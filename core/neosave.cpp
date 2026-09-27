// Neo Geo backup RAM <-> SD card (NeoTang, cart and CD bitstreams).
//
// The FPGA side is iosys_bl616's save channel, as for the PC Engine (pcesave.cpp), but on
// Neo Geo command codes -- 0x11 is the ROM region header there:
//   MCU -> FPGA 0x13 blk[15:0] <512 bytes>   write one block (restore)
//   MCU -> FPGA 0x14 blk[15:0]               request one block
//   FPGA -> MCU 0x0A blk[15:0] <512 bytes>   the requested block        (same as PCE)
//   FPGA -> MCU 0x0B 0x00                    backup RAM written          (same as PCE)
//
// The file is MiSTer's Neo Geo .sav, so saves move between the two: 72 KB, 0x00000-0x0FFFF
// the MVS backup SRAM (64 KB), 0x10000-0x11FFF the memory card (8 KB), each 16-bit RAM word
// low byte first. The CD console has only the card (its saves), so only blocks 0x80-0x8F
// travel; the rest of its file stays zero. Path: <drive>saves/neogeo[cd]/<game>.sav.
//
// Unlike pcesave (2 KB, two full-image buffers) this streams one 512-byte block at a time:
// 72 KB twice would not fit the MCU. A save is two passes: dump and checksum every block;
// only if one differs from what the file holds, dump again straight into the file.
#include "neosave.h"
#include "tc_utils.h"
#include "ff.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern void file_log(const char *msg);

static const uint16_t NS_BLK = 512, NS_FILE_BLOCKS = 0x90;      // 72 KB
static const uint16_t NS_CARD_FIRST = 0x80;                      // memory card: 0x80-0x8F

static uint8_t  ns_rx[512];                      // the block being dumped (RX task fills it)
static uint8_t  ns_io[512];                      // file I/O buffer
static uint32_t ns_sum[0x90];                    // checksum of each block as the file holds it
static char     ns_path[256], ns_dir[256], ns_root[256];
static uint16_t ns_first = 0, ns_count = 0;      // the blocks this bitstream has
static bool     ns_on = false;
static SemaphoreHandle_t ns_mutex, ns_blk_sem;
static TaskHandle_t ns_task;
static volatile uint16_t ns_rx_blk = 0xFFFF;
static volatile bool ns_dirty = false;           // set only by the FPGA's 0x0B notice

static void ns_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void ns_log(const char *fmt, ...) {
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    file_log(b);
}

static uint32_t ns_fnv(const uint8_t *p, uint16_t n) {           // FNV-1a
    uint32_t h = 2166136261u;
    while (n--) { h ^= *p++; h *= 16777619u; }
    return h;
}

// ---- RX task hooks (cheap: no SD I/O) ----
bool neosave_active(void) { return ns_on; }
void neosave_off(void) { ns_on = false; ns_dirty = false; }
void neosave_rx_byte(uint16_t blk, uint16_t off, uint8_t b) {
    if (off < NS_BLK) ns_rx[off] = b;
    (void)blk;
}
void neosave_rx_block_done(uint16_t blk) {
    ns_rx_blk = blk;
    if (ns_blk_sem) xSemaphoreGive(ns_blk_sem);
}
void neosave_rx_dirty(void) {
    ns_dirty = true;
    if (ns_task) xTaskNotifyGive(ns_task);
}

// ---- FPGA side ----
static void ns_send_block(uint16_t blk, const uint8_t *data) {      // 0x13
    fpga_tx_lock();
    fpga_tx_header(0x13, 1 + 2 + NS_BLK);
    fpga_tx_byte(blk >> 8); fpga_tx_byte(blk & 0xff);
    for (uint16_t i = 0; i < NS_BLK; i++) fpga_tx_byte(data[i]);
    fpga_tx_unlock();
}
static bool ns_fetch_block(uint16_t blk) {                          // 0x14 -> ns_rx
    xSemaphoreTake(ns_blk_sem, 0);                                  // drop a stale give
    fpga_tx_lock();
    fpga_tx_header(0x14, 3);
    fpga_tx_byte(blk >> 8); fpga_tx_byte(blk & 0xff);
    fpga_tx_unlock();
    // 515 bytes at 2 Mbaud ~2.6 ms; CD sector requests go first in the FPGA
    if (xSemaphoreTake(ns_blk_sem, pdMS_TO_TICKS(1000)) != pdTRUE || ns_rx_blk != blk) {
        ns_log("neosave: dump block %02x timed out", blk);
        return false;
    }
    return true;
}

// ---- API ----
void neosave_set_game(const char *fname, bool cd) {
    if (!ns_mutex) return;
    xSemaphoreTake(ns_mutex, portMAX_DELAY);
    const char *colon = strchr(fname, ':');
    char prefix[16] = "";
    if (colon && colon - fname < (int)sizeof prefix - 1) { memcpy(prefix, fname, colon - fname + 1); prefix[colon - fname + 1] = 0; }
    else snprintf(prefix, sizeof prefix, "%s", drv);
    const char *base = strrchr(fname, '/'); base = base ? base + 1 : (colon ? colon + 1 : fname);
    char name[200]; snprintf(name, sizeof name, "%s", base);
    char *dot = strrchr(name, '.'); if (dot) *dot = 0;
    snprintf(ns_root, sizeof ns_root, "%ssaves", prefix);
    snprintf(ns_dir, sizeof ns_dir, "%ssaves/%s", prefix, cd ? "neogeocd" : "neogeo");
    snprintf(ns_path, sizeof ns_path, "%s/%s.sav", ns_dir, name);
    ns_first = cd ? NS_CARD_FIRST : 0;
    ns_count = NS_FILE_BLOCKS - ns_first;
    ulTaskNotifyTake(pdTRUE, 0);                                    // drop the old game's pending save
    ns_dirty = false;
    ns_on = true;
    xSemaphoreGive(ns_mutex);
    ns_log("neosave: game set, save file %s (%u blocks from %02x)", ns_path, ns_count, ns_first);
}

void neosave_restore(void) {
    if (!ns_mutex || !ns_on) return;
    xSemaphoreTake(ns_mutex, portMAX_DELAY);
    char tmp[270]; snprintf(tmp, sizeof tmp, "%s.tmp", ns_path);
    FIL f; bool have = false;
    const char *src = "none (blank)";
    if (f_open(&f, ns_path, FA_READ) == FR_OK) { have = true; src = "file"; }
    else if (f_open(&f, tmp, FA_READ) == FR_OK) { have = true; src = "file.tmp (recovered)"; }
    if (have && f_lseek(&f, (FSIZE_t)ns_first * NS_BLK) != FR_OK) { f_close(&f); have = false; src = "none (short file)"; }
    // Always send every block: the RAM may still hold the previous game's data.
    for (uint16_t i = 0; i < ns_count; i++) {
        UINT br = 0;
        if (!have || f_read(&f, ns_io, NS_BLK, &br) != FR_OK || br != NS_BLK) memset(ns_io, 0, NS_BLK);
        ns_sum[ns_first + i] = ns_fnv(ns_io, NS_BLK);
        ns_send_block(ns_first + i, ns_io);
    }
    if (have) f_close(&f);
    xSemaphoreGive(ns_mutex);
    ns_log("neosave: restored from %s", src);
}

void neosave_flush_now(void) {
    if (!ns_mutex || !ns_on || !ns_dirty) return;
    if (xSemaphoreTake(ns_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return;
    // Clear BEFORE the dump: the FPGA clears its flag when block 0 is requested (cart), and a
    // write during the dump sends a fresh 0x0B that sets this again.
    ns_dirty = false;
    bool changed = false, ok = true;
    for (uint16_t i = 0; i < ns_count && ok; i++) {
        uint16_t blk = ns_first + i;
        if (!ns_fetch_block(blk)) ok = false;
        else if (ns_fnv(ns_rx, NS_BLK) != ns_sum[blk]) changed = true;
    }
    if (ok && changed) {
        // Second pass straight into the file: write-then-rename, so a power cut mid-write
        // leaves the previous save intact.
        f_mkdir(ns_root); f_mkdir(ns_dir);
        char tmp[270]; snprintf(tmp, sizeof tmp, "%s.tmp", ns_path);
        FIL f; UINT bw = 0;
        if (f_open(&f, tmp, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) { ns_log("neosave: cannot create %s", tmp); ok = false; }
        else {
            static uint32_t sum[0x90];
            memset(ns_io, 0, NS_BLK);
            for (uint16_t blk = 0; blk < ns_first && ok; blk++)              // CD: no SRAM, zeros
                if (f_write(&f, ns_io, NS_BLK, &bw) != FR_OK || bw != NS_BLK) ok = false;
            for (uint16_t i = 0; i < ns_count && ok; i++) {
                uint16_t blk = ns_first + i;
                if (!ns_fetch_block(blk)) { ok = false; break; }
                sum[blk] = ns_fnv(ns_rx, NS_BLK);
                if (f_write(&f, ns_rx, NS_BLK, &bw) != FR_OK || bw != NS_BLK) ok = false;
            }
            f_sync(&f); f_close(&f);
            if (ok) {
                f_unlink(ns_path);
                if (f_rename(tmp, ns_path) != FR_OK) { ns_log("neosave: rename to %s failed", ns_path); ok = false; }
                else {
                    for (uint16_t i = 0; i < ns_count; i++) ns_sum[ns_first + i] = sum[ns_first + i];
                    ns_log("neosave: saved %s", ns_path);
                }
            } else ns_log("neosave: write to %s failed", tmp);
        }
    }
    if (!ok) ns_dirty = true;                                       // try again next time
    xSemaphoreGive(ns_mutex);
}

static void neosave_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                    // the game wrote backup RAM
        // Debounce: wait until it has been quiet for 2 s, so one save covers a whole burst.
        while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) > 0) {}
        neosave_flush_now();
    }
}

void neosave_init(void) {
    ns_mutex = xSemaphoreCreateMutex();
    ns_blk_sem = xSemaphoreCreateBinary();
    xTaskCreate(neosave_task, "neosave", 4096, NULL, 1, &ns_task);
}
