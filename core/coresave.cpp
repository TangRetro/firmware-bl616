/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */

// Save RAM survives a power-off: PC Engine backup RAM, NES battery RAM, and any other core
// that exposes its save RAM the same way.
//
// The FPGA exposes the core's save RAM through iosys_bl616's save-RAM channel:
//   MCU -> FPGA 0x11 blk[15:0] <512 bytes>   write one block (restore)
//   MCU -> FPGA 0x12 blk[15:0]               request one block
//   FPGA -> MCU 0x0A blk[15:0] <512 bytes>   the requested block
//   FPGA -> MCU 0x0B 0x00                    the game wrote backup RAM since the last dump
//
// File layout follows MiSTer, so saves move between the two: the raw RAM image, one file per
// game. Path: <drive>saves/<core>/<game name>.sav
//   pce: 2 KB (TurboGrafx16).  nes: 32 KB, MiSTer NES's 64-sector .sav.
// A shorter existing file (MiSTer NES writes 8 KB for NES 2.0 carts that declare it) is read
// as far as it goes, the rest zero-filled.
//
// WHEN WE SAVE
//   - 2 s after the game's last backup-RAM write (the 0x0B notice, debounced). This is the
//     part MiSTer lacks: it saves only when the OSD opens, so pulling the plug without
//     opening the menu loses the save.
//   - when the in-game OSD opens (MiSTer's autosave point). Opening the OSD is the ONLY way
//     to switch games, so this also guarantees the old game is saved before a new one loads.
// WHEN WE NEVER SAVE
//   - during a game switch. By then the FPGA may have been reprogrammed and its backup RAM
//     reset to the blank default; dumping it would overwrite a real save with an empty one.
//     coresave_set_game() therefore only DROPS pending state, it never flushes.
#include "coresave.h"
#include "tc_utils.h"
#include "ff.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern void file_log(const char *msg);

static const uint16_t SV_BLK = 512;
static uint32_t sv_size = 0;                    // this game's save RAM, bytes
static uint16_t sv_blocks = 0;
static const uint8_t *sv_blank = nullptr;       // start of a fresh image
static uint32_t sv_blank_len = 0;
// beetle-pce-fast's BRAM_Init_String: a formatted, empty backup RAM.
static const uint8_t HUBM[8] = { 'H', 'U', 'B', 'M', 0x00, 0x88, 0x10, 0x80 };

static uint8_t sv_last[CORESAVE_MAX];           // what the SD card holds (skip identical writes)
static uint8_t sv_rx[CORESAVE_MAX];             // filled by the RX task during a dump
static char sv_path[256];                       // "" = no game with save RAM loaded
static char sv_dir[256];                        // "<drive>saves/<core>"
static char sv_root[256];                       // "<drive>saves"
static SemaphoreHandle_t sv_mutex, sv_blk_sem;
static TaskHandle_t sv_task;
static volatile uint16_t sv_rx_blk = 0xFFFF;
// Set only by the FPGA's 0x0B notice. Gates every dump, so a core without the save-RAM
// interface (or an older bitstream) is never asked for blocks it cannot send -- that
// would stall the OSD for the dump timeout.
static volatile bool sv_dirty = false;

static void sv_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void sv_log(const char *fmt, ...) {
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    file_log(b);
}

// ---- RX task hooks (must stay cheap: no SD I/O here) ----
void coresave_rx_byte(uint16_t blk, uint16_t off, uint8_t b) {
    if (blk < sv_blocks && off < SV_BLK) sv_rx[blk * SV_BLK + off] = b;
}
void coresave_rx_block_done(uint16_t blk) {
    sv_rx_blk = blk;
    if (sv_blk_sem) xSemaphoreGive(sv_blk_sem);
}
void coresave_rx_dirty(void) {
    sv_dirty = true;
    if (sv_task) xTaskNotifyGive(sv_task);
}

// ---- helpers ----
static void sv_send_block(uint16_t blk, const uint8_t *data) {      // 0x11
    fpga_tx_lock();
    fpga_tx_header(0x11, 1 + 2 + SV_BLK);
    fpga_tx_byte(blk >> 8); fpga_tx_byte(blk & 0xff);
    for (uint16_t i = 0; i < SV_BLK; i++) fpga_tx_byte(data[i]);
    fpga_tx_unlock();
}

// Read the whole save RAM back from the FPGA into sv_rx. False on any missing block.
static bool sv_dump(void) {
    for (uint16_t blk = 0; blk < sv_blocks; blk++) {
        xSemaphoreTake(sv_blk_sem, 0);                              // drop a stale give
        fpga_tx_lock();
        fpga_tx_header(0x12, 3);
        fpga_tx_byte(blk >> 8); fpga_tx_byte(blk & 0xff);
        fpga_tx_unlock();
        // 515 bytes at 2 Mbaud is ~2.6 ms; CD sector requests take priority in the FPGA,
        // so allow generously before calling it lost.
        if (xSemaphoreTake(sv_blk_sem, pdMS_TO_TICKS(1000)) != pdTRUE || sv_rx_blk != blk) {
            sv_log("coresave: dump block %u timed out, save NOT written", blk);
            return false;
        }
    }
    return true;
}

static bool sv_write_file(const uint8_t *img) {
    f_mkdir(sv_root);                                               // FR_EXIST is fine
    f_mkdir(sv_dir);
    char tmp[270]; snprintf(tmp, sizeof tmp, "%s.tmp", sv_path);
    FIL f; UINT bw = 0;
    if (f_open(&f, tmp, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) { sv_log("coresave: cannot create %s", tmp); return false; }
    FRESULT r = f_write(&f, img, sv_size, &bw);
    f_sync(&f); f_close(&f);
    if (r != FR_OK || bw != sv_size) { sv_log("coresave: short write %u to %s", bw, tmp); return false; }
    // Write-then-rename, so a power cut mid-write leaves the previous save intact.
    // (FatFs rename refuses an existing target, hence the unlink; coresave_restore also
    // accepts a lone .tmp, which covers a cut between the two.)
    f_unlink(sv_path);
    if (f_rename(tmp, sv_path) != FR_OK) { sv_log("coresave: rename to %s failed", sv_path); return false; }
    return true;
}

// A file shorter than the save RAM is read as far as it goes and zero-filled (MiSTer NES
// writes 8 KB for carts that declare it). An empty or unreadable file does not count.
static bool sv_read_file(const char *path, uint8_t *img) {
    FIL f; UINT br = 0;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;
    memset(img, 0, sv_size);
    FRESULT r = f_read(&f, img, sv_size, &br);
    f_close(&f);
    return r == FR_OK && br > 0;
}

// ---- API ----
void coresave_set_game(const char *fname, const char *subdir, uint32_t size,
                       const uint8_t *blank, uint32_t blank_len) {
    if (!sv_mutex) return;
    if (size > CORESAVE_MAX) size = CORESAVE_MAX;
    xSemaphoreTake(sv_mutex, portMAX_DELAY);
    sv_size = size & ~(uint32_t)(SV_BLK - 1);
    sv_blocks = sv_size / SV_BLK;
    sv_blank = blank; sv_blank_len = blank_len;
    // Drive prefix from the path itself ("sd:" / "usb:"), else the mounted drive.
    const char *colon = strchr(fname, ':');
    char prefix[16] = "";
    if (colon && colon - fname < (int)sizeof prefix - 1) { memcpy(prefix, fname, colon - fname + 1); prefix[colon - fname + 1] = 0; }
    else snprintf(prefix, sizeof prefix, "%s", drv);
    const char *base = strrchr(fname, '/'); base = base ? base + 1 : (colon ? colon + 1 : fname);
    char name[200]; snprintf(name, sizeof name, "%s", base);
    char *dot = strrchr(name, '.'); if (dot) *dot = 0;
    snprintf(sv_root, sizeof sv_root, "%ssaves", prefix);
    snprintf(sv_dir, sizeof sv_dir, "%ssaves/%s", prefix, subdir);
    snprintf(sv_path, sizeof sv_path, "%s/%s.sav", sv_dir, name);
    ulTaskNotifyTake(pdTRUE, 0);                                    // drop the old game's pending save
    sv_dirty = false;
    xSemaphoreGive(sv_mutex);
    sv_log("coresave: game set, save file %s (%u bytes)", sv_path, (unsigned)sv_size);
}

void coresave_no_game(void) {
    if (!sv_mutex) return;
    xSemaphoreTake(sv_mutex, portMAX_DELAY);
    sv_path[0] = 0;                                                 // never saves, never restores
    ulTaskNotifyTake(pdTRUE, 0);
    sv_dirty = false;
    xSemaphoreGive(sv_mutex);
}

void pcesave_set_game(const char *fname) {
    coresave_set_game(fname, "pce", 2048, HUBM, sizeof HUBM);
}

void coresave_restore(void) {
    if (!sv_mutex || !sv_path[0]) return;
    xSemaphoreTake(sv_mutex, portMAX_DELAY);
    char tmp[270]; snprintf(tmp, sizeof tmp, "%s.tmp", sv_path);
    const char *src = "none";
    if (sv_read_file(sv_path, sv_last)) src = "file";
    else if (sv_read_file(tmp, sv_last)) src = "file.tmp (recovered)";
    else {
        memset(sv_last, 0, sv_size);
        if (sv_blank) memcpy(sv_last, sv_blank, sv_blank_len < sv_size ? sv_blank_len : sv_size);
        src = "blank image";
    }
    // Always send SOMETHING: the save RAM may still hold the previous game's data if the
    // bitstream was not reloaded between games.
    for (uint16_t blk = 0; blk < sv_blocks; blk++) sv_send_block(blk, sv_last + blk * SV_BLK);
    xSemaphoreGive(sv_mutex);
    sv_log("coresave: restored %s from %s", sv_path, src);
}

void coresave_flush_now(void) {
    if (!sv_mutex || !sv_path[0] || !sv_dirty) return;
    if (xSemaphoreTake(sv_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return;
    // Clear BEFORE the dump: the FPGA clears its own flag when block 0 is requested, and a
    // write that lands during the dump sends a fresh 0x0B that sets this again.
    sv_dirty = false;
    if (!sv_dump()) {
        sv_dirty = true;                                            // try again next time
    } else {
        if (memcmp(sv_rx, sv_last, sv_size) == 0) {
            // unchanged: no SD write (saves card wear; the syscard touches BRAM at boot)
        } else if (sv_write_file(sv_rx)) {
            memcpy(sv_last, sv_rx, sv_size);
            sv_log("coresave: saved %s", sv_path);
        } else {
            sv_dirty = true;                                        // SD write failed: retry
        }
    }
    xSemaphoreGive(sv_mutex);
}

static void coresave_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                    // the game wrote backup RAM
        // Debounce: wait until it has been quiet for 2 s, so one save covers a whole burst.
        while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) > 0) {}
        coresave_flush_now();
    }
}

void coresave_init(void) {
    sv_mutex = xSemaphoreCreateMutex();
    sv_blk_sem = xSemaphoreCreateBinary();
    xTaskCreate(coresave_task, "coresave", 4096, NULL, 1, &sv_task);
}
