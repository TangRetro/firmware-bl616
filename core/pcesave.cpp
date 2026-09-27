/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */

// PC Engine backup RAM survives a power-off.
//
// The FPGA exposes the core's 2 KB backup RAM through iosys_bl616's save-RAM channel:
//   MCU -> FPGA 0x11 blk[15:0] <512 bytes>   write one block (restore)
//   MCU -> FPGA 0x12 blk[15:0]               request one block
//   FPGA -> MCU 0x0A blk[15:0] <512 bytes>   the requested block
//   FPGA -> MCU 0x0B 0x00                    the game wrote backup RAM since the last dump
//
// File layout follows MiSTer TurboGrafx16 (this core's donor): the raw 2 KB image, one file
// per game, so saves move between the two. Path: <drive>saves/pce/<game name>.sav
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
//     pcesave_set_game() therefore only DROPS pending state, it never flushes.
#include "pcesave.h"
#include "utils.h"
#include "overlay.h"
#include "ff.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern const char *drv;

static const uint16_t SV_BLOCKS = 4, SV_BLK = 512, SV_SIZE = SV_BLOCKS * SV_BLK;
// beetle-pce-fast's BRAM_Init_String: a formatted, empty backup RAM.
static const uint8_t HUBM[8] = { 'H', 'U', 'B', 'M', 0x00, 0x88, 0x10, 0x80 };

static uint8_t sv_last[SV_SIZE];                // what the SD card holds (skip identical writes)
static uint8_t sv_rx[SV_SIZE];                  // filled by the RX task during a dump
static char sv_path[256];                       // "" = no PCE game loaded
static char sv_dir[256];                        // "<drive>saves/pce"
static char sv_root[256];                       // "<drive>saves"
static SemaphoreHandle_t sv_mutex, sv_blk_sem;
static TaskHandle_t sv_task;
static volatile uint16_t sv_rx_blk = 0xFFFF;
// Set only by the FPGA's 0x0B notice. Gates every dump, so a core without the save-RAM
// interface (any non-PCE core, or an older PCE bitstream) is never asked for blocks it
// cannot send -- that would stall the OSD for the dump timeout.
static volatile bool sv_dirty = false;

static void sv_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void sv_log(const char *fmt, ...) {
    char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    DEBUG("%s\n", b);
}

// ---- RX task hooks (must stay cheap: no SD I/O here) ----
void pcesave_rx_byte(uint16_t blk, uint16_t off, uint8_t b) {
    if (blk < SV_BLOCKS && off < SV_BLK) sv_rx[blk * SV_BLK + off] = b;
}
void pcesave_rx_block_done(uint16_t blk) {
    sv_rx_blk = blk;
    if (sv_blk_sem) xSemaphoreGive(sv_blk_sem);
}
void pcesave_rx_dirty(void) {
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

// Read the whole backup RAM back from the FPGA into sv_rx. False on any missing block.
static bool sv_dump(void) {
    for (uint16_t blk = 0; blk < SV_BLOCKS; blk++) {
        xSemaphoreTake(sv_blk_sem, 0);                              // drop a stale give
        fpga_tx_lock();
        fpga_tx_header(0x12, 3);
        fpga_tx_byte(blk >> 8); fpga_tx_byte(blk & 0xff);
        fpga_tx_unlock();
        // 515 bytes at 2 Mbaud is ~2.6 ms; CD sector requests take priority in the FPGA,
        // so allow generously before calling it lost.
        if (xSemaphoreTake(sv_blk_sem, pdMS_TO_TICKS(1000)) != pdTRUE || sv_rx_blk != blk) {
            sv_log("pcesave: dump block %u timed out, save NOT written", blk);
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
    if (f_open(&f, tmp, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) { sv_log("pcesave: cannot create %s", tmp); return false; }
    FRESULT r = f_write(&f, img, SV_SIZE, &bw);
    f_sync(&f); f_close(&f);
    if (r != FR_OK || bw != SV_SIZE) { sv_log("pcesave: short write %u to %s", bw, tmp); return false; }
    // Write-then-rename, so a power cut mid-write leaves the previous save intact.
    // (FatFs rename refuses an existing target, hence the unlink; pcesave_restore also
    // accepts a lone .tmp, which covers a cut between the two.)
    f_unlink(sv_path);
    if (f_rename(tmp, sv_path) != FR_OK) { sv_log("pcesave: rename to %s failed", sv_path); return false; }
    return true;
}

static bool sv_read_file(const char *path, uint8_t *img) {
    FIL f; UINT br = 0;
    if (f_open(&f, path, FA_READ) != FR_OK) return false;
    FRESULT r = f_read(&f, img, SV_SIZE, &br);
    f_close(&f);
    return r == FR_OK && br == SV_SIZE;
}

// ---- API ----
void pcesave_set_game(const char *fname) {
    if (!sv_mutex) return;
    xSemaphoreTake(sv_mutex, portMAX_DELAY);
    // Drive prefix from the path itself ("sd:" / "usb:"), else the mounted drive.
    const char *colon = strchr(fname, ':');
    char prefix[16] = "";
    if (colon && colon - fname < (int)sizeof prefix - 1) { memcpy(prefix, fname, colon - fname + 1); prefix[colon - fname + 1] = 0; }
    else snprintf(prefix, sizeof prefix, "%s", drv);
    const char *base = strrchr(fname, '/'); base = base ? base + 1 : (colon ? colon + 1 : fname);
    char name[200]; snprintf(name, sizeof name, "%s", base);
    char *dot = strrchr(name, '.'); if (dot) *dot = 0;
    snprintf(sv_root, sizeof sv_root, "%ssaves", prefix);
    snprintf(sv_dir, sizeof sv_dir, "%ssaves/pce", prefix);
    snprintf(sv_path, sizeof sv_path, "%s/%s.sav", sv_dir, name);
    ulTaskNotifyTake(pdTRUE, 0);                                    // drop the old game's pending save
    sv_dirty = false;
    xSemaphoreGive(sv_mutex);
    sv_log("pcesave: game set, save file %s", sv_path);
}

void pcesave_restore(void) {
    if (!sv_mutex || !sv_path[0]) return;
    xSemaphoreTake(sv_mutex, portMAX_DELAY);
    char tmp[270]; snprintf(tmp, sizeof tmp, "%s.tmp", sv_path);
    const char *src = "none";
    if (sv_read_file(sv_path, sv_last)) src = "file";
    else if (sv_read_file(tmp, sv_last)) src = "file.tmp (recovered)";
    else { memset(sv_last, 0, SV_SIZE); memcpy(sv_last, HUBM, sizeof HUBM); src = "blank formatted image"; }
    // Always send SOMETHING: the backup RAM may still hold the previous game's data if the
    // bitstream was not reloaded between games.
    for (uint16_t blk = 0; blk < SV_BLOCKS; blk++) sv_send_block(blk, sv_last + blk * SV_BLK);
    xSemaphoreGive(sv_mutex);
    sv_log("pcesave: restored from %s", src);
}

void pcesave_flush_now(void) {
    if (!sv_mutex || !sv_path[0] || !sv_dirty) return;
    if (xSemaphoreTake(sv_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) return;
    // Clear BEFORE the dump: the FPGA clears its own flag when block 0 is requested, and a
    // write that lands during the dump sends a fresh 0x0B that sets this again.
    sv_dirty = false;
    if (!sv_dump()) {
        sv_dirty = true;                                            // try again next time
    } else {
        if (memcmp(sv_rx, sv_last, SV_SIZE) == 0) {
            // unchanged: no SD write (saves card wear; the syscard touches BRAM at boot)
        } else if (sv_write_file(sv_rx)) {
            memcpy(sv_last, sv_rx, SV_SIZE);
            sv_log("pcesave: saved %s", sv_path);
        } else {
            sv_dirty = true;                                        // SD write failed: retry
        }
    }
    xSemaphoreGive(sv_mutex);
}

static void pcesave_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                    // the game wrote backup RAM
        // Debounce: wait until it has been quiet for 2 s, so one save covers a whole burst.
        while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) > 0) {}
        pcesave_flush_now();
    }
}

void pcesave_init(void) {
    sv_mutex = xSemaphoreCreateMutex();
    sv_blk_sem = xSemaphoreCreateBinary();
    xTaskCreate(pcesave_task, "pcesave", 4096, NULL, 1, &sv_task);
}
