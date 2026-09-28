/*
 * Modifications copyright (c) 2026 Romain Tisserand.
 * Originally from nand2mario/firmware-bl616 (Apache-2.0); this file has been
 * modified in the pcetang fork.
 * SPDX-License-Identifier: Apache-2.0
 */

#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "tc_utils.h"
#include "cores.h"
#include "overlay.h"
#include "coresave.h"

// NES region for the core's "Region: Auto" (core_config[4:3]: 1 NTSC, 2 PAL, 3 Dendy).
// NES 2.0 header timing first; then iNES byte 9 bit 0, trusted only when bytes 12-15 are
// clean (old dumps carry tool signatures there); then the file name (No-Intro / GoodNES
// region tags). Default NTSC. "(USA, Europe)" style multi-region names run as NTSC.
// True if `tag` is one entry of a "(...)" list in the name: "(USA)", "(USA, Europe)", "(E)".
static bool region_tag(const char *name, const char *tag) {
    size_t n = strlen(tag);
    for (const char *p = strchr(name, '('); p; p = strchr(p + 1, '(')) {
        const char *q = p + 1;
        for (;;) {
            const char *end = q + strcspn(q, ",)");
            if ((size_t)(end - q) == n && strncasecmp(q, tag, n) == 0) return true;
            if (*end != ',') break;
            q = end + 1;
            while (*q == ' ') q++;
        }
    }
    return false;
}

static int nes_region(const uint8_t *h, const char *fname) {
    if ((h[7] & 0x0C) == 0x08) {                                    // NES 2.0
        int t = h[12] & 3;
        return t == 1 ? 2 : t == 3 ? 3 : 1;                         // 2 = multi-region: NTSC
    }
    if ((h[9] & 1) && !h[12] && !h[13] && !h[14] && !h[15])
        return 2;
    const char *base = strrchr(fname, '/');
    base = base ? base + 1 : fname;
    static const char *ntsc_tags[] = { "USA", "Japan", "U", "J", "JU", "UE", "JUE", "World" };
    static const char *pal_tags[] = { "Europe", "E", "PAL", "Australia", "Germany", "France",
        "Spain", "Italy", "Sweden", "Scandinavia", "Netherlands", "UK", "G", "F", "A" };
    for (const char *t : ntsc_tags) if (region_tag(base, t)) return 1;
    for (const char *t : pal_tags)  if (region_tag(base, t)) return 2;
    return 1;
}

// Load a NES ROM
// return 0 if successful
int loadnes(const char *fname) {
    unsigned int off = 0, br, total = 0;
    unsigned int size;
    int r = 1;
    bool battery = false;
    DEBUG("loadnes start\n");

    // check extension .nes
    char *p = strcasestr(fname, ".nes");
    if (p == NULL) {
        overlay_message("Only .nes supported", 1);
        goto loadnes_end;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        goto loadnes_end;
    }
    size = get_file_size(fname);

    // load actual ROM
    set_loading_state(1);
    core_running = false;

    // Send rom content
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadnes_snes_end;
    }


    do {
        if ((r = f_read(&fcore, fbuf, 1024 /*BLOCK_SIZE*/, &br)) != FR_OK)
            break;
        if (total == 0 && br >= 16 && memcmp(fbuf, "NES\x1a", 4) == 0) {
            // iNES header byte 6 bit 1: the cart has battery-backed RAM at $6000
            battery = (fbuf[6] & 0x02) != 0;
            // Region: resolved here for "Auto" and sent while the core is still loading
            // (a region change resets the NES).
            int region = nes_region((const uint8_t *)fbuf, fname);
            DEBUG("loadnes: region %d\n", region);
            set_core_config_auto(3u << 3, (uint32_t)region << 3);
            set_core_config(get_core_config());
        }
        // start rom loading command
        send_fbuf_data(br);
        taskYIELD();                // allow gamepad polling to run
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024 /*BLOCK_SIZE*/);

    DEBUG("loadnes: %d bytes\n", total);

    // Battery RAM: MiSTer NES's .sav (32 KB, saves/nes/<game>.sav). Sent while the core is
    // still held in loading, after the ROM; a bitstream without the save-RAM channel ignores
    // it and never asks for a save.
    if (r == FR_OK && battery) {
        coresave_set_game(fname, "nes", 32768, nullptr, 0);
        coresave_restore();
    } else {
        coresave_no_game();
    }

    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadnes_snes_end:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
loadnes_end:
    return r;
}
