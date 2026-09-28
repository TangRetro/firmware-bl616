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
        // iNES header byte 6 bit 1: the cart has battery-backed RAM at $6000
        if (total == 0 && br >= 16 && memcmp(fbuf, "NES\x1a", 4) == 0)
            battery = (fbuf[6] & 0x02) != 0;
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
