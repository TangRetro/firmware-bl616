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

int loadsms(const char *fname) {
    DEBUG("loadsms start\n");
    FRESULT r = FR_NO_FILE;

    // check extension .bin
    char *p = strcasestr(fname, ".sms");
    if (p == NULL) {
        overlay_message("Only .sms, .gg, and .sg supported", 1);
        return r;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }
    unsigned int off = 0, br, total = 0;
    unsigned int size = get_file_size(fname);
    off = size % 1024;          // skipping 512-byte header if there is one

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets the core
    core_running = false;

    // Send rom content to core
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadmd_close_file;
    }
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        send_fbuf_data(br);
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024);

    DEBUG("loadsms: %d bytes\n", total);

    // Cart RAM: 32 KB, MiSTer SMS's .sav, saves/sms/<game>.sav. SMS headers carry no
    // battery flag, so every game gets its RAM restored (zeros if no file), and a file is
    // only written once the game actually writes cart RAM (the core's 0x0B notice).
    if (r == FR_OK) {
        coresave_set_game(fname, "sms", 32768, nullptr, 0);
        coresave_restore();
    }

    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadmd_close_file:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
    return r;
}

