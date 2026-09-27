/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */
// PC Engine / TurboGrafx-16 HuCard loader for pcetang (core id 8).
#define _GNU_SOURCE
#include <string.h>

#include "utils.h"
#include "cores.h"
#include "overlay.h"

int loadpce(const char *fname) {
    DEBUG("loadpce start\n");
    FRESULT r = FR_NO_FILE;

    // .sgx is a SuperGrafx HuCard; the core handles both
    if (strcasestr(fname, ".pce") == NULL && strcasestr(fname, ".sgx") == NULL) {
        overlay_message("Only .pce or .sgx supported", 1);
        return r;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }
    unsigned int off = 0, br, total = 0;
    unsigned int size = get_file_size(fname);
    if ((size % 1024) == 512) {     // skip a 512-byte copier header
        off = 512;
        size -= 512;
    }

    set_loading_state(1);		// enable game loading, this resets the core
    core_running = false;

    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadpce_close_file;
    }
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        send_fbuf_data(br);
        taskYIELD();                // allow gamepad polling to run
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024 && total < size);

    DEBUG("loadpce: %d bytes\n", total);
    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadpce_close_file:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
    return r;
}
