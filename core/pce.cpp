/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 *
 * Loader structure follows nand2mario's core/*.cpp loaders in
 * nand2mario/firmware-bl616 (Apache-2.0).
 */

#define _GNU_SOURCE
#include <string.h>      // for strcasestr
#include <stdio.h>

#include "tc_utils.h"
#include "cores.h"
#include "overlay.h"
#include "pcecd.h"
#include "coresave.h"
#include "wifi_debug.h"

extern void file_log(const char *msg);   // Diagnostic, defined in main.cpp

// Load a PC Engine / TurboGrafx-16 HuCard ROM (pcetang core)
// return 0 if successful
int loadpce(const char *fname) {
    unsigned int off = 0, br, total = 0;
    unsigned int size;
    int r = 1;
    DEBUG("loadpce start\n");
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "loadpce: start fname=%s", fname);
        file_log(buf);
        wifi_log(buf);
    }

    // check extension .pce or .sgx
    //
    // SuperGrafx ROMs are conventionally named .sgx and are otherwise the SAME HuCard image
    // format -- header-less, optionally with a 512-byte copier header, loaded to the same
    // place. The core decides what to do with them (the SGX generic in the bitstream), so the
    // loader only has to accept the name. Without this every .sgx was refused with
    // "Only .pce supported" and SuperGrafx titles could not be loaded at all.
    char *p = strcasestr(fname, ".pce");
    if (p == NULL) p = strcasestr(fname, ".sgx");
    if (p == NULL) {
        file_log("loadpce: not a .pce/.sgx, abort");
        wifi_log("loadpce: not a .pce/.sgx, abort");
        overlay_message("Only .pce and .sgx supported", 1);
        goto loadpce_end;
    }

    r = f_open(&fcore, fname, FA_READ);
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "loadpce: f_open returned %d", r);
        file_log(buf);
        wifi_log(buf);
    }
    if (r) {
        overlay_status("Cannot open file");
        goto loadpce_end;
    }
    size = get_file_size(fname);
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "loadpce: size=%u", size);
        file_log(buf);
        wifi_log(buf);
    }

    // Some .pce dumps carry a 512-byte copier header ahead of the real HuCard
    // image (same convention real emulators use to detect it): a clean dump's
    // size is a whole multiple of 1KB, so a size that is off by exactly 512
    // bytes means a header is present. Skip it -- pcetang's own dynamic
    // ROM_SZ latch (pce_top.vhd) tracks byte count from whatever is streamed
    // here, so an unskipped header would silently shift and undersize the
    // whole image on the FPGA side.
    if ((size % 1024) == 512) {
        off = 512;
        size -= 512;
        DEBUG("loadpce: 512-byte copier header detected, skipping\n");
    }

    // load actual ROM
    set_loading_state(1);
    core_running = false;

    // Send rom content
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        file_log("loadpce: seek failure");
        wifi_log("loadpce: seek failure");
        overlay_status("Seek failure");
        goto loadpce_pce_end;
    }

    do {
        if ((r = f_read(&fcore, fbuf, 1024 /*BLOCK_SIZE*/, &br)) != FR_OK)
            break;
        // start rom loading command
        send_fbuf_data(br);
        taskYIELD();                // allow gamepad polling to run
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024 /*BLOCK_SIZE*/ && total < size);

    DEBUG("loadpce: %d bytes\n", total);
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "loadpce: sent total=%u bytes, r=%d", total, r);
        file_log(buf);
        wifi_log(buf);
    }
    overlay_status("Success");
    // Backup RAM goes in while the core is still held in the loading state. For a CD game
    // this runs for the syscard, but pcesave_set_game() was given the .chd, so the save is
    // the disc's, not the BIOS's.
    coresave_restore();
    core_running = true;

    overlay(0);		// turn off OSD
    file_log("loadpce: overlay off, core_running=true, success");
    wifi_log("loadpce: overlay off, core_running=true, success");

loadpce_pce_end:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
loadpce_end:
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "loadpce: returning %d", r);
        file_log(buf);
        wifi_log(buf);
    }
    return r;
}

// PC Engine and PC Engine CD share one bitstream (pcetang.bin) and one menu entry --
// extension is the real discriminator between a HuCard dump (.pce) and a CD image
// (.chd), checked here before either loader's own file-open path runs.
int loadpce_dispatch(const char *fname) {
    pcesave_set_game(fname);    // the image the user picked -- names the save file
    {
        char buf[160];
        snprintf(buf, sizeof(buf), "loadpce_dispatch: fname=%s", fname);
        file_log(buf);
        wifi_log(buf);
    }
    if (strcasestr(fname, ".chd") != NULL) {
        file_log("loadpce_dispatch: .chd -> loadpcecd");
        wifi_log("loadpce_dispatch: .chd -> loadpcecd");
        return loadpcecd(fname);
    }
    file_log("loadpce_dispatch: not .chd -> loadpce path (.pce/.sgx)");
    wifi_log("loadpce_dispatch: not .chd -> loadpce path (.pce/.sgx)");
    // Drop any disc left mounted from a prior .chd load -- but only if something
    // actually is mounted. pcecd_unload() always sends a mount(0) frame to the FPGA,
    // which is fine once the CD core is already listening (its 3 existing callers)
    // but not right after a fresh fpga_program(), before loadpce() below has even
    // reached set_loading_state(1).
    if (pcecd_is_mounted()) {
        file_log("loadpce_dispatch: disc was mounted, unloading first");
        wifi_log("loadpce_dispatch: disc was mounted, unloading first");
        pcecd_unload();
    }
    int r = loadpce(fname);
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "loadpce_dispatch: loadpce returned %d", r);
        file_log(buf);
        wifi_log(buf);
    }
    return r;
}
