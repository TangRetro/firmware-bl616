/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */

// Battery / backup save RAM <-> SD card, for any core with iosys_bl616's save-RAM channel.
// See coresave.cpp.
#pragma once
#include <stdint.h>

void coresave_init(void);                       // create the save task; call once at boot
// The game the user picked. subdir names saves/<subdir>/; size is the save RAM in bytes (a
// multiple of 512, at most CORESAVE_MAX); blank/blank_len is the start of the image sent
// when there is no save file yet (the rest is zeros).
#define CORESAVE_MAX 32768
void coresave_set_game(const char *fname, const char *subdir, uint32_t size,
                       const uint8_t *blank, uint32_t blank_len);
void coresave_no_game(void);                    // the loaded game has no save RAM
void coresave_restore(void);                    // send the save into the FPGA; core must not run yet
void coresave_flush_now(void);                  // save now if the game wrote since the last save

// PC Engine: 2 KB backup RAM, MiSTer TurboGrafx16 layout.
void pcesave_set_game(const char *fname);

// Called from uart1_rx_task only.
void coresave_rx_byte(uint16_t blk, uint16_t off, uint8_t b);
void coresave_rx_block_done(uint16_t blk);
void coresave_rx_dirty(void);
