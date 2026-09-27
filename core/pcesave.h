/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */

// PC Engine backup RAM <-> SD card. See pcesave.cpp.
#pragma once
#include <stdint.h>

void pcesave_init(void);                        // create the save task; call once at boot
void pcesave_set_game(const char *fname);       // the image the user picked (.pce/.sgx/.chd)
void pcesave_restore(void);                     // send the save into the FPGA; core must not run yet
void pcesave_flush_now(void);                   // save now if the game wrote since the last save

// Called from uart1_rx_task only.
void pcesave_rx_byte(uint16_t blk, uint16_t off, uint8_t b);
void pcesave_rx_block_done(uint16_t blk);
void pcesave_rx_dirty(void);
