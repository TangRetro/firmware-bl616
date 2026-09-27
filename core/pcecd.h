/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */
// PC Engine CD: serves a .chd disc image to pcetang's emulated CD drive.
#pragma once
#include <stdint.h>

int loadpcecd(const char *fname);           // mount the disc, load the System Card
void pcecd_unload(void);
bool pcecd_is_mounted(void);
void pcecd_serve_sector(uint32_t lba);      // 2048-byte data sector
void pcecd_serve_audio_sector(uint32_t lba);// 2352-byte CD-DA sector
void pcecd_tx_dma_init(void);               // once at boot, after UART1 is up
