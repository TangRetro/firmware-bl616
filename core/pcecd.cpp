/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */

// PC Engine CD for pcetang. The disc is a .chd image on the SD card; the FPGA's
// emulated CD drive asks for sectors and this file serves them.
//
// Protocol (iosys_bl616.v):
//   MCU -> FPGA 0x0e mounted[7:0]                          disc present or not
//   MCU -> FPGA 0x0f track control lba[23:0]               one TOC entry (track 100 = lead-out)
//   MCU -> FPGA 0x10 chunk[7:0] <payload>                  sector data: data sectors as two
//                                                          1024-byte chunks (0, 1), CD-DA as
//                                                          two 1176-byte chunks (0, 0xFF)
//   FPGA -> MCU 0x06 is_audio[7:0] lba[23:0]               sector request
// The System Card image (bios/syscard3.pce) is loaded as the HuCard.

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "FreeRTOS.h"
#include "task.h"

#include "utils.h"
#include "cores.h"
#include "overlay.h"
#include "pcecd.h"
#include "bflb_dma.h"
#include "bflb_uart.h"
#include "chd/chd_fatfs.h"
#include "libchdr/chd.h"

extern const char *drv;

#define PCECD_RAW_UNIT_BYTES     2448       // 2352 raw + 96 subcode per CHD unit
#define PCECD_USER_DATA_OFFSET   16         // Mode 1: sync(12) + header(4)
#define PCECD_USER_DATA_BYTES    2048
#define PCECD_AUDIO_BYTES        2352

// CD-DA sectors go out by DMA, so the next hunk can decode while the previous sector
// is still on the wire (11.8 ms at 2 Mbaud); a blocking send could not keep up with
// 75 sectors/s. The transfer holds the UART1 token and its completion interrupt gives
// it back. If a completion ever goes missing, DMA is turned off and sends fall back to
// blocking. The DMA engine does not snoop the D-cache, hence the nocache buffer.
#define PCECD_TX_CHUNK      (PCECD_AUDIO_BYTES / 2)
#define PCECD_TX_FRAME      (5 + PCECD_TX_CHUNK)        // 0xAA len16 cmd chunk + payload
#define PCECD_TX_DMA_WAIT_MS 250

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t pcecd_tx_stage[2 * PCECD_TX_FRAME];
static struct bflb_device_s *pcecd_tx_dma = NULL;
static struct bflb_dma_channel_lli_pool_s pcecd_tx_llipool[4];
static volatile uint8_t pcecd_dma_inflight = 0;

static void pcecd_tx_dma_isr(void *arg)
{
    (void)arg;
    if (!pcecd_dma_inflight) return;
    pcecd_dma_inflight = 0;
    bflb_uart_link_txdma(uart1_dev, false);     // UART1 back to CPU writes
    fpga_tx_unlock_from_isr();
}

void pcecd_tx_dma_init(void)
{
    struct bflb_dma_channel_config_s cfg;
    cfg.direction       = DMA_MEMORY_TO_PERIPH;
    cfg.src_req         = DMA_REQUEST_NONE;
    cfg.dst_req         = DMA_REQUEST_UART1_TX;
    cfg.src_addr_inc    = DMA_ADDR_INCREMENT_ENABLE;
    cfg.dst_addr_inc    = DMA_ADDR_INCREMENT_DISABLE;
    cfg.src_burst_count = DMA_BURST_INCR1;
    cfg.dst_burst_count = DMA_BURST_INCR1;
    cfg.src_width       = DMA_DATA_WIDTH_8BIT;
    cfg.dst_width       = DMA_DATA_WIDTH_8BIT;
    struct bflb_device_s *dev = bflb_device_get_by_name("dma0_ch0");
    if (dev == NULL) return;
    bflb_dma_channel_init(dev, &cfg);
    bflb_dma_channel_irq_attach(dev, pcecd_tx_dma_isr, NULL);
    pcecd_tx_dma = dev;
}

static void pcecd_send_sector_chunk(uint8_t chunk_idx, const uint8_t *data, uint16_t length) {
    fpga_tx_lock();
    fpga_tx_header(0x10, (int)length + 2);
    fpga_tx_byte(chunk_idx);
    for (uint16_t i = 0; i < length; i++)
        fpga_tx_byte(data[i]);
    fpga_tx_unlock();
}

static void pcecd_send_audio_blocking(const uint8_t *s) {
    pcecd_send_sector_chunk(0, s, PCECD_TX_CHUNK);
    pcecd_send_sector_chunk(0xFF, s + PCECD_TX_CHUNK, PCECD_TX_CHUNK);   // 0xFF = last
}

// Returns with the token held by the transfer; the completion interrupt releases it.
static void pcecd_send_audio_dma(const uint8_t *s)
{
    if (!fpga_tx_lock_timed(PCECD_TX_DMA_WAIT_MS)) {
        // the previous transfer never completed: give up on DMA for this session
        if (pcecd_tx_dma) bflb_dma_channel_stop(pcecd_tx_dma);
        bflb_uart_link_txdma(uart1_dev, false);
        pcecd_dma_inflight = 0;
        pcecd_tx_dma = NULL;
        fpga_tx_unlock();
        pcecd_send_audio_blocking(s);
        return;
    }
    uint8_t *d = pcecd_tx_stage;
    uint16_t n = 0;
    for (int c = 0; c < 2; c++) {
        uint16_t len = PCECD_TX_CHUNK + 2;      // cmd + chunk index + payload
        d[n++] = 0xAA;
        d[n++] = (uint8_t)(len >> 8);
        d[n++] = (uint8_t)(len & 0xFF);
        d[n++] = 0x10;
        d[n++] = (c == 0) ? 0x00 : 0xFF;
        memcpy(&d[n], s + c * PCECD_TX_CHUNK, PCECD_TX_CHUNK);
        n += PCECD_TX_CHUNK;
    }
    struct bflb_dma_channel_lli_transfer_s tr[1];
    tr[0].src_addr = (uint32_t)pcecd_tx_stage;
    tr[0].dst_addr = (uint32_t)DMA_ADDR_UART1_TDR;
    tr[0].nbytes   = n;
    pcecd_dma_inflight = 1;
    bflb_dma_channel_lli_reload(pcecd_tx_dma, pcecd_tx_llipool, 4, tr, 1);
    bflb_uart_link_txdma(uart1_dev, true);      // only for the length of this transfer
    bflb_dma_channel_start(pcecd_tx_dma);
}

static uint8_t pcecd_audio_buf[PCECD_AUDIO_BYTES];
static USB_NOCACHE_RAM_SECTION FIL f_chd;
static chd_file *pcecd_chd = NULL;
static const chd_header *pcecd_hdr = NULL;
static uint8_t *pcecd_hunk_buf = NULL;
static uint32_t pcecd_cached_hunk = 0xFFFFFFFF;
static uint32_t pcecd_sectors_per_hunk = 0;

// TOC. Index 1..99 = tracks, 100 = lead-out. A track's LBA and where its frames sit in
// the CHD file differ (pregaps not stored in the file, 4-frame padding per track), so
// both are kept.
#define PCECD_MAX_TRACKS 99
static uint32_t pcecd_toc_lba[101];
static uint8_t  pcecd_toc_control[101];
static uint32_t pcecd_toc_fofs[101];        // file frame of the track's first LBA
static uint32_t pcecd_toc_sectors[101];     // frames of the track present in the file
static int      pcecd_toc_num_tracks = 0;

static void pcecd_send_mount(uint8_t mounted) {
    fpga_tx_lock();
    fpga_tx_header(0x0e, 2);
    fpga_tx_byte(mounted);
    fpga_tx_unlock();
}

static void pcecd_send_toc_entry(uint8_t track, uint8_t control, uint32_t lba) {
    fpga_tx_lock();
    fpga_tx_header(0x0f, 6);
    fpga_tx_byte(track);
    fpga_tx_byte(control);
    fpga_tx_byte((lba >> 16) & 0xff);
    fpga_tx_byte((lba >> 8) & 0xff);
    fpga_tx_byte(lba & 0xff);
    fpga_tx_unlock();
}

static void pcecd_send_toc(void) {
    for (int t = 1; t <= pcecd_toc_num_tracks; t++)
        pcecd_send_toc_entry((uint8_t)t, pcecd_toc_control[t], pcecd_toc_lba[t]);
    pcecd_send_toc_entry(100, 0, pcecd_toc_lba[100]);
}

bool pcecd_is_mounted(void) {
    return pcecd_chd != NULL;
}

void pcecd_unload(void) {
    if (pcecd_chd) {
        chd_close(pcecd_chd);
        pcecd_chd = NULL;
        pcecd_hdr = NULL;
    }
    if (pcecd_hunk_buf) {
        free(pcecd_hunk_buf);
        pcecd_hunk_buf = NULL;
    }
    pcecd_cached_hunk = 0xFFFFFFFF;
    pcecd_sectors_per_hunk = 0;
    pcecd_send_mount(0);
}

static bool pcecd_lba_to_file_frame(uint32_t lba, uint32_t *out) {
    for (int t = 1; t <= pcecd_toc_num_tracks; t++) {
        uint32_t start = pcecd_toc_lba[t];
        if (lba >= start && lba < start + pcecd_toc_sectors[t]) {
            *out = pcecd_toc_fofs[t] + (lba - start);
            return true;
        }
    }
    return false;
}

// Decode the hunk holding `lba` (cached: consecutive sectors share a hunk) and return
// the start of its raw 2352-byte sector, or NULL.
static const uint8_t *pcecd_raw_sector(uint32_t lba) {
    if (!pcecd_chd || pcecd_sectors_per_hunk == 0 || lba >= pcecd_toc_lba[100])
        return NULL;
    uint32_t fframe;
    if (!pcecd_lba_to_file_frame(lba, &fframe))
        return NULL;
    uint32_t hunknum = fframe / pcecd_sectors_per_hunk;
    if (hunknum != pcecd_cached_hunk) {
        chd_error err = chd_read(pcecd_chd, hunknum, pcecd_hunk_buf);
        if (err != CHDERR_NONE) {
            DEBUG("pcecd: chd_read(hunk %u) = %s\n", (unsigned)hunknum, chd_error_string(err));
            return NULL;
        }
        pcecd_cached_hunk = hunknum;
    }
    return pcecd_hunk_buf + (fframe % pcecd_sectors_per_hunk) * PCECD_RAW_UNIT_BYTES;
}

void pcecd_serve_sector(uint32_t lba) {
    const uint8_t *raw = pcecd_raw_sector(lba);
    if (!raw) return;
    raw += PCECD_USER_DATA_OFFSET;
    pcecd_send_sector_chunk(0, raw, 1024);
    pcecd_send_sector_chunk(1, raw + 1024, 1024);
}

void pcecd_serve_audio_sector(uint32_t lba) {
    const uint8_t *raw = pcecd_raw_sector(lba);
    if (!raw) return;
    // CHD stores CD-DA big-endian; the core wants little-endian samples
    for (uint32_t i = 0; i < PCECD_AUDIO_BYTES; i += 2) {
        pcecd_audio_buf[i]     = raw[i + 1];
        pcecd_audio_buf[i + 1] = raw[i];
    }
    if (pcecd_tx_dma != NULL)
        pcecd_send_audio_dma(pcecd_audio_buf);
    else
        pcecd_send_audio_blocking(pcecd_audio_buf);
}

// Build the TOC from the CHD track metadata, the way MAME's cdrom.c lays tracks out.
static bool pcecd_read_toc(void) {
    int32_t plba = -150;
    int32_t fofs = 0;
    int track_count = 0;
    for (uint32_t idx = 0; idx < PCECD_MAX_TRACKS; idx++) {
        char metabuf[256];
        char type[64] = {0}, subtype[32] = {0}, pgtype[32] = {0}, pgsub[32] = {0};
        int tkid = 0, frames = 0, pregap = 0, postgap = 0;
        uint32_t resultlen = 0, resulttag = 0;
        uint8_t resultflags = 0;
        chd_error err = chd_get_metadata(pcecd_chd, CDROM_TRACK_METADATA2_TAG, idx,
                                          metabuf, sizeof(metabuf) - 1, &resultlen,
                                          &resulttag, &resultflags);
        if (err == CHDERR_NONE) {
            sscanf(metabuf, CDROM_TRACK_METADATA2_FORMAT, &tkid, type, subtype,
                   &frames, &pregap, pgtype, pgsub, &postgap);
        } else {
            err = chd_get_metadata(pcecd_chd, CDROM_TRACK_METADATA_TAG, idx,
                                    metabuf, sizeof(metabuf) - 1, &resultlen,
                                    &resulttag, &resultflags);
            if (err == CHDERR_NONE)
                sscanf(metabuf, CDROM_TRACK_METADATA_FORMAT, &tkid, type, subtype, &frames);
        }
        if (err != CHDERR_NONE)
            break;
        track_count++;
        // track 1's 2 s pregap is never in the file; a 'V' pregap is
        int pregap_lba  = (track_count == 1) ? 150 : (pgtype[0] == 'V') ? 0 : pregap;
        int pregap_file = (pgtype[0] == 'V') ? pregap : 0;
        plba += pregap_lba + pregap_file;
        pcecd_toc_lba[track_count]     = (uint32_t)plba;
        pcecd_toc_control[track_count] = (strcmp(type, "AUDIO") == 0) ? 0x00 : 0x04;
        fofs += pregap_file;
        pcecd_toc_fofs[track_count]    = (uint32_t)fofs;
        pcecd_toc_sectors[track_count] = (uint32_t)(frames - pregap_file);
        fofs += (frames - pregap_file) + postgap;
        fofs += ((frames + 3) & ~3) - frames;   // tracks are padded to 4 frames
        plba += (frames - pregap_file) + postgap;
    }
    pcecd_toc_num_tracks = track_count;
    if (track_count == 0)
        return false;
    pcecd_toc_lba[100] = (uint32_t)plba;
    DEBUG("pcecd: %d track(s), lead-out LBA %u\n", track_count, (unsigned)pcecd_toc_lba[100]);
    return true;
}

int loadpcecd(const char *fname) {
    int r = 1;
    DEBUG("loadpcecd: %s\n", fname);
    if (strcasestr(fname, ".chd") == NULL) {
        overlay_message("Only .chd supported", 1);
        return r;
    }
    pcecd_unload();

    chd_error err = chd_fatfs_open(fname, &f_chd, &pcecd_chd);
    if (err != CHDERR_NONE) {
        overlay_status("Cannot open CHD: %s", chd_error_string(err));
        return r;
    }
    pcecd_hdr = chd_get_header(pcecd_chd);
    if (pcecd_hdr->unitbytes == 0 || (pcecd_hdr->hunkbytes % pcecd_hdr->unitbytes) != 0) {
        overlay_status("Unexpected CHD sector layout");
        pcecd_unload();
        return r;
    }
    pcecd_sectors_per_hunk = pcecd_hdr->hunkbytes / pcecd_hdr->unitbytes;
    pcecd_hunk_buf = (uint8_t *)malloc(pcecd_hdr->hunkbytes);
    if (!pcecd_hunk_buf) {
        overlay_status("Out of memory for CD hunk buffer");
        pcecd_unload();
        return r;
    }
    if (!pcecd_read_toc()) {
        overlay_status("No track metadata in CHD");
        pcecd_unload();
        return r;
    }

    std::string syscard = std::string(drv) + "bios/syscard3.pce";
    r = loadpce(syscard.c_str());
    if (r != 0) {
        overlay_status("Cannot load bios/syscard3.pce");
        pcecd_unload();
        return r;
    }
    pcecd_send_toc();
    pcecd_send_mount(1);
    return 0;
}
