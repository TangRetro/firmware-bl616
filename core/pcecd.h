#pragma once

// Real PC Engine CD-ROM (.chd) loader -- see pcetang_cd_scsi_plan.md (pcetang project
// memory) for the full real wire-protocol design this implements (0x0e mount/0x10 sector
// chunk/0x06 sector request), verified against 3 real .chd dumps before this was written.

extern int loadpcecd(const char *fname);
// Generic: mount `fname`, run `boot` (BIOS load + core start; 0 = ok), then send TOC + mount.
extern int cdchd_load(const char *fname, int (*boot)(void));
// Real, idempotent disc-eject: safe to call with nothing mounted, but always sends a
// mount(0) frame to the FPGA -- fine for its 3 existing callers (all run once the
// active core is already the CD core and listening), NOT fine to call blind right
// after a fresh fpga_program() before the core has reached its own loading-state
// setup. Exposed (with pcecd_is_mounted() below) so loadpce_dispatch() (pce.cpp) can
// clear stale CD state before running a HuCard -- PCE and PCE-CD now share one menu
// entry, so switching between them no longer forces an FPGA reprogram in between.
extern void pcecd_unload(void);
extern bool pcecd_is_mounted(void);

// Real sector-request handlers, called from main.cpp's uart1_rx_task on a real 0x06 frame
// (cd_bridge.vhd's own SECTOR_REQ, relayed over UART by iosys_bl616.v). `lba` is a real
// absolute CD-ROM sector number. The frame's own is_audio bit (2026-08-31g, real
// SECTOR_IS_AUDIO tag) selects which one main.cpp calls: pcecd_serve_sector() answers
// with a real 2048-byte Mode-1 data sector, pcecd_serve_audio_sector() with a real
// 2352-byte raw CD-DA sector (direct 16-bit-LE stereo PCM, no header to strip).
extern void pcecd_serve_sector(uint32_t lba);
extern void pcecd_serve_audio_sector(uint32_t lba);
// Arrival time (mtimer us) of the request being served: the audio path logs the latency from
// here to its send (cdprog alat=avg/max). CD audio at 2 Mbaud needs it under ~1.5 ms on average.
extern volatile uint32_t pcecd_req_t_us;

// Called from cd_serve_task whenever its request queue goes quiet. Writes the per-request
// progress ring to the SD log once per quiet period, which is how a stalled boot says
// WHICH request it died on and how far that request got -- the hot path itself does no
// I/O, deliberately (file_log() f_syncs every line and has truncated request frames
// before). A new sector request re-arms it.
extern void pcecd_trace_idle_tick(void);

// DMA sector transmit (2026-09-16). pcecd_tx_dma_init() claims dma0_ch0 and links it to
// UART1 TX; call it once after the UART is up. Without it
// the sector path silently falls back to the old blocking write.
extern void pcecd_tx_dma_init(void);
