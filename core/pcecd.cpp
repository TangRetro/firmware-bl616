#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "FreeRTOS.h"
#include "task.h"

#include "tc_utils.h"
#include "cores.h"
#include "overlay.h"
#include "pcecd.h"
#include "bflb_dma.h"
#include "bflb_uart.h"
#include "chd/chd_fatfs.h"
#include "libchdr/chd.h"

extern void file_log(const char *msg);   // TEMP diagnostic, defined in main.cpp

// TEMP diagnostic (2026-09-10): largest single block newlib's heap will still hand out.
// Total free is the wrong question -- chd_open asks for a few sizeable contiguous blocks
// (codec instances, then the hunk buffer), so fragmentation, not the total, is what would
// bite. Probe downward and free immediately; costs nothing outside the log lines.
extern "C" void chd_dbg_log(const char *m) { file_log(m); }

// TEMP diagnostic (2026-09-10). The SDK's weak hooks are `printf(); while(1);` on UART0,
// which nothing is capturing here -- so both failures present as a silent hang with no
// reset, which is exactly the symptom being chased. Override them to leave a durable
// mark on the SD card first. Writing to FatFs from the overflow hook is not strictly
// safe (it can run from the scheduler's context), but at that point the system is
// already dead; a best-effort line costs nothing and names the failure.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    char buf[64];
    snprintf(buf, sizeof(buf), "FATAL: stack overflow in task '%.24s'", pcTaskName ? pcTaskName : "?");
    file_log(buf);
    for (;;) {}
}

extern "C" void vApplicationMallocFailedHook(void) {
    file_log("FATAL: malloc failed (pvPortMalloc)");
    for (;;) {}
}

extern "C" unsigned chd_dbg_stack_free_bytes(void) {
    return (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
}

// Non-allocating heap snapshot, for the leak/fragmentation hypothesis. Uses the SDK's
// own pfree_size(): the newlib port routes _malloc_r to bflb_malloc(PMEM_HEAP, ...),
// so PMEM is exactly the heap libchdr allocates from. NOT mallinfo() -- pulling
// <malloc.h> drags newlib's own allocator into the link and collides with the SDK's
// port_memory.c ("multiple definition of _malloc_r"). And NOT
// chd_dbg_largest_free_block() below, which allocates and has broken three separate
// hardware runs.
//
// Why this matters: libchdr calls plain malloc(), and the SDK's malloc failing returns
// NULL without ever reaching vApplicationMallocFailedHook -- it surfaces as a silent
// CHDERR_OUT_OF_MEMORY out of chd_read(). Heap exhaustion therefore looks exactly like
// the observed behaviour: serves a few sectors, then stops, with no FATAL line.
extern "C" {
#include "mem.h"
}
static void pcecd_heap_str(char *out, size_t n) {
    snprintf(out, n, "pfree=%u kfree=%u",
             (unsigned)pfree_size(), (unsigned)kfree_size());
}

extern "C" size_t chd_dbg_largest_free_block(void) {
    // Bounded and coarse on purpose. The first version walked 512KB down in 1KB steps,
    // i.e. up to 512 failing mallocs, each of which can drive newlib into _sbrk; that
    // made the probe itself a plausible hang and cost three hardware rounds of wrong
    // conclusions. 32 steps of 8KB, ceiling 256KB.
    for (size_t sz = 256u * 1024u; sz >= 8u * 1024u; sz -= 8u * 1024u) {
        void *p = malloc(sz);
        if (p) { free(p); return sz; }
    }
    return 0;
}

// Real PC Engine CD-ROM loader (pcetang core) -- see pcetang_cd_scsi_plan.md for the full
// real wire-protocol design (0x0e mount / 0x10 sector chunk / 0x06 sector request) and the
// real 2448-byte-raw-sector-to-2048-byte-user-data extraction this file implements, both
// verified against 3 real .chd dumps (toc_probe.c, a standalone host-side test, not part of
// this firmware) before being written here.

// Real CD-ROM raw-sector layout (Yellow Book Mode-1, real -- confirmed against every real
// .chd this session tested, not assumed): 12 sync + 4 header + 2048 user data + 4 EDC + 8
// zero + 172 P-parity + 104 Q-parity = 2352, + 96 subcode = 2448 real bytes/sector (matches
// libchdr's own `unitbytes` field exactly on every file tested). READ(6) (cd_bridge.vhd,
// FPGA side) needs exactly the 2048 real user-data bytes, matching real PCE-CD SCSI
// hardware (the drive's own decoder strips sync/header/ECC/subcode before delivery) --
// this loader does that stripping here, on the MCU side, before ever sending a byte over
// UART.
#define PCECD_RAW_UNIT_BYTES     2448
#define PCECD_USER_DATA_OFFSET   16
#define PCECD_USER_DATA_BYTES    2048

// Real raw CD-DA (audio-track) sector: unlike Mode-1 data tracks, an audio track's raw
// unit has NO sync/header/ECC structure at all -- all 2352 bytes are 16-bit stereo PCM
// (588 samples), starting at byte 0 of the raw unit (no offset to skip), confirmed
// against libchdr's own real per-track metadata (AUDIO vs MODE1 `type`) and every real
// .chd tested.
//
// BYTE ORDER, corrected 2026-09-16 -- this was the garbled-CD-audio bug. A CHD stores
// CD-DA samples BIG-ENDIAN, and the raw hunk bytes libchdr hands back are in that order.
// cd.vhd's CDDA_FIFO wants LITTLE-endian: it packs the wire bytes as
// `FIFO_D(7:0) <= CD_DATA` then `(15:8)`, `(23:16)`, `(31:24)` (cd.vhd:824-828), i.e.
// L-lsb, L-msb, R-lsb, R-msb. So every 16-bit sample must be byte-swapped on the way
// out. Data tracks are NOT swapped, which is exactly why this hid for so long: the
// byte-for-byte verification against beetle-pce-fast only ever covered MODE1 sectors.
//
// The reference implementation does the same swap, unconditionally, for every audio
// track -- mednafen/cdrom/CDAccess_CHD.c sets `RawAudioMSBFirst = 1` for every track
// whose type is AUDIO (line 183) and then:
//     case DI_FORMAT_AUDIO:
//        CDAccess_CHD_Read_CHD_Hunk_RAW(self, buf, lba, ct);
//        if (ct->RawAudioMSBFirst) Endian_A16_Swap(buf, 588 * 2);
// while MODE1_RAW/MODE2_RAW fall through with no swap at all.
//
// Measured on dd2.chd track 3, 20 sectors at the loudest probed passage, comparing the
// two interpretations by mean |sample-to-sample delta| against rms (music is strongly
// correlated, noise is not):
//     little-endian (correct)   mean|delta|   266   rms  4501   ratio 0.059
//     big-endian   (what we sent) mean|delta| 21786  rms 19048   ratio 1.144
// A ratio at or above 1 is white noise, and the wrong reading is also 4x louder -- which
// is precisely what "garbled sound" sounded like.
#define PCECD_AUDIO_BYTES        2352

// Real state for the currently-mounted disc. One disc at a time, same real assumption
// every other core loader in this firmware makes (fcore is a single global FIL too).
// ---------------------------------------------------------------------------
// DMA SECTOR TRANSMIT, second design (2026-09-17).
//
// Why DMA at all: the blocking write spins the CPU for the full 11.76 ms of wire time per
// 2352-byte sector. libchdr needs ~62 ms per 8-sector hunk, so decode THEN send is
// 62 + 94 = 156 ms to deliver 107 ms of audio = 68% of realtime -- measured 66-67%, the
// scratchy audio. With the sending done by DMA the next hunk decodes WHILE the previous
// sector goes out: ~113%.
//
// What the first design (2026-09-16) got wrong. Every disc stalled at a byte-identical
// point -- Bonk III at 163 requests, Rondo at 202 -- at the first DMA transfer ever
// attempted. Ruled out by measurement: the FPGA side (three builds, identical stalls), RX
// starvation (interrupt RX took the FIFO high-water from 23-31 to 9-12 with no drops, and
// it still stalled), and cache coherency (the staging buffer sits in ram_nocache). What
// it had lost was EXCLUSION: the old taskENTER_CRITICAL() around each blocking write was
// the only thing stopping other writers landing inside a frame, and a DMA transfer runs
// with interrupts enabled. The drain hook added to compensate covered some writers and
// not others, and it was itself deadlocking when reached from a critical section.
//
// This design makes exclusion explicit instead (see the UART1 token in utils.cpp):
//   - take the token        -- waits for the previous DMA transfer, or any other frame
//   - stage one frame, start DMA, return with the token STILL HELD
//   - DMA completion ISR    -- releases the token
// So there is exactly one transfer in flight, no ring, no drain hook, no critical section
// spanning a transfer, and exactly one chd_read() in flight at any time -- libchdr is
// never re-entered, which is why this is not a decode-ahead task.
//
// Every step is counted, and every wait is bounded, so the next hardware run MEASURES
// whether DMA works instead of letting it be inferred from a hang:
//   dma=started/done  -- a gap means completion interrupts are not arriving
//   dmams=last/max    -- ~12 ms = paced out by the UART; 0 ms = never actually sent
//   dmato             -- token waits that timed out (DMA disabled after the first)
//   lkto              -- timeouts on the ordinary frame writers
//
// Buffers are ATTR_NOCACHE_NOINIT_RAM_SECTION: the DMA engine does not snoop the D-cache.
#define PCECD_TX_DMA_ENABLE 1
#define PCECD_TX_CHUNK      (PCECD_AUDIO_BYTES / 2)
#define PCECD_TX_FRAME      (5 + PCECD_TX_CHUNK)       // 0xAA len16 cmd chunk_idx + payload
#define PCECD_TX_STAGE_SIZE (2 * PCECD_TX_FRAME)
// Generous against the ~12 ms a transfer should take, so only a genuinely lost
// completion ever reaches it.
#define PCECD_TX_DMA_WAIT_MS 250

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t pcecd_tx_stage[PCECD_TX_STAGE_SIZE];
static struct bflb_device_s *pcecd_tx_dma = NULL;
static struct bflb_dma_channel_lli_pool_s pcecd_tx_llipool[4];
static volatile uint8_t  pcecd_dma_inflight = 0;
static volatile TickType_t pcecd_dma_t0 = 0;
volatile uint32_t pcecd_dma_started = 0;
volatile uint32_t pcecd_dma_done = 0;
volatile uint32_t pcecd_dma_timeouts = 0;
volatile uint32_t pcecd_dma_last_ms = 0;
volatile uint32_t pcecd_dma_max_ms = 0;

static void pcecd_tx_dma_isr(void *arg)
{
    (void)arg;
    if (!pcecd_dma_inflight) return;          // spurious, or already recovered by timeout
    pcecd_dma_inflight = 0;
    uint32_t ms = (uint32_t)((xTaskGetTickCountFromISR() - pcecd_dma_t0) * portTICK_PERIOD_MS);
    pcecd_dma_last_ms = ms;
    if (ms > pcecd_dma_max_ms) pcecd_dma_max_ms = ms;
    pcecd_dma_done++;
    // Leave DMA-TX mode BEFORE releasing the token: the next owner may be a blocking
    // writer, and blocking writes into a UART in DMA-TX mode are what stalled the CD path.
    bflb_uart_link_txdma(uart1_dev, false);
    fpga_tx_unlock_from_isr();                // the transfer owned the token; release it
}

void pcecd_tx_dma_init(void)
{
    if (!PCECD_TX_DMA_ENABLE) return;         // pcecd_tx_dma stays NULL -> blocking send
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
    // UART1 is deliberately NOT put into DMA-TX mode here. It used to be, for the whole
    // session, and that alone broke the CD path: with UART_DMA_TX_EN set, frames written
    // the ordinary way (putchar) did not reliably finish going out. Measured 2026-09-17:
    // a build whose dma counter read 0/0 -- not one DMA transfer ever started -- still
    // stalled Bonk III at exactly request 163, identical to every earlier DMA build, while
    // the only build that passed was the one where this init never ran. Sector 163's tail
    // never reached the FPGA, so it never asked for 164.
    // DMA-TX mode is now switched on only for the lifetime of one transfer, while that
    // transfer owns the UART1 token; see pcecd_send_sector_dma and the completion ISR.
    pcecd_tx_dma = dev;
}

static void pcecd_send_sector_blocking(const uint8_t *swapped);

// Send one whole raw CD-DA sector (already byte-swapped). Returns as soon as the transfer
// is started; the token stays held until the completion ISR releases it.
static void pcecd_send_sector_dma(const uint8_t *swapped)
{
    if (!fpga_tx_lock_timed(PCECD_TX_DMA_WAIT_MS)) {
        // The previous transfer never reported completion (or another writer is stuck).
        // Stop the channel, give DMA up for the rest of the session, and carry on with the
        // blocking path so the game keeps running. The counters say what happened.
        pcecd_dma_timeouts++;
        if (pcecd_tx_dma) bflb_dma_channel_stop(pcecd_tx_dma);
        bflb_uart_link_txdma(uart1_dev, false);
        pcecd_dma_inflight = 0;
        pcecd_tx_dma = NULL;
        fpga_tx_unlock();                     // re-arm the token we could not obtain
        pcecd_send_sector_blocking(swapped);
        return;
    }

    uint8_t *d = pcecd_tx_stage;
    uint16_t n = 0;
    for (int c = 0; c < 2; c++) {
        uint16_t len = PCECD_TX_CHUNK + 2;    // chunk_idx + payload, as fpga_tx_header's len
        d[n++] = 0xAA;
        d[n++] = (uint8_t)(len >> 8);
        d[n++] = (uint8_t)(len & 0xFF);
        d[n++] = 0x10;
        d[n++] = (c == 0) ? 0x00 : 0xFF;      // 0xFF = final-chunk sentinel
        memcpy(&d[n], swapped + c * PCECD_TX_CHUNK, PCECD_TX_CHUNK);
        n += PCECD_TX_CHUNK;
    }

    struct bflb_dma_channel_lli_transfer_s tr[1];
    tr[0].src_addr = (uint32_t)pcecd_tx_stage;
    tr[0].dst_addr = (uint32_t)DMA_ADDR_UART1_TDR;
    tr[0].nbytes   = n;
    pcecd_dma_t0 = xTaskGetTickCount();
    pcecd_dma_inflight = 1;
    pcecd_dma_started++;
    bflb_dma_channel_lli_reload(pcecd_tx_dma, pcecd_tx_llipool, 4, tr, 1);
    bflb_uart_link_txdma(uart1_dev, true);   // only for this transfer, token held
    bflb_dma_channel_start(pcecd_tx_dma);
    // Token intentionally NOT released here.
}

// Byte-swapped CD-DA scratch, one raw sector. Static rather than a 2352-byte stack
// object: this runs on the UART RX task, whose stack this firmware has already had to
// raise once for the CD path.
static uint8_t pcecd_audio_buf[PCECD_AUDIO_BYTES];

static USB_NOCACHE_RAM_SECTION FIL f_chd;
static chd_file *pcecd_chd = NULL;
static const chd_header *pcecd_hdr = NULL;
static uint8_t *pcecd_hunk_buf = NULL;
static uint32_t pcecd_cached_hunk = 0xFFFFFFFF;   // real sentinel: nothing cached yet
static uint32_t pcecd_sectors_per_hunk = 0;

// Real, minimal per-track TOC state, computed by pcecd_read_toc() using the exact same
// plba/pregap/postgap arithmetic as Mednafen's own CDAccess_CHD::Load() (real source read
// this session, not derived from the CHD metadata spec directly -- see that function's own
// comment trail). Only what cd_bridge.vhd's real GETDIRINFO/SAPSP/READ(6)-bounds-check
// commands consume: real per-track start LBA + real control byte (audio=0x00/data=0x04).
// Indexed identically to cd_bridge.vhd's own toc_lba_tbl: 1..99 real tracks, 100 = real
// lead-out sentinel (LBA = total real sector count), 0 unused.
#define PCECD_MAX_TRACKS 99
static uint32_t pcecd_toc_lba[101];
static uint8_t  pcecd_toc_control[101];
// Logical LBA -> file frame. A CHD stores tracks back to back, each padded up to a
// multiple of CD_TRACK_PADDING (4) frames, and a pregap that is not in the file occupies
// LBAs but no bytes -- so file offset only equals LBA for a disc whose track 1 starts at
// LBA 0 with nothing skipped. Dungeon Explorer II is not one: track 1 is AUDIO (3365
// frames, padded to 3368) and track 2 carries a 225-frame pregap, putting the data track
// 222 frames apart in the two address spaces. The deltas differ per track (222, 371,
// 368 ... 545), so a single constant will not do.
// Accumulation follows beetle-pce-fast's CDAccess_CHD.cpp verbatim, including the
// postgap term my own derivation had missed, and its read mapping is the same:
//     file_frame = fileOffset[t] + (lba - LBA[t])
// Model checked against the real image before being written: summed padded frames came
// to 315468 against the file's own 39434 hunks x 8 = 315472, i.e. inside one hunk.
static uint32_t pcecd_toc_fofs[101];      // file frame where each track's LBA starts
static uint32_t pcecd_toc_sectors[101];   // playable sectors, for the track lookup
static int      pcecd_toc_num_tracks = 0;

static void pcecd_send_mount(uint8_t mounted) {
    fpga_tx_lock();
    fpga_tx_header(0x0e, 2);
    fpga_tx_byte(mounted);
    fpga_tx_unlock();
}

// Real, generalized (2026-08-31g, was hardcoded to 1024 bytes/chunk for the Mode-1 data
// path only): `chunk_idx` is passed straight through to iosys_bl616.v's own 0x10 RX
// handler, which real-ends the sector on EITHER `chunk_idx==1 && data_cnt==1024` (the
// original, unchanged, real 2x1024B Mode-1 data-sector shape) OR `chunk_idx==0xFF` at
// the frame's own last byte (a real, backward-compatible sentinel for any other
// chunk count/size -- see that file's own 0x10 handler comment). The real 2352-byte
// CD-DA path below uses the sentinel; the real 2048-byte data path keeps its original
// chunk_idx values (0, 1) and is byte-for-byte unchanged on the wire.
static void pcecd_send_sector_chunk(uint8_t chunk_idx, const uint8_t *data, uint16_t length) {
    fpga_tx_lock();     // waits out an in-flight DMA sector; see utils.cpp
    fpga_tx_header(0x10, (int)length + 2);
    fpga_tx_byte(chunk_idx);
    for (uint16_t i = 0; i < length; i++)
        fpga_tx_byte(data[i]);
    fpga_tx_unlock();
}

static void pcecd_send_sector_blocking(const uint8_t *swapped) {
    pcecd_send_sector_chunk(0, swapped, PCECD_AUDIO_BYTES / 2);
    pcecd_send_sector_chunk(0xFF, swapped + PCECD_AUDIO_BYTES / 2, PCECD_AUDIO_BYTES / 2);
}

// Real, one TOC entry per call -- matches cd_bridge.vhd's own real TOC_WR/TOC_TRACK/
// TOC_CONTROL/TOC_LBA one-write-per-track interface exactly.
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

// Real, sends every real TOC entry captured by pcecd_read_toc() (including the real
// lead-out at index 100) to the FPGA -- must run before pcecd_send_mount(1), matching the
// real ordering cd_bridge.vhd's own TOC_CAPTURE process assumes (see its header: TOC
// extents self-reset on DISC_MOUNTED's real falling edge, re-armed by the NEXT track=1
// write, so the far side must see the new disc's TOC before the syscard starts polling
// TEST UNIT READY on the new mount).
static void pcecd_send_toc(void) {
    for (int t = 1; t <= pcecd_toc_num_tracks; t++)
        pcecd_send_toc_entry((uint8_t)t, pcecd_toc_control[t], pcecd_toc_lba[t]);
    pcecd_send_toc_entry(100, 0, pcecd_toc_lba[100]);
}

bool pcecd_is_mounted(void) {
    return pcecd_chd != NULL;
}

// Clears the per-session trace one-shots and the request ring. Defined further down,
// with the state it owns; declared here because pcecd_unload() precedes all of it.
static void pcecd_trace_reset(void);

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
    // Re-arm the per-session one-shots. Without this a disc loaded a second time in one
    // MCU session inherits the first load's spent flags, so its SERVE-FAIL / DECODE-START
    // / SERVED lines never appear and the run reads as "no failure occurred". Defined
    // below, next to the counters it clears (they are declared after this function).
    pcecd_trace_reset();
    pcecd_send_mount(0);
}

// TEMP instrumentation (2026-09-10). The syscard boots and shows "JUST A MOMENT...",
// then sits there. The hot path below only logs on ERROR, so "no errors in the log" and
// "no sector request ever arrived" are indistinguishable -- which is exactly the gap that
// made the earlier hang take four rounds. These counters make the difference visible:
// zero requests means the fault is on the FPGA/cd_bridge side and nothing on the MCU is
// being asked for; a rising count with a slow rate means decode/IO throughput.
static uint32_t pcecd_req_count = 0;      // sector requests received
static uint32_t pcecd_hunk_reads = 0;     // actual chd_read() calls (cache misses)
static uint32_t pcecd_last_lba = 0;
static uint32_t pcecd_next_report = 1;

// 2026-09-11: the failure paths below used DEBUG(), which goes to dprint -> UART0, and
// NOTHING captures UART0 on this board. So a silent return looked identical to a request
// that was served fine, and the first real sector request stalled with no explanation.
// These now reach the SD log. They are one-shot (a stalled boot repeats nothing), so they
// cannot flood the UART RX path the way a per-sector log would.
// 2026-09-11: SERVE-PATH SD LOGGING OFF BY DEFAULT, and this is a real fix, not tidying.
// file_log() does f_write + f_sync to the SD card, and pcecd_serve_sector() runs INSIDE
// uart1_rx_task -- the sector request arrives as UART opcode 6. So every line logged from
// this path blocks the MCU's UART RX for milliseconds. cd_bridge fires SECTOR_REQ for the
// next sector the moment it has consumed 2048 bytes, which lands squarely in that window,
// and the request frame is dropped.
//
// Measured on hardware, trace tag 0xAF: SECTOR_DATA_VALID=2048, SECTOR_REQ=2 on the FPGA
// side, while the MCU's own counter reported reqs=1. The bridge asked twice; the MCU never
// heard the second. MCU TX is unaffected (all 2048 bytes reached the FPGA), only RX.
//
// Set to 1 only for a one-off diagnosis, and expect it to break multi-sector transfers
// while it is on. The FPGA-side 0xAE/0xAF counters are the safe way to watch this path:
// they accumulate in RTL and cost one frame per heartbeat regardless of activity.
// 2026-09-11: BACK ON, deliberately. The earlier reason for switching this off was
// that trace frames were thought to be starving the sector-request path -- that was
// tested and DISPROVEN: with the FPGA trace channel compiled out entirely
// (DBG_TRACE => 0), Prince of Persia behaves identically. Meanwhile the real failure
// is terminal and silent: PoP served its 18 boot sectors correctly, then delivered
// nothing across 146 retries of the same read, with no FATAL line -- so the MCU is
// alive and taking one of pcecd_serve_sector()'s silent early returns. Every line
// below is ONE-SHOT, so the whole diagnosis costs a handful of f_syncs, not a
// per-sector flood.
#define PCECD_TRACE_SERVE 1

// Log EVERY served sector as "SEC lba=<n> d=<first 8 user bytes>", for a direct diff
// against a golden trace from beetle-pce-fast (whose own PCESEC lines carry exactly the
// same two fields). Spot-checking one sector proved the mapping and the FIFO fix, but it
// cannot catch a divergence later in the stream -- this can.
//
// Only safe because sector serving now runs on cd_serve_task, NOT on uart1_rx_task:
// file_log() f_syncs every line, and doing that on the RX path is what truncated the
// request frames in the first place. It still costs an SD write per sector, so it
// visibly slows serving and CHANGES TIMING -- treat it as a diagnostic build, not a
// representative one, and turn it off before judging whether a game runs.
// OFF. This was for the one-time golden-trace diff against beetle-pce-fast (done, and
// every sector matched). Leaving it on costs an f_sync per sector, which made serves slow
// enough to trip cd_bridge's request watchdog and duplicate whole sectors -- 12 requests
// for a 2-sector read, and LOAD ERROR. Turn on only for another offline diff, never for
// judging whether a game runs.
#define PCECD_TRACE_SECTORS 0
#define PCECD_TRACE_SECTORS_MAX 1500
static uint32_t pcecd_sec_logged = 0;

static uint8_t pcecd_fail_logged = 0;
static uint8_t pcecd_served_logged = 0;
static uint8_t pcecd_decode_logged = 0;
static void pcecd_log_once(const char *msg) {
    if (!PCECD_TRACE_SERVE) return;   // never block UART RX on the CD path
    if (pcecd_fail_logged) return;
    pcecd_fail_logged = 1;
    file_log(msg);
}


// ---------------------------------------------------------------------------
// Per-request progress ring (2026-09-13).
//
// The board stalls with cd_bridge parked in SCSI_READ_WAIT_BYTE (trace tag 0xD2,
// state 5, on every one of 113 consecutive samples) -- it asked for a sector and
// no byte ever came back. REQ_WATCHDOG is false in cd_bridge.vhd, so that wait is
// permanent and every later host command is blocked behind it.
//
// Nothing in the log could say which request that was, because both of the existing
// probes go quiet exactly there:
//   * pcecd_progress_tick() logs on a 1,2,4,8,... schedule, so requests 5,6,7 are
//     silent and the next line would only come at 8;
//   * DECODE-START is gated by pcecd_decode_logged, a one-shot, so only the FIRST
//     hunk decode of the session is ever announced;
//   * SERVE-FAIL goes through pcecd_log_once(), also one-shot for the whole boot --
//     and this disc gets loaded more than once per session, so a real failure on the
//     second load is suppressed by the first load's flag.
//
// Logging per request on the hot path is not an option: file_log() f_syncs every
// line, which is what truncated sector-request frames before (see PCECD_TRACE_SERVE's
// own comment) and what duplicated whole sectors when PCECD_TRACE_SECTORS was left on.
// So record progress into RAM -- a few stores, no I/O -- and write it out only once
// serving has gone quiet, which is precisely the stalled state we want to inspect.
//
// stage values are cumulative high-water marks, so the LAST stage an entry reached is
// the answer: an entry stuck at 3 was inside chd_read() when everything stopped, which
// a log that simply ends cannot distinguish from a request that never arrived at all.
#define PCECD_RING_N 64
#define PCECD_RG_ENTER   1   // request dequeued, handler entered
#define PCECD_RG_MAPPED  2   // lba -> file frame -> hunk resolved
#define PCECD_RG_DECODE  3   // inside chd_read()
#define PCECD_RG_DECODED 4   // chd_read() returned
#define PCECD_RG_SENT    5   // both chunks handed to the UART
#define PCECD_RG_F_MOUNT 0x81 // no disc mounted
#define PCECD_RG_F_LEAD  0x82 // past lead-out
#define PCECD_RG_F_TRACK 0x83 // in no track extent
#define PCECD_RG_F_READ  0x84 // chd_read() returned an error

typedef struct {
    uint32_t lba;
    uint32_t hunk;
    uint32_t tick;
    uint8_t  stage;
    uint8_t  err;      // chd_error when stage == PCECD_RG_F_READ
    uint8_t  audio;
} pcecd_ring_e;

static pcecd_ring_e pcecd_ring[PCECD_RING_N];
static uint32_t pcecd_ring_used = 0;     // entries filled, saturates at PCECD_RING_N
static int32_t  pcecd_ring_cur  = -1;    // entry the in-flight request is using
static uint32_t pcecd_ring_last_tick = 0;
static uint8_t  pcecd_ring_flushed = 0;   // ring written for the current quiet period
static uint32_t pcecd_ring_flushed_upto = 0;  // entries already written, never reprinted

// Begin a ring entry. Requests past PCECD_RING_N are counted by pcecd_req_count but
// not traced -- the interesting window is the boot handful, and a fixed array keeps
// this allocation-free on a path where malloc has broken hardware runs before.
static void pcecd_ring_begin(uint32_t lba, uint8_t audio) {
    pcecd_ring_last_tick = (uint32_t)xTaskGetTickCount();
    pcecd_ring_flushed = 0;
    if (pcecd_ring_used >= PCECD_RING_N) { pcecd_ring_cur = -1; return; }
    pcecd_ring_cur = (int32_t)pcecd_ring_used++;
    pcecd_ring_e *e = &pcecd_ring[pcecd_ring_cur];
    e->lba = lba; e->hunk = 0xFFFFFFFF; e->tick = pcecd_ring_last_tick;
    e->stage = PCECD_RG_ENTER; e->err = 0; e->audio = audio;
}

static void pcecd_ring_mark(uint8_t stage, uint32_t hunk, uint8_t err) {
    pcecd_ring_last_tick = (uint32_t)xTaskGetTickCount();
    if (pcecd_ring_cur < 0) return;
    pcecd_ring_e *e = &pcecd_ring[pcecd_ring_cur];
    e->stage = stage;
    if (hunk != 0xFFFFFFFF) e->hunk = hunk;
    if (err) e->err = err;
}

static void pcecd_trace_reset(void) {
    pcecd_fail_logged = 0;
    pcecd_served_logged = 0;
    pcecd_decode_logged = 0;
    pcecd_sec_logged = 0;
    pcecd_req_count = 0;
    pcecd_hunk_reads = 0;
    pcecd_next_report = 1;
    pcecd_ring_used = 0;
    pcecd_ring_cur = -1;
    pcecd_ring_flushed = 0;
    pcecd_ring_flushed_upto = 0;
    // Measure the quiet window from the load, not from whatever the last session left
    // here -- otherwise the first idle tick after a load fires instantly.
    pcecd_ring_last_tick = (uint32_t)xTaskGetTickCount();
}

static const char *pcecd_ring_stage_name(uint8_t s) {
    switch (s) {
        case PCECD_RG_ENTER:   return "ENTER";
        case PCECD_RG_MAPPED:  return "MAPPED";
        case PCECD_RG_DECODE:  return "IN-CHD_READ";
        case PCECD_RG_DECODED: return "DECODED";
        case PCECD_RG_SENT:    return "SENT";
        case PCECD_RG_F_MOUNT: return "FAIL-NOMOUNT";
        case PCECD_RG_F_LEAD:  return "FAIL-LEADOUT";
        case PCECD_RG_F_TRACK: return "FAIL-NOTRACK";
        case PCECD_RG_F_READ:  return "FAIL-CHDREAD";
        default:               return "?";
    }
}

// Called from cd_serve_task when its queue receive times out, i.e. exactly when the CD
// path has gone quiet. Writes the ring once per quiet period; a new request re-arms it.
void pcecd_trace_idle_tick(void) {
    if (!PCECD_TRACE_SERVE) return;
    if (pcecd_ring_flushed) return;
    // Only once a disc is actually mounted: before that there is nothing to say, and
    // this runs from the UART RX task's idle path, which must stay cheap.
    if (!pcecd_chd) return;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((now - pcecd_ring_last_tick) < pdMS_TO_TICKS(3000)) return;
    pcecd_ring_flushed = 1;
    char b[160];
    // Printed even when nothing was traced. "0 traced of 0 total" is a positive
    // statement -- the MCU is alive, a disc is mounted, and no sector request ever
    // arrived -- which is a different fault from a log that simply stops, and the
    // difference between those two has misled this investigation twice already.
    snprintf(b, sizeof(b), "REQRING: %lu traced of %lu total, quiet %lums, hunk_reads=%lu",
             (unsigned long)pcecd_ring_used, (unsigned long)pcecd_req_count,
             (unsigned long)((now - pcecd_ring_last_tick) * portTICK_PERIOD_MS),
             (unsigned long)pcecd_hunk_reads);
    file_log(b);
    // Only entries not yet written. Re-arming on every request would otherwise redump
    // the whole ring after each >=3s pause in normal play -- up to 64 f_syncs, mid-game.
    for (uint32_t i = pcecd_ring_flushed_upto; i < pcecd_ring_used; i++) {
        const pcecd_ring_e *e = &pcecd_ring[i];
        snprintf(b, sizeof(b), "REQ %2lu lba=%-7lu hunk=%-7ld %s%s err=%d t=%lu",
                 (unsigned long)(i + 1), (unsigned long)e->lba,
                 (long)(int32_t)e->hunk, e->audio ? "AUDIO " : "",
                 pcecd_ring_stage_name(e->stage), (int)e->err,
                 (unsigned long)e->tick);
        file_log(b);
    }
    pcecd_ring_flushed_upto = pcecd_ring_used;
}

extern volatile uint16_t uart1_rx_hiwater;   // see main.cpp
// Software RX ring, added with interrupt-driven RX. rxhi now measures how well the ISR
// keeps the 32-byte hardware FIFO drained (should sit low); ringhi/ringdrop measure
// whether the PARSER task keeps up with the ring, which is a different failure.
extern volatile uint16_t u1rx_ring_hiwater;
extern volatile uint32_t u1rx_ring_drops;
static void pcecd_progress_tick(void) {
    // Log on a 1,2,4,8,... schedule: dense at the start where "did anything happen at
    // all" is the question, then rare, so a working stream cannot flood the SD card
    // (file_log f_syncs every line).
    if (!PCECD_TRACE_SERVE) return;   // never block UART RX on the CD path
    if (pcecd_req_count < pcecd_next_report)
        return;
    pcecd_next_report *= 2;
    char buf[176];
    snprintf(buf, sizeof(buf),
             "cdprog: reqs=%lu hunk_reads=%lu last_lba=%lu rxhi=%u ringhi=%u ringdrop=%lu "
             "dma=%lu/%lu dmams=%lu/%lu dmato=%lu lkto=%lu tick=%lu",
             (unsigned long)pcecd_req_count, (unsigned long)pcecd_hunk_reads,
             (unsigned long)pcecd_last_lba, (unsigned)uart1_rx_hiwater,
             (unsigned)u1rx_ring_hiwater, (unsigned long)u1rx_ring_drops,
             (unsigned long)pcecd_dma_started, (unsigned long)pcecd_dma_done,
             (unsigned long)pcecd_dma_last_ms, (unsigned long)pcecd_dma_max_ms,
             (unsigned long)pcecd_dma_timeouts, (unsigned long)fpga_tx_lock_timeouts,
             (unsigned long)xTaskGetTickCount());
    file_log(buf);
}

// Logical LBA -> file frame, per beetle-pce-fast: find the track containing this LBA,
// then file = fileOffset[t] + (lba - LBA[t]). Linear over <=99 tracks, called once per
// sector request, which is nothing next to a hunk decode. Returns false if the LBA falls
// in no track's playable extent (a gap), so the caller declines rather than serving
// whatever happens to sit at that file frame.
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

void pcecd_serve_sector(uint32_t lba) {
    pcecd_req_count++; pcecd_last_lba = lba; pcecd_ring_begin(lba, 0);
    pcecd_progress_tick();
    if (!pcecd_chd || pcecd_sectors_per_hunk == 0) {
        pcecd_ring_mark(PCECD_RG_F_MOUNT, 0xFFFFFFFF, 0);
        pcecd_log_once("SERVE-FAIL: no disc mounted (pcecd_chd NULL or spq 0)");
        return;
    }
    // Real bounds check -- cd_bridge.vhd's own READ(6) real TOC-lead-out check (see
    // cd_bridge.vhd) already rejects this before ever pulsing SECTOR_REQ for a real,
    // well-formed disc; this is real defense-in-depth against a genuinely malformed or
    // truncated .chd (real lead-out LBA computed from track metadata that doesn't match
    // the real hunk-backed file size), not a redundant no-op.
    if (lba >= pcecd_toc_lba[100]) {
        char b[80];
        snprintf(b, sizeof(b), "SERVE-FAIL: LBA %lu past lead-out %lu",
                 (unsigned long)lba, (unsigned long)pcecd_toc_lba[100]);
        pcecd_ring_mark(PCECD_RG_F_LEAD, 0xFFFFFFFF, 0);
        pcecd_log_once(b);
        return;
    }

    uint32_t fframe;
    if (!pcecd_lba_to_file_frame(lba, &fframe)) {
        char b[80];
        snprintf(b, sizeof(b), "SERVE-FAIL: LBA %lu in no track extent (ntracks=%d)",
                 (unsigned long)lba, pcecd_toc_num_tracks);
        pcecd_ring_mark(PCECD_RG_F_TRACK, 0xFFFFFFFF, 0);
        pcecd_log_once(b);
        return;
    }
    uint32_t hunknum = fframe / pcecd_sectors_per_hunk;
    uint32_t sector_in_hunk = fframe % pcecd_sectors_per_hunk;
    pcecd_ring_mark(PCECD_RG_MAPPED, hunknum, 0);

    if (hunknum != pcecd_cached_hunk) {
        // Announce the decode BEFORE running it. Without this, "returned early" and
        // "still inside chd_read" produce identical logs -- the same trap that made the
        // chd_open hang take three hardware rounds earlier today.
        if (PCECD_TRACE_SERVE && !pcecd_decode_logged) {
            pcecd_decode_logged = 1;
            char b[136];
            // NO heap probe here. chd_dbg_largest_free_block() walks malloc down from
            // 256KB and has now broken three separate hardware runs: it was the original
            // "f_open hangs" that cost three rounds, and adding it to THIS line silently
            // removed the DECODE-START output that the previous build printed fine.
            // Calling a probe that allocates, from inside the UART RX path, during a CD
            // load, is simply not safe. If heap state is needed, sample it somewhere
            // idle -- not on the path being diagnosed.
            snprintf(b, sizeof(b), "DECODE-START: lba=%lu fframe=%lu hunk=%lu sec=%lu",
                     (unsigned long)lba, (unsigned long)fframe,
                     (unsigned long)hunknum, (unsigned long)sector_in_hunk);
            file_log(b);
        }
        pcecd_ring_mark(PCECD_RG_DECODE, hunknum, 0);
        chd_error err = chd_read(pcecd_chd, hunknum, pcecd_hunk_buf);
        if (err != CHDERR_NONE) {
            pcecd_ring_mark(PCECD_RG_F_READ, hunknum, (uint8_t)err);
            char b[176], h[96];
            pcecd_heap_str(h, sizeof(h));
            snprintf(b, sizeof(b), "SERVE-FAIL: chd_read(hunk %lu) = %d (%s) after %lu hunks | %s",
                     (unsigned long)hunknum, (int)err, chd_error_string(err),
                     (unsigned long)pcecd_hunk_reads, h);
            pcecd_log_once(b);
            return;
        }
        pcecd_ring_mark(PCECD_RG_DECODED, hunknum, 0);
        pcecd_cached_hunk = hunknum;
        pcecd_hunk_reads++;
    }

    const uint8_t *raw = pcecd_hunk_buf + (uint32_t)sector_in_hunk * PCECD_RAW_UNIT_BYTES
                          + PCECD_USER_DATA_OFFSET;
    if (PCECD_TRACE_SECTORS && pcecd_sec_logged < PCECD_TRACE_SECTORS_MAX) {
        pcecd_sec_logged++;
        char b[80];
        snprintf(b, sizeof(b), "SEC lba=%lu d=%02x%02x%02x%02x%02x%02x%02x%02x",
                 (unsigned long)lba, raw[0], raw[1], raw[2], raw[3],
                 raw[4], raw[5], raw[6], raw[7]);
        file_log(b);
    }
    pcecd_send_sector_chunk(0, raw, 1024);
    pcecd_send_sector_chunk(1, raw + 1024, 1024);
    pcecd_ring_mark(PCECD_RG_SENT, 0xFFFFFFFF, 0);

    // One-shot proof that a sector was actually decoded AND sent, with the mapping that
    // produced it -- lba, the file frame it resolved to, the hunk, and the first bytes of
    // user data. For LBA 3590 on this disc the mapping should give file frame 3368, and a
    // Mode-1 data sector's user area starts with the disc's own content, not zeros.
    if (PCECD_TRACE_SERVE && !pcecd_served_logged) {
        pcecd_served_logged = 1;
        char b[160];
        snprintf(b, sizeof(b),
                 "SERVED: lba=%lu -> fframe=%lu hunk=%lu sec=%lu data=%02x%02x%02x%02x stack_free=%u",
                 (unsigned long)lba, (unsigned long)fframe, (unsigned long)hunknum,
                 (unsigned long)sector_in_hunk, raw[0], raw[1], raw[2], raw[3],
                 (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
        file_log(b);
        char h[96];
        pcecd_heap_str(h, sizeof(h));
        snprintf(b, sizeof(b), "SERVED-HEAP: %s", h);
        file_log(b);
    }
}

// Real raw CD-DA sector serving (2026-08-31g) -- same real bounds check and hunk-cache
// path as pcecd_serve_sector() above, but no PCECD_USER_DATA_OFFSET skip (see
// PCECD_AUDIO_BYTES' own comment: audio tracks have no header to strip) and a real
// 2352-byte length, split into two 1176-byte chunks (2352 divides evenly, both chunks
// comfortably under the wire protocol's real ~2047-byte single-frame cap). The second
// chunk is tagged with the real 0xFF "final chunk" sentinel (see
// pcecd_send_sector_chunk's own comment) -- cd_bridge.vhd doesn't know or care that
// this is 1176+1176 rather than 1024+1024, only that SECTOR_DATA_LAST pulses on the
// real last byte.
void pcecd_serve_audio_sector(uint32_t lba) {
    pcecd_req_count++; pcecd_last_lba = lba; pcecd_ring_begin(lba, 1);
    pcecd_progress_tick();
    // These three returns used DEBUG() -> dprint -> UART0, which nothing on this board
    // captures, so an audio request that was silently declined looked exactly like one
    // that was served -- and declining it wedges the bus, because cd_bridge waits in
    // SCSI_READ_WAIT_BYTE for a SECTOR_DATA_LAST that now never comes (REQ_WATCHDOG is
    // false). They record into the ring like the data path, which costs no I/O here.
    if (!pcecd_chd || pcecd_sectors_per_hunk == 0) {
        pcecd_ring_mark(PCECD_RG_F_MOUNT, 0xFFFFFFFF, 0);
        return;
    }
    if (lba >= pcecd_toc_lba[100]) {
        pcecd_ring_mark(PCECD_RG_F_LEAD, 0xFFFFFFFF, 0);
        return;
    }

    uint32_t fframe;
    if (!pcecd_lba_to_file_frame(lba, &fframe)) {
        pcecd_ring_mark(PCECD_RG_F_TRACK, 0xFFFFFFFF, 0);
        return;
    }
    uint32_t hunknum = fframe / pcecd_sectors_per_hunk;
    uint32_t sector_in_hunk = fframe % pcecd_sectors_per_hunk;
    pcecd_ring_mark(PCECD_RG_MAPPED, hunknum, 0);

    if (hunknum != pcecd_cached_hunk) {
        // Announce the decode BEFORE running it. Without this, "returned early" and
        // "still inside chd_read" produce identical logs -- the same trap that made the
        // chd_open hang take three hardware rounds earlier today.
        if (PCECD_TRACE_SERVE && !pcecd_decode_logged) {
            pcecd_decode_logged = 1;
            char b[136];
            // NO heap probe here. chd_dbg_largest_free_block() walks malloc down from
            // 256KB and has now broken three separate hardware runs: it was the original
            // "f_open hangs" that cost three rounds, and adding it to THIS line silently
            // removed the DECODE-START output that the previous build printed fine.
            // Calling a probe that allocates, from inside the UART RX path, during a CD
            // load, is simply not safe. If heap state is needed, sample it somewhere
            // idle -- not on the path being diagnosed.
            snprintf(b, sizeof(b), "DECODE-START: lba=%lu fframe=%lu hunk=%lu sec=%lu",
                     (unsigned long)lba, (unsigned long)fframe,
                     (unsigned long)hunknum, (unsigned long)sector_in_hunk);
            file_log(b);
        }
        pcecd_ring_mark(PCECD_RG_DECODE, hunknum, 0);
        chd_error err = chd_read(pcecd_chd, hunknum, pcecd_hunk_buf);
        if (err != CHDERR_NONE) {
            pcecd_ring_mark(PCECD_RG_F_READ, hunknum, (uint8_t)err);
            return;
        }
        pcecd_ring_mark(PCECD_RG_DECODED, hunknum, 0);
        pcecd_cached_hunk = hunknum;
        pcecd_hunk_reads++;
    }

    const uint8_t *raw = pcecd_hunk_buf + (uint32_t)sector_in_hunk * PCECD_RAW_UNIT_BYTES;

    // Big-endian -> little-endian, 16 bits at a time. See PCECD_AUDIO_BYTES' comment.
    //
    // Swapped into a separate buffer rather than in place: pcecd_hunk_buf is the CACHED
    // hunk, and the same sector can legitimately be requested twice (a re-read, or a
    // seek landing back in the same hunk). An in-place swap would double-swap on the
    // second request and hand back the original garbage, which would look exactly like
    // an intermittent version of the bug this fixes.
    for (uint32_t i = 0; i < PCECD_AUDIO_BYTES; i += 2) {
        pcecd_audio_buf[i]     = raw[i + 1];
        pcecd_audio_buf[i + 1] = raw[i];
    }

    // Hand the whole framed sector to the DMA ring and return immediately. The CPU is
    // then free to decode the next hunk while this one is still going out on the wire --
    // that overlap is the fix for the CD-DA underrun. Falls back to the old blocking
    // path if the DMA channel was not available at init.
    if (pcecd_tx_dma != NULL)
        pcecd_send_sector_dma(pcecd_audio_buf);
    else
        pcecd_send_sector_blocking(pcecd_audio_buf);
    pcecd_ring_mark(PCECD_RG_SENT, 0xFFFFFFFF, 0);
}

// Real TOC walk, computing real per-track start LBA + control byte using the exact same
// plba/pregap/postgap arithmetic as Mednafen's own CDAccess_CHD::Load() (real source read
// this session: mednafen/src/cdrom/CDAccess_CHD.cpp -- authoritative, not the generic CHD
// metadata spec, per this project's own verify-against-real-emulator-source rule). Real,
// deliberately narrower scope than Mednafen's own: this loader doesn't track pregap_dv
// ("virtual"/PGTYPE='V' pregaps not baked into the file) or per-track fileOffset skew --
// pcecd_serve_sector()'s own hunk math assumes fileOffset==LBA, true for track 1 always and
// for every subsequent track in the common real case (no virtual pregaps), which is what
// every real .chd this session tested actually has. A real, named gap for the rarer case,
// not a hidden one.
static bool pcecd_read_toc(void) {
    int32_t plba = -150;
    int32_t fofs = 0;      // running file frame offset, see pcecd_toc_fofs above
    int track_count = 0;

    for (uint32_t idx = 0; idx < 99; idx++) {
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
            // Real fallback: some real .chd dumps only carry the older v1 tag (no
            // pregap/postgap fields) -- confirmed a real, not hypothetical, case worth
            // handling (see pcetang_cd_scsi_plan.md's TOC-probe note). pregap/postgap
            // stay 0 (track 1's real 150-sector pregap is still applied below), pgtype
            // stays all-zero so the pgtype[0]=='V' check below is real, not garbage.
            err = chd_get_metadata(pcecd_chd, CDROM_TRACK_METADATA_TAG, idx,
                                    metabuf, sizeof(metabuf) - 1, &resultlen,
                                    &resulttag, &resultflags);
            if (err == CHDERR_NONE)
                sscanf(metabuf, CDROM_TRACK_METADATA_FORMAT, &tkid, type, subtype, &frames);
        }
        if (err != CHDERR_NONE)
            break;

        track_count++;
        if (track_count > PCECD_MAX_TRACKS) {
            DEBUG("pcecd_read_toc: too many real tracks (>%d), truncating\n", PCECD_MAX_TRACKS);
            track_count = PCECD_MAX_TRACKS;
            break;
        }

        int real_pregap = (track_count == 1) ? 150 : (pgtype[0] == 'V') ? 0 : pregap;
        int real_pregap_dv = (pgtype[0] == 'V') ? pregap : 0;

        plba += real_pregap + real_pregap_dv;
        pcecd_toc_lba[track_count] = (uint32_t)plba;
        pcecd_toc_control[track_count] = (strcmp(type, "AUDIO") == 0) ? 0x00 : 0x04;

        // File-frame accumulator, in lockstep with the logical one above.
        fofs += real_pregap_dv;                 // a pregap that IS in the file
        pcecd_toc_fofs[track_count]    = (uint32_t)fofs;
        pcecd_toc_sectors[track_count] = (uint32_t)(frames - real_pregap_dv);
        fofs += (frames - real_pregap_dv);
        fofs += postgap;
        fofs += ((frames + 3) & ~3) - frames;   // pad each track up to 4 frames

        plba += (frames - real_pregap_dv) + postgap;
    }

    pcecd_toc_num_tracks = track_count;
    if (track_count == 0) {
        DEBUG("pcecd_read_toc: no real track metadata\n");
        return false;
    }

    // Real lead-out -- matches Mednafen's own `tocd.tracks[100].lba = numsectors` (the
    // real running plba accumulator IS the total real sector count at this point, same
    // as Mednafen's parallel `numsectors` accumulator for a disc with no virtual pregaps).
    pcecd_toc_lba[100] = (uint32_t)plba;

    DEBUG("pcecd_read_toc: %d real track(s), lead-out LBA=%u\n",
          track_count, (unsigned)pcecd_toc_lba[100]);
    return true;
}

// Load a PC Engine CD-ROM (.chd) image. Real flow: mount the .chd, parse+validate its
// real TOC, then boot exactly like a real PCE-CD console does -- a real PCE-CD unit has
// no on-cart program at all, only the syscard, run from the CD side once cd_bridge.vhd's
// SCSI target (already real, gw_sh-verified) starts answering real disc requests. Loads
// syscard3.pce through the same real 0x07 rom_do path loadpce() already uses for a plain
// HuCard.
// The PC Engine CD's system card, run before the disc is announced.
static int pcecd_boot_syscard(void) {
    int r;
    std::string syscard = std::string(drv) + "bios/syscard3.pce";
    FILINFO fno;
    FRESULT sres = f_stat(syscard.c_str(), &fno);
    char buf[128];
    snprintf(buf, sizeof(buf), "loadpcecd: syscard=%s res=%d sz=%lu", syscard.c_str(), (int)sres,
        sres == FR_OK ? (unsigned long)fno.fsize : 0UL);
    file_log(buf);
    r = loadpce(syscard.c_str());
    snprintf(buf, sizeof(buf), "loadpcecd: loadpce(syscard) returned %d", r);
    file_log(buf);
    if (r != 0) overlay_status("Failed to load syscard3.pce");
    return r;
}

int loadpcecd(const char *fname) {
    return cdchd_load(fname, pcecd_boot_syscard);
}

// Mount a .chd and serve it over the shared CD transport (0x0f TOC, 0x0e mount, 0x06
// requests answered with 0x10 chunks): open, read the TOC, run the system's `boot` (load the
// BIOS/system card and start the core), then send the TOC and the mount. Used by the PC Engine
// CD and the Neo Geo CD, whose FPGA sides speak the same protocol.
int cdchd_load(const char *fname, int (*boot)(void)) {
    int r = 1;
    DEBUG("loadpcecd start: %s\n", fname);
    file_log("loadpcecd: start");

    char *p = strcasestr(fname, ".chd");
    if (p == NULL) {
        file_log("loadpcecd: not a .chd, abort");
        overlay_message("Only .chd supported", 1);
        goto loadpcecd_end;
    }

    pcecd_unload();   // real: drop any previously mounted disc first
    file_log("loadpcecd: pcecd_unload done");

    {
        chd_error err = chd_fatfs_open(fname, &f_chd, &pcecd_chd);
        char buf[96];
        snprintf(buf, sizeof(buf), "loadpcecd: chd_fatfs_open err=%d (%s)", (int)err, chd_error_string(err));
        file_log(buf);
        if (err != CHDERR_NONE) {
            overlay_status("Cannot open CHD: %s", chd_error_string(err));
            goto loadpcecd_end;
        }
    }

    pcecd_hdr = chd_get_header(pcecd_chd);
    if (pcecd_hdr->unitbytes == 0 || (pcecd_hdr->hunkbytes % pcecd_hdr->unitbytes) != 0) {
        file_log("loadpcecd: bad sector layout, abort");
        overlay_status("Unexpected CHD sector layout");
        chd_close(pcecd_chd);
        pcecd_chd = NULL;
        goto loadpcecd_end;
    }
    pcecd_sectors_per_hunk = pcecd_hdr->hunkbytes / pcecd_hdr->unitbytes;
    {
        char buf[96];
        snprintf(buf, sizeof(buf), "loadpcecd: hunkbytes=%u unitbytes=%u sectors_per_hunk=%u",
            (unsigned)pcecd_hdr->hunkbytes, (unsigned)pcecd_hdr->unitbytes, (unsigned)pcecd_sectors_per_hunk);
        file_log(buf);
    }

    pcecd_hunk_buf = (uint8_t *)malloc(pcecd_hdr->hunkbytes);
    if (!pcecd_hunk_buf) {
        file_log("loadpcecd: malloc for hunk buf FAILED");
        overlay_status("Out of memory for CD hunk buffer");
        chd_close(pcecd_chd);
        pcecd_chd = NULL;
        goto loadpcecd_end;
    }
    file_log("loadpcecd: hunk buf malloc ok");
    pcecd_cached_hunk = 0xFFFFFFFF;

    if (!pcecd_read_toc()) {
        file_log("loadpcecd: pcecd_read_toc FAILED, abort");
        overlay_status("No real track metadata in CHD");
        pcecd_unload();
        goto loadpcecd_end;
    }
    file_log("loadpcecd: TOC read ok");

    r = boot();
    if (r != 0) {
        pcecd_unload();
        goto loadpcecd_end;
    }

    // Real TOC send, BEFORE mount -- cd_bridge.vhd's own TOC_CAPTURE process (see its
    // header) re-arms its first/last-track extents on DISC_MOUNTED's real falling edge
    // (already sent by pcecd_unload() above) and expects the new disc's real TOC in place
    // before the syscard starts polling TEST UNIT READY on the new mount.
    pcecd_send_toc();
    file_log("loadpcecd: TOC sent");

    // Real disc mount, sent AFTER the syscard is running -- matches real hardware
    // sequencing (the syscard's own boot code polls TEST UNIT READY/REQUEST SENSE
    // before assuming a disc is present, per cd_bridge.vhd's own real command trace).
    pcecd_send_mount(1);
    file_log("loadpcecd: mount sent, done");
    r = 0;

loadpcecd_end:
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "loadpcecd: returning %d", r);
        file_log(buf);
    }
    return r;
}
