/*
 * Modifications copyright (c) 2026 Romain Tisserand.
 * Originally from nand2mario/firmware-bl616 (Apache-2.0); this file has been
 * modified in the pcetang fork.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * TangCore firmware for BL616 MCU
 *
 * (c) 2025, nand2mario <nand2mario@outlook.com>
 *
 * This source code is licensed under the Apache 2.0 license found in the
 * LICENSE file in the root directory of this source tree. 
 *
 */

#include <stdarg.h>
#include <string.h>
#include <vector>

extern "C" {
#include "board.h"
#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "bflb_clock.h"
#include "bl616_clock.h"

#ifdef TANGCORE_USB_CDC_DEBUG
#include "cdc_debug.h"
#else
#include "usbh_core.h"
#endif
#include "ff.h"
#include "fatfs_diskio_register.h"
}

#include "file_chooser.h"
#include "programmer.h"
#include "usb_gamepad.h"
#include "tc_utils.h"
#include "cores.h"
#include "overlay.h"
#include "chd_fatfs.h"
#include "core/pcecd.h"
#include "core/pcesave.h"
#include "core/neosave.h"
#include "init.h"
#include "menu_manager.h"
#include "options_menu.h"
#include "wifi_debug.h"

extern "C" char *strcasestr(const char *haystack, const char *needle);

// Uncomment this to enable UART console (use with caution. it may interfere with MCU-FPGA communication)
#define UART_CONSOLE

/////////////////////////////////////////////////////////////////////////////////
// Global state

int option_osd_key = OPTION_OSD_KEY_SELECT_RIGHT;
int16_t active_core = -1;           // firmware detected this core as active
bool core_running;                  // a rom is loaded and running on the core
struct core_info *core;

// UART
struct bflb_device_s *gpio_dev;
struct bflb_device_s *uart0_dev;
struct bflb_device_s *uart1_dev;

// USB and fatfs
#ifndef TANGCORE_USB_CDC_DEBUG
struct usbh_msc *msc;
#endif
const char *drv = "sd:";

// Tasks and shared state
TaskHandle_t main_task_handle;
TaskHandle_t uart1_rx_task_handle;

/* CD sector requests are handed to a worker task instead of being served inline.
 *
 * uart1_rx_task is the ONLY consumer of the UART1 RX FIFO and it polls
 * (bflb_uart_rxavailable). Serving a sector inline blocks that poll for tens of
 * milliseconds -- a libchdr hunk decode plus a 2048-byte TX -- during which anything the
 * FPGA sends queues into a 32-byte hardware FIFO and, once that is full, is LOST.
 *
 * Measured on hardware 2026-09-11. The FPGA asked for LBA 3591 (bytes 00 00 0E 07); the
 * MCU logged a request for 43520 = 0x00AA00, i.e. bytes 00 00 AA 00 -- the frame arrived
 * two bytes short and the parser back-filled from the NEXT frame's header (0xAA) and
 * length byte. 43520 is inside the disc, so the MCU then served a perfectly valid sector
 * from completely the wrong place and the syscard got garbage where the IPL belonged:
 * LOAD ERROR on Dungeon Explorer II and Bonk III, and a retry storm on the games whose
 * boot read is 16+ sectors (Prince of Persia, Double Dragon II).
 *
 * With this queue the RX task parses a frame, posts it, and goes straight back to
 * polling, so the FIFO is drained continuously and no frame is ever truncated. */
struct cd_req_t {
    uint32_t lba;
    bool     is_audio;
};
static QueueHandle_t cd_req_queue;
// Highest UART1 RX FIFO occupancy ever seen, reported in pcecd's cdprog line.
volatile uint16_t uart1_rx_hiwater = 0;
TaskHandle_t cd_serve_task_handle;

#ifdef TANG_CONSOLE60K
const char *BOARD_NAME = "console60k";
#elif defined(TANG_CONSOLE138K)
const char *BOARD_NAME = "console138k";
#elif defined(TANG_MEGA60K)
const char *BOARD_NAME = "mega60k";
#elif defined(TANG_MEGA138K)
const char *BOARD_NAME = "mega138k";
#elif defined(TANG_PRIMER25K)
const char *BOARD_NAME = "primer25k";
#elif defined(TANG_NANO20K)
const char *BOARD_NAME = "nano20k";
#else
const char *BOARD_NAME = "unknown";
#endif

// Override system printf() to send to FPGA
int __attribute__((weak)) putchar(int ch) {
    fpga_tx_lock();
    fpga_tx_header(0x05, 2);
    fpga_tx_byte(ch);
    fpga_tx_unlock();
    return ch;
}

// Diagnostic: raw UART1 console (the SDK's own debug console, set via
// bflb_uart_set_console in init_gpio_and_uart), independent of FPGA config
// state, so we can see boot progress even when the FPGA never gets configured.
// Diagnostic: log to a real file on the mounted drive instead of the
// screen -- overlay text is timing-sensitive and unreadable on a fast TV
// redraw, especially once a game core's own video output takes over from
// the menu core. Uses its own FIL handle so it never collides with fcore
// (the global handle used for ROM/core streaming).
FIL flog;
bool flog_open = false;
// Live debug output over UART0 (2026-09-06). UART0 is GPIO21/22, brought up at 2Mbaud
// by board_init(), and on Console 60K it leaves the board through the second USB-C as a
// CH340 USB-serial port -- so this streams straight to a terminal on a dev PC, with no
// SD-card swapping. Deliberately UART0 and NOT UART1: UART1 is the live MCU<->FPGA
// protocol link, and writing log text into it corrupts that protocol (uart_dbg() used
// to do exactly that -- fixed below).
// Never blocks on a missing device: if UART0 isn't up, this is a no-op.
void dbg_uart_puts(const char *s) {
    if (!uart0_dev) {
        uart0_dev = bflb_device_get_by_name("uart0");
        if (!uart0_dev) return;
    }
    bflb_uart_put(uart0_dev, (uint8_t*)s, strlen(s));
    bflb_uart_put(uart0_dev, (uint8_t*)"\r\n", 2);
}

// SD logging is ASYNCHRONOUS as of 2026-09-14. Two reasons, both measured:
//
// 1. CORRUPTION. file_log() was called from uart1_rx_task (priority 3) for every
//    RTL[..] trace frame while cd_serve_task (priority 2) sat inside f_read() on the
//    same volume. The logger preempts the reader, and FatFs keeps ONE sector window per
//    volume (fs->win/fs->winsect), so the interleave flushes a dirty window to the wrong
//    sector. That is structural damage, and fsck.exfat found exactly its signature:
//    duplicated directory entries whose clusters were already allocated to other files
//    (fbneo, fds, gameandwatch, gamegear) plus a bad cluster ref in debug.log itself.
//    FF_FS_REENTRANT (see fatfs_conf_user.h) makes concurrent access safe; this removes
//    the contention that was driving it.
// 2. RX STARVATION. f_sync() per line blocks for milliseconds, and cd_bridge fires
//    SECTOR_REQ the moment it has consumed 2048 bytes -- a request frame landing in that
//    window is dropped. That is the fault PCECD_TRACE_SERVE's own comment documents.
//    Writes now happen only on log_task, never on the RX path.
//
// Callers keep calling file_log() unchanged; it just enqueues. A full queue DROPS the
// line rather than blocking, because stalling the RX path is the exact thing this
// exists to prevent; the drop count is reported so silent loss is visible.
#define LOG_QUEUE_LEN 48
#define LOG_LINE_MAX  176
static QueueHandle_t log_queue = NULL;
static volatile uint32_t log_drops = 0;
static uint32_t log_drops_seen = 0;

static void file_log_sync(const char *msg);

void file_log(const char *msg) {
    // Live copy first, on the caller's task: works before any drive is mounted and
    // survives a hang that would eat the queued copy.
    dbg_uart_puts(msg);

#ifndef TANGCORE_SD_LOG
    // Release build: no debug.log on the SD card (a power cut mid-write can damage the
    // FAT). `make DEBUG=1` turns the SD log back on for bug hunting.
    return;
#endif
    if (log_queue == NULL) {          // before the logger task exists, write inline
        file_log_sync(msg);
        return;
    }
    char line[LOG_LINE_MAX];
    strncpy(line, msg, LOG_LINE_MAX - 1);
    line[LOG_LINE_MAX - 1] = 0;
    if (xQueueSend(log_queue, line, 0) != pdTRUE)
        log_drops++;                  // never block a caller on the SD card
}

// The ONLY place that touches the filesystem for logging.
static void file_log_sync(const char *msg) {
    if (!flog_open) {
        if (f_open(&flog, (std::string(drv) + "debug.log").c_str(), FA_WRITE | FA_OPEN_APPEND) != FR_OK)
            return;
        flog_open = true;
    }
    UINT bw;
    f_write(&flog, msg, strlen(msg), &bw);
    f_write(&flog, "\r\n", 2, &bw);
    f_sync(&flog);   // flush immediately so the log survives a hang/crash
}

void uart_dbg(const char *s) {
    // Was writing to uart1_dev -- the FPGA protocol link -- which injected raw log text
    // into the MCU<->core byte stream. Now goes to the SD log and UART0 only (file_log
    // calls dbg_uart_puts itself).
    file_log(s);
}


/////////////////////////////////////////////////////////////////////////////////
// Core loading and other file system operations

// nand2mario: these USB data structures cannot be CACHED as they are written to by hardware
USB_NOCACHE_RAM_SECTION FATFS fs;
USB_NOCACHE_RAM_SECTION FIL fcore;
USB_NOCACHE_RAM_SECTION BYTE __attribute__((aligned(64))) fbuf[BLOCK_SIZE];

FRESULT res_sd;
FileChooser file_chooser;

// #define PAGESIZE 22
// #define TOPLINE 2
// #define PWD_SIZE 1024
// char pwd[PWD_SIZE];
// one page of file names to display
// char file_names[PAGESIZE][256];
// int file_dir[PAGESIZE];         // this file is a directory
// int file_sizes[PAGESIZE];       
// int file_len;		            // number of files on this page

/////////////////////////////////////////////////////////////////////////////////
// Menu display and user interaction

// Menus for "NES", "SNES" ... entries
// dir: initial dir including the drive name (e.g. "sd:nes", "usb:cores")
// return 0: user chose a ROM (*choice), 1: no choice made, -1: error
// file chosen: pwd / file_name[*choice]
static int menu_loadrom(const char *dir) {
    string fname;
    file_chooser.rootdir = dir;
    file_chooser.curdir = dir;
    file_chooser.msg_return = "<< Return to main menu";
    bool r = file_chooser.choose_file(fname);
    if (!r) {
        overlay_status("No file chosen");
        return 1;
    }

    // now proceed to load the core and ROM
    joy1_state = 0; joy2_state = 0; // clear joypad states

    // load core if in cores/ dir
    if (fname.find(string(drv) + "cores") == 0) {
        overlay_status("Core: %s", fname.c_str());
        fpga_program(fname.c_str());
        _overlay_on = 1;                // turn on overlay after core is loaded
        return 0;       // return to main menu
    } 

    // find core info entry
    core_info *core = NULL;
    string path = fname.substr(fname.find(":")+1);
    // PC Engine's unified entry (id 8) owns both .pce (pce/) and .chd (pcenginecd/) --
    // extension is the real discriminator, checked first so a .chd under pcenginecd/
    // doesn't need to match a rom_dir prefix that entry no longer solely owns.
    // Neo Geo CD discs are .chd too: anything under neogeocd/ belongs to the Neo Geo CD
    // entry (id 9, its own bitstream), checked before PC Engine claims every .chd.
    if (path.find("neogeocd/") == 0) {
        core = find_core_by_id(9);
        file_log("menu_loadrom: neogeocd/ path -> Neo Geo CD");
    }
    else if (strcasestr(path.c_str(), ".pce") || strcasestr(path.c_str(), ".chd") ||
        strcasestr(path.c_str(), ".sgx")) {
        core = find_core_by_id(8);
        char buf[160];
        snprintf(buf, sizeof(buf), "menu_loadrom: .pce/.sgx/.chd extension match, path=%s core=%p", path.c_str(), (void*)core);
        file_log(buf);
        wifi_log(buf);
    }
    for (size_t i = 0; core == NULL && i < core_info_list.size(); i++) {
        core_info *c = &core_info_list[i];
        // match on a full path segment ("pc/" not just "pc") -- a loose
        // prefix check here matches "pc" (PC/XT) against "pce/..." and
        // "pcenginecd/..." paths too, since "pc" is a literal prefix of
        // both; PC/XT sits earlier in core_info_list so it always won the
        // race, silently loading pctang.bin for PCE/PCE-CD ROMs (real,
        // observed on Console 60K, 2026-09-01).
        if (path.find(std::string(c->rom_dir) + "/") == 0) {
            overlay_status("ROM for: %s", c->display_name);
            core = c;
            break;
        }
    }
    if (core == NULL) {
        overlay_status("Core not found: %s", path.c_str());
        return -1;
    }

    // user chose a ROM file
    active_core = get_core_id();

    // load core if needed
    if (core != NULL) {
        if (active_core != core->id) {      // active core is not what we need
            string fname_core;
            if (find_core_for_board(fname_core, core->core_file)) {
                // load core
                overlay_cursor(0, 10);
                overlay_printf("DBG fname=%s id=%d          ", fname_core.c_str(), core->id);
                {
                    char buf[128];
                    snprintf(buf, sizeof(buf), "menu_loadrom: loading core fname=%s want_id=%d", fname_core.c_str(), core->id);
                    file_log(buf);
                }
                bool prog_ok = fpga_program(fname_core.c_str());
                overlay_cursor(0, 11);
                overlay_printf("DBG fpga_program=%d          ", prog_ok ? 1 : 0);
                {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "menu_loadrom: fpga_program returned %d", prog_ok ? 1 : 0);
                    file_log(buf);
                }
                _overlay_on = 1;

                // allow 2 seconds for core to start
                uint64_t start = bflb_mtimer_get_time_ms();
                int16_t last_seen = -99;
                while (bflb_mtimer_get_time_ms() - start < 2000) {
                    send_blank_packet();
                    active_core = get_core_id();
                    last_seen = active_core;
                    if (active_core == core->id)
                        break;
                }
                overlay_cursor(0, 12);
                overlay_printf("DBG poll last_active_core=%d          ", last_seen);
                {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "menu_loadrom: poll done, last_active_core=%d want=%d", last_seen, core->id);
                    file_log(buf);
                    wifi_log(buf);   // safe here: fpga_program()'s critical section already exited
                }
            }
        }

        // Attemp to load ROM
        if (active_core == core->id) {
            // A freshly configured bitstream starts with core_config = 0: restore the
            // options saved for this core before the game starts.
            apply_saved_core_options(core->id);
            char buf[160];
            snprintf(buf, sizeof(buf), "menu_loadrom: calling load_rom fname=%s", fname.c_str());
            file_log(buf);
            wifi_log(buf);
            overlay_status("Loading ROM: %s\n", fname.c_str());
            core->load_rom(fname.c_str());
            file_log("menu_loadrom: load_rom returned");
            wifi_log("menu_loadrom: load_rom returned");
            return 1;
        } else {
            file_log("menu_loadrom: Core failed to load (active_core != core->id)");
            wifi_log("menu_loadrom: Core failed to load (active_core != core->id)");
            overlay_status("Core failed to load\n");
            delay(1000);
            return -1;
        }
    }
    return -1;
}

// Main menu "Options": the options of the game core currently loaded, if any. At boot
// the menu runs on monitor.bin, which has none worth showing.
static void menu_options(void) {
    menu_clear();
    push_menu(std::unique_ptr<Menu>(create_options_menu(active_core > 0 ? active_core : -1)));
    menu_current()->do_redraw();
    menu_input_loop();
    menu_clear();
}

// keep sending HID state to core until OSD is turned on
static void send_hid_to_core(void) {
    uint16_t hid1_old = 0, hid2_old = 0;
    bool first = true;
    dprint("Start sending HID to core...");
    while (1) {
        uint16_t joy1=0, joy2=0, hid1=0, hid2=0;    
        get_joypad_states(&joy1, &joy2, &hid1, &hid2);
        if (first || hid1 != hid1_old || hid2 != hid2_old) {    // send HID if changed
            // Runs continuously during gameplay, so it is the writer most likely to meet
            // an in-flight sector. The token makes the whole frame atomic against it.
            fpga_tx_lock();
            fpga_tx_header(0x09, 5);
            fpga_tx_byte(hid1 >> 8);
            fpga_tx_byte(hid1 & 0xff);
            fpga_tx_byte(hid2 >> 8);
            fpga_tx_byte(hid2 & 0xff);
            fpga_tx_unlock();
            hid1_old = hid1;
            hid2_old = hid2;
            first = false;
        }
        if (joy1 == OSD_KEY_CODE || joy2 == OSD_KEY_CODE || hid1 == OSD_KEY_CODE || hid2 == OSD_KEY_CODE) {
            break;
        }
        if (overlay_on())      // turned off by keyboard
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    dprint("Stopped sending HID to core.");
    // The in-game OSD is opening. Save now if the game wrote backup RAM (MiSTer's autosave
    // point) -- and since the OSD is the only way to switch games, this is also what makes
    // sure the old game is saved before another one loads.
    pcesave_flush_now();
    neosave_flush_now();
}

// // (R L X A RT LT DN UP START SELECT Y B)
// Return: 1 button B pressed, 4: button A pressed, 2: next page, 3: previous page
// active is the entry chosen
int joy_choice(int start_line, int len, int *active, int overlay_key_code) {
    if (*active < 0 || *active >= len)
        *active = 0;
    uint16_t joy1=0, joy2=0, hid1=0, hid2=0;    
    int last = *active;

    get_joypad_states(&joy1, &joy2, &hid1, &hid2);
    joy1 |= hid1;
    joy2 |= hid2;

    if ((joy1 == overlay_key_code) || (joy2 == overlay_key_code)) {
        overlay_status("OSD: %s", overlay_on() ? "ON" : "OFF");
        overlay(!overlay_on());    // toggle OSD
        delay(300);
    }

    if (!overlay_on()) {           // keep sending HID state to core when OSD is off
        send_hid_to_core();
        return 0;
    }

    // Check buttons BEFORE direction: a controller that reports both a
    // direction bit and a button bit in the same poll (debounce overlap,
    // e.g. real DS2-style pads over PMOD) must not shift *active on the
    // exact poll that also confirms/cancels -- that produces an off-by-one
    // selection (real, observed on Console 60K's DS2 controller, 2026-09-01).
    if ((joy1 & 0x40) || (joy2 & 0x40))
        return 3;      // previous page
    if ((joy1 & 0x80) || (joy2 & 0x80))
        return 2;      // next page
    if ((joy1 & 0x100) || (joy2 & 0x100))
        return 4;      // button A pressed
    if ((joy1 & 0x1) || (joy2 & 0x1))
        return 1;      // button B pressed

    if ((joy1 & 0x10) || (joy2 & 0x10)) {
        if (*active > 0) (*active)--;
    }
    if ((joy1 & 0x20) || (joy2 & 0x20)) {
        if (*active < len-1) (*active)++;
    }

    overlay_cursor(0, start_line + (*active));
    overlay_printf(">");

    // overlay_cursor(0, 27);
    // overlay_printf(" j1=%04x j2=%04x h1=%04x h2=%04x", joy1, joy2, hid1, hid2);
    if (last != *active) {
        overlay_cursor(0, start_line + last);
        overlay_printf(" ");
        delay(100);     // button debounce
    }    
    return 0;
}

/* 2026-09-10: was 2048 words (8KB). A CD load hangs inside FatFs f_open, and the SD
 * card is exFAT: FF_USE_LFN 2 puts the LFN working buffer on the STACK, costing
 * (FF_MAX_LFN+1)*2 + (FF_MAX_LFN+44)/15*32 = 1120 bytes per call with exFAT enabled,
 * and f_open is reached eight frames deep through C++ code holding std::string.
 * FreeRTOS's stack-overflow hook is `printf(); while(1);` on UART0, which nobody is
 * capturing -- so an overflow looks exactly like the observed silent hang. 32KB costs
 * 24KB of a 447KB region that is 12.77% used. The high-water mark is logged next to
 * the CD open so this stops being a guess. */
// 8192 words = 32KB. REVERTED 2026-09-12 after shrinking this to 2048 words caused
// "FATAL: stack overflow in task 'main_task'" during loadpcecd -- the core was then never
// programmed into the FPGA at all, so the board showed nothing, not even the system card.
//
// My justification for shrinking it was a measurement of 29736 bytes still free, but that
// was sampled at an idle moment, NOT during the CHD load path (libchdr chd_open plus
// FatFs) which runs on this task. A high-water figure only bounds the paths actually
// exercised when it was taken.
//
// There is no RAM pressure here to justify trimming: ram_memory sits at ~13% of 447KB.
// Do not shrink again without a high-water mark captured DURING a CHD load.
#define MAIN_TASK_STACK_SIZE  8192
#define MAIN_TASK_PRIORITY    3
/* 2026-09-11: was 512 words (2KB), and the SD-logged stack-overflow hook caught it:
 * "FATAL: stack overflow in task 'uart1_rx_task'" right after DECODE-START. The CD
 * sector request arrives as UART opcode 6, so pcecd_serve_sector() -- and with it the
 * whole chd_read() decode, LZMA/zlib/FLAC included -- runs ON THIS TASK'S STACK. miniz's
 * tinfl_decompress_mem_to_* frames alone measure ~8.4KB on rv32imafc -O2, so 2KB never
 * had a chance; the decode simply ran off the end and the SDK's own hook (printf on an
 * uncaptured UART0, then while(1)) made it look like chd_read was hanging.
 *
 * NOTE this is the task that actually needed the space -- MAIN_TASK_STACK_SIZE was raised
 * to 8192 earlier while chasing the same symptom and measured 29736 bytes still free,
 * because the decode was never on main_task at all. See the high-water marks logged next
 * to SERVED: to right-size BOTH of these afterwards. */
// 8192 words = 32KB. Also reverted: this task still handles the floppy read/write paths
// (a 512-byte buffer plus FatFs f_read/f_lseek) and the overlay, so "it only parses
// 5-byte frames now" was wrong. Shrinking two stacks in one step also meant a single
// failure could not be attributed to either.
#define UART1_RX_TASK_STACK_SIZE  8192
#define UART1_RX_TASK_PRIORITY    3

// Serves CD sector requests off the RX path. Priority is BELOW uart1_rx_task so that
// draining the UART always wins over decoding a hunk -- the whole point of the split.
// 4096 words = 16KB. libchdr's decode runs here. Measured peak on hardware was ~2.2KB
// (uxTaskGetStackHighWaterMark reports minimum-ever-free, so it does capture chd_read),
// but that was one disc's codec path -- 16KB keeps ~7x margin for a CHD that decodes
// differently, because a stack overflow here fails silently and cost a full day once.
// Drains the log queue onto the SD card. Priority 1 -- BELOW cd_serve_task (2) and
// uart1_rx_task (3) -- so logging can never preempt a hunk decode or the UART RX path.
// That ordering is the point: the old code logged from the highest-priority task.
#define LOG_TASK_STACK_SIZE   2048
#define LOG_TASK_PRIORITY     1
static void log_task(void *pvParameters)
{
    char line[LOG_LINE_MAX];
    for (;;) {
        if (xQueueReceive(log_queue, line, portMAX_DELAY) == pdTRUE)
            file_log_sync(line);
        // Report dropped lines once the burst has passed, so a full queue is never
        // silent -- absence of a line would otherwise read as absence of the event.
        if (log_drops != log_drops_seen && uxQueueMessagesWaiting(log_queue) == 0) {
            char b[64];
            snprintf(b, sizeof(b), "LOG-DROPS: %lu lines lost (queue full)",
                     (unsigned long)log_drops);
            log_drops_seen = log_drops;
            file_log_sync(b);
        }
    }
}

#define CD_SERVE_TASK_STACK_SIZE  8192
#define CD_SERVE_TASK_PRIORITY    2
static void cd_serve_task(void *pvParameters)
{
    cd_req_t req;
    for (;;) {
        // Bounded wait rather than portMAX_DELAY: a timeout here means the CD path has
        // gone quiet, which is exactly the stalled state worth dumping. Serving is
        // unaffected -- a queued request still wakes this immediately.
        if (xQueueReceive(cd_req_queue, &req, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (req.is_audio)
                pcecd_serve_audio_sector(req.lba);
            else
                pcecd_serve_sector(req.lba);
        } else {
            pcecd_trace_idle_tick();
        }
    }
}

// ---------------------------------------------------------------------------
// INTERRUPT-DRIVEN UART1 RX (2026-09-16).
//
// uart1_rx_task used to poll bflb_uart_getchar() directly, so the 32-byte hardware FIFO
// was only drained while that task was scheduled. That has always been the fragile part
// of this link -- rxhi peaked at 24 of 32 even before any of the CD work, i.e. 75% full
// with no margin -- and it is how sector-request frames get truncated when the CPU is
// busy (see ef4ffd7: serving inline on this task produced LBA 3591 arriving as
// 0x00AA00). Adding DMA sector transmit spent what margin was left: rxhi reached 31 of
// 32, one byte from loss, and requests stopped reaching the MCU entirely -- the FPGA
// waiting forever for a sector, the MCU idle on an empty queue, the SCSI bus parked in
// COMMAND phase.
//
// The ISR below drains the FIFO into a multi-kilobyte software ring, and the task parses
// from the ring. An interrupt preempts every task, so no amount of DMA bookkeeping,
// libchdr decode or vTaskDelay can now cost a received byte. The hardware FIFO should sit
// near empty from here on, which is what uart1_rx_hiwater keeps measuring.
//
// Single producer (ISR), single consumer (task), power-of-two size: head and tail need no
// lock as long as each side only writes its own index.
#define U1RX_RING_SIZE  4096u
#define U1RX_RING_MASK  (U1RX_RING_SIZE - 1u)
static uint8_t u1rx_ring[U1RX_RING_SIZE];
static volatile uint32_t u1rx_head = 0;   // ISR writes
static volatile uint32_t u1rx_tail = 0;   // task writes
// High-water of the SOFTWARE ring, and a count of bytes dropped because it was full.
// A nonzero drop count means the TASK is too slow, which is a different fault from the
// FIFO overrun this replaces -- worth distinguishing rather than conflating.
volatile uint16_t u1rx_ring_hiwater = 0;
volatile uint32_t u1rx_ring_drops = 0;

static void uart1_isr(int irq, void *arg)
{
    (void)irq; (void)arg;
    uint32_t st = bflb_uart_get_intstatus(uart1_dev);

    if (st & (UART_INTSTS_RX_FIFO | UART_INTSTS_RTO)) {
        // Keep the hardware-FIFO measurement: it is now the proof the ISR is keeping up.
        int rxcnt = bflb_uart_feature_control(uart1_dev, UART_CMD_GET_RX_FIFO_CNT, 0);
        if (rxcnt > (int)uart1_rx_hiwater)
            uart1_rx_hiwater = (uint16_t)rxcnt;

        while (bflb_uart_rxavailable(uart1_dev)) {
            uint8_t ch = bflb_uart_getchar(uart1_dev);
            uint32_t nxt = (u1rx_head + 1u) & U1RX_RING_MASK;
            if (nxt == (u1rx_tail & U1RX_RING_MASK)) {
                u1rx_ring_drops++;      // ring full: the task is behind, not the FIFO
                continue;               // keep draining so the FIFO never overruns
            }
            u1rx_ring[u1rx_head & U1RX_RING_MASK] = ch;
            u1rx_head = nxt;
        }

        uint32_t used = (u1rx_head - u1rx_tail) & U1RX_RING_MASK;
        if (used > u1rx_ring_hiwater)
            u1rx_ring_hiwater = (uint16_t)used;
    }
    // RTO is what delivers a frame's tail when it is shorter than the FIFO threshold.
    if (st & UART_INTSTS_RTO)
        bflb_uart_int_clear(uart1_dev, UART_INTCLR_RTO);
}

static inline bool u1rx_get(uint8_t *ch)
{
    if (u1rx_tail == u1rx_head) return false;
    *ch = u1rx_ring[u1rx_tail & U1RX_RING_MASK];
    u1rx_tail = (u1rx_tail + 1u) & U1RX_RING_MASK;
    return true;
}

void uart1_rx_irq_init(void)
{
    bflb_uart_rxint_mask(uart1_dev, false);
    bflb_irq_attach(uart1_dev->irq_num, uart1_isr, NULL);
    bflb_irq_enable(uart1_dev->irq_num);
}

// Receive joypad updates and other UART responses from the FPGA
static void uart1_rx_task(void *pvParameters)
{
    uint8_t buffer[5];
    // uint16_t, not uint8_t: frame payloads run to 514 bytes (save-RAM blocks, floppy
    // sectors). As uint8_t the floppy handler's `pos == 6+511` could never be reached.
    uint16_t pos = 0;
    uint8_t type = 0;
    uint16_t sv_blk = 0;
    uint16_t len = 0;
    uint8_t dbg_trace_tag = 0;
    uint8_t dbg_trace_buf[8] = {0};

    while (1) {
        uint8_t ch;
        if (u1rx_get(&ch)) {
            
            if (pos == 0) {          // expecting 0xAA
                if (ch == 0xAA) 
                    pos++;
            } else if (pos == 1) {   // len msb
                len = (uint16_t)ch << 8;
                pos++;
            } else if (pos == 2) {   // len lsb
                len += ch;
                pos++;
            } else if (pos == 3) {   // command type
                type = ch;
                pos++;

            ////// pos >= 4 //////
            } else if (type == 1) {     // response to command 1 (get core ID)
                if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                    core_id = ch;
                    xSemaphoreGive(state_mutex);
                }
                pos = 0;
            } else if (type == 2) {                 // config string, len-1 bytes
                static char conf_rx[CORE_CONF_MAX];
                uint16_t i = pos - 4;
                if (i < CORE_CONF_MAX) conf_rx[i] = ch;
                if (pos == len+2) {
                    uint16_t n = len - 1 < CORE_CONF_MAX ? len - 1 : CORE_CONF_MAX;
                    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                        memcpy(core_conf_str, conf_rx, n);
                        core_conf_str[n] = 0;
                        core_conf_len = n;
                        xSemaphoreGive(state_mutex);
                    }
                    pos = 0;
                } else
                    pos++;
            } else if (type == 3) {                 // periodic joypad state
                buffer[pos-4] = ch;
                // Complete packet received
                if (pos == 7) {
                    // Combine bytes into 16-bit values
                    uint16_t joy1 = (buffer[0] << 8) | buffer[1];
                    uint16_t joy2 = (buffer[2] << 8) | buffer[3];
                    
                    // Update global state with mutex protection
                    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                        joy1_state = joy1;
                        joy2_state = joy2;
                        xSemaphoreGive(state_mutex);
                    }
                    pos = 0; // Reset for next packet
                } else
                    pos++;
            } else if (type == 4) {              // floppy write
                if (pos < 6)
                    buffer[pos-4] = ch;
                else
                    fbuf[pos-6] = ch;
                if (pos == 6+511) {
                    uint16_t drive = buffer[0] >> 7;
                    uint16_t sector = (buffer[0] & 0x7f) << 8 | buffer[1];
                    if (floppy[drive]) {
                        UINT br;
                        f_lseek(&f_floppy[drive], sector * 512);
                        if (f_write(&f_floppy[drive], fbuf, 512, &br) != FR_OK) {
                            overlay_status("Failed to write floppy");
                        }
                    }
                    pos = 0;   // reset for next packet
                } else
                    pos++;
            } else if (type == 5) {              // floppy read
                buffer[pos-4] = ch;
                if (pos == 5) {
                    uint16_t drive = buffer[0] >> 7;
                    uint16_t sector = (buffer[0] & 0x7f) << 8 | buffer[1];
                    if (floppy[drive]) {
                        UINT br;
                        f_lseek(&f_floppy[drive], sector * 512);
                        if (f_read(&f_floppy[drive], fbuf, 512, &br) == FR_OK) {
                            fpga_tx_lock();
                            fpga_tx_header(0x0a, br+1);
                            for (UINT i = 0; i < br; i++) {
                                fpga_tx_byte(fbuf[i]);
                            }
                            fpga_tx_unlock();
                        } else {
                            overlay_status("Failed to read floppy");
                        }
                    }
                    pos = 0;   // reset for next packet
                } else
                    pos++;

            } else if (type == 6) {              // real CD sector request (is_audio[7:0] lba[23:0])
                buffer[pos-4] = ch;
                if (pos == 7) {
                    // Real (2026-08-31g): byte 0 was always 0 (top byte of a real CD LBA,
                    // genuinely unused as address bits, real CD max is ~330K sectors/19
                    // bits) -- repurposed on the FPGA side (iosys_bl616.v's own
                    // SEND_CD_SECTOR_REQ) to carry SECTOR_IS_AUDIO. See cd_bridge.vhd's
                    // own SECTOR_IS_AUDIO port comment for the real FPGA-side design.
                    bool is_audio = (buffer[0] & 0x01) != 0;
                    uint32_t lba = ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8)
                                  | buffer[3];
                    // Post, never serve inline -- see cd_req_queue's comment above.
                    // Non-blocking on purpose: if the queue were ever full, dropping the
                    // request is still better than stalling the RX poll, which is the
                    // very failure this exists to prevent.
                    cd_req_t req = { lba, is_audio };
                    if (cd_req_queue)
                        xQueueSend(cd_req_queue, &req, 0);
                    pos = 0;
                } else
                    pos++;

            } else if (type == 9) {              // RTL debug trace (see iosys_bl616.v)
                // A general FPGA->MCU debug channel: the core sends a 1-byte tag plus
                // 8 payload bytes, and they land as a line in debug.log on the SD card.
                // This exists because there is no UART or JTAG into the running core --
                // before it, the only way to see an internal RTL signal was to paint it
                // on the HDMI output and read it off the screen by eye.
                // NB: deliberately does NOT use `buffer` -- that is only 5 bytes, while
                // this frame carries 9 payload bytes. Writing through it here overflowed
                // the stack and crashed the MCU mid-log.
                if (pos == 4) {
                    dbg_trace_tag = ch;
                    pos++;
                } else if (pos < 4 + 1 + 8) {
                    dbg_trace_buf[pos - 5] = ch;
                    pos++;
                    if (pos == 4 + 1 + 8) {
                        char b[128];
                        snprintf(b, sizeof(b),
                            "RTL[%02x] %02x %02x %02x %02x %02x %02x %02x %02x",
                            dbg_trace_tag,
                            dbg_trace_buf[0], dbg_trace_buf[1], dbg_trace_buf[2], dbg_trace_buf[3],
                            dbg_trace_buf[4], dbg_trace_buf[5], dbg_trace_buf[6], dbg_trace_buf[7]);
                        file_log(b);
                        pos = 0;
                    }
                } else {
                    pos = 0;
                }

            } else if (type == 0x0A) {           // save-RAM block: blk[15:0] + 512 bytes
                uint16_t k = pos - 4;
                if (k == 0)      sv_blk = (uint16_t)ch << 8;
                else if (k == 1) sv_blk |= ch;
                else             pcesave_rx_byte(sv_blk, k - 2, ch);
                if (k == 2 + 511) { pcesave_rx_block_done(sv_blk); pos = 0; }
                else pos++;
            } else if (type == 0x0B) {           // the game wrote backup RAM (1 pad byte)
                pcesave_rx_dirty();
                pos = 0;
            } else {
                pos = 0; // Reset if we get out of sync
            }
        }

        // Idle path of the RX task -- reached only when no byte is pending. This task
        // runs at UART1_RX_TASK_PRIORITY (3), ABOVE cd_serve_task (2), so it keeps
        // running even when a sector serve is wedged inside chd_read() and never
        // returns to its own queue-receive timeout. That is precisely the case the
        // request ring exists to name, so the flush cannot live only in the serve task.
        // Self-gating on "a disc is mounted and 3s quiet", so it costs one tick
        // comparison per millisecond and writes nothing while sectors are flowing.
        pcecd_trace_idle_tick();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Display main menu and call other menu functions
static void main_task(void *pvParameters)
{
    uint32_t last_redraw_time = 0;
    // volatile uint32_t *reg_gpio0 = (volatile uint32_t *)0x200008c4;     // bl616 reference 4.8.5
    // volatile uint32_t *reg_gpio1 = (volatile uint32_t *)0x200008c8;
    // volatile uint32_t *reg_gpio2 = (volatile uint32_t *)0x200008cc;
    // volatile uint32_t *reg_gpio3 = (volatile uint32_t *)0x200008d0;

    // wait for drive to be ready
    uint64_t start = bflb_mtimer_get_time_ms();
    FRESULT res;
    overlay_status("Mounting sd card...", drv);
    uart_dbg("BOOT: mounting sd:");
    while ((res = f_mount(&fs, "sd:", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 500)
        delay(100);

    if (res == FR_OK) {
        overlay_status("SD card mounted in %d ms", bflb_mtimer_get_time_ms() - start);
        uart_dbg("BOOT: sd: mounted");
    } else  {
#ifdef TANGCORE_USB_CDC_DEBUG
        // No USB host in this build, so there is no usb: drive to fall back to.
        overlay_status("SD not found (no usb: fallback in CDC debug build)");
        uart_dbg("BOOT: sd: not found, no usb: fallback (USB_CDC_DEBUG build)");
#else
        overlay_status("SD not found. Mounting USB...");
        uart_dbg("BOOT: sd: not found, mounting usb:");
        drv = "usb:";
        start = bflb_mtimer_get_time_ms();
        while ((res = f_mount(&fs, "usb:", 1)) != FR_OK && bflb_mtimer_get_time_ms() - start < 15000)
            delay(100);
        if (res != FR_OK) {
            overlay_status("Failed to mount USB drive");
            uart_dbg("BOOT: usb: mount FAILED (timeout)");
        } else {
            overlay_status("USB drive mounted in %d ms", bflb_mtimer_get_time_ms() - start);
            uart_dbg("BOOT: usb: mounted");
        }
#endif
    }

    // load monitor core at startup
    string fname;
    uart_dbg("BOOT: looking for monitor.bin");
    if (find_core_for_board(fname, "monitor.bin")) {
        uart_dbg("BOOT: monitor.bin found, calling fpga_program");
        bool ok = fpga_program(fname.c_str());
        uart_dbg(ok ? "BOOT: fpga_program returned OK" : "BOOT: fpga_program returned FAIL");
    } else {
        overlay_status("No monitor.bin found for board.");
        uart_dbg("BOOT: monitor.bin NOT FOUND");
    }

    int line_start;
    int menu_cnt = main_menu_config.size();
    line_start = 13 - (menu_cnt+2+2) / 2;       // 2 lines for version, 2 lines for "TangCore"

    while (1) {
        bool redraw = true;
        int choice = 0;
        for (;;) {
#ifdef TANGCORE_USB_CDC_DEBUG
            cdc_debug_poll();   // service one pending PC command per iteration
#endif
            uint32_t now = bflb_mtimer_get_time_ms();
            if (active_core == -1) {
                // send_blank_packet();
                active_core = get_core_id();            // 200ms timeout
                overlay_status("core_id=%d", active_core);
                if (active_core > 0) apply_saved_core_options(active_core);
                if (active_core >= 0) redraw = true;    // redraw immediately if core is detected
            }
            // if (core < 0) continue;         // do not draw or process input if core is not ready
            if (now - last_redraw_time > 5000) 
                redraw = true;
            if (redraw) {
                active_core = get_core_id();            // allow jtag to change core underneath us
                overlay(overlay_on());                  // set correct overlay state
                overlay_clear();

                int line = line_start;
                overlay_cursor(0, line++);
                //              01234567890123456789012345678901
                overlay_printf("       -== TangCore ==-");
                line++;

                // display all menu items
                for (int i = 0; i < menu_cnt; i++) {
                    overlay_cursor(2, line++);
                    if (main_menu_config[i] > 0) {
                        for (size_t j = 0; j < core_info_list.size(); j++) {
                            if (core_info_list[j].id == main_menu_config[i]) {
                                overlay_printf("%s", core_info_list[j].display_name);
                                break;
                            }
                        }
                    } else if (main_menu_config[i] == -1) {
                        overlay_printf("Cores");
                    } else if (main_menu_config[i] == -2) {
                        overlay_printf("Options");
                    }
                }

                line++;
                overlay_cursor(2, line++);
                overlay_printf("Version: ");
                overlay_printf(__DATE__);
                last_redraw_time = now;
                redraw = false;

                // print some debug stats to UART
                // uint16_t joy1=0, joy2=0;
                // get_joypad_states(&joy1, &joy2);
                // overlay_status("core=%d, j1=%04x, j2=%04x", active_core, joy1, joy2);
                // overlay_status("Mtimer frequency: %d MHz", bflb_mtimer_get_freq() / 1000000);
                // overlay_status("CPU frequency: %d MHz", bflb_clk_get_system_clock(BL_SYSTEM_CLOCK_MCU_CLK) / 1000000);
                // overlay_status("GPIO0-3 status: %08x %08x %08x %08x", *reg_gpio0, *reg_gpio1, *reg_gpio2, *reg_gpio3);
            }

            bool before_overlay = overlay_on();
            int r = joy_choice(line_start+2, menu_cnt, &choice, OSD_KEY_CODE);
            if (r == 1) break;

            if (!before_overlay && overlay_on() && active_core > 0) {
                // overlay is turned back on, now display the pop-up menu
                DEBUG("Displaying pop-up menu\n");
                menu_clear();
                // Menu *menu = core->create_menu(core->rom_dir)
                core_info *core = find_core_by_id(active_core);
                if (core != NULL) {
                    DEBUG("Found core_info. Displaying menu\n");
                    Menu *menu;
                    if (active_core == 6) {
                        menu = create_pcxt_menu(std::string(drv).append(core->rom_dir).c_str());
                    } else {
                        menu = create_default_menu(std::string(drv).append(core->rom_dir).c_str());
                    }
                    std::unique_ptr<Menu> menu_ptr(menu);
                    push_menu(std::move(menu_ptr));
                    menu->do_redraw();
                    menu_input_loop();
                    redraw = true;
                }
            }

            delay(20);
        }

        if (main_menu_config[choice] > 0) {
            // Load rom or core from USB drive
            struct core_info *core = NULL;
            for (size_t i = 0; i < core_info_list.size(); i++) {
                if (core_info_list[i].id == main_menu_config[choice]) {
                    core = &core_info_list[i];
                    break;
                }
            }
            if (core) {
                // PC Engine's unified entry (id 8) owns both pce/ (.pce) and
                // pcenginecd/ (.chd) -- FileChooser can't navigate above the dir it's
                // opened with, so browse from the drive root instead of rom_dir for
                // this one core, matching PC/XT's existing (active_core == 6)
                // special-case just below.
                std::string dir = (core->id == 8) ? std::string(drv)
                                                   : std::string(drv).append(core->rom_dir);
                menu_loadrom(dir.c_str());
            }
        } else if (main_menu_config[choice] == -1) {
            // load cores manually
            std::string dir = std::string(drv).append("cores");
            menu_loadrom(dir.c_str());
        } else if (main_menu_config[choice] == -2) {
            // Options
            menu_options();
        } 

        delay(300);
    }
}

static void print_system_info(void) {
    // this is viewable with scripts/liveuart.py
    overlay_status("TangCore %s", __DATE__);
    overlay_status("TangBoard: %s", BOARD_NAME);
    overlay_status("System clock: %u MHz", bflb_clk_get_system_clock(BL_SYSTEM_CLOCK_MCU_CLK) / 1000000);
    // UART registers
    // Clock comes from XCLK/160M/BCLK and goes through a divider and becomes UART_CLK
    overlay_status("GLB_UART_CFG0: %08x", BL_RD_WORD(0x20000150));
    overlay_status("GLB_UART_CFG1: %08x", BL_RD_WORD(0x20000154));
    overlay_status("GLB_UART_CFG2: %08x", BL_RD_WORD(0x20000158));
    overlay_status("UART0 clock: %u", Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_UART0));
    overlay_status("UART1 clock: %u", Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_UART1));

    // 10.3.5: baudrate = UART_clk / (uart_prd + 1)
    // This causes memory exception.
    // overlay_status("UART_BIT_PRD: %08x", BL_RD_WORD(0x40010008));
    //              01234567890123456789012345678901
    // overlay_status("                                ");
}

// Initialize things, then start main_task and uart1_rx_task to do actual work

int main(void)
{
    /* Board init */
    board_init();
    init_core_list();

    // Initialize GPIO and UART
    init_gpio_and_uart();
    uart_dbg("BOOT: init_gpio_and_uart done, console alive");

    // CD sector transmit moves to DMA so the CPU is free during the 11.76 ms of wire
    // time per sector -- that is where the ~62 ms libchdr hunk decode has to hide if
    // CD-DA is to keep up. See core/pcecd.cpp's DMA SECTOR TRANSMIT comment.
    pcecd_tx_dma_init();

    print_system_info();

    // Create mutex for joypad states
    state_mutex = xSemaphoreCreateMutex();

    overlay_status("Initializing SDH...");
    fatfs_sdh_driver_register();        // calls SDH_Init()
    // f_mount(&fs_sd, "sd:", 0);          // registers SDMMC drive 

#ifdef TANGCORE_USB_CDC_DEBUG
    // USB DEVICE mode instead of host: brings up a CDC-ACM link to a PC on the same
    // connector. No gamepads and no USB mass-storage drive in this build -- see
    // usb/cdc_debug.c. This is also the boot step at which the board is observed to hang
    // when cabled to a PC in a normal (host) build, which is the leading explanation for
    // that hang: two hosts on one cable.
    overlay_status("Initializing USB CDC debug...");
    cdc_debug_init();
#else
    // Initializing USB host...
    overlay_status("Initializing USB host...");
    usbh_initialize();
    fatfs_usbh_driver_register();
    usb_gamepad_init();
#endif

    overlay_status("Creating tasks...");
    // Create the tasks
    // Logger first: file_log() falls back to a direct (blocking) write while
    // log_queue is NULL, so create it before anything that logs.
    log_queue = xQueueCreate(LOG_QUEUE_LEN, LOG_LINE_MAX);
    xTaskCreate(log_task, "log_task", LOG_TASK_STACK_SIZE, NULL, LOG_TASK_PRIORITY, NULL);
    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &main_task_handle);
    // Arm interrupt-driven RX before the parser task runs, so no byte arrives while the
    // FIFO is still unattended. See uart1_isr's comment for why polling was not enough.
    uart1_rx_irq_init();
    xTaskCreate(uart1_rx_task, "uart1_rx_task", UART1_RX_TASK_STACK_SIZE, NULL, UART1_RX_TASK_PRIORITY, &uart1_rx_task_handle);
    cd_req_queue = xQueueCreate(16, sizeof(cd_req_t));
    neosave_init();                     // Neo Geo backup RAM <-> SD card, see core/neosave.cpp
    pcesave_init();                     // backup RAM <-> SD card, see core/pcesave.cpp
    xTaskCreate(cd_serve_task, "cd_serve_task", CD_SERVE_TASK_STACK_SIZE, NULL, CD_SERVE_TASK_PRIORITY, &cd_serve_task_handle);
    wifi_debug_start();     // real no-op unless built with WIFI_DEBUG=1

    vTaskStartScheduler();

    while (1) {
    }
}
