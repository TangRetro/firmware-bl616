/*
 * Modifications copyright (c) 2026 Romain Tisserand.
 * Originally from nand2mario/firmware-bl616 (Apache-2.0); this file has been
 * modified in the pcetang fork.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tc_utils.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// UART1 OWNERSHIP (2026-09-17).
//
// UART1 is the FPGA link, and more than one task writes frames to it: sector data, TOC,
// joypad/HID, overlay text, floppy, keyboard. A frame is only valid if its bytes go out
// contiguously -- iosys_bl616.v counts `len` bytes after the 0xAA header, so any foreign
// byte landing inside a frame shifts every byte after it and the frame never completes.
//
// That exclusion used to come from taskENTER_CRITICAL() around each blocking write. It
// worked, but it was a side effect: with interrupts off for a whole 1026-byte data chunk
// (~5 ms at 2 Mbaud) the RX side cannot be serviced either, and a 32-byte RX FIFO at
// 2 Mbaud overflows in ~1.6 ms. And it cannot express a DMA transfer at all, which runs
// for 11.8 ms with interrupts ENABLED -- which is exactly why DMA hung: once the critical
// section was gone, other writers could land inside a sector frame.
//
// So exclusion is now explicit: a binary semaphore is the UART1 token. Blocking writers
// take it and give it back. A DMA transfer takes it in task context and the DMA
// COMPLETION ISR gives it back (a binary semaphore, not a mutex, precisely so an ISR may
// release it). The task that started the DMA is free to go and decode the next hunk while
// the token is held by the transfer.
//
// Takes time out instead of blocking forever: a DMA completion that never arrives would
// otherwise silence every writer on the board, the menu included. A timeout is counted in
// fpga_tx_lock_timeouts and the writer proceeds -- degraded and visible, never wedged.
static SemaphoreHandle_t fpga_tx_sem = NULL;
volatile uint32_t fpga_tx_lock_timeouts = 0;
static volatile uint8_t fpga_tx_owned_by_isr = 0;

void fpga_tx_lock_init(void)
{
    if (fpga_tx_sem == NULL) {
        fpga_tx_sem = xSemaphoreCreateBinary();
        if (fpga_tx_sem) xSemaphoreGive(fpga_tx_sem);
    }
}

// Before the scheduler starts there is one thread of execution, so no lock is needed --
// and blocking is not allowed. Same for a missing semaphore.
static bool fpga_tx_lock_active(void)
{
    return fpga_tx_sem != NULL && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

bool fpga_tx_lock_timed(uint32_t ms)
{
    if (!fpga_tx_lock_active()) return true;
    if (xSemaphoreTake(fpga_tx_sem, pdMS_TO_TICKS(ms)) == pdTRUE) return true;
    fpga_tx_lock_timeouts++;
    return false;
}

void fpga_tx_lock(void)   { (void)fpga_tx_lock_timed(500); }

void fpga_tx_unlock(void)
{
    if (!fpga_tx_lock_active()) return;
    xSemaphoreGive(fpga_tx_sem);    // harmless no-op if already available
}

// Called only from an ISR (the DMA transfer-complete callback).
void fpga_tx_unlock_from_isr(void)
{
    if (fpga_tx_sem == NULL) return;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(fpga_tx_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

void fpga_tx_header(int cmd, int len) {
    bflb_uart_putchar(uart1_dev, 0xAA);
    bflb_uart_putchar(uart1_dev, len >> 8);
    bflb_uart_putchar(uart1_dev, len & 0xFF);
    bflb_uart_putchar(uart1_dev, cmd);
}

void fpga_tx_byte(uint8_t b) {
    bflb_uart_putchar(uart1_dev, b);
}

uint32_t get_file_size(const char *fname) {
    FILINFO fno;
    FRESULT r = f_stat(fname, &fno);
    if (r != FR_OK) return 0;
    return fno.fsize;
}

// Send a romdata packet to core of len bytes in `fbuf`
void send_fbuf_data(uint16_t len) {
    taskENTER_CRITICAL();
    fpga_tx_header(0x07, len+1);
    for (int i = 0; i < len; i ++) {
        fpga_tx_byte(fbuf[i]);
    }
    taskEXIT_CRITICAL();
}

// set loading state
void set_loading_state(int state) {
    taskENTER_CRITICAL();
    fpga_tx_header(0x06, 2);
    fpga_tx_byte(state);        
    taskEXIT_CRITICAL();
}


// bring FPGA to a good state by sending a few 0's
void send_blank_packet(void) {
    taskENTER_CRITICAL();
    for (int i = 0; i < 8; i++) {
        fpga_tx_byte(0);
    }
    taskEXIT_CRITICAL();
}

#include <string>
#include <algorithm>

const char *cstr_find_ignore_case(const char *str, const char *substr) {
    std::string s(str);
    std::string ss(substr);
    auto it = std::search(s.begin(), s.end(), ss.begin(), ss.end(), [](char a, char b) {
        return std::tolower(a) == std::tolower(b);
    });
    if (it == s.end()) return NULL;
    return str + (it - s.begin());
}

static uint32_t core_config;
// "Auto" choices resolved by the firmware (e.g. NES region from the ROM header): when the
// field under auto_mask is 0 in the word the options menu keeps, auto_bits go to the core
// instead. The menu and the .opt file still hold 0, so "Auto" stays Auto.
static uint32_t auto_mask = 0, auto_bits = 0;

uint32_t get_core_config(void) {
    return core_config;
}

void set_core_config_auto(uint32_t mask, uint32_t bits) {
    auto_mask = mask;
    auto_bits = bits & mask;
}

void set_core_config(uint32_t config) {
    core_config = config;
    if (auto_mask && (config & auto_mask) == 0) config |= auto_bits;
    // The options menu runs in-game, so this can meet a CD sector frame in flight: take
    // the TX token like the HID sender does, so the two frames cannot interleave.
    fpga_tx_lock();
    taskENTER_CRITICAL();
    fpga_tx_header(0x03, 5);
    fpga_tx_byte(config >> 24);
    fpga_tx_byte(config >> 16);
    fpga_tx_byte(config >> 8);
    fpga_tx_byte(config);
    taskEXIT_CRITICAL();
    fpga_tx_unlock();
}

/////////////////////////////////////////////////////////////////////////////////
// Shared state among tasks
volatile uint16_t joy1_state = 0;
volatile uint16_t joy2_state = 0;
volatile uint16_t hid1_state = 0;
volatile uint16_t hid2_state = 0;
volatile int16_t core_id = -1;
// Core config string (MiSTer-style CONF_STR, iosys command 0x02), published by
// uart1_rx_task. core_conf_len is -1 until a complete string has arrived.
char core_conf_str[CORE_CONF_MAX + 1];
volatile int16_t core_conf_len = -1;
volatile uint8_t key_buf[4] = {0};
SemaphoreHandle_t state_mutex;              // for all global state access

// read joypad states
void get_joypad_states(uint16_t *joy1, uint16_t *joy2, uint16_t *hid1, uint16_t *hid2)
{
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        *joy1 = joy1_state;
        *joy2 = joy2_state;
        *hid1 = hid1_state;
        *hid2 = hid2_state;
        xSemaphoreGive(state_mutex);
    }
}

// Ask the core for its config string. Returns false on timeout (300 ms), e.g. a core
// whose iosys predates command 0x02.
bool get_core_conf_string(std::string &out) {
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        core_conf_len = -1;
        xSemaphoreGive(state_mutex);
    }
    fpga_tx_lock();
    taskENTER_CRITICAL();
    fpga_tx_header(0x02, 1);
    taskEXIT_CRITICAL();
    fpga_tx_unlock();
    uint64_t start = bflb_mtimer_get_time_ms();
    while (bflb_mtimer_get_time_ms() - start < 300) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            int16_t n = core_conf_len;
            if (n >= 0) out.assign(core_conf_str, n);
            xSemaphoreGive(state_mutex);
            if (n >= 0) return true;
        }
        delay(10);
    }
    return false;
}

// query over UART to return if the correct core is loaded
// return >= 0 if request is successful, -1 if timeout (200ms)
int16_t get_core_id(void) {
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        core_id = -1;
        xSemaphoreGive(state_mutex);
    }

    // send command 1
    fpga_tx_header(0x01, 1);

    // TODO: use a queue for better performance
    uint64_t start = bflb_mtimer_get_time_ms();
    while (bflb_mtimer_get_time_ms() - start < 200) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            int16_t res = core_id;
            if (res >= 0) {
                xSemaphoreGive(state_mutex);
                return res;
            }
            xSemaphoreGive(state_mutex);
        }
        delay(10);
    }
    return -1;
}