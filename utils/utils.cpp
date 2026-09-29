#include "utils.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// UART1 is the FPGA link and several tasks write frames to it. A frame is only valid if
// its bytes go out back to back (iosys counts `len` bytes after the 0xAA header), so
// writers hold this token for a whole frame. It replaces taskENTER_CRITICAL(): with
// interrupts off for a 1 KB ROM chunk (~5 ms at 2 Mbaud) the 32-byte RX FIFO overflows,
// and a DMA transfer cannot sit inside a critical section at all. A DMA sender takes the
// token and its completion interrupt gives it back, hence a binary semaphore, not a
// mutex. Waits time out (500 ms) so a lost completion cannot silence every writer.
static SemaphoreHandle_t fpga_tx_sem = NULL;
volatile uint32_t fpga_tx_lock_timeouts = 0;

void fpga_tx_lock_init(void)
{
    if (fpga_tx_sem == NULL) {
        fpga_tx_sem = xSemaphoreCreateBinary();
        if (fpga_tx_sem) xSemaphoreGive(fpga_tx_sem);
    }
}

// Before the scheduler runs there is one thread and blocking is not allowed.
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
    xSemaphoreGive(fpga_tx_sem);
}

// DMA transfer-complete callback only.
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
    fpga_tx_lock();
    fpga_tx_header(0x07, len+1);
    for (int i = 0; i < len; i ++) {
        fpga_tx_byte(fbuf[i]);
    }
    fpga_tx_unlock();
}

// set loading state
void set_loading_state(int state) {
    fpga_tx_lock();
    fpga_tx_header(0x06, 2);
    fpga_tx_byte(state);        
    fpga_tx_unlock();
}


// bring FPGA to a good state by sending a few 0's
void send_blank_packet(void) {
    fpga_tx_lock();
    for (int i = 0; i < 8; i++) {
        fpga_tx_byte(0);
    }
    fpga_tx_unlock();
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

uint32_t get_core_config(void) {
    return core_config;
}

void set_core_config(uint32_t config) {
    core_config = config;
    fpga_tx_lock();
    fpga_tx_header(0x03, 5);
    fpga_tx_byte(config >> 24);
    fpga_tx_byte(config >> 16);
    fpga_tx_byte(config >> 8);
    fpga_tx_byte(config);
    fpga_tx_unlock();
}

/////////////////////////////////////////////////////////////////////////////////
// Shared state among tasks
volatile uint16_t joy1_state = 0;
volatile uint16_t joy2_state = 0;
volatile uint16_t hid1_state = 0;
volatile uint16_t hid2_state = 0;
volatile int16_t core_id = -1;
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

// query over UART to return if the correct core is loaded
// return >= 0 if request is successful, -1 if timeout (200ms)
int16_t get_core_id(void) {
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        core_id = -1;
        xSemaphoreGive(state_mutex);
    }

    // send command 1
    fpga_tx_lock();
    fpga_tx_header(0x01, 1);
    fpga_tx_unlock();

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