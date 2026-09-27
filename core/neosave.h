// Neo Geo (cart and CD) backup RAM <-> SD card. See neosave.cpp.
#pragma once
#include <stdint.h>
#include <stdbool.h>

void neosave_init(void);                        // create the save task; call once at boot
// The game being loaded: names the save file and picks the RAM the core has.
// cd = the Neo Geo CD bitstream (memory card only); otherwise the cart one (SRAM + card).
void neosave_set_game(const char *fname, bool cd);
void neosave_restore(void);                     // send the save into the FPGA; core must not run yet
void neosave_flush_now(void);                   // save now if the game wrote since the last save
bool neosave_active(void);                      // a Neo Geo game owns the save channel
void neosave_off(void);                         // another core's game took the channel

// Called from uart1_rx_task only (via pcesave's hooks while neosave_active()).
void neosave_rx_byte(uint16_t blk, uint16_t off, uint8_t b);
void neosave_rx_block_done(uint16_t blk);
void neosave_rx_dirty(void);
