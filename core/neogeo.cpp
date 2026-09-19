#define _GNU_SOURCE
#include <string.h>
#include <stdio.h>

#include "utils.h"
#include "cores.h"
#include "overlay.h"

// Neo Geo .neo (TerraOnion) file format:
// 0x000  'N' 'E' 'O' 0x01
// 0x004  u32le PSize, SSize, MSize, V1Size, V2Size, CSize
// 0x01c  u32le Year, Genre, Screenshot, NGH
// 0x02c  char  Name[33]
// 0x04d  char  Manu[17]
// ...    zero-fill to 0x1000
// 0x1000 P, S, M, V1, V2, C (concatenated)

// Region IDs for FPGA dispatch (must match neotang_console60k.sv INDEX_*)
enum neo_region {
    NEO_REG_P1     = 4,   // INDEX_P1ROM_A
    NEO_REG_P2     = 6,   // INDEX_P2ROM
    NEO_REG_S      = 8,   // INDEX_S1ROM
    NEO_REG_M1     = 9,   // INDEX_M1ROM
    NEO_REG_C      = 15,  // INDEX_CROM0
    NEO_REG_V1     = 16,  // INDEX_VROMS
    // Note: region C-ROMs are contiguous at CROM_START;
    //       V1/V2 are in the VROMS bank.
};

#define NEO_HEADER_SIZE 0x1000
#define NEO_DATA_OFFSET 0x1000

// return 0 if successful
int loadneogeo(const char *fname) {
    int r = 1;
    DEBUG("loadneogeo start");

    // check extension .neo
    char *p = strcasestr(fname, ".neo");
    if (p == NULL) {
        overlay_message("Only .neo supported", 1);
        return r;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }

    unsigned int br;
    int file_size = get_file_size(fname);

    // Read and parse header
    if ((r = f_lseek(&fcore, 0)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadneogeo_close;
    }

    uint8_t hdr[256];
    if ((r = f_read(&fcore, hdr, 256, &br)) != FR_OK || br != 256) {
        overlay_status("Header read failure");
        goto loadneogeo_close;
    }

    // Check magic
    if (hdr[0] != 'N' || hdr[1] != 'E' || hdr[2] != 'O' || hdr[3] != 0x01) {
        overlay_status("Not a .neo file");
        goto loadneogeo_close;
    }

    // Extract region sizes (little-endian u32le at offsets 0x04-0x18)
    uint32_t p_size = (hdr[4] | (hdr[5] << 8) | (hdr[6] << 16) | (hdr[7] << 24));
    uint32_t s_size = (hdr[8] | (hdr[9] << 8) | (hdr[10] << 16) | (hdr[11] << 24));
    uint32_t m_size = (hdr[12] | (hdr[13] << 8) | (hdr[14] << 16) | (hdr[15] << 24));
    uint32_t v1_size = (hdr[16] | (hdr[17] << 8) | (hdr[18] << 16) | (hdr[19] << 24));
    uint32_t v2_size = (hdr[20] | (hdr[21] << 8) | (hdr[22] << 16) | (hdr[23] << 24));
    uint32_t c_size = (hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24));

    // Extract name (for display)
    char game_name[34];
    memcpy(game_name, hdr + 44, 33);
    game_name[33] = '\0';
    overlay_status("Loading: %s", game_name);
    delay(100);

    // Enable game loading on FPGA
    set_loading_state(1);
    core_running = false;

    // Seek to data start (0x1000)
    if ((r = f_lseek(&fcore, NEO_DATA_OFFSET)) != FR_OK) {
        overlay_status("Seek to data failed");
        goto loadneogeo_end;
    }

    // Stream regions in order: P, S, M, V1, V2, C
    // Each region:
    //   1. Send region ID + size to FPGA
    //   2. Stream region bytes
    struct {
        uint32_t size;
        uint8_t region_id;
        const char *name;
    } regions[] = {
        { p_size,  NEO_REG_P1, "P" },
        { s_size,  NEO_REG_S,  "S" },
        { m_size,  NEO_REG_M1, "M" },
        { v1_size, NEO_REG_V1, "V1" },
        { v2_size, NEO_REG_V1 + 16, "V2" },  // V2 is V1+16 in VROMS bank
        { c_size,  NEO_REG_C,  "C" },
        { 0, 0, NULL }
    };

    uint32_t total_bytes = 0;
    for (int i = 0; regions[i].size > 0; i++) {
        uint32_t region_size = regions[i].size;
        uint8_t region_id = regions[i].region_id;
        const char *region_name = regions[i].name;

        overlay_status("Loading %s (%dK)...", region_name, region_size >> 10);

        // Send region marker: 0x08 (custom command for region header)
        // Format: region_id (1 byte) + size (4 bytes little-endian)
        taskENTER_CRITICAL();
        fpga_tx_header(0x08, 5);  // custom region header
        fpga_tx_byte(region_id);
        fpga_tx_byte(region_size & 0xFF);
        fpga_tx_byte((region_size >> 8) & 0xFF);
        fpga_tx_byte((region_size >> 16) & 0xFF);
        fpga_tx_byte((region_size >> 24) & 0xFF);
        taskEXIT_CRITICAL();

        // Stream region data in chunks
        uint32_t remaining = region_size;
        while (remaining > 0) {
            uint16_t chunk_size = remaining > 1024 ? 1024 : remaining;
            if ((r = f_read(&fcore, fbuf, chunk_size, &br)) != FR_OK) {
                overlay_status("Read failure at %s", region_name);
                goto loadneogeo_end;
            }
            if (br == 0) break;

            // Send data via normal 0x07 command (rom_do stream)
            send_fbuf_data(br);
            total_bytes += br;
            remaining -= br;

            if ((total_bytes & 0x3FFFF) == 0) {  // Progress every 256KB
                overlay_status("Loaded %dK/%dK", total_bytes >> 10, 
                             (p_size + s_size + m_size + v1_size + v2_size + c_size) >> 10);
            }
        }
    }

    overlay_status("Success! %dK loaded", total_bytes >> 10);
    core_running = true;
    delay(200);
    overlay(0);  // turn off OSD

loadneogeo_end:
    set_loading_state(0);  // turn off game loading, this starts core
loadneogeo_close:
    f_close(&fcore);
    return r;
}

// Register Neo Geo core
const core_t neogeo_core = {
    .name = "Neo Geo",
    .load_rom = loadneogeo,
};
