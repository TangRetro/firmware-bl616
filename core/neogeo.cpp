#define _GNU_SOURCE
#include <string.h>
#include <stdio.h>
#include <string>

#include "tc_utils.h"
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
//
// The .neo file holds ONLY the game ROMs. The MVS system BIOS is separate and is
// loaded first (M1.6): SPROM (sp-u2.sp1), LO (000-lo.lo), SFIX (sfix.sfix),
// SM1 (sm1.sm1) -- each a 128KB file in the SD "neogeo/" directory.

// Region IDs for FPGA dispatch (must match neotang_console60k.sv map_region_to_index)
enum neo_region {
    // BIOS (system) regions
    NEO_REG_SPROM  = 0,   // -> INDEX_SPROM   (SDRAM 0x0000000)
    NEO_REG_LO     = 1,   // -> INDEX_LOROM   (on-chip dpram)
    NEO_REG_SFIX   = 2,   // -> INDEX_SFIXROM (SDRAM 0x0020000)
    // Game regions
    NEO_REG_P1     = 4,   // -> INDEX_P1ROM_A (SDRAM 0x0200000)
    NEO_REG_P2     = 6,   // -> INDEX_P2ROM   (SDRAM 0x0300000)
    NEO_REG_S      = 8,   // -> INDEX_S1ROM   (cart S ROM = fix layer, SDRAM 0x0080000)
    NEO_REG_M1     = 9,   // -> INDEX_M1ROM   (game sound, MROM)
    NEO_REG_C      = 15,  // -> INDEX_CROM0   (sprites)
    NEO_REG_V1     = 16,  // -> INDEX_VROMS   (ADPCM)
    NEO_REG_V2     = 32,  // V1+16 in VROMS bank
    // System sound -- shares the MROM path with the game M1 (region 9). Loaded
    // FIRST so the game M1 overwrites it (the Z80 runs the game's M1 in play).
    NEO_REG_SM1    = 20,  // -> INDEX_M1ROM
};

#define NEO_HEADER_SIZE 0x1000
#define NEO_DATA_OFFSET 0x1000

static FIL fbios;   // second FatFs handle for the BIOS files

// Send a region header (0x08: region_id + u32le size) then stream `size` bytes
// from `file` (file pointer positioned at the region start) via the 0x07 stream.
static int neo_stream_region(FIL *file, uint8_t region_id, uint32_t size, const char *name) {
    unsigned int br;

    overlay_status("Loading %s (%dK)...", name, size >> 10);

    taskENTER_CRITICAL();
    fpga_tx_header(0x11, 5);                 // Neo Geo region header (see iosys_bl616.v)
    fpga_tx_byte(region_id);
    fpga_tx_byte(size & 0xFF);
    fpga_tx_byte((size >> 8) & 0xFF);
    fpga_tx_byte((size >> 16) & 0xFF);
    fpga_tx_byte((size >> 24) & 0xFF);
    taskEXIT_CRITICAL();

    uint32_t remaining = size;
    while (remaining > 0) {
        uint16_t chunk = remaining > 1024 ? 1024 : (uint16_t)remaining;
        if (f_read(file, fbuf, chunk, &br) != FR_OK) return 1;
        if (br == 0) break;
        send_fbuf_data(br);
        remaining -= br;
    }
    return 0;
}

// Load one BIOS file from the SD card as a region. required=1 -> fatal if absent.
static int neo_load_bios(const char *subpath, uint8_t region_id, const char *name, int required) {
    std::string path(drv);
    path.append(subpath);

    FILINFO fno;
    if (f_stat(path.c_str(), &fno) != FR_OK) {
        if (required) { overlay_status("Missing BIOS: %s", name); return 1; }
        DEBUG("optional BIOS not found: %s", name);
        return 0;
    }
    if (f_open(&fbios, path.c_str(), FA_READ) != FR_OK) {
        overlay_status("Cannot open %s", name);
        return 1;
    }
    int r = neo_stream_region(&fbios, region_id, fno.fsize, name);
    f_close(&fbios);
    return r;
}

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

    // All locals declared up front so the gotos below never cross an initialization.
    unsigned int br = 0;
    uint8_t hdr[256];
    uint32_t p_size = 0, s_size = 0, m_size = 0, v1_size = 0, v2_size = 0, c_size = 0;
    uint32_t total_bytes = 0;
    char game_name[34];

    struct neo_region_desc {
        uint32_t size;
        uint8_t region_id;
        const char *name;
    } regions[7];

    // Read and parse header
    if ((r = f_lseek(&fcore, 0)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadneogeo_close;
    }
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
    p_size  = (hdr[4]  | (hdr[5]  << 8) | (hdr[6]  << 16) | (hdr[7]  << 24));
    s_size  = (hdr[8]  | (hdr[9]  << 8) | (hdr[10] << 16) | (hdr[11] << 24));
    m_size  = (hdr[12] | (hdr[13] << 8) | (hdr[14] << 16) | (hdr[15] << 24));
    v1_size = (hdr[16] | (hdr[17] << 8) | (hdr[18] << 16) | (hdr[19] << 24));
    v2_size = (hdr[20] | (hdr[21] << 8) | (hdr[22] << 16) | (hdr[23] << 24));
    c_size  = (hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24));

    // Extract name (for display)
    memcpy(game_name, hdr + 44, 33);
    game_name[33] = '\0';
    overlay_status("Loading: %s", game_name);
    delay(100);

    // Game regions (streamed from the .neo file, after the BIOS)
    regions[0] = { p_size,  NEO_REG_P1, "P"  };
    regions[1] = { s_size,  NEO_REG_S,  "S"  };
    regions[2] = { m_size,  NEO_REG_M1, "M1" };
    regions[3] = { v1_size, NEO_REG_V1, "V1" };
    regions[4] = { v2_size, NEO_REG_V2, "V2" };
    regions[5] = { c_size,  NEO_REG_C,  "C"  };
    regions[6] = { 0, 0, NULL };

    // Enable loading on FPGA
    set_loading_state(1);
    core_running = false;

    // ---- M1.6: load the MVS system BIOS first (separate files on SD card) ----
    if (neo_load_bios("neogeo/sp-u2.sp1",  NEO_REG_SPROM, "SPROM (BIOS)", 1)) goto loadneogeo_end;
    if (neo_load_bios("neogeo/000-lo.lo",  NEO_REG_LO,    "LO",           1)) goto loadneogeo_end;
    if (neo_load_bios("neogeo/sfix.sfix",  NEO_REG_SFIX,  "SFIX",         1)) goto loadneogeo_end;
    if (neo_load_bios("neogeo/sm1.sm1",    NEO_REG_SM1,   "SM1",          0)) goto loadneogeo_end;

    // ---- game regions from the .neo file ----
    // Seek to data start (0x1000)
    if ((r = f_lseek(&fcore, NEO_DATA_OFFSET)) != FR_OK) {
        overlay_status("Seek to data failed");
        goto loadneogeo_end;
    }
    for (int i = 0; regions[i].size > 0; i++) {
        if (neo_stream_region(&fcore, regions[i].region_id, regions[i].size, regions[i].name)) {
            overlay_status("Read failure in %s", regions[i].name);
            goto loadneogeo_end;
        }
        total_bytes += regions[i].size;
    }

    overlay_status("Success! %dK loaded", total_bytes >> 10);
    core_running = true;
    delay(200);
    overlay(0);  // turn off OSD

loadneogeo_end:
    set_loading_state(0);  // turn off loading, this starts core
loadneogeo_close:
    f_close(&fcore);
    return r;
}
