#define _GNU_SOURCE
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
    NEO_REG_V2     = 48,  // -> INDEX_VROMS+32 (ADPCMB half; bit 24 of VROM_LOAD_ADDR)
    // System sound -- shares the MROM path with the game M1 (region 9). Loaded
    // FIRST so the game M1 overwrites it (the Z80 runs the game's M1 in play).
    NEO_REG_SM1    = 20,  // -> INDEX_M1ROM
};

#define NEO_HEADER_SIZE 0x1000
#define NEO_DATA_OFFSET 0x1000

static FIL fbios;   // second FatFs handle for the BIOS files

// Send a region header (UART cmd 0x11: region_id + u32le size) then stream `size` bytes
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

// ---------------------------------------------------------------------------
// Cart hardware configuration (the FPGA's `cfg` word)
//
// Neo Geo carts carry protection/bank silicon that the core has to reproduce: NEO-SMA and
// NEO-PVC bank chips, PRO-CT0, the link-play MCU, cart RAM, and the fix-layer
// bankswitching the CMC chips do. MiSTer gets the per-game settings from its MRA file; we
// have to send them ourselves, as three words on region 10 (INDEX_MEMCP).
//
// The `.neo` file does NOT carry this information -- its header is just sizes, year,
// genre, screenshot, NGH, name and manufacturer, then zero fill. NGH (the SNK game number)
// is the only identifier, and it is not sufficient on its own:
//   * NGH collides between unrelated titles (0x066 is both Digger Man (prototype) and
//     Karnov's Revenge),
//   * revisions and bootlegs of the SAME title share an NGH but need different settings
//     (garou vs garouh, mslug4 vs ms4plus, mslug5 vs ms5plus, svc and kof2003 bootlegs),
//   * homebrew fills the field freely -- plenty ship 0x0000 or a copied template value.
// So every entry below VERIFIES before it acts, exactly as geolith does: an NGH match plus
// a P-size window and/or a byte probe at a known file offset. Anything unrecognised gets
// cfg = 0, which is the correct answer for a plain cart -- and that is what homebrew and
// the current wave of new commercial releases are, since nobody is fabricating SMA/PVC
// silicon any more. A wrong guess here is much worse than no guess: enabling SMA on an
// innocent homebrew that happens to use NGH 0x253 would break a game that otherwise runs.
//
// The rules and probe offsets come from geolith's geo_neo.c (BSD-3, Rupert Carmichael),
// which is the maintained reference for exactly this problem. Kept keyed on NGH + probe
// rather than on filename, since filenames vary by ROM set.
//
// `neogeo/boards.cfg` on the SD card overrides the built-in table, so a new release, a
// bootleg or a fix can be handled without reflashing the firmware. Format, one per line:
//   # comment
//   0253 00500000      <- NGH (4 hex) then cfg (8 hex)
//   garou.neo 00500000 <- or a file name, which wins over an NGH match
// ---------------------------------------------------------------------------

#define CFG_MS5P_BANK    (1u << 17)
#define CFG_XRAM         (1u << 18)
#define CFG_ADPCMA_EXT   (1u << 19)
#define CFG_PCHIP(n)     ((uint32_t)(n) << 20)   // 2=PVC, 3=kof99, 4=garou, 5=garouh,
                                                 // 6=mslug3, 7=kof2000 (neo_sma TYPE)
#define CFG_USE_PCM      (1u << 23)
#define CFG_CART_CHIP(n) ((uint32_t)(n) << 24)   // 1=PRO-CT0, 2=link-play MCU
#define CFG_CMC(n)       ((uint32_t)(n) << 26)   // fix bankswitch: 1=per line, 2=per tile

#define NEO_P_BASE 0x1000                        // P ROM starts here in the file

// Read one byte at an absolute offset in the .neo. Returns 0xFF00 on failure (no valid
// probe value, so a failed read never matches an entry).
static int neo_probe(FIL *f, uint32_t offset) {
    uint8_t b; unsigned int br;
    if (f_lseek(f, offset) != FR_OK) return 0xFF00;
    if (f_read(f, &b, 1, &br) != FR_OK || br != 1) return 0xFF00;
    return b;
}

// Built-in table, straight from geolith's geo_neo.c per-NGH handling.
static uint32_t neo_builtin_cfg(FIL *f, uint32_t ngh, uint32_t psz, const char **why) {
    uint32_t cfg = 0;
    *why = NULL;

    switch (ngh) {
        // ---- link-play MCU: Riding Hero, League Bowling, Thrash Rally ----
        case 0x006: case 0x019: case 0x038:
            cfg = CFG_CART_CHIP(2); *why = "link MCU"; break;

        // ---- cart RAM: Jockey Grand Prix, V-Liner ----
        case 0x008: case 0x3e7: case 0x999:
            cfg = CFG_XRAM; *why = "cart RAM"; break;

        // ---- PRO-CT0: Fatal Fury 2, Super Sidekicks ----
        case 0x047: case 0x052:
            cfg = CFG_CART_CHIP(1); *why = "PRO-CT0"; break;

        // ---- NEO-SMA: KOF 99 (only the encrypted P set is SMA) ----
        case 0x151: case 0x251:
            if (psz > 0x500000) { cfg = CFG_PCHIP(3); *why = "SMA (kof99)"; }
            break;

        // ---- Garou: two SMA variants, told apart by a byte in the P ROM ----
        case 0x253:
            if (psz > 0x500000) {
                int v = neo_probe(f, 0xc1000 + 0x3e481);
                if (v == 0x9f)      { cfg = CFG_PCHIP(5) | CFG_CMC(1); *why = "SMA (garouh) + fix bank"; }
                else if (v == 0x41) { cfg = CFG_PCHIP(4) | CFG_CMC(1); *why = "SMA (garou) + fix bank"; }
                // bootleg/prototype sets use neither
            }
            break;

        // ---- Metal Slug 3: fix bankswitching always, SMA only on the encrypted set ----
        case 0x256:
            cfg = CFG_CMC(1); *why = "fix bank";
            if (psz > 0x500000) {
                // geolith splits mslug3 / mslug3a here; the core's neo_sma has one
                // Metal Slug 3 table (TYPE 6), so mslug3a is not supported.
                cfg |= CFG_PCHIP(6); *why = "SMA (mslug3) + fix bank";
            }
            break;

        // ---- KOF 2000: per-tile fix bankswitching, SMA on the encrypted set ----
        case 0x257:
            cfg = CFG_CMC(2); *why = "fix bank";
            if (psz > 0x500000) { cfg |= CFG_PCHIP(7); *why = "SMA (kof2000) + fix bank"; }
            break;

        // ---- Metal Slug 4: mslug4/mslug4h bank the fix layer, ms4plus does not ----
        case 0x263:
            if (neo_probe(f, NEO_P_BASE + 0x809) != 0x0c) { cfg = CFG_CMC(1); *why = "fix bank"; }
            break;

        // ---- Matrimelee ----
        // (geolith also rebuilds matrimbl's fix layer from the C ROM in software, which we
        //  cannot do on the FPGA -- that bootleg will have a corrupt fix layer.)
        case 0x266:
            cfg = CFG_CMC(2); *why = "fix bank"; break;

        // ---- Metal Slug 5: official = NEO-PVC, Plus = its own bank scheme ----
        case 0x268:
            if (neo_probe(f, NEO_P_BASE + 0x26b) == 0xb9)      { cfg = CFG_MS5P_BANK; *why = "MS5 Plus bank"; }
            else if (neo_probe(f, NEO_P_BASE + 0x267) == 0x4f) { cfg = CFG_PCHIP(2);  *why = "PVC"; }
            break;

        // ---- SVC Chaos: PVC and fix banking are probed independently ----
        case 0x269:
            if (neo_probe(f, NEO_P_BASE + 0x3d25) == 0xc4) { cfg |= CFG_CMC(2); }
            if (neo_probe(f, NEO_P_BASE + 0x2f8f) == 0xc0) { cfg |= CFG_PCHIP(2); }
            if (cfg) *why = "PVC / fix bank";
            break;

        // ---- KOF 2003: official uses PVC + per-tile fix banking; bootlegs need boards
        //      the core does not implement ----
        case 0x271:
            if (neo_probe(f, NEO_P_BASE + 0x689) == 0x10)    *why = "UNSUPPORTED (kf2k3bla)";
            else if (neo_probe(f, NEO_P_BASE + 0xc1) == 0x02) *why = "UNSUPPORTED (kf2k3bl)";
            else { cfg = CFG_PCHIP(2) | CFG_CMC(2); *why = "PVC + fix bank"; }
            break;

        // ---- protections with no equivalent in the core ----
        case 0x242: *why = "UNSUPPORTED (KOF98 protection)";   break;
        case 0x250: *why = "UNSUPPORTED (Metal Slug X prot)";  break;
        case 0x275: *why = "UNSUPPORTED (KOF 10th Anniv.)";    break;
        case 0x5003:*why = "UNSUPPORTED (CTHD2003 bootleg)";   break;

        default: break;   // plain cart (and every homebrew / new release): cfg = 0
    }
    return cfg;
}

// neogeo/boards.cfg override. Returns true (and sets *cfg) when a line matches.
static bool neo_cfg_override(const char *fname, uint32_t ngh, uint32_t *cfg) {
    std::string path(drv);
    path.append("neogeo/boards.cfg");

    FIL fo;
    if (f_open(&fo, path.c_str(), FA_READ) != FR_OK) return false;

    // Basename of the loaded file, for a filename match
    const char *base = fname, *p;
    for (p = fname; *p; p++) if (*p == '/' || *p == '\\' || *p == ':') base = p + 1;

    // FatFs here is built without FF_USE_STRFUNC, so no f_gets: read in chunks and split
    // lines by hand. boards.cfg is a handful of lines, so one pass is plenty.
    char chunk[128], line[96];
    unsigned int br = 0, li = 0;
    bool found = false;

    do {
        if (f_read(&fo, chunk, sizeof(chunk), &br) != FR_OK) break;
        for (unsigned int i = 0; i < br && !found; i++) {
            char c = chunk[i];
            if (c != '\n' && c != '\r') {
                if (li < sizeof(line) - 1) line[li++] = c;
                continue;
            }
            line[li] = '\0';
            li = 0;
            if (line[0] == '\0' || line[0] == '#' || line[0] == ';') continue;

            char key[64]; unsigned long val;
            if (sscanf(line, " %63s %lx", key, &val) != 2) continue;
            if (key[0] == '#' || key[0] == ';') continue;

            if (strcasecmp(key, base) == 0) { *cfg = (uint32_t)val; found = true; break; }

            char *endp;
            unsigned long k = strtoul(key, &endp, 16);
            if (*endp == '\0' && k == ngh) { *cfg = (uint32_t)val; found = true; }
        }
    } while (br == sizeof(chunk) && !found);

    f_close(&fo);
    return found;
}

// Send the three config words as region 10: {1'b1, idx}, cfg[15:0], cfg[31:16].
// Bit 15 of the first word selects config mode (cp_op = 0) rather than a memory copy.
static int neo_send_cfg(uint32_t cfg) {
    taskENTER_CRITICAL();
    fpga_tx_header(0x11, 5);
    fpga_tx_byte(10);
    fpga_tx_byte(6); fpga_tx_byte(0); fpga_tx_byte(0); fpga_tx_byte(0);
    taskEXIT_CRITICAL();

    fbuf[0] = 0x00;  fbuf[1] = 0x80;                     // word 0 = 0x8000
    fbuf[2] = cfg & 0xff;         fbuf[3] = (cfg >> 8) & 0xff;
    fbuf[4] = (cfg >> 16) & 0xff; fbuf[5] = (cfg >> 24) & 0xff;
    send_fbuf_data(6);
    return 0;
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
    uint32_t ngh = 0;
    uint32_t cart_cfg = 0;
    const char *cfg_why = NULL;
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

    ngh = (hdr[40] | (hdr[41] << 8) | (hdr[42] << 16) | (hdr[43] << 24));

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

    // Cart hardware options: SD override first, else the built-in NGH table, else 0.
    if (neo_cfg_override(fname, ngh, &cart_cfg)) {
        cfg_why = "boards.cfg";
    } else {
        cart_cfg = neo_builtin_cfg(&fcore, ngh, p_size, &cfg_why);
    }
    DEBUG("neogeo: NGH %03lX cfg %08lX (%s)", (unsigned long)ngh,
          (unsigned long)cart_cfg, cfg_why ? cfg_why : "plain cart");
    if (cfg_why && strncmp(cfg_why, "UNSUPPORTED", 11) == 0) {
        overlay_status("Unsupported cart: %s", cfg_why + 12);
        delay(1500);
    }

    // Enable loading on FPGA
    set_loading_state(1);
    core_running = false;

    // The config words go first: they select the cart's bank/protection hardware for the
    // whole session (and keep cd_en clear, i.e. arcade rather than Neo Geo CD).
    neo_send_cfg(cart_cfg);

    // ---- M1.6: load the MVS system BIOS first (separate files on SD card) ----
    if (neo_load_bios("neogeo/sp-u2.sp1",  NEO_REG_SPROM, "SPROM (BIOS)", 1)) goto loadneogeo_end;
    // LO is the sprite vertical-zoom table. It lands in an on-chip 8-bit dpram that takes
    // one entry per WORD address, so the FPGA side (rom_stream_pack) expands this region
    // one byte per word rather than packing pairs. 000-lo.lo is two identical 64 KB
    // halves and the dpram holds 65536 entries, so sending the whole file writes it twice.
    if (neo_load_bios("neogeo/000-lo.lo",  NEO_REG_LO,    "LO",           1)) goto loadneogeo_end;
    if (neo_load_bios("neogeo/sfix.sfix",  NEO_REG_SFIX,  "SFIX",         1)) goto loadneogeo_end;
    // SM1 is REQUIRED, not optional: it is the Z80's BIOS sound program, and the MVS BIOS
    // boot path blocks on the Z80's reply to sound command 1 (sp-u2.sp1 $C11FE8, 50000
    // polls, then error code 8 -> the BIOS error screen). With no SM1 the Z80 has nothing
    // to run and every game stops on that screen.
    if (neo_load_bios("neogeo/sm1.sm1",    NEO_REG_SM1,   "SM1",          1)) goto loadneogeo_end;

    // ---- game regions from the .neo file ----
    // Seek to data start (0x1000)
    if ((r = f_lseek(&fcore, NEO_DATA_OFFSET)) != FR_OK) {
        overlay_status("Seek to data failed");
        goto loadneogeo_end;
    }
    // Walk all six regions. A zero-size region is SKIPPED, not a terminator: V2 is empty
    // on most single-V-ROM games (diggerma, kotm, ...), and stopping there dropped the C
    // ROM -- the sprites -- entirely. regions[6] is the real sentinel (name == NULL).
    for (int i = 0; regions[i].name != NULL; i++) {
        if (regions[i].size == 0) continue;
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
