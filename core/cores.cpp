#include <vector>
#include <string>
#include <stdio.h>
#include "cores.h"

#include "ff.h"
#include "menu_manager.h"
#include "overlay.h"

extern void file_log(const char *msg);   // TEMP diagnostic, defined in main.cpp

// null-terminated list of core info
std::vector<core_info> core_info_list;
// Main menu listing:
// >0: core id, -1: cores menu, -2: options menu, 0: end of list
std::vector<int16_t> main_menu_config;

struct core_info *find_core_by_id(uint16_t id) {
    for (auto &core : core_info_list) {
        if (core.id == id)
            return &core;
    }
    return NULL;
}

Menu *create_default_menu(const char *imgdir) {
    dprint("Creating default menu\n");
    return new DefaultMenu();
}

Menu *create_pcxt_menu(const char *imgdir) {
    dprint("Creating PCXT menu\n");
    return new PcxtMenu(imgdir);
}

void init_core_list() {
    core_info_list = {
        {1, "NES", "nes", "nestang.bin", loadnes, create_default_menu},
        {2, "SNES", "snes", "snestang.bin", loadsnes, create_default_menu},
        {3, "Game Boy Advance", "gba", "gbatang.bin", loadgba, create_default_menu},
        {4, "MegaDrive / Genesis", "genesis", "mdtang.bin", loadmd, create_default_menu},
        {5, "Sega Master System", "sms", "smstang.bin", loadsms, create_default_menu},
        {6, "IBM PC/XT", "pc", "pctang.bin", loadpc, create_pcxt_menu},
        // Neo Geo (arcade) -- TerraOnion .neo format, MVS BIOS (SPROM/LO/SFIX/SM1)
        // loaded from neogeo/ ahead of the game. CORE_ID 7 (matches the NeoTang
        // FPGA iosys .CORE_ID(7)); free between PC/XT(6) and PC Engine(8).
        {7, "Neo Geo", "neogeo", "neotang.bin", loadneogeo, create_default_menu},
        // Real (2026-09-01): PC Engine and PC Engine CD unified into one entry --
        // one real bitstream (pcetang.bin, cd_bridge.vhd's SCSI target is baked into
        // every CD combo build regardless of what's loaded), one CORE_ID (0x0008,
        // matches all 3 CD boards' VHDL). loadpce_dispatch() (pce.cpp) picks loadpce
        // vs loadpcecd by file extension (.pce vs .chd) at load time. rom_dir "pce"
        // is only this entry's browse root (see main.cpp's menu_loadrom call site,
        // which opens the drive root instead for this one core so the chooser can
        // reach both pce/ and pcenginecd/ -- FileChooser confines navigation to
        // rootdir's own subtree, so a single-dir root can't reach both); ownership
        // matching in menu_loadrom() short-circuits on .pce/.chd before the generic
        // rom_dir-prefix loop, see pcetang_cd_scsi_plan.md for the protocol this
        // shares with the CD path.
        {8, "PC Engine", "pce", "pcetang.bin", loadpce_dispatch, create_default_menu}
    };

    main_menu_config = {1,2,
#if defined(TANG_MEGA60K) || defined(TANG_MEGA138K) || defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
        3,4,5,6,
#endif
#if defined(TANG_CONSOLE60K)
        // Neo Geo (NeoTang) -- 60K-only until built/verified on the other boards.
        7,
#endif
#if defined(TANG_PRIMER25K) || defined(TANG_CONSOLE60K) || defined(TANG_NANO20K)
        // pcetang: gw_sh-verified on these boards (see pcetang project
        // memory pcetang_hw_bringup.md / status matrix) -- not enabled on
        // other boards' menus until built and verified there too.
        8,
#endif
        -1, -2
    };
}


extern const char *BOARD_NAME;
extern char *drv;

// Find a core file in the search order:
// usb:cores/${BOARD_NAME}/${core_name}
// usb:cores/${core_name}
bool find_core_for_board(std::string &fname, const char *core_name) {
    // check sd|usb:cores/${BOARD_NAME}/${core_name}
    fname = std::string(drv) + "cores/" + BOARD_NAME + "/" + core_name;
    FILINFO fno;
    FRESULT res1 = f_stat(fname.c_str(), &fno);
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "try1=%s res=%d sz=%lu", fname.c_str(), (int)res1,
            res1 == FR_OK ? (unsigned long)fno.fsize : 0UL);
        file_log(buf);
    }
    if (res1 == FR_OK && fno.fsize > 0) {
        return true;
    }

    // check sd|usb:cores/${core_name}
    fname = std::string(drv) + "cores/" + core_name;
    FRESULT res2 = f_stat(fname.c_str(), &fno);
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "try2=%s res=%d sz=%lu", fname.c_str(), (int)res2,
            res2 == FR_OK ? (unsigned long)fno.fsize : 0UL);
        file_log(buf);
    }
    if (res2 == FR_OK && fno.fsize > 0) {
        return true;
    }
    return false;
}
