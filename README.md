
<h1 align="center">
    <img src="https://raw.githubusercontent.com/Senryoku/Deecy/refs/heads/main/src/assets/logo-256.png" width="128">
    <div>Deecy</div>
</h1>

Deecy is an experimental Dreamcast emulator written in Zig.

Videos: [Soul Calibur](https://www.youtube.com/watch?v=IuY1Qi1YygM) (May 2024), [Grandia II](https://www.youtube.com/watch?v=YQG3SSVfeis) (July 2024), [DCA3](https://www.youtube.com/watch?v=RAj67PZbVnc) (January 2025)

![image](https://github.com/user-attachments/assets/cf0027bb-b136-45d3-bec9-623c407660fa)

## Installation

 - Download the latest version for your platform from the [Release Page](https://github.com/Senryoku/Deecy/releases).
 - Decompress the zip archive.
 - Copy your bios and flash dumps as `dc_boot.bin` and `dc_flash.bin` to the `data` folder.
 - Launch the Deecy executable and click `Change Directory` to select the folder where you store your DC games.

## Usage 

### Keybindings

Deecy should detect, map and use controllers automatically. Host controllers can be manually assigned to guest controller ports in the Settings.
Keyboard bindings can be customized in the settings. Defaults for controller 1:
| DC Controller | Host Keyboard (AZERTY) |
| ------------- | ------------- |
| A, B, X, Y    | A, Z, Q, S |
| L, R triggers | W, X |
| Start         | Enter |
| Left Stick    | Numpad 8, 4, 5, 6 | 
| Dpad          | Directional Arrows |

### Shortcuts

| Key    | Action |
| ------ | ------ | 
| Escape | Show/Hide the UI |
| F      | Toggle Fullscreen |
| L      | Toggle unlimited emulation speed |
| F1-F4  | Save state 1-4 |
| F5-F8  | Load state 1-4 |
| D      | Toggle debug UI |
| N      | Next Frame (Experimental) |

### CLI options

| Option | Action | Argument | Notes |
| ------ | ------ | -------- | ----- |
| -g     | Load and Execute a disc file (.gdi/.cdi/.chd) | Path to a disc file |
| --vmu  | Replace default vmu file | Path to an vmu file | Use with -g |
| --no-realtime | Starts with unlimited emulation speed |
| --load-state | Loads a save state on startup | Path to a Deecy save state | Use with -g |
| --stop | Prevent automatic start of emulation | | Use with -g or -b |
| --skip-bios | Skip default BIOS. | | Experimental |
| -b     | Load and Execute a .bin file | Path to a .bin file | Will also skip the BIOS. Experimental |
| -i     | Replace default IP.BIN file | Path to an IP.BIN file | Use with -b only |

## Compatibility

[See issue #33](https://github.com/Senryoku/Deecy/issues/33)

## Build

Install the correct zig version (see `.zigversion`, I try to keep up with [Mach nominated version](https://machengine.org/docs/nominated-zig/) when not on a stable release). 
You can use [zigup](https://github.com/marler8997/zigup) to manage your installed zig versions, or get it from https://machengine.org/docs/nominated-zig/.
```sh
zigup 0.14.1
```
Clone and build. Zig will fetch all dependencies automatically.
```sh
git clone https://github.com/Senryoku/Deecy         # Clone the repo
cd Deecy
zig build run                                       # Build and run in debug mode without any argument
zig build run --release=fast -- -g "path/game.gdi"  # Build and run in release mode and loads a disc
```

You will also need to provide copies of `dc_boot.bin` and `dc_flash.bin` files in the `data/` directory.

### Linux 

`nfd-zig` (native file dialog) needs these additional dependencies on Linux:
```sh
sudo apt install libgtk-3-dev
```

## Things I know I have to do

-   Debug, debug, debug.
-   SH4:
    -   MMU:
        -   Windows CE: Test and Debug more games.
        -   Optimize.
-   AICA:
    -   Debug
-   Renderer:
    -   Framebuffer:
        -   Improve detection of writes to framebuffer (false positives?)
        -   Write back for games that need it.
    -   Modifier Volumes.
        -   Implemented: Inclusion volumes and shadow bit over opaque and transparent geometry.
        -   Missing: Exclusion volumes.
        -   Missing: Translucent MV in pre-sort mode.
    -   Region Array Data Configuration are mostly ignored.
        -   Flush Accumulate (Secondary accumulate buffer)
    -   Fog LUT Mode 2.
    -   User Tile Clip, only the simplest version is supported.
    -   Secondary accumulate buffer (very low priority, not sure if many games use this feature).
    -   Mipmaps for palette textures?
    -   Follow ISP_FEED_CFG discard mode flag? (Find a game that turns it off)

### Nice to have

-   Some (rendering) performance metrics directly in the emulator?
-   GDROM-DMA: Uses a superfluous memcpy (gdrom -> dma-queue -> ram). Not a huge deal on my main system, but I bet it's noticeable on lower end devices.

## Some sources

-   http://www.shared-ptr.com/sh_insns.html
-   SH4 Hardware Manual / Programming Manual
-   https://segaretro.org/Dreamcast_official_documentation
-   Dreamcast Programming by Marcus Comstedt : https://mc.pp.se/dc/
-   Boot ROM disassembly by Lars Olsson (https://lars-olsson.sizious.com/, originally https://www.ludd.ltu.se/~jlo/dc/)
-   [Neill Corlett's Yamaha AICA notes](https://github.com/Senryoku/dreamcast-docs/raw/refs/heads/master/AICA/DOCS/myaica.txt)
-   AICA ARM7 Core tester by snickerbockers: https://github.com/snickerbockers/dc-arm7wrestler/
-   Reicast https://github.com/skmp/reicast-emulator

## Dependencies

Dependencies are managed by the `build.zig.zon` file.

-   Multiple libraries from https://github.com/zig-gamedev (MIT)
-   ndf-zig: https://github.com/fabioarnold/nfd-zig (MIT)
-   zig-lz4: https://github.com/SnorlaxAssist/zig-lz4 (MIT), bindings for LZ4 https://github.com/lz4/lz4 (BSD 2-Clause)

## Thanks

-   Huge thanks to drk||Raziel and MetalliC for their respective contributions to the scene, and for answering my questions!
-   Thanks to originaldave\_ for the sh4 tests (https://github.com/SingleStepTests/sh4)
-   And overall thanks to everyone participating in the EmuDev Discord :)

## Licence
 
 Uses data from MAME under the BSD-3-Clause licence (see `src/gdrom_secu.zig`). CHD related features (`src/disc/chd.zig`) are also
 inspired by, or even direct ports of, their equivalent from the MAME project.

 Rest is MIT.
