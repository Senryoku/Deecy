# Deecy

Deecy is a very experimental Dreamcast emulator written in Zig.

Videos: [Soul Calibur](https://www.youtube.com/watch?v=IuY1Qi1YygM) (May 2024), [Grandia II](https://www.youtube.com/watch?v=YQG3SSVfeis) (July 2024)

![image](https://github.com/user-attachments/assets/cf0027bb-b136-45d3-bec9-623c407660fa)

## Build and Run

Install the correct zig version (see `.zigversion`). You can use [zigup](https://github.com/marler8997/zigup) to manage your installed zig versions.

```sh
zigup 0.13.0-dev.351+64ef45eb0
```

```sh
git clone --recurse-submodules https://github.com/Senryoku/Deecy     # Clone the repo and its submodules
cd Deecy
zig build run                                                        # Build and run in debug mode without any argument
zig build run -Doptimize=ReleaseFast -- -g game/game.gdi             # Build and run in release mode and loads a gdi
```

You will also need to provide copies of `dc_boot.bin` and `dc_flash.bin` files in the `bin/` directory.

### Linux 

Linux isn't fully supported yet.
nfd-zig (native file dialog) needs these additional dependencies on Linux:
```sh
sudo apt install librust-atk-dev
sudo apt install libgtk-3-dev
```

## Things I know I have to do

-   Debug, debug, debug.
-   Better low level emulation: Most games do not work as well when not using the HLE'd syscalls.
-   ch0-DMA and ch1-DMA?
-   What's "Maple V blank over interrupt"?
    "This interrupt is generated when a Maple interface transmission/reception operation spans V-Blank_In."
-   AICA:
    -   Stereo debugging
    -   DSP
    -   More debug
-   Renderer:
    -   Framebuffer:
        -   Somehow detect writes to framebuffer and display it.
        -   Write back for games that need it.
    -   Modifier Volumes.
        -   Implemented: Inclusion volumes and shadow bit over opaque and transparent geometry.
        -   Missing: Exclusion volumes.
        -   "Region Array Data Configuration" (written by the CPU directly to VRAM) are completely ignored. I don't know if it's actually used much.
    -   Fog LUT Mode 2.
    -   Secondary accumulate buffer (very low priority, not sure if many games use this feature).
    -   Bump mapping.
    -   Handle and generate Mipmaps.
    -   Sort-DMA?
    -   User Tile Clip, only the simplest version is supported.
-   MMU: Only supported for store queue writes using the pref intruction (used by Ikaruga for example)  

### Nice to have

-   Save states
-   Display VMU animation :^)
-   Some (rendering) performance metrics directly in the emulator?
-   GDROM-DMA: Uses a superfluous memcpy (gdrom -> dma-queue -> ram). Not a huge deal on my main system, but I bet it's noticeable on lower end devices.

## Things I don't know I have to do

-   A lot more than the previous list

## Some sources

-   http://www.shared-ptr.com/sh_insns.html
-   SH4 Hardware Manual / Programming Manual
-   https://segaretro.org/Dreamcast_official_documentation
-   Dreamcast Programming by Marcus Comstedt : https://mc.pp.se/dc/
-   Boot ROM disassembly by Lars Olsson (https://lars-olsson.sizious.com/, originally https://www.ludd.ltu.se/~jlo/dc/)
-   AICA ARM7 Core tester by snickerbockers: https://github.com/snickerbockers/dc-arm7wrestler/
-   Reicast https://github.com/skmp/reicast-emulator

## Dependencies

-   Multiple libraries from https://github.com/michal-z/zig-gamedev, included in libs/ as a submodule.
-   ndf-zig: https://github.com/fabioarnold/nfd-zig

## Thanks

-   Huge thanks to drk||Raziel and MetalliC for their respective contributions to the scene, and for answering my questions!
-   Thanks to originaldave\_ for the sh4 tests (https://github.com/SingleStepTests/sh4)
-   And overall thanks to everyone participating in the EmuDev Discord :)

## Licence

-   Uses data from MAME under the BSD-3-Clause licence (see `src/gdrom_secu.zig`).
