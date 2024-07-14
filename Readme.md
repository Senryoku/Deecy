# Deecy

Deecy is a very experimental Dreamcast emulator written in Zig.

[Video of Soul Calibur running in Deecy](https://www.youtube.com/watch?v=IuY1Qi1YygM) (May 2024)

![image](https://github.com/Senryoku/Deecy/assets/1338143/5818d263-8f62-4f33-a799-5682f1fa94aa)

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

## Things I know I have to do

-   Debug, debug, debug.
-   Better low level emulation: Most games do not work as well when not using the HLE'd syscalls.
-   ch0-DMA and ch1-DMA?
-   What's "Maple V blank over interrupt"?
    "This interrupt is generated when a Maple interface transmission/reception operation spans V-Blank_In."
-   VMU/Storage
-   AICA:
    - Stereo
    - DSP
-   Renderer:
    - Modifier Volumes.
        - Implemented: Inclusion volumes and shadow bit over opaque geometry.
        - TODO: Exclusion volumes.
        - TODO: Handle polygons with 'Two Volumes' (another way to use the modifier volumes).
        - TODO: Same, but for the translucent pass.
    - Fog LUT Mode 2.
    - Secondary accumulate buffer (very low priority, not sure if many games use this feature)
    - Handle and generate Mipmaps.
    - Sort-DMA?
    - User Tile Clip, only the simplest version is supported.
    - (Improvement?) OIT: Add an option to allocate lists per slice to allow processing them in parallel when memory is not an issue (we're currently limited by the limit on binding buffer size, not total memory).
        No idea of the potential gain, if any.
      
- Ikaruga need some sort of MMU support :( 
    Apparently only around the Store Queue, but I'm still unsure how much of the MMU I need to implement to circumvent this (and the interpreter is already painfully slow).
    The fact that the issue arise before issuing a pref instruction with the MMU on lead me to think there's more to it.

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

## Thanks

-   Huge thanks to drk||Raziel and MetalliC for their respective contributions to the scene, and for answering my questions!
-   Thanks to originaldave_ for the sh4 tests (https://github.com/SingleStepTests/sh4)
-   And overall thanks to everyone participating in the EmuDev Discord :)

## Licence

- Uses data from MAME under the BSD-3-Clause licence (see `src/gdrom_secu.zig`).
