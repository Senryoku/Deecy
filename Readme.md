# Deecy

Deecy is a very experimental Dreamcast emulator written in Zig.

![image](https://github.com/Senryoku/Deecy/assets/1338143/5818d263-8f62-4f33-a799-5682f1fa94aa)

## Things I know I have to do

-   Better low level emulation: Boot ROM currently hangs after the logo animation.
-   ch0-DMA and ch1-DMA?
-   What's "Maple V blank over interrupt"?
    "This interrupt is generated when a Maple interface transmission/reception operation spans V-Blank_In."
-   VMU/Storage
-   The whole AICA Chip. Please have mercy.
-   Renderer:
    - Modifier Volumes.
    - Fog.
    - Secondary accumulate buffer (very low priority, not sure if many games use this feature)
    - Handle and generate Mipmaps.
    - Sort-DMA?
    - User Tile Clip, only the simplest version is supported.
    - (Improvement?) OIT: Add an option to allocate lists per slice to allow processing them in parallel when memory is not an issue (we're currently limited by the limit on binding buffer size, not total memory).
        No idea of the potential gain, if any.
      
- Ikaruga need some sort of MMU support :( 
    Apparently only around the Store Queue, but I'm still unsure how much of the MMU I need to implement to circumvent this (and the interpreter is already painfully slow).
    The fact that the issue arise before issuing a pref instruction with the MMU on lead me to think there's more to it.
- Some games (e.g. Legacy of Kain) seem to try to directly access the GDROM, I was hoping I could skip it by only implementing the related syscalls :(


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

## Licence

- Uses data from MAME under the BSD-3-Clause licence (see `src/gdrom_secu.zig`).
