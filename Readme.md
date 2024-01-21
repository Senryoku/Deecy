#

## Notes

-   Some games (e.g. Legacy of Kain) seem to try to directly access the GDROM, I was hoping I could skip it by only implementing the related syscalls :(


## Things I know I have to do

-   ch0-DMA and ch1-DMA?
-   Sort-DMA? (And the corresponding end interrupt)

-   What's "Maple V blank over interrupt"?
    "This interrupt is generated when a Maple interface transmission/reception operation spans V-Blank_In."
-   VMU/Storage

-   The whole AICA Chip. Please have mercy.

-   Renderer:
    - Handle and generate Mipmaps.
    - Order independent transparency.
    - Secondary accumulate buffer (very low priority, not sure if many games use this feature)
      
- Ikaruga need some sort of MMU support :( 
    Apparently only around the Store Queue, but I'm still unsure how much of the MMU I need to implement to circumvent this (and the interpreter is already painfully slow).
    The fact that the issue arise before issuing a pref instruction with the MMU on lead me to think there's more to it.


## Things I don't know I have to do

-   A lot more than the previous list

## Some sources

-   http://www.shared-ptr.com/sh_insns.html
-   https://www.renesas.com/us/en/document/mas/sh-4-software-manual / h14th002d2.pdf
-   DreamcastDevBoxSystemArchitecture.pdf
-   Dreamcast Programming by Marcus Comstedt : https://mc.pp.se/dc/
-   Boot ROM disassembly by Lars Olsson (https://lars-olsson.sizious.com/, originally https://www.ludd.ltu.se/~jlo/dc/)
-   AICA ARM7 Core tester by snickerbockers: https://github.com/snickerbockers/dc-arm7wrestler/
-   Reicast https://github.com/skmp/reicast-emulator

## Dependencies

-   Multiple libraries from https://github.com/michal-z/zig-gamedev, included in libs/


## Licence

- Uses data from MAME under the BSD-3-Clause licence (see `src/gdrom_secu.zig`).
