#

## Notes

-   Some games (e.g. Legacy of Kain) seem to try to directly access the GDROM, I was hoping I could skip it by only implementing the related syscalls :(


## Things I know I have to do

-   ch0-DMA and ch1-DMA?
-   Sort-DMA? (And the corresponding end interrupt)

-   What's "Maple V blank over interrupt"?
    "This interrupt is generated when a Maple interface transmission/reception operation spans V-Blank_In."

-   The whole AICA Chip. Please have mercy.

-   Renderer:
  - Handle ClampUV and FlipUV, idealy using appropriate samplers and adressing modes.
  - Handle and generate Mipmaps.


## Things I don't know I have to do

-   A lot more than the previous list

## Some sources

-   http://www.shared-ptr.com/sh_insns.html
-   https://www.renesas.com/us/en/document/mas/sh-4-software-manual / h14th002d2.pdf
-   DreamcastDevBoxSystemArchitecture.pdf
-   Dreamcast Programming by Marcus Comstedt : https://mc.pp.se/dc/
-   Boot ROM disassembly by Lars Olsson (https://lars-olsson.sizious.com/, originally https://www.ludd.ltu.se/~jlo/dc/)

-   Reicast https://github.com/skmp/reicast-emulator

## Dependencies

-   Multiple libraries from https://github.com/michal-z/zig-gamedev, included in libs/


## Licence

- Uses data from MAME under the BSD-3-Clause licence (see `src/gdrom_secu.zig`).