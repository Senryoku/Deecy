#

## Notes
 - Some games (e.g. Legacy of Kain) seem to try to directly access the GDROM, I was hoping I could skip it by only implementing the related syscalls :(


## Some sources
 - http://www.shared-ptr.com/sh_insns.html
 - https://www.renesas.com/us/en/document/mas/sh-4-software-manual / h14th002d2.pdf
 - DreamcastDevBoxSystemArchitecture.pdf
 - Dreamcast Programming by Marcus Comstedt : https://mc.pp.se/dc/
 - Boot ROM disassembly by Lars Olsson (https://lars-olsson.sizious.com/, originally https://www.ludd.ltu.se/~jlo/dc/)
 
 - Reicast https://github.com/skmp/reicast-emulator


## Dependencies
 - Multiple libraries from https://github.com/michal-z/zig-gamedev, included in libs/
