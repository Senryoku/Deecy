dc-arm7wrestler.bin by snickerbockers (https://github.com/snickerbockers/dc-arm7wrestler)

Compiled using the a slightly modified Makefile, adding "-fno-builtin" for the sh4 steps (gcc was generating a memset call):

```makefile
AS=sh4-linux-gnu-as
LD=sh4-linux-gnu-ld
CC=sh4-linux-gnu-gcc
OBJCOPY=sh4-linux-gnu-objcopy

all: dc-arm7wrestler.bin

clean:
        rm -f init.o dc-arm7wrestler.elf dc-arm7wrestler.bin main.o

init.o: init.s
        $(AS) -little -o init.o init.s

dc-arm7wrestler.elf: init.o main.o
        $(CC) -Wl,-e_start,-Ttext,0x8c010000 init.o main.o -o dc-arm7wrestler.elf -nostartfiles -nostdlib -lgcc -fno-builtin

dc-arm7wrestler.bin: dc-arm7wrestler.elf
        $(OBJCOPY) -O binary -j .text -j .data -j .bss -j .rodata  --set-section-flags .bss=alloc,load,contents dc-arm7wrestler.elf dc-arm7wrestler.bin

main.o: main.c arm_prog.h
        $(CC) -c main.c -nostartfiles -nostdlib -Os -fno-builtin
```