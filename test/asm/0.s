mov #8, r0
mov #9, r1
add r1, r0
mov #17,r2
cmp/eq r0,r2
bt 4
trapa #1
nop
mov #0,r14
