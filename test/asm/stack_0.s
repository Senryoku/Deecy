mov #0, r0
mov r15, r13
mov.l @r0, r1
mov.l @r15+,r1
cmp/eq r15,r13
bf 4
trapa #0
mov.l r2,@-r15
cmp/eq r1,r2
bt 4
trapa #1
cmp/eq r15,r13
bt 4
trapa #2
mov #0,r14
