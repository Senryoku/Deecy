const std = @import("std");

buffer: []u8,
cycles: u32 = 0,

pub fn init(buffer: []u8) @This() {
    return .{
        .buffer = buffer,
    };
}

pub fn execute(self: *@This(), user_data: *anyopaque) void {
    // Saving XMM registers before the call.
    // FIXME: This is stupid, but I'm trying to reduce the number of possible cause for errors.
    asm volatile (
        \\ subq $11*16,%rsp
        \\ movdqa %xmm5,  0(%rsp) 
        \\ movdqa %xmm6,  16(%rsp)
        \\ movdqa %xmm7,  32(%rsp)
        \\ movdqa %xmm8,  48(%rsp)
        \\ movdqa %xmm9,  64(%rsp)
        \\ movdqa %xmm10, 80(%rsp)
        \\ movdqa %xmm11, 96(%rsp)
        \\ movdqa %xmm12, 112(%rsp)
        \\ movdqa %xmm13, 128(%rsp)
        \\ movdqa %xmm14, 144(%rsp)
        \\ movdqa %xmm15, 160(%rsp)
    );

    @as(*const fn (*anyopaque) void, @ptrCast(self.buffer.ptr))(user_data);

    asm volatile (
        \\ movdqa 0(%rsp), %xmm5 
        \\ movdqa 16(%rsp), %xmm6 
        \\ movdqa 32(%rsp), %xmm7 
        \\ movdqa 48(%rsp), %xmm8 
        \\ movdqa 64(%rsp), %xmm9 
        \\ movdqa 80(%rsp), %xmm10 
        \\ movdqa 96(%rsp), %xmm11 
        \\ movdqa 112(%rsp), %xmm12 
        \\ movdqa 128(%rsp), %xmm13 
        \\ movdqa 144(%rsp), %xmm14 
        \\ movdqa 160(%rsp), %xmm15 
        \\ addq $11*16,%rsp
    );
}
