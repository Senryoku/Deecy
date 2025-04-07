pub const Exception = enum {
    PowerOnReset,
    ManualReset,
    HitachiUDIReset,
    InstructionTLBMultipleHit,
    DataTLBMultipleHit,
    UserBreakBeforeInstructionExecution,
    InstructionAddressError,
    InstructionTLBMiss,
    InstructionTLBProtectionViolation,
    GeneralIllegalInstruction,
    SlotIllegalInstruction,
    GeneralFPUDisable,
    SlotFPUDisable,
    DataAddressErrorRead,
    DataAddressErrorWrite,
    DataTLBMissRead,
    DataTLBMissWrite,
    DataTLBProtectionViolationRead,
    DataTLBProtectionViolationWrite,
    FPU,
    InitialPageWrite,
    UnconditionalTrap,
    UserBreakAfterInstructionExecution,
    NonmaskableInterrupt,

    pub fn code(self: @This()) u16 {
        return switch (self) {
            .PowerOnReset => 0x000,
            .ManualReset => 0x020,
            .HitachiUDIReset => 0x000,
            .InstructionTLBMultipleHit => 0x140,
            .DataTLBMultipleHit => 0x140,
            .UserBreakBeforeInstructionExecution => 0x1E0,
            .InstructionAddressError => 0x0E0,
            .InstructionTLBMiss => 0x040,
            .InstructionTLBProtectionViolation => 0x0A0,
            .GeneralIllegalInstruction => 0x180,
            .SlotIllegalInstruction => 0x1A0,
            .GeneralFPUDisable => 0x800,
            .SlotFPUDisable => 0x820,
            .DataAddressErrorRead => 0x0E0,
            .DataAddressErrorWrite => 0x100,
            .DataTLBMissRead => 0x040,
            .DataTLBMissWrite => 0x060,
            .DataTLBProtectionViolationRead => 0x0A0,
            .DataTLBProtectionViolationWrite => 0x0C0,
            .FPU => 0x120,
            .InitialPageWrite => 0x080,
            .UnconditionalTrap => 0x160,
            .UserBreakAfterInstructionExecution => 0x1E0,
            .NonmaskableInterrupt => 0x1C0,
        };
    }

    pub fn offset(self: @This()) u32 {
        return switch (self) {
            .InstructionTLBMiss, .DataTLBMissRead, .DataTLBMissWrite => 0x400,
            else => 0x100,
        };
    }
};
