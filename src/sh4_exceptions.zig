const ExceptionCode = enum(u16) {
    PowerOnReset = 0x000,
    ManualReset = 0x020,
    HitachiUDIReset = 0x000,
    InstructionTLBMultipleHitException = 0x140,
    DataTLBMultipleHitException = 0x140,
    UserBreakBeforeInstructionExecution = 0x1E0,
    InstructionAddressError = 0x0E0,
    InstructionTLBMissException = 0x040,
    InstructionTLBProtectionViolationException = 0x0A0,
    GeneralIllegalInstructionException = 0x180,
    SlotIllegalInstructionException = 0x1A0,
    GeneralFPUDisableException = 0x800,
    SlotFPUDisableException = 0x820,
    DataAddressErrorRead = 0x0E0,
    DataAddressErrorWrite = 0x100,
    DataTLBMissExceptionRead = 0x040,
    DataTLBmissExceptionWrite = 0x060,
    DataTLBProtectionViolationExceptionRead = 0x0A0,
    DataTLBProtectionViolationExceptionWrite = 0x0C0,
    FPUException = 0x120,
    InitialPageWriteException = 0x080,
    UnconditionalTraprap = 0x160,
    UserBreakAfterInstructionExecution = 0x1E0,
    NonmaskableInterrupt = 0x1C0,
    // TODO External interrupts (IRL3-IRL0)
    // TODO Peripheral module interrupt (module/source)
};