const std = @import("std");

// Holly Video Chip

// Tile Accelerator and PowerVR2 core

const TileAccelerator = struct {};

const PowerVR2 = struct {
    ram: [8 * 1024 * 1024]u8,
};
