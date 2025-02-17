const std = @import("std");

pub const Track = @import("track.zig");

pub const Area = enum { SingleDensity, DoubleDensity };

first_track: u32, // 0 based index
last_track: u32, // 0 based index
start_fad: u32,
end_fad: u32,
