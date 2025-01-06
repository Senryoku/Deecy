const std = @import("std");

pub const Track = @import("track.zig");

pub const Area = enum { SingleDensity, DoubleDensity };

first_track: u32,
last_track: u32,
start_fad: u32,
end_fad: u32,
