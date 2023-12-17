pub fn yellow(comptime str: []const u8) []const u8 {
    return "\u{001b}[33m" ++ str ++ "\u{001b}[0m";
}

pub fn red(comptime str: []const u8) []const u8 {
    return "\u{001b}[31m" ++ str ++ "\u{001b}[0m";
}
