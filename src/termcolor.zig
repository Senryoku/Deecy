pub fn green(comptime str: []const u8) []const u8 {
    return "\u{001b}[32m" ++ str ++ "\u{001b}[0m";
}

pub fn yellow(comptime str: []const u8) []const u8 {
    return "\u{001b}[33m" ++ str ++ "\u{001b}[0m";
}

pub fn red(comptime str: []const u8) []const u8 {
    return "\u{001b}[31m" ++ str ++ "\u{001b}[0m";
}

pub fn blue(comptime str: []const u8) []const u8 {
    return "\u{001b}[34m" ++ str ++ "\u{001b}[0m";
}

pub fn grey(comptime str: []const u8) []const u8 {
    return "\u{001b}[90m" ++ str ++ "\u{001b}[0m";
}
