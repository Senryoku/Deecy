/// Calling this with an unique identifier (e.g. `Once(@src())`) will return true only the first time.
/// Useful for initialization, or for logging:
///   ```if (Once(@src())) std.log.info("This is wrong, I won't repeat myself.", .{});```
pub fn Once(comptime id: anytype) bool {
    const static = struct {
        const src = id;
        pub var first: bool = true;
    };
    const value = static.first;
    static.first = false;
    return value;
}
