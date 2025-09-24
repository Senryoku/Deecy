// Interleave 2 16-bit coordinates into a 32bit Morton code
fn morton2(c: vec2<u32>) -> u32 {
	var r = c; // We assume both c coordinates fit in 16 bits
    r = (r | (r << vec2<u32>(8))) & vec2<u32>(0x00FF00FF);
    r = (r | (r << vec2<u32>(4))) & vec2<u32>(0x0F0F0F0F);
    r = (r | (r << vec2<u32>(2))) & vec2<u32>(0x33333333);
    r = (r | (r << vec2<u32>(1))) & vec2<u32>(0x55555555);
    return r.x | (r.y << 1);
}
