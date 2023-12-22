 @group(0) @binding(1) var texture_array: texture_2d_array<f32>;
 @group(0) @binding(2) var image_sampler: sampler;

@fragment
fn main(
    @location(0) color: vec4<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) @interpolate(flat) tex: u32,
) -> @location(0) vec4<f32> {
    var safe_tex = tex;
    if tex == 0xFFFFFFFF {
        safe_tex = 0; // No idea if this is actually an issue, but I'd rather avoid sampling from a non existing texture.
    }
    let tex_color = textureSample(texture_array, image_sampler, uv, safe_tex);

    if tex != 0xFFFFFFFF {
        return tex_color;
    } else {
        return color;
    }
}