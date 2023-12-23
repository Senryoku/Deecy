 @group(0) @binding(1) var texture_array: texture_2d_array<f32>;
 @group(0) @binding(2) var image_sampler: sampler;

@fragment
fn main(
    @location(0) color: vec4<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) @interpolate(flat) tex: vec2<u32>,
) -> @location(0) vec4<f32> {

    // TODO
    let offset_color = vec4<f32>(0.0, 0.0, 0.0, 0.0);

    let tex_color = textureSample(texture_array, image_sampler, uv, tex[0]);
    if (tex[1] & 1) == 1 {
        let shading = (tex[1] >> 1) & 0x3;
        let ignore_tex_alpha = ((tex[1] >> 3) & 0x1) == 1;
        let tex_a = select(tex_color.a, 1.0, ignore_tex_alpha);
        switch(shading)  {
            // Decal
            case 0: {
                let rgb = tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                return vec4<f32>(rgb, a);
            }
            // Modulate
            case 1 : {
                let rgb = color.rgb * tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                return vec4<f32>(rgb, a);
            }
            // Decal Alpha
            case 2 : {
                let rgb = tex_a * tex_color.rgb + (1.0 - tex_a) * color.rgb + offset_color.rgb;
                let a = color.a;
                return vec4<f32>(rgb, a);
            }
            // Modulate Alpha
            case 3 : {
                let rgb = color.rgb * tex_color.rgb + offset_color.rgb;
                let a = color.a * tex_a;
                return vec4<f32>(rgb, a);
            }
            default: { return color; }
        }
    }

    return color;
}