struct Uniforms {
    depth_min: f32,
    depth_max: f32, 
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var texture_array_8x8: texture_2d_array<f32>;
@group(0) @binding(2) var texture_array_16x16: texture_2d_array<f32>;
@group(0) @binding(3) var texture_array_32x32: texture_2d_array<f32>;
@group(0) @binding(4) var texture_array_64x64: texture_2d_array<f32>;
@group(0) @binding(5) var texture_array_128x128: texture_2d_array<f32>;
@group(0) @binding(6) var texture_array_256x256: texture_2d_array<f32>;
@group(0) @binding(7) var texture_array_512x512: texture_2d_array<f32>;
@group(0) @binding(8) var texture_array_1024x1024: texture_2d_array<f32>;

@group(1) @binding(0) var image_sampler: sampler;

fn tex_sample(uv: vec2<f32>, control: u32, index: u32) -> vec4<f32> {
    // textureSample can't be called in non-uniform context, because of this derivative, I guess.
    // This kinda feel like a hack, but works.
    let ddx = dpdx(uv);
    let ddy = dpdy(uv);
    switch((control >> 4) & 7)  {
        case 0: { return textureSampleGrad(texture_array_8x8, image_sampler, uv, index, ddx, ddy); }
        case 1: { return textureSampleGrad(texture_array_16x16, image_sampler, uv, index, ddx, ddy); }
        case 2: { return textureSampleGrad(texture_array_32x32, image_sampler, uv, index, ddx, ddy); }
        case 3: { return textureSampleGrad(texture_array_64x64, image_sampler, uv, index, ddx, ddy); }
        case 4: { return textureSampleGrad(texture_array_128x128, image_sampler, uv, index, ddx, ddy); }
        case 5: { return textureSampleGrad(texture_array_256x256, image_sampler, uv, index, ddx, ddy); }
        case 6: { return textureSampleGrad(texture_array_512x512, image_sampler, uv, index, ddx, ddy); }
        case 7: { return textureSampleGrad(texture_array_1024x1024, image_sampler, uv, index, ddx, ddy); }
        default: { return vec4<f32>(1.0, 0.0, 0.0, 1.0); } 
    }
}

fn fragment_color(
    base_color: vec4<f32>,
    offset_color: vec4<f32>,
    uv: vec2<f32>,
    tex: vec2<u32>
) -> vec4<f32> {
    let tex_color = tex_sample(uv, tex[1], tex[0]);

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
                let rgb = base_color.rgb * tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                return vec4<f32>(rgb, a);
            }
            // Decal Alpha
            case 2 : {
                let rgb = tex_a * tex_color.rgb + (1.0 - tex_a) * base_color.rgb + offset_color.rgb;
                let a = base_color.a;
                return vec4<f32>(rgb, a);
            }
            // Modulate Alpha
            case 3 : {
                let rgb = base_color.rgb * tex_color.rgb + offset_color.rgb;
                let a = base_color.a * tex_a;
                return vec4<f32>(rgb, a);
            }
            default: { return base_color + offset_color; }
        }
    } else {
        return base_color + offset_color;
    }
}

