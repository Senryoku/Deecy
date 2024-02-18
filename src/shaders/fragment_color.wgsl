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
        case 0u: { return textureSampleGrad(texture_array_8x8, image_sampler, uv, index, ddx, ddy); }
        case 1u: { return textureSampleGrad(texture_array_16x16, image_sampler, uv, index, ddx, ddy); }
        case 2u: { return textureSampleGrad(texture_array_32x32, image_sampler, uv, index, ddx, ddy); }
        case 3u: { return textureSampleGrad(texture_array_64x64, image_sampler, uv, index, ddx, ddy); }
        case 4u: { return textureSampleGrad(texture_array_128x128, image_sampler, uv, index, ddx, ddy); }
        case 5u: { return textureSampleGrad(texture_array_256x256, image_sampler, uv, index, ddx, ddy); }
        case 6u: { return textureSampleGrad(texture_array_512x512, image_sampler, uv, index, ddx, ddy); }
        case 7u: { return textureSampleGrad(texture_array_1024x1024, image_sampler, uv, index, ddx, ddy); }
        default: { return vec4<f32>(1.0, 0.0, 0.0, 1.0); } 
    }
}

fn tex_size(idx: u32) -> f32 {
    switch(idx & 7)  {
        case 0u: { return 8.0; }
        case 1u: { return 16.0; }
        case 2u: { return 32.0; }
        case 3u: { return 64.0; }
        case 4u: { return 128.0; }
        case 5u: { return 256.0; }
        case 6u: { return 512.0; }
        case 7u: { return 1024.0; }
        default: { return 8.0; }
    }
}

fn fog_alpha_lut(inv_w: f32) -> f32 {
    let val: f32 = clamp(inv_w * uniforms.fog_density, 1.0, 255.9999);
    let uval = bitcast<u32>(val);
    // Bit 6-4: Lower 3 bits for the 1/W index
    // Bit 3-0: Upper 4 bits for the 1/W mantissa
    let index =  ((((uval >> 23) + 1) & 0x7) << 4) | ((uval >> 19) & 0xF);
    let low = f32((uniforms.fog_lut[index / 4][index % 4] >> 8) & 0xFF) / 255.0;           
    let high = f32((uniforms.fog_lut[index / 4][index % 4]) & 0xFF) / 255.0;
    return mix(low, high, f32((uval >> 11) & 0xFF) / 255.0); // Residual fractional part
}

fn apply_fog(shading_instructions: u32, inv_w: f32, color: vec4<f32>, offset_alpha: f32) -> vec4<f32> {
    // TODO: Color Clamping
    switch((shading_instructions >> 19) & 0x3) {
        case 0x0u: {
            // Lookup table mode
            let fog_alpha = fog_alpha_lut(inv_w);
            return vec4<f32>(mix(color.rgb, uniforms.fog_col_pal.rgb, fog_alpha), color.a);
        }
        case 0x1u: {
            // Per vertex mode
            if(((shading_instructions >> 21) & 1) == 1) { // Using Offset color?
                return vec4<f32>(mix(color.rgb, uniforms.fog_col_vert.rgb, offset_alpha), color.a);
            } else {
                // If the polygon is not set up to use an Offset Color, Fog processing is not performed.
                return color;
            }
        }
        case 0x2u: {
            // No Fog
            return color;
        }
        case 0x3u: {
            // Look up table Mode 2
            // Substitutes the polygon color for the Fog Color, and the polygon α value for the Fog α value.
            let fog_alpha = fog_alpha_lut(inv_w);
            return vec4<f32>(1.0, 0.0, 0.0, 1.0); // FIXME: This is untested, output a solid red to identify some places where it's used.
            //return vec4<f32>(uniforms.fog_col_pal.rgb, fog_alpha);
        }
        default: { return color; }
    }
}

fn fragment_color(
    base_color: vec4<f32>,
    offset_color: vec4<f32>,
    uv: vec2<f32>,
    tex: vec2<u32>,
    inv_w: f32,
) -> vec4<f32> {
    let u_size: f32 = tex_size((tex[1] >> 4) & 7);
    let v_size: f32 = tex_size((tex[1] >> 7) & 7);
    let tex_color = tex_sample( vec2<f32>(1.0, v_size / u_size) * uv, tex[1], tex[0]);

    var final_color = base_color + vec4<f32>(offset_color.rgb, 0.0);

    if (tex[1] & 1) == 1 {
        let shading = (tex[1] >> 1) & 0x3;
        let ignore_tex_alpha = ((tex[1] >> 3) & 0x1) == 1;
        let tex_a = select(tex_color.a, 1.0, ignore_tex_alpha);
        switch(shading)  {
            // Decal
            case 0u: {
                let rgb = tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                final_color = vec4<f32>(rgb, a);
            }
            // Modulate
            case 1u: {
                let rgb = base_color.rgb * tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                final_color = vec4<f32>(rgb, a);
            }
            // Decal Alpha
            case 2u: {
                let rgb = tex_a * tex_color.rgb + (1.0 - tex_a) * base_color.rgb + offset_color.rgb;
                let a = base_color.a;
                final_color = vec4<f32>(rgb, a);
            }
            // Modulate Alpha
            case 3u: {
                let rgb = base_color.rgb * tex_color.rgb + offset_color.rgb;
                let a = base_color.a * tex_a;
                final_color = vec4<f32>(rgb, a);
            }
            default: { final_color = base_color + vec4<f32>(offset_color.rgb, 0.0); }
        }
    } 
    
    return apply_fog(tex[1], inv_w, final_color, offset_color.a);
}

