@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var texture_array_8x8: texture_2d_array<f32>;
@group(0) @binding(2) var texture_array_16x16: texture_2d_array<f32>;
@group(0) @binding(3) var texture_array_32x32: texture_2d_array<f32>;
@group(0) @binding(4) var texture_array_64x64: texture_2d_array<f32>;
@group(0) @binding(5) var texture_array_128x128: texture_2d_array<f32>;
@group(0) @binding(6) var texture_array_256x256: texture_2d_array<f32>;
@group(0) @binding(7) var texture_array_512x512: texture_2d_array<f32>;
@group(0) @binding(8) var texture_array_1024x1024: texture_2d_array<f32>;
@group(0) @binding(10) var<storage, read> palette: array<u32>;

@group(1) @binding(0) var image_sampler: sampler;

fn bilinear_interpolation(u_min_v_min: vec4<f32>, u_min_v_max: vec4<f32>, u_max_v_min: vec4<f32>, u_max_v_max: vec4<f32>, frac: vec2<f32>) -> vec4<f32> {
    let v_min = mix(u_min_v_min, u_max_v_min, frac.x);
    let v_max = mix(u_min_v_max, u_max_v_max, frac.x);
    return mix(v_min, v_max, frac.y);
}

fn tex_array_sample(tex_array: texture_2d_array<f32>, uv: vec2<f32>, duvdx: vec2<f32>, duvdy: vec2<f32>, palette_instructions: u32, control: u32, index: u32) -> vec4<f32> {
    if index >= textureNumLayers(tex_array) { return vec4<f32>(1.0, 0.0, 0.0, 1.0); }

    if (palette_instructions & 1) == 1 {
        // Palette Texture
        let palette_selector = ((palette_instructions >> 2) & 0x3F) << 4;

        if ((palette_instructions >> 1) & 1) == 1 {
            // with Bilinear filtering
            let texel_coord = vec2<f32>(textureDimensions(tex_array)) * uv;

            if true { // Fallback while the textureGather version is not working.
                let palette_indices = vec4<u32>(
                    pack4x8unorm(textureSampleLevel(tex_array, image_sampler, (texel_coord + vec2<f32>(0.0, 0.0)) / vec2<f32>(textureDimensions(tex_array)), index, 0).zyxw),
                    pack4x8unorm(textureSampleLevel(tex_array, image_sampler, (texel_coord + vec2<f32>(0.0, 1.0)) / vec2<f32>(textureDimensions(tex_array)), index, 0).zyxw),
                    pack4x8unorm(textureSampleLevel(tex_array, image_sampler, (texel_coord + vec2<f32>(1.0, 0.0)) / vec2<f32>(textureDimensions(tex_array)), index, 0).zyxw),
                    pack4x8unorm(textureSampleLevel(tex_array, image_sampler, (texel_coord + vec2<f32>(1.0, 1.0)) / vec2<f32>(textureDimensions(tex_array)), index, 0).zyxw),
                );
                return bilinear_interpolation(
                    unpack4x8unorm(palette[palette_selector + palette_indices[0] ]),
                    unpack4x8unorm(palette[palette_selector + palette_indices[1] ]),
                    unpack4x8unorm(palette[palette_selector + palette_indices[2] ]),
                    unpack4x8unorm(palette[palette_selector + palette_indices[3] ]),
                    fract(texel_coord),
                ).zyxw;
            } else {
                // FIXME: This should be more efficient, but doesn't work as expected. (For a failure example: Soul Calibur characted selection uses filtered palette textures for the background and "New" text) 
                // Palette data is 4 or 8bits, we don't need every components.
                let z = textureGather(2, tex_array, image_sampler, uv, index);
                let palette_indices = vec4<u32>(
                    pack4x8unorm(vec4<f32>(z.x, 0, 0, 0)), // Umin, Vmax (per WebGPU spec)
                    pack4x8unorm(vec4<f32>(z.y, 0, 0, 0)), // Umax, Vmax
                    pack4x8unorm(vec4<f32>(z.z, 0, 0, 0)), // Umax, Vmin
                    pack4x8unorm(vec4<f32>(z.w, 0, 0, 0)), // Umin, Vmin
                );
                // FIXME: The error looks like these parameters aren't in order, but I think I tried every permutation at this point...
                return bilinear_interpolation(
                    unpack4x8unorm(palette[palette_selector + palette_indices[3] ]),
                    unpack4x8unorm(palette[palette_selector + palette_indices[0] ]),
                    unpack4x8unorm(palette[palette_selector + palette_indices[2] ]),
                    unpack4x8unorm(palette[palette_selector + palette_indices[1] ]),
                    fract(texel_coord),
                ).zyxw;
            }
        } else {
            var sample = textureSampleLevel(tex_array, image_sampler, uv, index, 0);
            let palette_index = pack4x8unorm(sample.zyxw);
            return unpack4x8unorm(palette[palette_selector + palette_index]).zyxw;
        }
    } else {
        return textureSampleGrad(tex_array, image_sampler, uv, index, duvdx, duvdy);
    }
}

fn tex_sample(uv: vec2<f32>, duvdx: vec2<f32>, duvdy: vec2<f32>, palette_instructions: u32, control: u32, index: u32) -> vec4<f32> {
    switch(max((control >> 4) & 7, (control >> 7) & 7))  {
        case 0u: { return tex_array_sample(texture_array_8x8, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 1u: { return tex_array_sample(texture_array_16x16, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 2u: { return tex_array_sample(texture_array_32x32, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 3u: { return tex_array_sample(texture_array_64x64, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 4u: { return tex_array_sample(texture_array_128x128, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 5u: { return tex_array_sample(texture_array_256x256, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 6u: { return tex_array_sample(texture_array_512x512, uv, duvdx, duvdy, palette_instructions, control, index); }
        case 7u: { return tex_array_sample(texture_array_1024x1024, uv, duvdx, duvdy, palette_instructions, control, index); }
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

fn fog_alpha_lut(z: f32) -> f32 {
    let val: f32 = clamp(z * uniforms.fog_density, 1.0, 255.9999);
    let uval = bitcast<u32>(val);
    // Bit 6-4: Lower 3 bits for the 1/W index
    // Bit 3-0: Upper 4 bits for the 1/W mantissa
    let index = ((((uval >> 23) + 1) & 0x7) << 4) | ((uval >> 19) & 0xF);
    let low = f32((uniforms.fog_lut[index / 4][index % 4] >> 8) & 0xFF) / 255.0;
    let high = f32((uniforms.fog_lut[index / 4][index % 4]) & 0xFF) / 255.0;
    return mix(low, high, f32((uval >> 11) & 0xFF) / 255.0); // Residual fractional part
}

fn apply_fog(shading_instructions: u32, z: f32, color: vec4<f32>, offset_alpha: f32) -> vec4<f32> {
    // TODO: Color Clamping
    switch((shading_instructions >> 19) & 0x3) {
        case 0x0u: {
            // Lookup table mode
            let fog_alpha = fog_alpha_lut(z);
            return vec4<f32>(mix(color.rgb, uniforms.fog_col_pal.rgb, fog_alpha), color.a);
        }
        case 0x1u: {
            // Per vertex mode
            if ((shading_instructions >> 21) & 1) == 1 { // Using Offset color?
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
            let fog_alpha = fog_alpha_lut(z);
            return vec4<f32>(1.0, 0.0, 0.0, 1.0); // FIXME: This is untested, output a solid red to identify some places where it's used.
            //return vec4<f32>(uniforms.fog_col_pal.rgb, fog_alpha);
        }
        default: { return color; }
    }
}


fn area_color(
    base_color: vec4<f32>,
    offset_color: vec4<f32>,
    uv: vec2<f32>,
    duvdx: vec2<f32>,
    duvdy: vec2<f32>,
    texture_index: u32,
    palette_instructions: u32,
    shading_instructions: u32,
    z: f32,
    punch_through: bool,
) -> vec4<f32> {
    var final_color = base_color + vec4<f32>(offset_color.rgb, 0.0);

    if (shading_instructions & 1) == 1 {
        let u_size: f32 = tex_size((shading_instructions >> 4) & 7);
        let v_size: f32 = tex_size((shading_instructions >> 7) & 7);
        let uv_factor = select(vec2<f32>(1.0, v_size / u_size), vec2<f32>(u_size / v_size, 1.0), u_size < v_size);
        let tex_color = tex_sample(uv_factor * uv, uv_factor * duvdx, uv_factor * duvdy, palette_instructions, shading_instructions, texture_index);

        let shading = (shading_instructions >> 1) & 0x3;
        let ignore_tex_alpha = ((shading_instructions >> 3) & 0x1) == 1;
        let tex_a = select(tex_color.a, 1.0, ignore_tex_alpha);

        if punch_through && tex_a < uniforms.pt_alpha_ref {
            discard;
        }

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

    return apply_fog(shading_instructions, z, final_color, offset_color.a);
}

struct FragmentColor {
    area0: vec4<f32>,
    area1: vec4<f32>,
}

fn fragment_color(
    varying_base_color: vec4<f32>,
    flat_base_color: vec4<f32>,
    varying_offset_color: vec4<f32>,
    flat_offset_color: vec4<f32>,
    uv: vec2<f32>,
    texture_index_palette: u32,
    shading_instructions: u32,
    area1_varying_base_color: vec4<f32>,
    area1_flat_base_color: vec4<f32>,
    area1_varying_offset_color: vec4<f32>,
    area1_flat_offset_color: vec4<f32>,
    area1_uv: vec2<f32>,
    area1_texture_index_palette: u32,
    area1_shading_instructions: u32,
    z: f32,
    punch_through: bool,
) -> FragmentColor {
    var output: FragmentColor;

    let gouraud_area0 = ((shading_instructions >> 23) & 1) == 1;
    let gouraud_area1 = ((area1_shading_instructions >> 23) & 1) == 1;

    let base_color = select(flat_base_color, varying_base_color, gouraud_area0);
    let offset_color = select(flat_offset_color, varying_offset_color, gouraud_area0);

    let area1_base_color = select(area1_flat_base_color, area1_varying_base_color, gouraud_area1);
    let area1_offset_color = select(area1_flat_offset_color, area1_varying_offset_color, gouraud_area1);

    let duvdx = dpdx(uv);
    let duvdy = dpdy(uv);
    let area1_duvdx = dpdx(area1_uv);
    let area1_duvdy = dpdy(area1_uv);

    let texture_index: u32 = texture_index_palette & 0xFFFF;
    let palette_instructions: u32 = texture_index_palette >> 16;

    output.area0 = area_color(
        base_color,
        offset_color,
        uv,
        duvdx,
        duvdy,
        texture_index,
        palette_instructions,
        shading_instructions,
        z,
        punch_through
    );

    // Shadow Bit | Volume Bit | Explanation
    // -------------------------------------
    //      0     |     0      | Normal polygons, or polygons for which shadow processing is not performed (in Intensity Volume mode)
    //      0     |     1      | Reserved
    //      1     |     0      | Polygons for which shadow processing is performed (in Intensity Volume mode)
    //      1     |     1      | Polygons in "with Two Volumes" format

    let shadow_bit = ((shading_instructions >> 22) & 1) == 1;
    let volume_bit = ((shading_instructions >> 24) & 1) == 1;

    let area1_texture_index: u32 = area1_texture_index_palette & 0xFFFF;
    let area1_palette_instructions: u32 = area1_texture_index_palette >> 16;

    if shadow_bit {
        if volume_bit { // Volume Bit
            output.area1 = area_color(
                area1_base_color,
                area1_offset_color,
                area1_uv,
                area1_duvdx,
                area1_duvdy,
                area1_texture_index,
                area1_texture_index_palette,
                area1_shading_instructions,
                z,
                punch_through
            );
        } else {
            // FIXME: This might not work properly for Decal and Decal Alpha.
            //        I think we're supposed to only multiple base_color and offset_color, not the tex color (which is included in this final output)?
            output.area1 = vec4<f32>(uniforms.fpu_shad_scale * output.area0.rgb, output.area0.a);
        }
    } else {
        output.area1 = output.area0;
    }

    return output;
}

