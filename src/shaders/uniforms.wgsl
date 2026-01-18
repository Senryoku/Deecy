struct Uniforms {
    depth_min: f32,
    depth_max: f32,
    framebuffer_width: f32,
    framebuffer_height: f32,
    fpu_shad_scale: f32,
    fog_density: f32, // Should be a f16? 
    pt_alpha_ref: f32,  
    fog_col_pal: u32,
    fog_col_vert: u32,
    fog_clamp_min: u32,
    fog_clamp_max: u32,
    _padding: u32,
    fog_lut: array<vec4<u32>, 0x80 / 4>, // actually 2 * 8bits; vec4 for alignment issues.
};
