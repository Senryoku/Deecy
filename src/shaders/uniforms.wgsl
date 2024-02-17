struct Uniforms {
    depth_min: f32,
    depth_max: f32,
	_padding: f32,
    fog_density: f32, // Should be a f16? 
    fog_col_pal: vec4<f32>,
    fog_col_vert: vec4<f32>,
    fog_lut: array<vec4<u32>, 0x80 / 4>, // actually 2 * 8bits; vec4 for alignment issues.
};
