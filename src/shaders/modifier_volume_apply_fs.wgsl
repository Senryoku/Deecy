struct Uniforms {
    fpu_shad_scale: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var opaque_texture: texture_2d<f32>;

@fragment
fn main(
    @builtin(position) position: vec4<f32>,
) -> @location(0) vec4<f32> {
    let frag_coords = vec2<i32>(position.xy);
    let opaque = textureLoad(opaque_texture, frag_coords, 0);
    return uniforms.fpu_shad_scale * opaque;
}
