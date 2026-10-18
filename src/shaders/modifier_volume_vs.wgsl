struct Uniforms {
    depth_min: f32,
    depth_max: f32,
    framebuffer_width: f32,
    framebuffer_height: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
 
struct VertexOut {
    @builtin(position) position_clip: vec4<f32>,
    @location(0) @interpolate(flat) volume_index: u32,
 }

@vertex
fn main(
    @location(0) position: vec4<f32>,
    @builtin(instance_index) instance_index: u32
) -> VertexOut {
    var output: VertexOut;
    output.position_clip = to_position_clip(position.xyz, uniforms.depth_max);
    output.volume_index = instance_index;
    return output;
}