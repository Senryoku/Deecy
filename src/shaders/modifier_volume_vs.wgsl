struct Uniforms {
    depth_min: f32,
    depth_max: f32,
    framebuffer_width: f32,
    framebuffer_height: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
 
struct VertexOut {
     @builtin(position) position_clip: vec4<f32>,
 }

@vertex
fn main(
    @location(0) position: vec4<f32>,
) -> VertexOut {
    var output: VertexOut;
    output.position_clip = to_position_clip(position.xyz, uniforms.depth_max);
    return output;
}