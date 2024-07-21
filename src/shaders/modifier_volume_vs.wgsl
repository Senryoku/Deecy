struct Uniforms {
    depth_min: f32,
    depth_max: f32,
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

    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

    let inv_w = 1.0 / position.z;

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    output.position_clip.x = inv_w * (position.x * 2.0 / screen_size.x - 1.0);
    output.position_clip.y = inv_w * (position.y * -2.0 / screen_size.y + 1.0);
    output.position_clip.z = 1.0 / uniforms.depth_max;
    output.position_clip.w = inv_w;

    return output;
}