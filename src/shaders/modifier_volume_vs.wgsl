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

    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    if(uniforms.depth_max > uniforms.depth_min) {
        output.position_clip.z = ((1.0 / position.z) - uniforms.depth_min) / (uniforms.depth_max - uniforms.depth_min); // Remap to the [0.0..1.0] range used by WGPU
    } else {
        output.position_clip.z = (1.0 / position.z);
    }
    output.position_clip.w = 1.0;

    return output;
}