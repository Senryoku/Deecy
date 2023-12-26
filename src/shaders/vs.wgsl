
 @group(0) @binding(0) var<uniform> max_depth: vec4<f32>;
 
struct VertexOut {
     @builtin(position) position_clip: vec4<f32>,
     @location(0) color: vec4<f32>,
     @location(1) uv: vec2<f32>,
     @location(2) @interpolate(flat) tex: vec2<u32>,
 }

@vertex
fn main(
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) tex: vec2<u32>,
    @location(4) tex_size: vec2<u32>, // u16 doesn't exist in WGSL, apparently.
    @location(5) uv_offset: vec2<f32>, // TODO
) -> VertexOut {
    var output: VertexOut;

    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    output.position_clip.z = (1.0 / position.z) /  max_depth[0]; // Remap to the [0.0..1.0] range used by WGPU
    output.position_clip.w = 1.0;

    output.color = color;
    output.uv = vec2<f32>(tex_size) / 1024.0 * uv ; // Adjust for the actual texture size (We're storing everything in 1024x1024 texture for now.)
    output.tex = tex;

    return output;
}