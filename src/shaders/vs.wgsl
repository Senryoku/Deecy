
 @group(0) @binding(0) var<uniform> max_depth: vec4<f32>;
 
struct VertexOut {
     @builtin(position) position_clip: vec4<f32>,
     @location(0) color: vec4<f32>,
     @location(1) uv: vec2<f32>,
     @location(2) @interpolate(flat) tex: u32,
 }

@vertex
fn main(
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) tex: u32,
    @location(4) tex_size: vec2<u32>, // u16 doesn't exist in WGSL, apparently.
    @location(5) uv_offset: vec2<f32>, // TODO
) -> VertexOut {
    var output: VertexOut;
     
     // output.position_clip = vec4(position, 1.0) * conversion_matrix;
     
     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    let w = 1.0 / position.z;
    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    output.position_clip.z = (1.0 - position.z); // / max_depth; // IDK, polygons are supposed to be sorted by the CPU or the TA
    output.position_clip.w = 1.0;

     // Should we undo the projection? Not if we set w to 1.0, I think.
    // output.position_clip.z = w;
    // output.position_clip.w = w;
    // output.position_clip.x *= w;
    // output.position_clip.y *= w;

    output.color = color;
    if tex == 0xFFFFFFFF {
        output.uv = vec2<f32>(0, 0);
    } else {
        output.uv = vec2<f32>(tex_size) / 1024.0 * uv ; // Adjust for the actual texture size (We're storing everything in 1024x1024 texture for now.)
    }
    output.tex = tex;
    return output;
}