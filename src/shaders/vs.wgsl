
struct Uniforms {
    depth_min: f32,
    depth_max: f32, 
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
 
struct VertexOut {
     @builtin(position) position_clip: vec4<f32>,
     @location(0) base_color: vec4<f32>,
     @location(1) offset_color: vec4<f32>,
     @location(2) uv: vec2<f32>,
     @location(3) w: f32,
     @location(4) @interpolate(flat) tex: vec2<u32>,
 }

@vertex
fn main(
    @location(0) position: vec3<f32>,
    @location(1) base_color: vec4<f32>,
    @location(2) offset_color: vec4<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) tex: vec2<u32>, // Texture index and Texture control word
) -> VertexOut {
    var output: VertexOut;

    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    output.position_clip.z = (1.0 / position.z) / uniforms.depth_max; // Remap to the [0.0..1.0] range used by WGPU
    output.position_clip.w = 1.0;

    let w = position.z;
    
    output.base_color = w * base_color;
    output.offset_color = w * offset_color;

    output.uv = w * uv;
    output.w = w;
    output.tex = tex;

    return output;
}