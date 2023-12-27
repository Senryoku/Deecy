
 @group(0) @binding(0) var<uniform> max_depth: vec4<f32>;
 
struct VertexOut {
     @builtin(position) position_clip: vec4<f32>,
     @location(0) color: vec4<f32>,
     @location(1) uv: vec2<f32>,
     @location(2) @interpolate(flat) tex: vec2<u32>,
 }

fn tex_size(idx: u32) -> f32 {
    switch(idx & 7)  {
        case 0: { return 8.0; }
        case 1: { return 16.0; }
        case 2: { return 32.0; }
        case 3: { return 64.0; }
        case 4: { return 128.0; }
        case 5: { return 256.0; }
        case 6: { return 512.0; }
        case 7: { return 1024.0; }
        default: { return 8.0; }
    }
}

@vertex
fn main(
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) tex: vec2<u32>, // Texture index and Texture control word
    @location(4) uv_offset: vec2<f32>, // TODO
) -> VertexOut {
    var output: VertexOut;

    let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.

     // Positions are supplied in screen space (0..640, 0..480)
     // Convert it to wgpu clip space (-1..1, -1..1)
    output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
    output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;

    output.position_clip.z = (1.0 / position.z) / max_depth[0]; // Remap to the [0.0..1.0] range used by WGPU
    output.position_clip.w = 1.0;

    output.color = color;

    let u_size: f32 = tex_size((tex[1] >> 4) & 7);
    let v_size: f32 = tex_size((tex[1] >> 7) & 7);

    output.uv = vec2<f32>(1.0, v_size / u_size) * uv ;
    output.tex = tex;

    return output;
}