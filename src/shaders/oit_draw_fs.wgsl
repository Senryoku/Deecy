// https://webgpu.github.io/webgpu-samples/samples/A-buffer 

struct Uniforms {
    depth_min: f32,
    depth_max: f32, 
    max_fragments: u32,
    target_width: u32,
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var texture_array_8x8: texture_2d_array<f32>;
@group(0) @binding(2) var texture_array_16x16: texture_2d_array<f32>;
@group(0) @binding(3) var texture_array_32x32: texture_2d_array<f32>;
@group(0) @binding(4) var texture_array_64x64: texture_2d_array<f32>;
@group(0) @binding(5) var texture_array_128x128: texture_2d_array<f32>;
@group(0) @binding(6) var texture_array_256x256: texture_2d_array<f32>;
@group(0) @binding(7) var texture_array_512x512: texture_2d_array<f32>;
@group(0) @binding(8) var texture_array_1024x1024: texture_2d_array<f32>;

@group(1) @binding(0) var image_sampler: sampler;

struct Heads {
  fragment_count: atomic<u32>,
  data: array<atomic<u32>>
};

struct LinkedListElement {
  color: vec4<f32>,
  depth: f32,
  blend_mode: u32,
  next: u32
};

struct LinkedList {
  data: array<LinkedListElement>
};

@group(2) @binding(0) var opaque_depth_texture: texture_depth_2d;
@group(2) @binding(1) var<storage, read_write> heads: Heads;
@group(2) @binding(2) var<storage, read_write> linked_list: LinkedList;


fn tex_sample(uv: vec2<f32>, control: u32, index: u32) -> vec4<f32> {
    // textureSample can't be called in non-uniform context, because of this derivative, I guess.
    // This kinda feel like a hack, but works.
    let ddx = dpdx(uv);
    let ddy = dpdy(uv);
    switch((control >> 4) & 7)  {
        case 0: { return textureSampleGrad(texture_array_8x8, image_sampler, uv, index, ddx, ddy); }
        case 1: { return textureSampleGrad(texture_array_16x16, image_sampler, uv, index, ddx, ddy); }
        case 2: { return textureSampleGrad(texture_array_32x32, image_sampler, uv, index, ddx, ddy); }
        case 3: { return textureSampleGrad(texture_array_64x64, image_sampler, uv, index, ddx, ddy); }
        case 4: { return textureSampleGrad(texture_array_128x128, image_sampler, uv, index, ddx, ddy); }
        case 5: { return textureSampleGrad(texture_array_256x256, image_sampler, uv, index, ddx, ddy); }
        case 6: { return textureSampleGrad(texture_array_512x512, image_sampler, uv, index, ddx, ddy); }
        case 7: { return textureSampleGrad(texture_array_1024x1024, image_sampler, uv, index, ddx, ddy); }
        default: { return vec4<f32>(1.0, 0.0, 0.0, 1.0); } 
    }
}

@fragment
fn main(
    @builtin(position) position: vec4<f32>,
    @location(0) base_color: vec4<f32>,
    @location(1) offset_color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) @interpolate(flat) tex: vec2<u32>,
) {
    let frag_coords = vec2<i32>(position.xy);
    let opaque_depth = textureLoad(opaque_depth_texture, frag_coords, 0);

    let depth_compare = (tex[1] >> 16) & 0x7;

    // NOTE: Comparisons are inversed compared to Holly's 1/z depth.
    //       Also, the label denotes when the fragment is kept, not when it's discarded.
    switch(depth_compare) {
        case 0: { discard; } // Never
        case 1: { // Less
            if position.z <= opaque_depth { discard; }
        }
        case 2: { // Equal
            if position.z != opaque_depth { discard; }
        }
        case 3: { // Less or Equal
            if position.z < opaque_depth { discard; }
        }
        case 4: { // Greater
            if position.z >= opaque_depth { discard; }
        }
        case 5: { // Not Equal
            if position.z == opaque_depth { discard; }
        }
        case 6: { // Greater or Equal
            if position.z > opaque_depth { discard; }
        }
        case 7: {} // Always
        default: {}
    }
    // Normal rendering
    let tex_color = tex_sample(uv, tex[1], tex[0]);

    var final_color: vec4<f32>;

    if (tex[1] & 1) == 1 {
        let shading = (tex[1] >> 1) & 0x3;
        let ignore_tex_alpha = ((tex[1] >> 3) & 0x1) == 1;
        let tex_a = select(tex_color.a, 1.0, ignore_tex_alpha);
        switch(shading)  {
            // Decal
            case 0: {
                let rgb = tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                final_color = vec4<f32>(rgb, a);
            }
            // Modulate
            case 1 : {
                let rgb = base_color.rgb * tex_color.rgb + offset_color.rgb;
                let a = tex_a;
                final_color = vec4<f32>(rgb, a);
            }
            // Decal Alpha
            case 2 : {
                let rgb = tex_a * tex_color.rgb + (1.0 - tex_a) * base_color.rgb + offset_color.rgb;
                let a = base_color.a;
                final_color = vec4<f32>(rgb, a);
            }
            // Modulate Alpha
            case 3 : {
                let rgb = base_color.rgb * tex_color.rgb + offset_color.rgb;
                let a = base_color.a * tex_a;
                final_color = vec4<f32>(rgb, a);
            }
            default: { final_color = base_color + offset_color; }
        }
    } else {
        final_color = base_color + offset_color;
    }

    // Add the fragment to the linked list

    // The index in the heads buffer corresponding to the head data for the fragment at
    // the current location.
    let heads_index = u32(frag_coords.y) * uniforms.target_width + u32(frag_coords.x);
    
    // The index in the linkedList buffer at which to store the new fragment
    let frag_index = atomicAdd(&heads.fragment_count, 1u);

    // If we run out of space to store the fragments, we just lose them
    if frag_index < uniforms.max_fragments {
        let last_head = atomicExchange(&heads.data[heads_index], frag_index);
        linked_list.data[frag_index].depth = position.z;
        linked_list.data[frag_index].color = final_color;
        linked_list.data[frag_index].blend_mode = (tex[1] >> 10) & 0x3F;
        linked_list.data[frag_index].next = last_head;
    }
}