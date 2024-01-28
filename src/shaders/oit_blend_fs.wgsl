// https://webgpu.github.io/webgpu-samples/samples/A-buffer

struct Uniforms {
    depth_min: f32,
    depth_max: f32, 
    max_fragments: u32,
    target_width: u32,
};

struct Heads {
  fragment_count: u32, // No concurency here, no need for atomics.
  data: array<u32>
};

struct LinkedListElement {
  color: vec4<f32>,
  depth: f32,
  blend_mode: u32,
  next: u32,
};

struct LinkedList {
  data: array<LinkedListElement>
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var<storage, read_write> heads: Heads;
@group(0) @binding(2) var<storage, read_write> linked_list: LinkedList;
@group(0) @binding(3) var opaque_texture: texture_2d<f32>;

fn get_blend_factor(factor: u32, src: vec4<f32>, dst: vec4<f32>) -> vec4<f32> {
    switch(factor) {
    case 0: { return vec4<f32>(0); }
    case 1: { return vec4<f32>(1); }
    case 4: { return vec4<f32>(src.a); }
    case 5: { return vec4<f32>(1.0 - src.a); }
    case 6: { return vec4<f32>(dst.a); }
    case 7: { return vec4<f32>(1.0 - dst.a); }
    default: { return vec4<f32>(1.0, 0.0, 0.0, 1.0); }
  }
}

fn get_src_factor(factor: u32, src: vec4<f32>, dst: vec4<f32>) -> vec4<f32> {
    switch(factor) {
    case 2: { return dst; }
    case 3: { return (1.0 - dst); }
    default: { return get_blend_factor(factor, src, dst); }
  }
} 

fn get_dst_factor(factor: u32, src: vec4<f32>, dst: vec4<f32>) -> vec4<f32> {
    switch(factor) {
    case 2: { return src; }
    case 3: { return (1.0 - src); }
    default: { return get_blend_factor(factor, src, dst); }
  }
} 

@fragment
fn main(@builtin(position) position: vec4<f32>) -> @location(0) vec4<f32> {
    let frag_coords = vec2<i32>(position.xy);
    let heads_index = u32(frag_coords.y) * uniforms.target_width + u32(frag_coords.x);

    // The maximum layers we can process for any pixel
    const MaxLayers = 24u;

    var layers: array<LinkedListElement, MaxLayers>;

    var layer_count = 0u;
    var element_index = heads.data[heads_index];

    // copy the list elements into an array up to the maximum amount of layers
    while element_index != 0xFFFFFFFFu && layer_count < MaxLayers {
        layers[layer_count] = linked_list.data[element_index];
        layer_count++;
        element_index = linked_list.data[element_index].next;
    }

    if layer_count == 0u {
      discard;
    }
  
    // sort the fragments by depth
    for (var i = 1u; i < layer_count; i++) {
        let to_insert = layers[i];
        var j = i;

        while j > 0u && to_insert.depth > layers[j - 1u].depth {
            layers[j] = layers[j - 1u];
            j--;
        }

        layers[j] = to_insert;
    }

    var color = textureLoad(opaque_texture, frag_coords, 0);
    color.a = 1.0;

    // blend the remaining layers
    for (var i = 0u; i < layer_count; i++) {
        let src = layers[i].color;
        let dst = color;
        color = src * get_src_factor(layers[i].blend_mode & 7, src, dst) + dst * get_dst_factor((layers[i].blend_mode >> 3) & 7, src, dst);
    }

    return color;
}