// Modified from https://webgpu.github.io/webgpu-samples/samples/A-buffer - BSD-3-Clause license 

struct Heads {
  fragment_count: u32, // No concurency here, no need for atomics. Although I'm not sure this has any impact :)
  data: array<u32>
};

@group(0) @binding(0) var<uniform> oit_uniforms: OITUniforms;
@group(0) @binding(1) var<storage, read_write> heads: Heads;
@group(0) @binding(2) var<storage, read_write> linked_list: LinkedList;
@group(0) @binding(3) var opaque_texture: texture_2d<f32>; // FIXME: Should be the same as output_texture, but WGPU doesn't support reading from storage textures.
@group(0) @binding(4) var output_texture: texture_storage_2d<bgra8unorm, write>;
@group(0) @binding(5) var<storage, read_write> modvols: array<VolumesInterfaces>;

fn get_blend_factor(factor: u32, src_a: f32, dst_a: f32) -> f32 {
    switch(factor & 7) {
    case 0: { return 0; }
    case 1: { return 1; }
    case 4: { return src_a; }
    case 5: { return 1.0 - src_a; }
    case 6: { return dst_a; }
    case 7: { return 1.0 - dst_a; }
    default: { return 1.0; }
  }
}

fn get_src_factor(factor: u32, src: vec4<f32>, dst: vec4<f32>) -> vec4<f32> {
    switch(factor & 7) {
    case 2: { return dst; }
    case 3: { return (1.0 - dst); }
    default: { return vec4<f32>(get_blend_factor(factor, src.a, dst.a)); }
  }
} 

fn get_dst_factor(factor: u32, src: vec4<f32>, dst: vec4<f32>) -> vec4<f32> {
    switch(factor & 7) {
    case 2: { return src; }
    case 3: { return (1.0 - src); }
    default: { return vec4<f32>(get_blend_factor(factor, src.a, dst.a)); }
  }
} 

// The maximum layers we can process for any pixel
const MaxFragments = 24u;

struct Fragment {
  depth: f32,
  index_and_blend_modes: u32,
  color_area0: u32,
  color_area1: u32,
}

var<workgroup> fragments: array<array<Fragment, MaxFragments>, 8 * 4>;

@compute @workgroup_size(8, 4, 1)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>, @builtin(local_invocation_index) local_idx: u32) {
    let heads_index = global_id.y * oit_uniforms.target_width + global_id.x;

    var element_index = heads.data[heads_index];

    if element_index != 0xFFFFFFFFu { 
      var frag_coords = global_id.xy;
      frag_coords.y += oit_uniforms.start_y;
      var color = textureLoad(opaque_texture, frag_coords, 0);
      color.a = 1.0;

      let fragment = linked_list.data[element_index];
      fragments[local_idx][0] = Fragment(
        fragment.depth,
        fragment.index_and_blend_modes,
        fragment.color_area0,
        fragment.color_area1,
      );
      element_index = linked_list.data[element_index].next;

      var layer_count = 1u;
      // Follow the linked list of fragments, inserting them in our sorted array as we go.
      while element_index != 0xFFFFFFFFu && layer_count < MaxFragments {
          let fragment = linked_list.data[element_index];
          element_index = fragment.next;

          let to_insert = Fragment(
            fragment.depth,
            fragment.index_and_blend_modes,
            fragment.color_area0,
            fragment.color_area1,
          );
          var j = layer_count;

          // Look back into the sorted array until we find where we should insert the new fragment, moving up previous fragments as needed.
          while j > 0u && (to_insert.depth < fragments[local_idx][j - 1u].depth || // If the depths are equal, use the draw order (vertex index) as a tie breaker.
              (to_insert.depth == fragments[local_idx][j - 1u].depth && (to_insert.index_and_blend_modes >> 12) < (fragments[local_idx][j - 1u].index_and_blend_modes >> 12))) {
              fragments[local_idx][j] = fragments[local_idx][j - 1u];
              j--;
          }

          fragments[local_idx][j] = to_insert;

          layer_count++;
      }

      var depth_interfaces_count = 0u;
      while(depth_interfaces_count < MaxVolumesInterfaces && modvols[heads_index].interfaces[depth_interfaces_count] > 0.0) {
        depth_interfaces_count++;
      }

      var curr_depth_interface = 0u;
      var use_area1 = false; // Start in area0

      // Blend the translucent fragments
      for (var i = 0u; i < layer_count; i++) {
          let fragment = fragments[local_idx][i];

          while curr_depth_interface < depth_interfaces_count && modvols[heads_index].interfaces[curr_depth_interface] < fragment.depth {
              // Crossed the interface between area0 and area1.
              use_area1 = !use_area1;
              curr_depth_interface++;
          }

          let src = unpack4x8unorm(select(fragment.color_area0, fragment.color_area1, use_area1));
          let dst = color;
          let blend_modes = extractBits(fragment.index_and_blend_modes, select(0u, 6u, use_area1), 6);
          color = src * get_src_factor(extractBits(blend_modes, 0, 3), src, dst) + dst * get_dst_factor(extractBits(blend_modes, 3, 3), src, dst);
          color = clamp(color, vec4<f32>(0.0), vec4<f32>(1.0));
      }

      textureStore(output_texture, frag_coords, color);
  }

  // Reset the heads buffer for the next pass.
  heads.data[heads_index] = 0xFFFFFFFFu;
  // Reset modifier volumes for the next pass.
  modvols[heads_index].interfaces[0] = -1.0;
  if all(global_id == vec3<u32>(0, 0, 0)) {
    heads.fragment_count = 0;
  }
}
