// Takes a list of fragment generated by rendering a closed translucent modifier volume and
// add them to the list of already rendered segments, merging them if necessary.

struct Heads {
  fragment_count: u32,
  data: array<u32>
};

@group(0) @binding(0) var<uniform> oit_uniforms: OITUniforms;
@group(0) @binding(1) var<storage, read_write> modvol_heads: Heads;
@group(0) @binding(2) var<storage, read_write> modvol_linked_list: VolumeLinkedList;
@group(0) @binding(3) var<storage, read_write> out_modvols: array<Volumes>;

const MaxFragments = 16u;

struct VolumeLinkedListElementData {
  depth: f32,
  volume_index: u32,
};

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let heads_index = global_id.y * oit_uniforms.target_width + global_id.x;

    var frag_index = modvol_heads.data[heads_index];
    // Reset the heads buffer for the next pass.
    modvol_heads.data[heads_index] = 0xFFFFFFFFu;
    if all(global_id == vec3<u32>(0, 0, 0)) {
        modvol_heads.fragment_count = 0;
    }

    // Group the Modifier Volume fragments by volume and sort by depth.
    var fragments: array<VolumeLinkedListElementData, MaxFragments>;
    var frag_count = 0u;
    while frag_index != 0xFFFFFFFFu && frag_count < MaxFragments {
        var to_insert = VolumeLinkedListElementData(modvol_linked_list.data[frag_index].depth, modvol_linked_list.data[frag_index].volume_index);
        frag_index = modvol_linked_list.data[frag_index].next;

        var j = frag_count;

        // Look back into the sorted array until we find where we should insert the new fragment, moving up previous fragment as needed.
        while j > 0u && (to_insert.volume_index < fragments[j - 1u].volume_index || (to_insert.volume_index == fragments[j - 1u].volume_index && to_insert.depth < fragments[j - 1u].depth)) {
            fragments[j] = fragments[j - 1u];
            j--;
        }

        fragments[j] = to_insert;

        frag_count++;
    }

    var volumes = Volumes();
    volumes.count = 0;

    var curr_fragment = 0u;
    while curr_fragment < frag_count {
        let start_fragment = curr_fragment;
        let curr_volume = fragments[start_fragment].volume_index;
        curr_fragment++;
        while curr_fragment < frag_count && fragments[curr_fragment].volume_index == curr_volume {
            curr_fragment++;
        }
        let fragment_count = curr_fragment - start_fragment;

        // Convert the sorted list of depth values into a list of intervals
        var new_volumes = Volumes();
        new_volumes.count = fragment_count / 2;
        for (var i = 0u; i < new_volumes.count; i++) {
            new_volumes.intervals[i] = vec2<f32>(fragments[start_fragment + 2 * i].depth, fragments[start_fragment + 2 * i + 1].depth);
        }
        if fragment_count % 2 == 1 { // Last volume is open (backside behind the depth plane)
            new_volumes.intervals[new_volumes.count] = vec2<f32>(fragments[start_fragment + 2 * new_volumes.count].depth, 10.0);
            new_volumes.count++;
	    }

        volumes = volume_union(new_volumes, volumes);
    }

    out_modvols[heads_index] = volumes;
}

fn volume_union(a: Volumes, b: Volumes) -> Volumes {
    if a.count == 0 { return b; }
    if b.count == 0 { return a; }

    var union_volumes = Volumes();

	// Insert and Merge volumes
    var a_index = 0u;
    var b_index = 0u;
    if a.intervals[0].x < b.intervals[0].x {
        union_volumes.intervals[0] = a.intervals[0];
        a_index++;
    } else {
        union_volumes.intervals[0] = b.intervals[0];
        b_index++;
    }
    union_volumes.count = 1u;

    var to_insert: vec2<f32>;
    while a_index < a.count || b_index < b.count {
        if a_index >= a.count {
            to_insert = b.intervals[b_index];
            b_index++;
        } else if b_index >= b.count {
            to_insert = a.intervals[a_index];
            a_index++;
        } else {
            if a.intervals[a_index].x < b.intervals[b_index].x {
                to_insert = a.intervals[a_index];
                a_index++;
            } else {
                to_insert = b.intervals[b_index];
                b_index++;
            }
        }

        if to_insert.x <= union_volumes.intervals[union_volumes.count - 1].y {
			// Merge
            union_volumes.intervals[union_volumes.count - 1].y = max(to_insert.y, union_volumes.intervals[union_volumes.count - 1].y);
        } else {
            union_volumes.intervals[union_volumes.count] = to_insert;
            union_volumes.count++;
        }

        if union_volumes.count >= MaxVolumes {
            return union_volumes;
        }
    }

    return union_volumes;
} 