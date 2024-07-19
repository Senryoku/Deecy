struct OITUniforms {
    max_fragments: u32,
    target_width: u32,
    start_y: u32,
};

struct LinkedListElement {
  color: u32,
  depth: f32,
  index_and_blend_mode: u32,
  next: u32,
};

struct LinkedList {
  data: array<LinkedListElement>
};
