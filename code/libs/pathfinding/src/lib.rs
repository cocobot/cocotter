//! Pathfinding utilities
//!
//! A graph is built from a static list of nodes and edges.
//! Obstacles can be added or removed at runtime to block some nodes or edges.
//!
//! Internally, distances are stored as `i16` for performance and to avoid floating-point issues.
//! `PathGraph` methods accept nodes as references. If provided nodes are not from the graph,
//! behavior is undefined.
use std::collections::{HashSet, HashMap};
use amatheur::XY;


/// Graph node
pub struct PathNode {
    xy: (i16, i16),
    /// Range of neighbors in graph's `edges`
    neighbors_range: std::ops::Range<usize>,
}

/// Graph obstacle (circular)
pub struct PathObstacle {
    xy: (i16, i16),
    /// Radius, squared
    radius2: u32,
}

/// Graph use for pathfinding
pub struct PathGraph {
    /// Graph nodes (frozen)
    nodes: Vec<PathNode>,
    /// Graph edges, as a single list of neighbors (frozen)
    ///
    /// Concatenated list of all node neighbors.
    /// Each node references sub-range of this list.
    edges: Vec<usize>,
    /// Graph nodes, can be modified at runtime
    pub obstacles: Vec<PathObstacle>,
    /// Extra cost (internal distance) added to edge
    extra_edge_cost: u32,
}


/// Collect nodes and edges then build a graph
///
/// The node indexes will be preserved in the built graph.
/// Edges are not oriented. Duplicate edges will be ignored.
#[derive(Default)]
pub struct PathGraphBuilder {
    nodes: Vec<(XY, HashSet<usize>)>,
}


impl PathGraphBuilder {
    /// Create a new graph build
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a new node, return its index
    pub fn add_node(&mut self, xy: XY) -> usize {
        self.nodes.push((xy, HashSet::new()));
        self.nodes.len() - 1
    }

    /// Add a new edge, indexes must be valid
    pub fn add_edge(&mut self, index_a: usize, index_b: usize) {
        self.nodes.get_mut(index_a).expect("invalid node index").1.insert(index_b);
        self.nodes.get_mut(index_b).expect("invalid node index").1.insert(index_a);
    }

    /// Add a triangle grid with an horizontal symmetry
    ///
    /// Grid has the following layout:
    /// 
    ///  2 o---o---o
    ///     \ / \ /
    ///  1 --o---o--
    ///     / \ / \
    ///  0 o---o---o
    ///  -2 -1 0 1 2
    /// 
    ///
    /// Don't add nodes if there is already one closer than `radius / 2`.
    /// Add edges between grid nodes and existing nodes closer than `1.5 * radius`.
    pub fn add_triangle_grid(&mut self, x1: f32, y0: f32, y1: f32, radius: f32) {
        let dx = radius / 2.0;
        let dy = radius * core::f32::consts::FRAC_PI_3.sin();
        let ix_max = (x1 / dx) as i16;
        let iy_max = ((y1 - y0) / dy) as i16;

        let max_grid_count = ((2 * ix_max + 1) * (iy_max + 1) / 2 + 1) as usize;
        // Helper to iterate on "valid" grid nodes
        let iter_grid_pos = || {
            (-ix_max..=ix_max).flat_map(|ix| (0..=iy_max).map(move |iy| (ix, iy)))
                // Skip points in the middle of horizontal sides
                .filter(|(ix, iy)| (ix + iy) % 2 == 0)
        };

        // Keep track of added nodes, to built edges later on
        let mut added_nodes: HashMap<(i16, i16), usize> = HashMap::with_capacity(max_grid_count);

        // Add nodes
        let previous_nodes_len = self.nodes.len();
        self.nodes.reserve(max_grid_count);
        for (ix, iy) in iter_grid_pos() {
            let xy = XY::new(ix as f32 * dx, iy as f32 * dy);
            // Skip positions close to an existing node
            if self.closest_node(&xy, radius / 2.0).is_none() {
                added_nodes.insert((ix, iy), self.add_node(xy));
            }
        }

        // Add edges for grid neighbors   \ /
        // Check edges above and right     o---  
        for (ix, iy) in iter_grid_pos() {
            if let Some(index0) = added_nodes.get(&(ix, iy)) {
                for (dix, diy) in [(-1, 1), (1, 1), (2, 0)] {
                    if let Some(index1) = added_nodes.get(&(ix + dix, iy + diy)) {
                        self.add_edge(*index0, *index1);
                    }
                }
            }
        }

        // Add edges for grid nodes close to existing nodes
        let close_distance2 = (1.5 * radius) * (1.5 * radius);
        for prev_index in 0..previous_nodes_len {
            for new_index in previous_nodes_len..self.nodes.len() {
                if (self.nodes[prev_index].0 - self.nodes[new_index].0).length2() < close_distance2 {
                    self.add_edge(prev_index, new_index);
                }
            }
        }
    }

    /// Return index of node closest to given position, within given margin
    pub fn closest_node(&self, xy: &XY, margin: f32) -> Option<usize> {
        self.nodes.iter().enumerate()
            .map(|(index, (node, _))| (index, (xy - node).length2()))
            .min_by_key(|(_, dist2)| *dist2 as u32)
            .filter(|(_, dist2)| *dist2 <= margin * margin)
            .map(|(index, _)| index)
    }

    /// Build the graph
    pub fn build(self, extra_edge_cost: f32) -> PathGraph {
        let extra_edge_cost = mm_to_internal(extra_edge_cost) as u32;
        let edge_count = self.nodes.iter().map(|(_, neighbors)| neighbors.len()).sum();
        let mut nodes = Vec::with_capacity(self.nodes.len());
        let mut edges = Vec::with_capacity(edge_count);
        for (xy, neighbors) in self.nodes {
            let n0 = edges.len();
            edges.extend(neighbors);
            nodes.push(PathNode {
                xy: xy_mm_to_internal(&xy),
                neighbors_range: n0..edges.len(),
            });
        }

        PathGraph {
            nodes,
            edges,
            obstacles: vec![],
            extra_edge_cost,
        }
    }
}


impl PathGraph {
    /// Get node from given index
    pub fn get_node(&self, index: usize) -> Option<&PathNode> {
        self.nodes.get(index)
    }

    /// Find a path in the graph from `start` to `goal`
    pub fn find_path(&self, start: &PathNode, goal: &PathNode) -> Option<Vec<&PathNode>> {
        // Fail if start node is blocked
        if self.node_is_blocked(start) || self.node_is_blocked(goal) {
            return None;
        }

        #[derive(Clone, Copy, PartialEq, Eq)]
        enum NodeState {
            Open,
            Pending,
            Closed,
        }

        #[derive(Clone)]
        struct NodeInfo {
            state: NodeState,
            /// Index of most efficient previous node
            previous_index: Option<usize>,
            /// Cost from start to this node (distance)
            partial_cost: u32,
            /// Estimated total cost (distance)
            total_cost: u32,
        }

        impl NodeInfo {
            fn new() -> Self {
                Self {
                    state: NodeState::Pending,
                    previous_index: None,
                    partial_cost: u32::MAX,
                    total_cost: u32::MAX,
                }
            }
        }


        // Initalize node information
        let mut node_infos: Vec<NodeInfo> = vec![NodeInfo::new(); self.nodes.len()];
        let start_index = self.node_index(start);
        let goal_index = self.node_index(goal);
        {
            let start_info = &mut node_infos[start_index];
            start_info.state = NodeState::Open;
            start_info.partial_cost = 0;
            start_info.total_cost = self.estimated_edge_cost(start, goal);
        }

        loop {
            // Get the next node to process in the main loop
            let index = node_infos.iter().enumerate()
                .filter(|(_, info)| info.state == NodeState::Open)
                .min_by_key(|(_, info)| info.total_cost)?.0;

            if index == goal_index {
                // Solution found, rebuild the path
                let goal_to_start: Vec<usize> = std::iter::successors(Some(index), |i| node_infos[*i].previous_index).collect();
                let start_to_goal: Vec<&PathNode> = goal_to_start.into_iter().rev().map(|i| &self.nodes[i]).collect();
                return Some(start_to_goal);
            }

            let node = &self.nodes[index];
            node_infos[index].state = NodeState::Closed;
            let node_partial_cost = node_infos[index].partial_cost;  // Get a copy now, to avoid borrow checker issues

            for edge_index in node.neighbors_range.clone() {
                let neighbor_index = self.edges[edge_index];
                let neighbor = &self.nodes[neighbor_index];
                // Note: closed neighbors could be detected during init
                // But it's better to do it here, only for nodes that need it
                let neighbor_info = &mut node_infos[neighbor_index];
                if neighbor_info.state == NodeState::Pending {
                    if self.node_is_blocked(node) {
                        neighbor_info.state = NodeState::Closed;
                    }
                }
                if neighbor_info.state == NodeState::Closed {
                    continue;
                }
                if self.edge_is_blocked(node, neighbor) {
                    continue;
                }

                neighbor_info.state = NodeState::Open;  // May be a no-op
                let cost = node_partial_cost + self.estimated_edge_cost(node, neighbor);
                if cost < neighbor_info.partial_cost {
                    // Better origin path
                    neighbor_info.previous_index = Some(index);
                    neighbor_info.partial_cost = cost;
                    neighbor_info.total_cost = cost + self.estimated_edge_cost(neighbor, goal);
                }
            }
        }
    }

    /// Return the node closest to the given coordinates and not blocked
    pub fn nearest_node(&self, xy: XY) -> &PathNode {
        let target_xy = (mm_to_internal(xy.x), mm_to_internal(xy.y));
        self.nodes.iter()
            .filter(|node| !self.node_is_blocked(node))
            .min_by_key(|node| internal_distance2(target_xy, node.xy))
            .unwrap()  // Assume at least 1 node
    }

    /// Return true if given node is currently blocked by an obstacle
    fn node_is_blocked(&self, node: &PathNode) -> bool {
        self.obstacles.iter().any(|ob| ob.blocks_node(node))
    }

    /// Return true if an edge is currently blocked by an obstacle
    fn edge_is_blocked(&self, a: &PathNode, b: &PathNode) -> bool {
        self.obstacles.iter().any(|ob| ob.blocks_edge(a, b))
    }

    /// Return a node index from its reference
    ///
    /// If `node` does not point to an item of `self.nodes`, result is undefined.
    fn node_index(&self, node: &PathNode) -> usize {
        let first_ptr = self.nodes.as_ptr();
        let node_ptr = node as *const PathNode;
        unsafe { node_ptr.offset_from(first_ptr) as usize }
    }

    /// Compute the estimated distance from a node to another
    fn estimated_edge_cost(&self, from: &PathNode, to: &PathNode) -> u32 {
        let dist = internal_distance2(from.xy, to.xy).isqrt();
        dist + self.extra_edge_cost
    }
}


impl PathNode {
    pub fn xy(&self) -> XY {
        xy_mm_from_internal(&self.xy)
    }
}


impl PathObstacle {
    pub fn new(xy: &XY, radius: f32) -> Self {
        let r = mm_to_internal(radius) as u32;
        Self {
            xy: xy_mm_to_internal(xy),
            radius2: r * r,
        }
    }

    pub fn xy(&self) -> XY {
        xy_mm_from_internal(&self.xy)
    }

    /// Return true if obstacle blocks the given node
    fn blocks_node(&self, node: &PathNode) -> bool {
        if self.radius2 > 0 {
            internal_distance2(self.xy, node.xy) < self.radius2
        } else {
            false
        }
    }

    /// Return true if obstacle blocks the path between two given *unblocked* nodes
    ///
    /// Important: this method assumes edges are NOT blocked.
    fn blocks_edge(&self, a: &PathNode, b: &PathNode) -> bool {
        if self.radius2 <= 0 {
            return false;
        }
        let d2_ao = internal_distance2(a.xy, self.xy);
        let d2_ab = internal_distance2(a.xy, b.xy);
        // Early check: if AO > AB, no intersection
        // This is true because nodes are not blocked
        if d2_ao > d2_ab {
            return false;
        }
        // u is the distance along AB of the projection P, of O on AB, scaled by AB²
        // then: AP = AB * u / AB² = u / AB
        let u = {
            let dx_ab = a.xy.0.saturating_sub(b.xy.0) as i32;
            let dx_ao = a.xy.0.saturating_sub(self.xy.0) as i32;
            let dy_ab = a.xy.1.saturating_sub(b.xy.1) as i32;
            let dy_ao = a.xy.1.saturating_sub(self.xy.1) as i32;
            (dx_ab * dx_ao + dy_ab * dy_ao) as u32
        };
        // P must be in AB for an intersection: 0 < u < AB²
        // Note: we already handled cases where the A or B are in the obstacle.
        if u <= 0 || u >= d2_ab {
            return false;
        }
        // The obstacle intersects if:  OP² = AO² - u² / AB² < r²
        let u2_ab2 = ((u as u64 * u as u64) / d2_ab as u64) as u32;
        d2_ao - u2_ab2 < self.radius2
    }
}


/// Compute the squared distance between two internal positions
const fn internal_distance2(a: (i16, i16), b: (i16, i16)) -> u32 {
    let dx = a.0.saturating_sub(b.0) as i32;
    let dy = a.1.saturating_sub(b.1) as i32;
    (dx * dx + dy * dy) as u32
}

/// Convert a distance in millimetter to internal value
///
/// Out-of-range values are saturated.
const fn mm_to_internal(v: f32) -> i16 { v as i16 }
/// Convert a distance from internal value to common floating point value
const fn mm_from_internal(v: i16) -> f32 { v as f32 }

const fn xy_mm_to_internal(xy: &XY) -> (i16, i16) {
    (
        mm_to_internal(xy.x),
        mm_to_internal(xy.y),
    )
}

const fn xy_mm_from_internal(xy: &(i16, i16)) -> XY {
    XY {
        x: mm_from_internal(xy.0),
        y: mm_from_internal(xy.1),
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn grid_builder(size: usize) -> PathGraphBuilder {
        let mut builder = PathGraphBuilder::new();
        // Nodes
        for y in 0..size {
            for x in 0..size {
                builder.add_node(XY::new(x as f32 * 100.0, y as f32 * 100.0));
            }
        }
        // Horizontal edges
        for y in 0..size {
            for x in 0..size-1 {
                builder.add_edge(y * size + x, y * size + x + 1);
            }
        }
        // Vertical edges
        for y in 0..size-1 {
            for x in 0..size {
                builder.add_edge(y * size + x, (y + 1) * size + x);
            }
        }
        builder
    }

    fn grid_index(size: usize, x: usize, y: usize) -> usize {
        x + y * size
    }

    /// Find a path, convert the result to a vector of plain coordinates
    fn grid_path(graph: &PathGraph, start: usize, goal: usize) -> Option<Vec<(i32, i32)>> {
        let path = graph.find_path(graph.get_node(start).unwrap(), graph.get_node(goal).unwrap())?;
        let path = path.into_iter().map(|node| {
            let XY { x, y } = node.xy();
            ((x / 100.0) as i32, (y / 100.0) as i32)
        }).collect();
        Some(path)
    }

    /// Remove edges pointing to given nodes
    /// This method is only in tests, because real code should not let unsued nodes
    fn remove_edges_of(builder: &mut PathGraphBuilder, index: usize) {
        let neighbors: Vec<usize> = builder.nodes[index].1.drain().collect();
        for neighbor in neighbors {
            builder.nodes[neighbor].1.remove(&index);
        }
    }

    /// Print graph builder nodes as SVG
    ///
    /// Can be used with `cargo test -- --nocapture` for visualization.
    fn print_builder_svg(builder: &PathGraphBuilder) {
        println!(r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="-650 -50 1300 700">"#);
        println!(r#"  <defs>"#);
        println!(r#"    <style type="text/css"><![CDATA["#);
        println!(r#"      .node {{ fill: black; stroke: none; }}"#);
        println!(r#"      .edge {{ stroke: black; stroke-width: 2px; }}"#);
        println!(r#"    ]]></style>"#);
        println!(r#"  <defs>"#);
        println!(r#"]]></style></defs>"#);
        println!(r#"<g>"#);
        for (xy, _) in &builder.nodes {
            println!(r#"<circle class="node" cx="{}" cy="{}" r="20" />"#, xy.x, xy.y);
        }
        // Note: each edge will be added two times
        for (xy1, neighbors) in &builder.nodes {
            for neighbor in neighbors {
                let xy2 = builder.nodes[*neighbor].0;
                println!(r#"<line class="edge" x1="{}" y1="{}" x2="{}" y2="{}" />"#, xy1.x, xy1.y, xy2.x, xy2.y);
            }
        }
        println!(r#"</g></svg>"#);
    }


    #[test]
    fn test_basic_path() {
        //  o---o---o---o---o
        //  |   |   |   |   |
        //  o---o---G---o---o
        //  |               |
        //  o   o   o   o   o
        //  |               |
        //  o---o---o---S---o
        //  |   |   |   |   |
        //  o---o---o---o---o
        let mut builder = grid_builder(5);
        remove_edges_of(&mut builder, grid_index(5, 1, 2));
        remove_edges_of(&mut builder, grid_index(5, 2, 2));
        remove_edges_of(&mut builder, grid_index(5, 3, 2));

        let mut graph = builder.build(0.0);
        let start = grid_index(5, 3, 3);
        let goal = grid_index(5, 2, 1);

        // Check "normal" path
        let path1 = vec![(3, 3), (4, 3), (4, 2), (4, 1), (3, 1), (2, 1)];
        assert_eq!(Some(&path1), grid_path(&graph, start, goal).as_ref());

        // Add an obstacle on the top-right corner, but too small to block
        graph.obstacles.push(PathObstacle::new(&XY::new(400.0, 0.0), 95.0));
        assert_eq!(Some(&path1), grid_path(&graph, start, goal).as_ref());

        // Change the obstacle, it's now blocking the path
        graph.obstacles[0] = PathObstacle::new(&XY::new(400.0, 0.0), 105.0);
        let path2 = vec![(3, 3), (2, 3), (1, 3), (0, 3), (0, 2), (0, 1), (1, 1), (2, 1)];
        assert_eq!(Some(&path2), grid_path(&graph, start, goal).as_ref());

        // Add an obstacle blocking the path (not node) near the goal on the left
        graph.obstacles.push(PathObstacle::new(&XY::new(150.0, 110.0), 20.0));
        let path3 = vec![(3, 3), (2, 3), (1, 3), (0, 3), (0, 2), (0, 1), (1, 1), (1, 0), (2, 0), (2, 1)];
        assert_eq!(Some(&path3), grid_path(&graph, start, goal).as_ref());

        // Add another obstacle on the left, there is no valid path anymore
        graph.obstacles.push(PathObstacle::new(&XY::new(0.0, 0.0), 120.0));
        assert!(grid_path(&graph, start, goal).is_none());
    }


    #[test]
    fn test_triangle_grid() {
        let mut builder = PathGraphBuilder::new();
        builder.add_node(XY::new(0.0, 50.0));
        builder.add_node(XY::new(-150.0, 50.0));
        builder.add_triangle_grid(620.0, 0.0, 620.0, 100.0);
        print_builder_svg(&builder);
    }
}
