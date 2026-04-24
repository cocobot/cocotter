//! 2D ray-vs-segment raycasting against static walls.
//!
//! Kept simple and allocation-free: walls are flat [ax, ay, bx, by] tuples,
//! the ray is an (origin, unit direction) pair, distance returned in the
//! same units as the inputs (millimeters).

/// Return the distance along `(dx,dy)` to the closest intersection with the
/// segment `(ax,ay)-(bx,by)`, or `None` if the ray misses it.
///
/// Assumes `(dx,dy)` is a unit vector, then the returned value is the true
/// distance.
pub fn ray_segment_dist(
    ox: f32, oy: f32, dx: f32, dy: f32,
    ax: f32, ay: f32, bx: f32, by: f32,
) -> Option<f32> {
    let ex = bx - ax;
    let ey = by - ay;
    let det = ex * dy - dx * ey;
    if det.abs() < 1e-9 {
        return None; // ray parallel to segment
    }
    let s = (ex * (ay - oy) - ey * (ax - ox)) / det;
    let t = (dx * (ay - oy) - (ax - ox) * dy) / det;
    if s >= 0.0 && (0.0..=1.0).contains(&t) {
        Some(s)
    } else {
        None
    }
}

/// Raycast origin→direction against a list of wall segments; returns the
/// closest hit distance, clamped to `max_dist`.
pub fn raycast(
    origin: (f32, f32),
    angle_rad: f32,
    max_dist: f32,
    walls: &[[f32; 4]],
) -> f32 {
    let dx = angle_rad.cos();
    let dy = angle_rad.sin();
    let mut best = max_dist;
    for &[ax, ay, bx, by] in walls {
        if let Some(s) = ray_segment_dist(origin.0, origin.1, dx, dy, ax, ay, bx, by) {
            if s < best {
                best = s;
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_hits_wall_in_front() {
        // Room 3000 wide. Robot at (1500, 1000), facing +x. Wall at x=3000.
        let walls = [[3000.0, 0.0, 3000.0, 2000.0]];
        let d = raycast((1500.0, 1000.0), 0.0, 10_000.0, &walls);
        assert!((d - 1500.0).abs() < 0.1, "d={d}");
    }

    #[test]
    fn ray_misses_returns_max() {
        let walls = [[3000.0, 0.0, 3000.0, 2000.0]];
        let d = raycast((1500.0, 1000.0), std::f32::consts::PI, 10_000.0, &walls);
        assert!((d - 10_000.0).abs() < 0.1);
    }
}
