//! Field + robot configuration loaded from `table.toml`.

use serde::Deserialize;
use sim_protocol::{Pose2D, RobotKind};

#[derive(Debug, Deserialize, Clone)]
pub struct Config {
    pub field: FieldConfig,
    pub start_poses: StartPosesConfig,
    pub galipeur: RobotConfig,
    pub pami: RobotConfig,
    #[serde(default)]
    pub humans: HumansConfig,
}

#[derive(Debug, Deserialize, Clone)]
pub struct HumansConfig {
    /// Off by default. Turn on via `--humans` on the CLI or `enabled = true`
    /// in the TOML.
    #[serde(default)]
    pub enabled: bool,
    /// Spawn count picked uniformly in `[count_min, count_max]` at startup.
    #[serde(default = "default_count_min")]
    pub count_min: usize,
    #[serde(default = "default_count_max")]
    pub count_max: usize,
    /// Body height range (mm), picked uniformly per human.
    #[serde(default = "default_height_min")]
    pub body_height_min_mm: f32,
    #[serde(default = "default_height_max")]
    pub body_height_max_mm: f32,
    /// Walking speed range (mm/s), picked uniformly per human.
    #[serde(default = "default_speed_min")]
    pub walk_speed_min_mm_s: f32,
    #[serde(default = "default_speed_max")]
    pub walk_speed_max_mm_s: f32,
    /// Footprint bounds (mm) — width and length of the bbox.
    #[serde(default = "default_width_min")]
    pub width_min_mm: f32,
    #[serde(default = "default_width_max")]
    pub width_max_mm: f32,
    /// Margin outside the table in which humans can roam (mm).
    #[serde(default = "default_margin")]
    pub margin_mm: f32,
    /// RNG seed. `0` → non-deterministic (use wall-clock).
    #[serde(default)]
    pub seed: u64,
}

impl Default for HumansConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            count_min: default_count_min(),
            count_max: default_count_max(),
            body_height_min_mm: default_height_min(),
            body_height_max_mm: default_height_max(),
            walk_speed_min_mm_s: default_speed_min(),
            walk_speed_max_mm_s: default_speed_max(),
            width_min_mm: default_width_min(),
            width_max_mm: default_width_max(),
            margin_mm: default_margin(),
            seed: 0,
        }
    }
}

fn default_count_min() -> usize { 3 }
fn default_count_max() -> usize { 6 }
fn default_height_min() -> f32 { 1600.0 }
fn default_height_max() -> f32 { 1950.0 }
fn default_speed_min() -> f32 { 400.0 }
fn default_speed_max() -> f32 { 1200.0 }
fn default_width_min() -> f32 { 350.0 }
fn default_width_max() -> f32 { 550.0 }
fn default_margin() -> f32 { 1500.0 }

#[derive(Debug, Deserialize, Clone)]
pub struct FieldConfig {
    pub width_mm: u32,
    pub height_mm: u32,
    /// Height of the table above the real floor. Purely visual — it does
    /// not shift the simulation coordinate frame.
    #[serde(default)]
    pub stand_height_mm: f32,
    /// Optional "wallpaper" texture that covers the entire field
    /// `[0, 0] — [width_mm, height_mm]`. Obstacles with `use_playmat = true`
    /// show the crop of this texture that matches their AABB.
    #[serde(default)]
    pub playmat: Option<Playmat>,
    /// Axis-aligned rectangular obstacles with a vertical height. The
    /// ground itself is expected to be declared as an obstacle with
    /// `height_mm = 0` and `use_playmat = true`.
    #[serde(default)]
    pub obstacles: Vec<Obstacle>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Playmat {
    pub path: String,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Obstacle {
    #[serde(default)]
    pub name: String,
    /// `[x_min, y_min, x_max, y_max]` in millimeters.
    pub aabb_mm: [f32; 4],
    /// Vertical extent from the ground plane. `0.0` means a flat ground
    /// tile (rendered as a plane, invisible to any sensor above it).
    pub height_mm: f32,
    /// sRGB hex color `"#RRGGBB"` (leading `#` optional). Defaults to RAL 7032
    /// ("Kieselgrau"), the usual table-wall color.
    #[serde(default)]
    pub color: Option<String>,
    /// Surface finish. `"smooth"` (default) uses a flat color material;
    /// `"wood"` applies a procedurally-generated wood-grain texture
    /// modulated on the base color. Ignored if `texture` is set.
    #[serde(default)]
    pub finish: Finish,
    /// Optional path to a PNG image used as the surface texture. Path is
    /// relative to the cwd (typically the workspace root). Overrides `finish`.
    #[serde(default)]
    pub texture: Option<String>,
    /// If `true`, render this obstacle with the crop of `[field.playmat]`
    /// matching its AABB. Overrides `texture` and `finish`.
    #[serde(default)]
    pub use_playmat: bool,
}

#[derive(Debug, Deserialize, Clone, Copy, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Finish {
    #[default]
    Smooth,
    Wood,
}

/// RAL 7032 ≈ sRGB #CBC4B0 (203, 196, 176).
const RAL_7032: [f32; 3] = [203.0 / 255.0, 196.0 / 255.0, 176.0 / 255.0];

impl Obstacle {
    /// Return the 4 perimeter segments of the AABB as `[ax, ay, bx, by]`
    /// tuples suitable for ray-vs-segment raycasting.
    pub fn segments(&self) -> [[f32; 4]; 4] {
        let [x0, y0, x1, y1] = self.aabb_mm;
        [
            [x0, y0, x1, y0],
            [x1, y0, x1, y1],
            [x1, y1, x0, y1],
            [x0, y1, x0, y0],
        ]
    }

    /// Resolved sRGB color in [0.0, 1.0] per channel.
    pub fn srgb_color(&self) -> [f32; 3] {
        self.color
            .as_deref()
            .and_then(parse_hex_color)
            .unwrap_or(RAL_7032)
    }
}

fn parse_hex_color(s: &str) -> Option<[f32; 3]> {
    let s = s.strip_prefix('#').unwrap_or(s);
    if s.len() != 6 {
        return None;
    }
    let r = u8::from_str_radix(&s[0..2], 16).ok()?;
    let g = u8::from_str_radix(&s[2..4], 16).ok()?;
    let b = u8::from_str_radix(&s[4..6], 16).ok()?;
    Some([r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0])
}

impl FieldConfig {
    /// Segments visible to a horizontal beam at `sensor_height_mm`: an
    /// obstacle contributes its 4 perimeter segments iff its vertical
    /// extent reaches the beam (`height_mm >= sensor_height_mm`).
    pub fn obstacle_segments_visible_from(&self, sensor_height_mm: f32) -> Vec<[f32; 4]> {
        let mut out = Vec::with_capacity(self.obstacles.len() * 4);
        for o in &self.obstacles {
            if o.height_mm >= sensor_height_mm && sensor_height_mm > 0.0 {
                out.extend_from_slice(&o.segments());
            } else if sensor_height_mm == 0.0 && o.height_mm > 0.0 {
                // Fallback for sensors with unspecified (0) height — still
                // exclude flat ground tiles (height_mm == 0).
                out.extend_from_slice(&o.segments());
            }
        }
        out
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct StartPosesConfig {
    pub galipeur: TeamPoses,
    pub pami: TeamPoses,
}

#[derive(Debug, Deserialize, Clone)]
pub struct TeamPoses {
    pub blue: Pose2DToml,
    pub yellow: Pose2DToml,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Pose2DToml {
    pub x_mm: f32,
    pub y_mm: f32,
    pub theta_rad: f32,
}

impl From<Pose2DToml> for Pose2D {
    fn from(p: Pose2DToml) -> Self {
        Pose2D { x_mm: p.x_mm, y_mm: p.y_mm, theta_rad: p.theta_rad }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct RobotConfig {
    #[serde(default)]
    pub kinematics: Option<KinematicsConfig>,
    pub bbox: BBoxConfig,
    /// Height above the ground of the robot's main distance sensor
    /// (top lidar for galipeur, VLX for pami). Used to filter obstacles
    /// in the raycast — the sensor only "sees" obstacles at least this
    /// tall.
    #[serde(default)]
    pub lidar_height_mm: f32,
    /// Optional 3D model (visual glb + simplified collision primitives).
    /// When absent the robot renders as its `bbox` cuboid and the raycast
    /// uses the AABB silhouette.
    #[serde(default)]
    pub model: Option<RobotModel>,
    /// Neopixel fixtures wired to the robot's LED output. The order of
    /// this list defines the wire order (the first fixture's LEDs start
    /// at strip index 0, the next picks up where it left off, etc.).
    #[serde(default)]
    pub neopixels: Vec<NeopixelFixture>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum NeopixelFixture {
    /// Single LED at a robot-local position.
    Single {
        #[serde(default)]
        name: String,
        /// `[x_forward, y_left, z_up]` in robot body frame (mm).
        position_mm: [f32; 3],
        /// Visual size of the pixel (mm diameter).
        #[serde(default = "default_pixel_size_mm")]
        pixel_size_mm: f32,
    },
    /// Ring of `count` LEDs, evenly spaced around `center_mm` in the
    /// robot's XY plane (Y-forward convention matches the rest of the
    /// robot body frame).
    Ring {
        #[serde(default)]
        name: String,
        /// `[x_forward, y_left, z_up]` — centre of the ring (mm).
        center_mm: [f32; 3],
        radius_mm: f32,
        count: u32,
        /// Pixel 0 starts at this angle (rad, 0 = +X forward), rotating
        /// counter-clockwise (toward +Y).
        #[serde(default)]
        start_angle_rad: f32,
        #[serde(default = "default_pixel_size_mm")]
        pixel_size_mm: f32,
    },
}

fn default_pixel_size_mm() -> f32 {
    8.0
}

impl NeopixelFixture {
    pub fn count(&self) -> usize {
        match self {
            Self::Single { .. } => 1,
            Self::Ring { count, .. } => *count as usize,
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct RobotModel {
    /// PNG/glb path relative to the workspace cwd — loaded as a Bevy scene.
    #[serde(default)]
    pub visual: Option<String>,
    /// Union of 2D polygons (extruded) and cylinders forming the simplified
    /// collision / raycast shape.
    #[serde(default)]
    pub collision: Vec<CollisionPrimitive>,
    /// Reference to the Onshape source for regeneration. Informational at
    /// runtime; consumed by the `onshape_fetch` CLI.
    #[serde(default)]
    pub onshape: Option<OnshapeSource>,
    /// Optional `[x_forward, y_up, z_side]` translation (mm, robot body
    /// frame) applied to the visual scene on spawn. Use the `y_up` slot
    /// when the Onshape origin is below the chassis (wheel axle etc.) and
    /// the model ends up floating under the table.
    #[serde(default)]
    pub visual_offset_mm: [f32; 3],
}

#[derive(Debug, Deserialize, Clone)]
pub struct OnshapeSource {
    pub url: String,
    #[serde(default)]
    pub exclude: Vec<String>,
    #[serde(default)]
    pub include: Vec<String>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum CollisionPrimitive {
    Polygon {
        /// Vertices in robot-local millimetres, counter-clockwise.
        points_mm: Vec<[f32; 2]>,
        #[serde(default)]
        z_base_mm: f32,
        height_mm: f32,
    },
    Cylinder {
        center_mm: [f32; 2],
        radius_mm: f32,
        #[serde(default)]
        z_base_mm: f32,
        height_mm: f32,
    },
}

impl CollisionPrimitive {
    /// Whether a horizontal sensor beam at `sensor_z_mm` intersects this
    /// primitive vertically.
    pub fn visible_at(&self, sensor_z_mm: f32) -> bool {
        let (z0, h) = match self {
            Self::Polygon { z_base_mm, height_mm, .. } => (*z_base_mm, *height_mm),
            Self::Cylinder { z_base_mm, height_mm, .. } => (*z_base_mm, *height_mm),
        };
        sensor_z_mm >= z0 && sensor_z_mm <= z0 + h
    }

    /// 2D silhouette as `[ax, ay, bx, by]` segments in robot-local
    /// millimetres. Cylinders are approximated with an inscribed regular
    /// polygon with `cyl_n` sides (err on the side of slightly
    /// undersizing, so the raycast doesn't shoot through).
    pub fn to_segments_local(&self, cyl_n: usize) -> Vec<[f32; 4]> {
        match self {
            Self::Polygon { points_mm, .. } => {
                let n = points_mm.len();
                if n < 2 {
                    return Vec::new();
                }
                let mut out = Vec::with_capacity(n);
                for i in 0..n {
                    let [ax, ay] = points_mm[i];
                    let [bx, by] = points_mm[(i + 1) % n];
                    out.push([ax, ay, bx, by]);
                }
                out
            }
            Self::Cylinder { center_mm, radius_mm, .. } => {
                let n = cyl_n.max(3);
                let mut pts = Vec::with_capacity(n);
                for i in 0..n {
                    let a = (i as f32) * std::f32::consts::TAU / (n as f32);
                    pts.push([
                        center_mm[0] + radius_mm * a.cos(),
                        center_mm[1] + radius_mm * a.sin(),
                    ]);
                }
                let mut out = Vec::with_capacity(n);
                for i in 0..n {
                    let [ax, ay] = pts[i];
                    let [bx, by] = pts[(i + 1) % n];
                    out.push([ax, ay, bx, by]);
                }
                out
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polygon_segments_square() {
        let p = CollisionPrimitive::Polygon {
            points_mm: vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
            z_base_mm: 0.0,
            height_mm: 10.0,
        };
        let segs = p.to_segments_local(16);
        assert_eq!(segs.len(), 4);
        assert_eq!(segs[0], [-1.0, -1.0, 1.0, -1.0]);
    }

    #[test]
    fn cylinder_produces_n_segments() {
        let c = CollisionPrimitive::Cylinder {
            center_mm: [0.0, 0.0],
            radius_mm: 10.0,
            z_base_mm: 0.0,
            height_mm: 10.0,
        };
        let segs = c.to_segments_local(16);
        assert_eq!(segs.len(), 16);
    }

    #[test]
    fn visible_at_is_within_z_range() {
        let c = CollisionPrimitive::Cylinder {
            center_mm: [0.0, 0.0],
            radius_mm: 1.0,
            z_base_mm: 50.0,
            height_mm: 100.0,
        };
        assert!(!c.visible_at(0.0));
        assert!(c.visible_at(100.0));
        assert!(!c.visible_at(200.0));
    }
}

#[derive(Debug, Deserialize, Clone)]
#[serde(untagged)]
pub enum KinematicsConfig {
    Holo {
        velocities_to_consigns: [[f32; 3]; 3],
        encoders_to_position: [[f32; 3]; 3],
    },
    Diff {
        wheel_base_mm: f32,
        wheel_diameter_mm: f32,
        encoder_ticks_per_rev: u32,
        max_wheel_speed_mm_s: f32,
    },
}

#[derive(Debug, Deserialize, Clone)]
pub struct BBoxConfig {
    pub width_mm: f32,
    pub length_mm: f32,
    /// Height above the table top. Used for raycast filtering (so tall
    /// robots remain visible to high-mounted lidars of other robots).
    #[serde(default = "default_robot_height")]
    pub height_mm: f32,
}

fn default_robot_height() -> f32 { 300.0 }


impl Config {
    pub fn load(path: &str) -> std::io::Result<Self> {
        let text = std::fs::read_to_string(path)?;
        toml::from_str(&text)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }

    /// Return the default start pose for a given robot kind (blue team for now).
    pub fn default_start_for(&self, kind: RobotKind) -> Pose2D {
        match kind {
            RobotKind::Galipeur => self.start_poses.galipeur.blue.clone().into(),
            RobotKind::Pami => self.start_poses.pami.blue.clone().into(),
        }
    }
}
