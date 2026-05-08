use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use asserv::maths::XYA;
use flume::Sender;
use crate::led::{LedMessage, OpponentLedPixel};

/// Conservative adversary footprint. Eurobot opponent is Ø450mm;
/// the LD06 typically catches the thin antenna, so we inflate each
/// hit by the base radius to trip as soon as any part of the puck
/// would intersect the danger area.
pub const ADVERSARY_RADIUS_MM: f32 = 225.0;

/// Number of consecutive net hits required to trip must_stop.
/// Also used as saturation cap for the hit counter.
const TRIP_THRESHOLD: u8 = 3;

const LED_COUNT: usize = 40;
const SCAN_BUFFER_SIZE: usize = 512;

// ------------------------------------------------------------------
// ScanBuffer — ring buffer of raw lidar points for preflight checks
// ------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
struct ScanPoint {
    angle_deg: f32,
    distance_mm: u16,
    robot_pos: XYA,
}

struct ScanBuffer {
    points: [ScanPoint; SCAN_BUFFER_SIZE],
    write_idx: usize,
    len: usize,
}

impl ScanBuffer {
    fn new() -> Self {
        Self {
            points: [ScanPoint::default(); SCAN_BUFFER_SIZE],
            write_idx: 0,
            len: 0,
        }
    }

    fn push(&mut self, angle_deg: f32, distance_mm: u16, robot_pos: XYA) {
        self.points[self.write_idx] = ScanPoint { angle_deg, distance_mm, robot_pos };
        self.write_idx = (self.write_idx + 1) % SCAN_BUFFER_SIZE;
        if self.len < SCAN_BUFFER_SIZE {
            self.len += 1;
        }
    }

    fn iter(&self) -> impl Iterator<Item = &ScanPoint> + '_ {
        let start = if self.len < SCAN_BUFFER_SIZE {
            0
        } else {
            self.write_idx
        };
        (0..self.len).map(move |i| {
            &self.points[(start + i) % SCAN_BUFFER_SIZE]
        })
    }
}

struct LedAccumulator {
    pixels: [OpponentLedPixel; LED_COUNT],
    last_angle: f32,
    initialized: bool,
    sender: Sender<LedMessage>,
    /// Offset (degrees) from robot 0° to LED pixel 1.
    led_angle_offset: f32,
}

impl LedAccumulator {
    fn new(sender: Sender<LedMessage>, led_angle_offset: f32) -> Self {
        Self {
            pixels: [OpponentLedPixel::Off; LED_COUNT],
            last_angle: 0.0,
            initialized: false,
            sender,
            led_angle_offset,
        }
    }

    fn angle_to_led(&self, angle_deg: f32) -> usize {
        let a = (angle_deg + self.led_angle_offset).rem_euclid(360.0);
        ((a / 360.0 * LED_COUNT as f32) as usize).min(LED_COUNT - 1)
    }

    fn set_pixel(&mut self, idx: usize, state: OpponentLedPixel) {
        if (state as u8) > (self.pixels[idx] as u8) {
            self.pixels[idx] = state;
        }
    }

    /// Compute the zone baseline (which LEDs show the zone shape).
    fn zone_baseline(zone: &Zone, compiled: &CompiledZone) -> [OpponentLedPixel; LED_COUNT] {
        let mut pixels = [OpponentLedPixel::Off; LED_COUNT];
        match zone {
            Zone::Inactive => {}
            Zone::Cylinder { .. } => {
                for i in (0..LED_COUNT).step_by(3) {
                    pixels[i] = OpponentLedPixel::Zone;
                }
            }
            Zone::Corridor { .. } => {
                // Check at several distances to find which angles the corridor covers
                for i in 0..LED_COUNT {
                    let angle_rad = (i as f32 * 360.0 / LED_COUNT as f32).to_radians();
                    for &d in &[200.0_f32, 500.0, 1000.0, 1500.0] {
                        let x = d * angle_rad.cos();
                        let y = d * angle_rad.sin();
                        if !matches!(compiled.trip_state(x, y), TripState::Clear) {
                            pixels[i] = OpponentLedPixel::Zone;
                            break;
                        }
                    }
                }
            }
        }
        pixels
    }

    /// Check if a revolution wrapped, emit overlay and reset.
    fn check_revolution(&mut self, angle_deg: f32, zone: &Zone, compiled: &CompiledZone) {
        if !self.initialized {
            self.last_angle = angle_deg;
            self.initialized = true;
            self.pixels = Self::zone_baseline(zone, compiled);
            return;
        }
        // Detect wrap: current angle much smaller than last (crossed 360°→0°)
        if angle_deg < self.last_angle - 180.0 {
            // Revolution complete — send overlay and reset
            self.sender.send(LedMessage::OpponentOverlay(self.pixels)).ok();
            self.pixels = Self::zone_baseline(zone, compiled);
        }
        self.last_angle = angle_deg;
    }
}

// ------------------------------------------------------------------
// Zone
// ------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Zone {
    Inactive,
    /// Disk around robot origin (body frame). Used for pure rotation.
    Cylinder { radius_mm: f32 },
    /// Forward-facing rectangle in body frame. Used for translation.
    /// `direction_rad` is the heading of motion in body frame.
    /// `[0..stop_until_mm]` → Stop, `[stop_until_mm..length_mm]` → Slow.
    Corridor {
        half_width_mm: f32,
        length_mm: f32,
        direction_rad: f32,
        stop_until_mm: f32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TripState {
    Clear,
    Slow,
    Stop,
}

impl Zone {
    pub fn trip_state(&self, x_mm: f32, y_mm: f32, inflate_mm: f32) -> TripState {
        match *self {
            Zone::Inactive => TripState::Clear,
            Zone::Cylinder { radius_mm } => {
                let r = radius_mm + inflate_mm;
                if r > 0.0 && x_mm * x_mm + y_mm * y_mm <= r * r {
                    TripState::Stop
                } else {
                    TripState::Clear
                }
            }
            Zone::Corridor {
                half_width_mm,
                length_mm,
                direction_rad,
                stop_until_mm,
            } => {
                if half_width_mm <= 0.0 || length_mm <= 0.0 {
                    return TripState::Clear;
                }
                let (sin, cos) = direction_rad.sin_cos();
                let along = cos * x_mm + sin * y_mm;
                let lateral = -sin * x_mm + cos * y_mm;
                let lateral_clamped = lateral.clamp(-half_width_mm, half_width_mm);
                let dy = lateral - lateral_clamped;
                let inflate_sq = inflate_mm * inflate_mm;

                let stop_until = stop_until_mm.clamp(0.0, length_mm);
                let along_stop = along.clamp(0.0, stop_until);
                let dx_stop = along - along_stop;
                if dx_stop * dx_stop + dy * dy <= inflate_sq {
                    return TripState::Stop;
                }

                let along_full = along.clamp(0.0, length_mm);
                let dx_full = along - along_full;
                if dx_full * dx_full + dy * dy <= inflate_sq {
                    return TripState::Slow;
                }

                TripState::Clear
            }
        }
    }

    pub fn compile(self, inflate_mm: f32) -> CompiledZone {
        let (sin_dir, cos_dir) = match self {
            Zone::Corridor { direction_rad, .. } => direction_rad.sin_cos(),
            _ => (0.0, 0.0),
        };
        let max_dist_f32 = match self {
            Zone::Inactive => 0.0,
            Zone::Cylinder { radius_mm } => (radius_mm + inflate_mm).max(0.0),
            Zone::Corridor { half_width_mm, length_mm, .. } => {
                let h = (half_width_mm + inflate_mm).max(0.0);
                let l = (length_mm + inflate_mm).max(0.0);
                (l * l + h * h).sqrt()
            }
        };
        let max_dist_mm = if max_dist_f32 >= u16::MAX as f32 {
            u16::MAX
        } else {
            max_dist_f32.ceil() as u16
        };
        CompiledZone {
            inner: self,
            inflate_mm,
            inflate_sq_mm: inflate_mm * inflate_mm,
            max_dist_sq_mm: (max_dist_mm as u32).saturating_mul(max_dist_mm as u32),
            sin_dir,
            cos_dir,
        }
    }
}

// ------------------------------------------------------------------
// CompiledZone — pre-computed for hot-path
// ------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub struct CompiledZone {
    pub inner: Zone,
    pub inflate_mm: f32,
    pub inflate_sq_mm: f32,
    pub max_dist_sq_mm: u32,
    pub sin_dir: f32,
    pub cos_dir: f32,
}

impl CompiledZone {
    #[inline]
    pub fn distance_reject(&self, distance_mm: u16) -> bool {
        if matches!(self.inner, Zone::Inactive) {
            return true;
        }
        if distance_mm == 0 {
            return true;
        }
        let d = distance_mm as u32;
        d * d > self.max_dist_sq_mm
    }

    #[inline]
    pub fn trip_state(&self, x_mm: f32, y_mm: f32) -> TripState {
        match self.inner {
            Zone::Inactive => TripState::Clear,
            Zone::Cylinder { radius_mm } => {
                let r = radius_mm + self.inflate_mm;
                if r > 0.0 && x_mm * x_mm + y_mm * y_mm <= r * r {
                    TripState::Stop
                } else {
                    TripState::Clear
                }
            }
            Zone::Corridor {
                half_width_mm,
                length_mm,
                stop_until_mm,
                ..
            } => {
                if half_width_mm <= 0.0 || length_mm <= 0.0 {
                    return TripState::Clear;
                }
                let along = self.cos_dir * x_mm + self.sin_dir * y_mm;
                let lateral = -self.sin_dir * x_mm + self.cos_dir * y_mm;
                let lateral_clamped = lateral.clamp(-half_width_mm, half_width_mm);
                let dy = lateral - lateral_clamped;

                let stop_until = stop_until_mm.clamp(0.0, length_mm);
                let along_stop = along.clamp(0.0, stop_until);
                let dx_stop = along - along_stop;
                if dx_stop * dx_stop + dy * dy <= self.inflate_sq_mm {
                    return TripState::Stop;
                }

                let along_full = along.clamp(0.0, length_mm);
                let dx_full = along - along_full;
                if dx_full * dx_full + dy * dy <= self.inflate_sq_mm {
                    return TripState::Slow;
                }

                TripState::Clear
            }
        }
    }
}

// ------------------------------------------------------------------
// AtomicZone — lock-free single-producer / multi-consumer
// ------------------------------------------------------------------

const KIND_INACTIVE: u8 = 0;
const KIND_CYLINDER: u8 = 1;
const KIND_CORRIDOR: u8 = 2;

pub struct AtomicZone {
    kind: AtomicU8,
    f0: AtomicU32,
    f1: AtomicU32,
    f2: AtomicU32,
    f3: AtomicU32,
}

impl AtomicZone {
    pub const fn new() -> Self {
        Self {
            kind: AtomicU8::new(KIND_INACTIVE),
            f0: AtomicU32::new(0),
            f1: AtomicU32::new(0),
            f2: AtomicU32::new(0),
            f3: AtomicU32::new(0),
        }
    }

    pub fn store(&self, zone: Zone) {
        self.kind.store(KIND_INACTIVE, Ordering::Release);
        match zone {
            Zone::Inactive => {}
            Zone::Cylinder { radius_mm } => {
                self.f0.store(radius_mm.to_bits(), Ordering::Relaxed);
                self.kind.store(KIND_CYLINDER, Ordering::Release);
            }
            Zone::Corridor {
                half_width_mm,
                length_mm,
                direction_rad,
                stop_until_mm,
            } => {
                self.f0.store(half_width_mm.to_bits(), Ordering::Relaxed);
                self.f1.store(length_mm.to_bits(), Ordering::Relaxed);
                self.f2.store(direction_rad.to_bits(), Ordering::Relaxed);
                self.f3.store(stop_until_mm.to_bits(), Ordering::Relaxed);
                self.kind.store(KIND_CORRIDOR, Ordering::Release);
            }
        }
    }

    pub fn load(&self) -> Zone {
        match self.kind.load(Ordering::Acquire) {
            KIND_CYLINDER => Zone::Cylinder {
                radius_mm: f32::from_bits(self.f0.load(Ordering::Relaxed)),
            },
            KIND_CORRIDOR => Zone::Corridor {
                half_width_mm: f32::from_bits(self.f0.load(Ordering::Relaxed)),
                length_mm: f32::from_bits(self.f1.load(Ordering::Relaxed)),
                direction_rad: f32::from_bits(self.f2.load(Ordering::Relaxed)),
                stop_until_mm: f32::from_bits(self.f3.load(Ordering::Relaxed)),
            },
            _ => Zone::Inactive,
        }
    }
}

impl Default for AtomicZone {
    fn default() -> Self {
        Self::new()
    }
}

// ------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------

pub struct OpponentDetectionConf {
    pub table: TableConfig,
    pub led_angle_offset: f32,
    pub corridor_half_width_mm: f32,
    pub corridor_stop_until_mm: f32,
    pub rotation_radius_mm: f32,
}

// ------------------------------------------------------------------
// Detection mode
// ------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DetectionMode {
    Off = 0,
    OnTable = 1,
    Always = 2,
}

impl DetectionMode {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::OnTable,
            2 => Self::Always,
            _ => Self::Off,
        }
    }
}

// ------------------------------------------------------------------
// Table config
// ------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub struct TableConfig {
    pub width_mm: f32,
    pub height_mm: f32,
    pub margin_mm: f32,
}

impl TableConfig {
    fn point_on_table(&self, tx: f32, ty: f32) -> bool {
        let m = self.margin_mm;
        tx >= m && tx <= self.width_mm - m && ty >= m && ty <= self.height_mm - m
    }
}

// ------------------------------------------------------------------
// OpponentDetection
// ------------------------------------------------------------------

struct OpponentDetectionInner {
    zone: AtomicZone,
    must_stop: AtomicBool,
    hit_count: AtomicU8,
    mode: AtomicU8,
    robot_position: Mutex<XYA>,
    conf: OpponentDetectionConf,
    led_accum: Mutex<LedAccumulator>,
    scan_buffer: Mutex<ScanBuffer>,
}

#[derive(Clone)]
pub struct OpponentDetection {
    inner: Arc<OpponentDetectionInner>,
}

impl OpponentDetection {
    pub fn new(conf: OpponentDetectionConf, led_sender: Sender<LedMessage>) -> Self {
        let led_offset = conf.led_angle_offset;
        Self {
            inner: Arc::new(OpponentDetectionInner {
                zone: AtomicZone::new(),
                must_stop: AtomicBool::new(false),
                hit_count: AtomicU8::new(0),
                mode: AtomicU8::new(DetectionMode::Off as u8),
                robot_position: Mutex::new(XYA::new(0.0, 0.0, 0.0)),
                conf,
                led_accum: Mutex::new(LedAccumulator::new(led_sender, led_offset)),
                scan_buffer: Mutex::new(ScanBuffer::new()),
            }),
        }
    }

    // --- Configuration ---

    pub fn set_mode(&self, mode: DetectionMode) {
        self.inner.mode.store(mode as u8, Ordering::Relaxed);
        if matches!(mode, DetectionMode::Off) {
            self.inner.must_stop.store(false, Ordering::Relaxed);
            self.inner.hit_count.store(0, Ordering::Relaxed);
        }
    }

    pub fn mode(&self) -> DetectionMode {
        DetectionMode::from_u8(self.inner.mode.load(Ordering::Relaxed))
    }

    pub fn conf(&self) -> &OpponentDetectionConf {
        &self.inner.conf
    }

    // --- Zone (written by asserv callback) ---

    pub fn update_zone(&self, zone: Zone) {
        self.inner.zone.store(zone);
        if !matches!(zone, Zone::Inactive) {
            self.clear_stop();
        }
    }

    // --- Robot position (written by asserv loop) ---

    pub fn update_robot_position(&self, pos: XYA) {
        *self.inner.robot_position.lock().unwrap() = pos;
    }

    // --- Query side (read by strat wait loop) ---

    #[inline]
    pub fn must_stop(&self) -> bool {
        self.inner.must_stop.load(Ordering::Relaxed)
    }

    pub fn clear_stop(&self) {
        self.inner.must_stop.store(false, Ordering::Relaxed);
        self.inner.hit_count.store(0, Ordering::Relaxed);
    }

    // --- Preflight (called by asserv callback before accepting a trajectory segment) ---

    /// Check stored scan points against a candidate zone.
    /// Transforms each stored point: body(stored) → table → body(current), then tests.
    /// Returns `true` if the zone is clear, `false` if an opponent is detected.
    /// Sets `must_stop` when returning `false`.
    pub fn preflight(&self, zone: Zone) -> bool {
        let robot_pos = *self.inner.robot_position.lock().unwrap();
        let compiled = zone.compile(ADVERSARY_RADIUS_MM);
        let scan_buf = self.inner.scan_buffer.lock().unwrap();

        let cur_cos = robot_pos.a.cos();
        let cur_sin = robot_pos.a.sin();

        let mut hits = 0u32;
        for pt in scan_buf.iter() {
            // Stored body frame
            let angle_rad = pt.angle_deg.to_radians();
            let dist = pt.distance_mm as f32;
            let bx_old = dist * angle_rad.cos();
            let by_old = dist * angle_rad.sin();

            // Body(stored) → table
            let old_cos = pt.robot_pos.a.cos();
            let old_sin = pt.robot_pos.a.sin();
            let tx = pt.robot_pos.x + bx_old * old_cos - by_old * old_sin;
            let ty = pt.robot_pos.y + bx_old * old_sin + by_old * old_cos;

            // Table → body(current)
            let dx = tx - robot_pos.x;
            let dy = ty - robot_pos.y;
            let bx = cur_cos * dx + cur_sin * dy;
            let by = -cur_sin * dx + cur_cos * dy;

            if matches!(compiled.trip_state(bx, by), TripState::Stop) {
                hits += 1;
                if hits >= TRIP_THRESHOLD as u32 {
                    self.inner.must_stop.store(true, Ordering::Relaxed);
                    return false;
                }
            }
        }
        true
    }

    // --- Feed (called by sensors thread on each lidar packet, ~40Hz) ---

    /// Check a chunk of lidar points against the active zone.
    /// Points are `(angle_deg, distance_mm, intensity)` in body frame.
    /// The hit counter persists across calls — treat the stream as continuous.
    pub fn feed(&self, points: &[(f32, u16, u8)]) {
        let mode = self.mode();
        if matches!(mode, DetectionMode::Off) {
            return;
        }

        let zone = self.inner.zone.load();
        let compiled = if matches!(zone, Zone::Inactive) {
            None
        } else {
            Some(zone.compile(ADVERSARY_RADIUS_MM))
        };

        let robot_pos = *self.inner.robot_position.lock().unwrap();
        let on_table_filter = matches!(mode, DetectionMode::OnTable);

        let mut count = self.inner.hit_count.load(Ordering::Relaxed);
        let mut led_accum = self.inner.led_accum.lock().unwrap();
        let mut scan_buf = self.inner.scan_buffer.lock().unwrap();

        for &(angle_deg, distance_mm, _intensity) in points {
            if let Some(compiled) = &compiled {
                led_accum.check_revolution(angle_deg, &zone, compiled);
            }

            if distance_mm == 0 {
                continue;
            }

            // Store raw point for preflight (always, even when zone is Inactive)
            let angle_rad = angle_deg.to_radians();
            let dist = distance_mm as f32;
            let bx = dist * angle_rad.cos();
            let by = dist * angle_rad.sin();

            let is_on_table = if on_table_filter {
                let cos_a = robot_pos.a.cos();
                let sin_a = robot_pos.a.sin();
                let tx = robot_pos.x + bx * cos_a - by * sin_a;
                let ty = robot_pos.y + bx * sin_a + by * cos_a;
                self.inner.conf.table.point_on_table(tx, ty)
            } else {
                true
            };

            if is_on_table {
                scan_buf.push(angle_deg, distance_mm, robot_pos);
            }

            // Zone hit detection
            if let Some(compiled) = &compiled {
                let in_range = !compiled.distance_reject(distance_mm);
                let is_hit = in_range && is_on_table && matches!(compiled.trip_state(bx, by), TripState::Stop);

                if is_hit {
                    count = count.saturating_add(1).min(TRIP_THRESHOLD);
                } else {
                    count = count.saturating_sub(1);
                }
                if count >= TRIP_THRESHOLD {
                    self.inner.must_stop.store(true, Ordering::Relaxed);
                }

                let led_idx = led_accum.angle_to_led(angle_deg);
                if is_hit {
                    led_accum.set_pixel(led_idx, OpponentLedPixel::Hit);
                } else if is_on_table && in_range {
                    led_accum.set_pixel(led_idx, OpponentLedPixel::Detected);
                }
            }
        }

        self.inner.hit_count.store(count, Ordering::Relaxed);
    }

}

// ------------------------------------------------------------------
// Tests
// ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cylinder_trip() {
        let z = Zone::Cylinder { radius_mm: 250.0 };
        assert_eq!(z.trip_state(0.0, 0.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(100.0, 100.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(200.0, 200.0, 0.0), TripState::Clear);
    }

    #[test]
    fn cylinder_inflate() {
        let z = Zone::Cylinder { radius_mm: 250.0 };
        assert_eq!(z.trip_state(400.0, 0.0, 0.0), TripState::Clear);
        assert_eq!(z.trip_state(400.0, 0.0, ADVERSARY_RADIUS_MM), TripState::Stop);
    }

    #[test]
    fn corridor_stop_slow() {
        let z = Zone::Corridor {
            half_width_mm: 250.0,
            length_mm: 1500.0,
            direction_rad: 0.0,
            stop_until_mm: 600.0,
        };
        assert_eq!(z.trip_state(300.0, 0.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(900.0, 0.0, 0.0), TripState::Slow);
        assert_eq!(z.trip_state(1800.0, 0.0, 0.0), TripState::Clear);
    }

    #[test]
    fn corridor_lateral() {
        let z = Zone::Corridor {
            half_width_mm: 250.0,
            length_mm: 1500.0,
            direction_rad: 0.0,
            stop_until_mm: 1500.0,
        };
        assert_eq!(z.trip_state(500.0, 200.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(500.0, 400.0, 0.0), TripState::Clear);
    }

    #[test]
    fn compiled_distance_reject() {
        let z = Zone::Cylinder { radius_mm: 250.0 };
        let c = z.compile(ADVERSARY_RADIUS_MM);
        assert!(c.distance_reject(0));
        assert!(!c.distance_reject(300));
        assert!(c.distance_reject(1000));
    }

    #[test]
    fn compiled_trip_matches_zone() {
        let z = Zone::Corridor {
            half_width_mm: 200.0,
            length_mm: 1000.0,
            direction_rad: 0.5,
            stop_until_mm: 500.0,
        };
        let c = z.compile(ADVERSARY_RADIUS_MM);
        for (x, y) in [(300.0, 100.0), (0.0, 0.0), (800.0, 400.0), (2000.0, 0.0)] {
            assert_eq!(
                z.trip_state(x, y, ADVERSARY_RADIUS_MM),
                c.trip_state(x, y),
                "mismatch at ({x}, {y})"
            );
        }
    }

    #[test]
    fn atomic_roundtrip_cylinder() {
        let cell = AtomicZone::new();
        cell.store(Zone::Cylinder { radius_mm: 320.0 });
        match cell.load() {
            Zone::Cylinder { radius_mm } => assert!((radius_mm - 320.0).abs() < 1e-3),
            other => panic!("expected Cylinder, got {other:?}"),
        }
    }

    #[test]
    fn atomic_roundtrip_corridor() {
        let cell = AtomicZone::new();
        cell.store(Zone::Corridor {
            half_width_mm: 200.0,
            length_mm: 1500.0,
            direction_rad: -1.234,
            stop_until_mm: 800.0,
        });
        match cell.load() {
            Zone::Corridor { half_width_mm, length_mm, direction_rad, stop_until_mm } => {
                assert!((half_width_mm - 200.0).abs() < 1e-3);
                assert!((length_mm - 1500.0).abs() < 1e-3);
                assert!((direction_rad - -1.234).abs() < 1e-4);
                assert!((stop_until_mm - 800.0).abs() < 1e-3);
            }
            other => panic!("expected Corridor, got {other:?}"),
        }
    }

    #[test]
    fn atomic_default_inactive() {
        let cell = AtomicZone::new();
        assert!(matches!(cell.load(), Zone::Inactive));
    }

    #[test]
    fn table_filter() {
        let table = TableConfig { width_mm: 3000.0, height_mm: 2000.0, margin_mm: 50.0 };
        assert!(table.point_on_table(1500.0, 1000.0));
        assert!(!table.point_on_table(10.0, 1000.0));
        assert!(!table.point_on_table(2990.0, 1000.0));
        assert!(!table.point_on_table(1500.0, 1980.0));
    }

    fn make_det() -> OpponentDetection {
        let (tx, _rx) = flume::unbounded();
        let det = OpponentDetection::new(OpponentDetectionConf {
            table: TableConfig { width_mm: 3000.0, height_mm: 2000.0, margin_mm: 50.0 },
            led_angle_offset: 0.0,
            corridor_half_width_mm: 250.0,
            corridor_stop_until_mm: 600.0,
            rotation_radius_mm: 400.0,
        }, tx);
        det.set_mode(DetectionMode::Always);
        det.update_zone(Zone::Cylinder { radius_mm: 400.0 });
        det
    }

    /// A point at (angle=0°, distance=300mm) is inside a 400mm+225mm cylinder.
    fn hit_point() -> (f32, u16, u8) {
        (0.0, 300, 100)
    }

    /// A point at (angle=0°, distance=5000mm) is far outside.
    fn miss_point() -> (f32, u16, u8) {
        (0.0, 5000, 100)
    }

    #[test]
    fn single_noise_point_no_trip() {
        let det = make_det();
        // 1 hit surrounded by misses
        let mut points = vec![miss_point(); 12];
        points[6] = hit_point();
        det.feed(&points);
        assert!(!det.must_stop(), "single noise point should not trip");
    }

    #[test]
    fn two_noise_points_no_trip() {
        let det = make_det();
        let mut points = vec![miss_point(); 12];
        points[5] = hit_point();
        points[6] = hit_point();
        det.feed(&points);
        assert!(!det.must_stop(), "two consecutive noise points should not trip");
    }

    #[test]
    fn three_consecutive_hits_trip() {
        let det = make_det();
        let mut points = vec![miss_point(); 12];
        points[4] = hit_point();
        points[5] = hit_point();
        points[6] = hit_point();
        det.feed(&points);
        assert!(det.must_stop(), "three consecutive hits should trip");
    }

    #[test]
    fn trip_latched_across_feeds() {
        let det = make_det();
        // Trip with 3 hits
        det.feed(&[hit_point(); 3]);
        assert!(det.must_stop());
        // Feed all misses — must_stop stays latched
        det.feed(&[miss_point(); 12]);
        assert!(det.must_stop(), "must_stop should stay latched");
    }

    #[test]
    fn clear_stop_resets() {
        let det = make_det();
        det.feed(&[hit_point(); 3]);
        assert!(det.must_stop());
        det.clear_stop();
        assert!(!det.must_stop());
        // Counter is also reset, so misses don't re-trip
        det.feed(&[miss_point(); 12]);
        assert!(!det.must_stop());
    }

    #[test]
    fn hits_accumulate_across_feeds() {
        let det = make_det();
        // 2 hits, then miss, then 2 hits in next feed
        det.feed(&[hit_point(), hit_point(), miss_point()]);
        assert!(!det.must_stop(), "only 2 hits minus 1 miss = 1, not enough");
        // Counter is at 1, feed 2 more hits
        det.feed(&[hit_point(), hit_point()]);
        assert!(det.must_stop(), "accumulated to 3, should trip");
    }
}
