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

struct ScanHalf {
    points: Vec<ScanPoint>,
}

impl ScanHalf {
    fn new() -> Self {
        Self {
            points: Vec::with_capacity(SCAN_BUFFER_SIZE),
        }
    }

    fn clear(&mut self) {
        self.points.clear();
    }

    fn push(&mut self, pt: ScanPoint) {
        if self.points.len() < SCAN_BUFFER_SIZE {
            self.points.push(pt);
        }
    }

    fn len(&self) -> usize {
        self.points.len()
    }

    fn iter(&self) -> impl Iterator<Item = &ScanPoint> + '_ {
        self.points.iter()
    }
}

/// Double-buffered scan: `writing` accumulates the current revolution,
/// `complete` holds the last full revolution for preflight reads.
struct ScanBuffer {
    complete: ScanHalf,
    writing: ScanHalf,
    last_angle: f32,
    initialized: bool,
}

impl ScanBuffer {
    fn new() -> Self {
        Self {
            complete: ScanHalf::new(),
            writing: ScanHalf::new(),
            last_angle: 0.0,
            initialized: false,
        }
    }

    fn push(&mut self, angle_deg: f32, distance_mm: u16, robot_pos: XYA) {
        // Detect revolution wrap: swap buffers
        if self.initialized && angle_deg > self.last_angle + 180.0 {
            std::mem::swap(&mut self.complete, &mut self.writing);
            self.writing.clear();
        }
        self.last_angle = angle_deg;
        self.initialized = true;

        self.writing.push(ScanPoint { angle_deg, distance_mm, robot_pos });
    }

    /// Iterate over the last complete revolution.
    fn iter(&self) -> impl Iterator<Item = &ScanPoint> + '_ {
        self.complete.iter()
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
    fn check_revolution(&mut self, angle_deg: f32, zone: &Zone, compiled: Option<&CompiledZone>) {
        if !self.initialized {
            self.last_angle = angle_deg;
            self.initialized = true;
            if let Some(compiled) = compiled {
                self.pixels = Self::zone_baseline(zone, compiled);
            }
            return;
        }
        // Detect wrap: current angle much smaller than last (crossed 360°→0°)
        if angle_deg > self.last_angle + 180.0 {
            // Revolution complete — send overlay and reset
            self.sender.send(LedMessage::OpponentOverlay(self.pixels)).ok();
            self.pixels = if let Some(compiled) = compiled {
                Self::zone_baseline(zone, compiled)
            } else {
                [OpponentLedPixel::Off; LED_COUNT]
            };
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
    pub fn trip_state(&self, x_mm: f32, y_mm: f32) -> TripState {
        match *self {
            Zone::Inactive => TripState::Clear,
            Zone::Cylinder { radius_mm } => {
                if radius_mm > 0.0 && x_mm * x_mm + y_mm * y_mm <= radius_mm * radius_mm {
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
                if along < 0.0 || along > length_mm {
                    return TripState::Clear;
                }
                let lateral = (-sin * x_mm + cos * y_mm).abs();
                if lateral > half_width_mm {
                    return TripState::Clear;
                }

                let stop_until = stop_until_mm.clamp(0.0, length_mm);
                if along <= stop_until {
                    TripState::Stop
                } else {
                    TripState::Slow
                }
            }
        }
    }

    pub fn compile(self) -> CompiledZone {
        let (sin_dir, cos_dir) = match self {
            Zone::Corridor { direction_rad, .. } => direction_rad.sin_cos(),
            _ => (0.0, 0.0),
        };
        let max_dist_f32 = match self {
            Zone::Inactive => 0.0,
            Zone::Cylinder { radius_mm } => radius_mm.max(0.0),
            Zone::Corridor { half_width_mm, length_mm, .. } => {
                (length_mm * length_mm + half_width_mm * half_width_mm).sqrt()
            }
        };
        let max_dist_mm = if max_dist_f32 >= u16::MAX as f32 {
            u16::MAX
        } else {
            max_dist_f32.ceil() as u16
        };
        CompiledZone {
            inner: self,
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
                if radius_mm > 0.0 && x_mm * x_mm + y_mm * y_mm <= radius_mm * radius_mm {
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
                if along < 0.0 || along > length_mm {
                    return TripState::Clear;
                }
                let lateral = (-self.sin_dir * x_mm + self.cos_dir * y_mm).abs();
                if lateral > half_width_mm {
                    return TripState::Clear;
                }

                let stop_until = stop_until_mm.clamp(0.0, length_mm);
                if along <= stop_until {
                    TripState::Stop
                } else {
                    TripState::Slow
                }
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
    /// Cruise speed cap applied when an opponent is in the Slow zone.
    pub slow_cruise_speed: f32,
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
        let half_w = self.width_mm / 2.0;
        tx >= -half_w + m && tx <= half_w - m && ty >= m && ty <= self.height_mm - m
    }
}

// ------------------------------------------------------------------
// OpponentDetection
// ------------------------------------------------------------------

/// Number of consecutive clean revolutions required to auto-clear must_slow.
const SLOW_CLEAR_REVOLUTIONS: u8 = 3;

struct SlowRevTracker {
    last_angle: f32,
    initialized: bool,
    /// Did this revolution have any slow hit?
    rev_had_slow: bool,
    /// Consecutive clean revolutions (no slow hits).
    clean_revs: u8,
}

impl SlowRevTracker {
    fn new() -> Self {
        Self { last_angle: 0.0, initialized: false, rev_had_slow: false, clean_revs: 0 }
    }

    fn reset(&mut self) {
        self.rev_had_slow = false;
        self.clean_revs = 0;
    }

    /// Call for each point. Returns true when a revolution boundary is crossed
    /// and `SLOW_CLEAR_REVOLUTIONS` consecutive clean revolutions have passed.
    fn update(&mut self, angle_deg: f32, had_slow_hit: bool) -> bool {
        if had_slow_hit {
            self.rev_had_slow = true;
        }

        if !self.initialized {
            self.last_angle = angle_deg;
            self.initialized = true;
            return false;
        }

        // Detect revolution wrap
        if angle_deg > self.last_angle + 180.0 {
            let had_slow = self.rev_had_slow;
            if self.rev_had_slow {
                self.clean_revs = 0;
            } else {
                self.clean_revs = self.clean_revs.saturating_add(1);
            }
            let should_clear = !had_slow && self.clean_revs >= SLOW_CLEAR_REVOLUTIONS;
            self.rev_had_slow = false;
            self.last_angle = angle_deg;
            return should_clear;
        }

        self.last_angle = angle_deg;
        false
    }
}

struct OpponentDetectionInner {
    zone: AtomicZone,
    must_stop: AtomicBool,
    hit_count: AtomicU8,
    must_slow: AtomicBool,
    slow_count: AtomicU8,
    mode: AtomicU8,
    robot_position: Mutex<XYA>,
    /// Target position for the current trajectory segment (table frame).
    /// Used to dynamically cap stop_until_mm based on remaining distance.
    target_x: AtomicU32,
    target_y: AtomicU32,
    conf: OpponentDetectionConf,
    led_accum: Mutex<LedAccumulator>,
    scan_buffer: Mutex<ScanBuffer>,
    /// Tracks slow-clear revolution logic (behind scan_buffer mutex for convenience).
    slow_rev: Mutex<SlowRevTracker>,
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
                must_slow: AtomicBool::new(false),
                slow_count: AtomicU8::new(0),
                mode: AtomicU8::new(DetectionMode::Off as u8),
                robot_position: Mutex::new(XYA::new(0.0, 0.0, 0.0)),
                target_x: AtomicU32::new(0),
                target_y: AtomicU32::new(0),
                conf,
                led_accum: Mutex::new(LedAccumulator::new(led_sender, led_offset)),
                scan_buffer: Mutex::new(ScanBuffer::new()),
                slow_rev: Mutex::new(SlowRevTracker::new()),
            }),
        }
    }

    // --- Configuration ---

    pub fn set_mode(&self, mode: DetectionMode) {
        self.inner.mode.store(mode as u8, Ordering::Relaxed);
        if matches!(mode, DetectionMode::Off) {
            self.inner.must_stop.store(false, Ordering::Relaxed);
            self.inner.hit_count.store(0, Ordering::Relaxed);
            self.inner.must_slow.store(false, Ordering::Relaxed);
            self.inner.slow_count.store(0, Ordering::Relaxed);
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

    /// Set the target position for dynamic stop zone capping.
    pub fn set_target(&self, x: f32, y: f32) {
        self.inner.target_x.store(x.to_bits(), Ordering::Relaxed);
        self.inner.target_y.store(y.to_bits(), Ordering::Relaxed);
    }

    // --- Robot position (written by asserv loop) ---

    pub fn update_robot_position(&self, pos: XYA) {
        log::info!("RPOS {:?}", pos);
        *self.inner.robot_position.lock().unwrap() = pos;
    }

    // --- Query side (read by strat wait loop) ---

    #[inline]
    pub fn must_stop(&self) -> bool {
        self.inner.must_stop.load(Ordering::Relaxed)
    }

    #[inline]
    pub fn must_slow(&self) -> bool {
        self.inner.must_slow.load(Ordering::Relaxed)
    }

    pub fn slow_cruise_speed(&self) -> f32 {
        self.inner.conf.slow_cruise_speed
    }

    pub fn clear_stop(&self) {
        self.inner.must_stop.store(false, Ordering::Relaxed);
        self.inner.hit_count.store(0, Ordering::Relaxed);
        self.inner.must_slow.store(false, Ordering::Relaxed);
        self.inner.slow_count.store(0, Ordering::Relaxed);
        self.inner.slow_rev.lock().unwrap().reset();
    }

    // --- Preflight (called by asserv callback before accepting a trajectory segment) ---

    /// Check stored scan points against a candidate zone.
    /// Transforms each stored point: body(stored) → table → body(current), then tests.
    /// Returns `true` if the zone is clear, `false` if an opponent is detected.
    /// Sets `must_stop` when returning `false`.
    pub fn preflight(&self, zone: Zone) -> bool {

        let mode = self.mode();
        if matches!(mode, DetectionMode::Off) {
            return true;
        }

        let robot_pos = *self.inner.robot_position.lock().unwrap();
        let compiled = zone.compile();
        let scan_buf = self.inner.scan_buffer.lock().unwrap();
        let on_table_filter = matches!(mode, DetectionMode::OnTable);

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

            // Filter: skip points outside table when mode is OnTable
            if on_table_filter && !self.inner.conf.table.point_on_table(tx, ty) {
                continue;
            }

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

        let robot_pos = *self.inner.robot_position.lock().unwrap();
        let mut zone = self.inner.zone.load();

        // Dynamically cap stop/slow zones based on remaining distance to target
        if let Zone::Corridor { ref mut stop_until_mm, ref mut length_mm, .. } = zone {
            let tx = f32::from_bits(self.inner.target_x.load(Ordering::Relaxed));
            let ty = f32::from_bits(self.inner.target_y.load(Ordering::Relaxed));
            let dx = tx - robot_pos.x;
            let dy = ty - robot_pos.y;
            let remaining = (dx * dx + dy * dy).sqrt() + ADVERSARY_RADIUS_MM;
            *stop_until_mm = stop_until_mm.min(remaining);
            *length_mm = length_mm.min(remaining * 1.5);
        }

        let compiled = if matches!(zone, Zone::Inactive) {
            None
        } else {
            Some(zone.compile())
        };
        let mut stop_count = self.inner.hit_count.load(Ordering::Relaxed);
        let mut slow_count = self.inner.slow_count.load(Ordering::Relaxed);
        let mut led_accum = self.inner.led_accum.lock().unwrap();
        let mut scan_buf = self.inner.scan_buffer.lock().unwrap();
        let mut slow_rev = self.inner.slow_rev.lock().unwrap();

        for &(angle_deg, distance_mm, _intensity) in points {
            led_accum.check_revolution(angle_deg, &zone, compiled.as_ref());

            // Track revolution for slow auto-clear (must see all angles, even distance=0)
            if compiled.is_some() && self.inner.must_slow.load(Ordering::Relaxed) {
                if slow_rev.update(angle_deg, false) {
                    self.inner.must_slow.store(false, Ordering::Relaxed);
                    slow_count = 0;
                    self.inner.slow_count.store(0, Ordering::Relaxed);
                }
            }

            if distance_mm == 0 {
                continue;
            }

            // Store raw point for preflight (always, even when zone is Inactive)
            let angle_rad = angle_deg.to_radians();
            let dist = distance_mm as f32;
            let bx = dist * angle_rad.cos();
            let by = dist * angle_rad.sin();

            let is_on_table = if distance_mm <= 3000 {
                let cos_a = robot_pos.a.cos();
                let sin_a = robot_pos.a.sin();
                let tx = robot_pos.x + bx * cos_a - by * sin_a;
                let ty = robot_pos.y + bx * sin_a + by * cos_a;
                self.inner.conf.table.point_on_table(tx, ty)
            } else {
                false
            };

            if is_on_table {
                scan_buf.push(angle_deg, distance_mm, robot_pos);
                let led_idx = led_accum.angle_to_led(angle_deg);
                led_accum.set_pixel(led_idx, OpponentLedPixel::Detected);
            }

            // Zone hit detection
            if let Some(compiled) = &compiled {
                let in_range = !compiled.distance_reject(distance_mm);
                let trip = if in_range && is_on_table {
                    compiled.trip_state(bx, by)
                } else {
                    TripState::Clear
                };

                // Stop counter: triggers on Stop hits
                let is_stop = matches!(trip, TripState::Stop);
                if is_stop {
                    stop_count = stop_count.saturating_add(1).min(TRIP_THRESHOLD);
                } else {
                    stop_count = stop_count.saturating_sub(1);
                }
                if stop_count >= TRIP_THRESHOLD {
                    self.inner.must_stop.store(true, Ordering::Relaxed);
                }

                // Slow counter: triggers on Stop OR Slow hits (anything in the corridor)
                let is_slow = matches!(trip, TripState::Stop | TripState::Slow);
                if is_slow {
                    slow_count = slow_count.saturating_add(1).min(TRIP_THRESHOLD);
                } else {
                    slow_count = slow_count.saturating_sub(1);
                }
                if slow_count >= TRIP_THRESHOLD && !self.inner.must_slow.load(Ordering::Relaxed) {
                    self.inner.must_slow.store(true, Ordering::Relaxed);
                }

                // Mark slow hits for revolution tracker
                if is_slow {
                    slow_rev.rev_had_slow = true;
                }

                // Upgrade LED pixel for zone hits
                let led_idx = led_accum.angle_to_led(angle_deg);
                if is_stop {
                    led_accum.set_pixel(led_idx, OpponentLedPixel::Hit);
                } else if is_slow {
                    led_accum.set_pixel(led_idx, OpponentLedPixel::Slow);
                }
            }
        }

        self.inner.hit_count.store(stop_count, Ordering::Relaxed);
        self.inner.slow_count.store(slow_count, Ordering::Relaxed);
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
        assert_eq!(z.trip_state(0.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(100.0, 100.0), TripState::Stop);
        assert_eq!(z.trip_state(200.0, 200.0), TripState::Clear);
    }

    #[test]
    fn cylinder_sized_for_adversary() {
        let z = Zone::Cylinder { radius_mm: 250.0 + ADVERSARY_RADIUS_MM };
        assert_eq!(z.trip_state(400.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(500.0, 0.0), TripState::Clear);
    }

    #[test]
    fn corridor_stop_slow() {
        let z = Zone::Corridor {
            half_width_mm: 250.0,
            length_mm: 1500.0,
            direction_rad: 0.0,
            stop_until_mm: 600.0,
        };
        assert_eq!(z.trip_state(300.0, 0.0), TripState::Stop);
        assert_eq!(z.trip_state(900.0, 0.0), TripState::Slow);
        assert_eq!(z.trip_state(1800.0, 0.0), TripState::Clear);
    }

    #[test]
    fn corridor_lateral() {
        let z = Zone::Corridor {
            half_width_mm: 250.0,
            length_mm: 1500.0,
            direction_rad: 0.0,
            stop_until_mm: 1500.0,
        };
        assert_eq!(z.trip_state(500.0, 200.0), TripState::Stop);
        assert_eq!(z.trip_state(500.0, 300.0), TripState::Clear);
    }

    #[test]
    fn corridor_behind() {
        let z = Zone::Corridor {
            half_width_mm: 250.0,
            length_mm: 1500.0,
            direction_rad: 0.0,
            stop_until_mm: 600.0,
        };
        assert_eq!(z.trip_state(-100.0, 0.0), TripState::Clear);
    }

    #[test]
    fn compiled_distance_reject() {
        let z = Zone::Cylinder { radius_mm: 250.0 };
        let c = z.compile();
        assert!(c.distance_reject(0));
        assert!(!c.distance_reject(200));
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
        let c = z.compile();
        for (x, y) in [(300.0, 100.0), (0.0, 0.0), (800.0, 400.0), (2000.0, 0.0)] {
            assert_eq!(
                z.trip_state(x, y),
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
        // X: -1500..1500, Y: 0..2000, margin 50
        let table = TableConfig { width_mm: 3000.0, height_mm: 2000.0, margin_mm: 50.0 };
        assert!(table.point_on_table(0.0, 1000.0));      // center
        assert!(table.point_on_table(1400.0, 1000.0));    // near right edge
        assert!(table.point_on_table(-1400.0, 1000.0));   // near left edge
        assert!(!table.point_on_table(1480.0, 1000.0));   // outside right margin
        assert!(!table.point_on_table(-1480.0, 1000.0));  // outside left margin
        assert!(!table.point_on_table(0.0, 1980.0));      // outside top margin
        assert!(!table.point_on_table(0.0, 10.0));        // outside bottom margin
    }

    fn make_det() -> OpponentDetection {
        let (tx, _rx) = flume::unbounded();
        let det = OpponentDetection::new(OpponentDetectionConf {
            table: TableConfig { width_mm: 3000.0, height_mm: 2000.0, margin_mm: 50.0 },
            led_angle_offset: 0.0,
            corridor_half_width_mm: 250.0,
            corridor_stop_until_mm: 600.0,
            rotation_radius_mm: 400.0,
            slow_cruise_speed: 1.0,
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
