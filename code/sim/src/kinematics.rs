//! Kinematic integration for robots.
//!
//! "Perfect asserv" assumption: we don't model motor physics. We take the
//! PWM consigns the robot sent, invert the robot's `velocities_to_consigns`
//! matrix to recover the commanded body velocity, integrate over `dt`, then
//! synthesize consistent encoder deltas via the `encoders_to_position`
//! inverse so that the robot's asserv closes the loop as it would with a
//! real (noiseless, instant-response) actuator.

use sim_protocol::Pose2D;

pub type Matrix3 = [[f32; 3]; 3];

/// Compute the inverse of a 3x3 matrix. Returns `None` if singular.
pub fn inverse3(m: Matrix3) -> Option<Matrix3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let mut inv = [[0.0f32; 3]; 3];
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    Some(inv)
}

pub fn mat_vec3(m: Matrix3, v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Holonomic state + kinematics for a single robot.
pub struct HoloState {
    pub pose: Pose2D,
    /// velocities_to_consigns: consigns = M @ [vx, vy, va]
    consigns_from_vel: Matrix3,
    /// inverse: [vx, vy, va] = M^-1 @ consigns
    vel_from_consigns: Matrix3,
    /// encoders_to_position: [dx_robot, dy_robot, da] = M @ encoder_deltas
    pos_from_enc: Matrix3,
    /// inverse: encoder_deltas = M^-1 @ [dx_robot, dy_robot, da]
    enc_from_pos: Matrix3,
}

impl HoloState {
    pub fn new(
        start: Pose2D,
        velocities_to_consigns: Matrix3,
        encoders_to_position: Matrix3,
    ) -> Self {
        let vel_from_consigns = inverse3(velocities_to_consigns)
            .expect("velocities_to_consigns is singular");
        let enc_from_pos = inverse3(encoders_to_position)
            .expect("encoders_to_position is singular");
        Self {
            pose: start,
            consigns_from_vel: velocities_to_consigns,
            vel_from_consigns,
            pos_from_enc: encoders_to_position,
            enc_from_pos,
        }
    }

    /// Integrate one step. Returns (encoder_delta, gyro_delta) consistent
    /// with the new pose.
    ///
    /// PWM consigns are clamped to `[-4095, 4095]` before inversion (motor
    /// driver saturation). The recovered velocity is then clamped to
    /// realistic physical limits — without this the asserv's max-PWM output
    /// translates into supersonic "commanded" body velocity through the
    /// motors matrix.
    pub fn step(&mut self, consigns: [f32; 3], dt_s: f32) -> ([f32; 3], f32) {
        const MAX_LIN_MM_S: f32 = 1500.0; // realistic galipeur ceiling
        const MAX_ANG_RAD_S: f32 = 8.0;

        let c_clamped = [
            consigns[0].clamp(-4095.0, 4095.0),
            consigns[1].clamp(-4095.0, 4095.0),
            consigns[2].clamp(-4095.0, 4095.0),
        ];
        let v_body = mat_vec3(self.vel_from_consigns, c_clamped);
        let (mut vx, mut vy, mut va) = (v_body[0], v_body[1], v_body[2]);

        // Scale linear velocity uniformly if the norm exceeds MAX_LIN_MM_S.
        let v_norm = (vx * vx + vy * vy).sqrt();
        if v_norm > MAX_LIN_MM_S {
            let k = MAX_LIN_MM_S / v_norm;
            vx *= k;
            vy *= k;
        }
        va = va.clamp(-MAX_ANG_RAD_S, MAX_ANG_RAD_S);

        // Robot-frame displacement over dt.
        let dx_r = vx * dt_s;
        let dy_r = vy * dt_s;
        let da = va * dt_s;

        // Rotate into world frame and update pose.
        let c = self.pose.theta_rad.cos();
        let s = self.pose.theta_rad.sin();
        self.pose.x_mm += c * dx_r - s * dy_r;
        self.pose.y_mm += s * dx_r + c * dy_r;
        self.pose.theta_rad += da;

        // Synthesize encoder deltas from the robot-frame displacement.
        let enc_delta = mat_vec3(self.enc_from_pos, [dx_r, dy_r, da]);
        (enc_delta, da)
    }

    #[allow(dead_code)]
    pub fn consigns_from_vel(&self) -> Matrix3 { self.consigns_from_vel }
    #[allow(dead_code)]
    pub fn pos_from_enc(&self) -> Matrix3 { self.pos_from_enc }
}

/// Differential-drive state for pami-like robots.
pub struct DiffState {
    pub pose: Pose2D,
    pub wheel_base_mm: f32,
    pub tick_to_mm: f32,
    /// Max physical wheel speed at consign=±1000.
    pub max_wheel_speed_mm_s: f32,
}

impl DiffState {
    pub fn new(
        start: Pose2D,
        wheel_base_mm: f32,
        wheel_diameter_mm: f32,
        encoder_ticks_per_rev: u32,
        max_wheel_speed_mm_s: f32,
    ) -> Self {
        let tick_to_mm = core::f32::consts::PI * wheel_diameter_mm / encoder_ticks_per_rev as f32;
        Self { pose: start, wheel_base_mm, tick_to_mm, max_wheel_speed_mm_s }
    }

    /// Consume `[left, right]` consigns (±1000 range) and return the pair
    /// of encoder deltas (ticks) corresponding to the integrated motion.
    pub fn step(&mut self, consigns: [f32; 2], dt_s: f32) -> [f32; 2] {
        let scale = self.max_wheel_speed_mm_s / 1000.0;
        let vl = consigns[0].clamp(-1000.0, 1000.0) * scale;
        let vr = consigns[1].clamp(-1000.0, 1000.0) * scale;

        let v_body = 0.5 * (vl + vr);
        let omega = (vr - vl) / self.wheel_base_mm;

        let c = self.pose.theta_rad.cos();
        let s = self.pose.theta_rad.sin();
        self.pose.x_mm += c * v_body * dt_s;
        self.pose.y_mm += s * v_body * dt_s;
        self.pose.theta_rad += omega * dt_s;

        let dl_mm = vl * dt_s;
        let dr_mm = vr * dt_s;
        [dl_mm / self.tick_to_mm, dr_mm / self.tick_to_mm]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Galipeur matrices from galipeur/src/main.rs.
    const VEL2CONS: Matrix3 = [
        [0.137193775559, -0.227742535811, 32.7587578324],
        [-0.267514745628, 0.000225842067981, 32.2910980339],
        [0.138273262887, 0.235015679279, 32.2670974911],
    ];
    const ENC2POS: Matrix3 = [
        [-1.24627114282, 2.4735001584, -1.21007913871],
        [2.15287736186, -0.0169008404017, -2.16876778164],
        [-0.0103397573436, -0.010476522571, -0.0100097003094],
    ];

    #[test]
    fn inverse_roundtrip() {
        let inv = inverse3(VEL2CONS).unwrap();
        // M * M^-1 ≈ I
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += VEL2CONS[i][k] * inv[k][j];
                }
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((sum - expected).abs() < 1e-4, "[{i}][{j}] = {sum}");
            }
        }
    }

    #[test]
    fn forward_motion_produces_positive_x() {
        // Construct consigns that correspond to pure vx (no vy, no va).
        // consigns = VEL2CONS @ [100, 0, 0]
        let consigns = mat_vec3(VEL2CONS, [100.0, 0.0, 0.0]);
        let mut state = HoloState::new(
            Pose2D { x_mm: 0.0, y_mm: 0.0, theta_rad: 0.0 },
            VEL2CONS,
            ENC2POS,
        );
        // Integrate 1 second in 10ms ticks.
        for _ in 0..100 {
            state.step(consigns, 0.010);
        }
        // Should have moved ~100mm forward in x direction.
        assert!((state.pose.x_mm - 100.0).abs() < 1.0, "x = {}", state.pose.x_mm);
        assert!(state.pose.y_mm.abs() < 1.0, "y drift = {}", state.pose.y_mm);
        assert!(state.pose.theta_rad.abs() < 0.01);
    }

    #[test]
    fn rotation_in_place() {
        let consigns = mat_vec3(VEL2CONS, [0.0, 0.0, 1.0]); // va = 1 rad/s
        let mut state = HoloState::new(
            Pose2D { x_mm: 0.0, y_mm: 0.0, theta_rad: 0.0 },
            VEL2CONS,
            ENC2POS,
        );
        for _ in 0..100 {
            state.step(consigns, 0.010);
        }
        assert!((state.pose.theta_rad - 1.0).abs() < 0.01);
    }
}
