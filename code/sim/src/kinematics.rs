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
    /// Rotation (rad) that maps a vector from the asserv body frame to
    /// the sim body frame. Used to correct a rotational mismatch between
    /// the physical motor/encoder mounting and the sim's orientation
    /// convention — the asserv thinks +X_body is its forward, but the
    /// visual/chassis has that axis rotated by this angle.
    asserv_to_sim_rad: f32,
    /// When `true`, the asserv frame is mirrored compared to the sim
    /// (left-handed). Applied *before* `asserv_to_sim_rad` as
    /// `(vx, va) → (-vx, -va)` which flips both the X axis and the
    /// rotation sense.
    asserv_mirror: bool,
}

impl HoloState {
    pub fn new(
        start: Pose2D,
        velocities_to_consigns: Matrix3,
        encoders_to_position: Matrix3,
        asserv_to_sim_rad: f32,
        asserv_mirror: bool,
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
            asserv_to_sim_rad,
            asserv_mirror,
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
    pub fn step(
        &mut self,
        consigns: [f32; 3],
        dt_s: f32,
        obstacles: &[crate::config::Obstacle],
        robot_shape: &crate::collide::RobotShape,
    ) -> ([f32; 3], f32) {
        const MAX_LIN_MM_S: f32 = 1500.0; // realistic galipeur ceiling
        const MAX_ANG_RAD_S: f32 = 8.0;

        let c_clamped = [
            consigns[0].clamp(-4095.0, 4095.0),
            consigns[1].clamp(-4095.0, 4095.0),
            consigns[2].clamp(-4095.0, 4095.0),
        ];
        let v_body_asserv = mat_vec3(self.vel_from_consigns, c_clamped);
        // Optional mirror (left-handed asserv frame): flip X and the
        // rotation sense. Applied *before* the rotation.
        let mirror = if self.asserv_mirror { -1.0 } else { 1.0 };
        let vxa = mirror * v_body_asserv[0];
        let vya = v_body_asserv[1];
        let vaa = mirror * v_body_asserv[2];
        // Rotate the (possibly mirrored) asserv velocity into the sim
        // body frame.
        let (ca, sa) = (self.asserv_to_sim_rad.cos(), self.asserv_to_sim_rad.sin());
        let mut vx = ca * vxa - sa * vya;
        let mut vy = sa * vxa + ca * vya;
        let mut va = vaa;

        let v_norm = (vx * vx + vy * vy).sqrt();
        if v_norm > MAX_LIN_MM_S {
            let k = MAX_LIN_MM_S / v_norm;
            vx *= k;
            vy *= k;
        }
        va = va.clamp(-MAX_ANG_RAD_S, MAX_ANG_RAD_S);

        let dx_r = vx * dt_s;
        let dy_r = vy * dt_s;
        let da = va * dt_s;

        let c = self.pose.theta_rad.cos();
        let s = self.pose.theta_rad.sin();
        let world_dx = c * dx_r - s * dy_r;
        let world_dy = s * dx_r + c * dy_r;

        // Clamp the fraction of the motion that would traverse an obstacle.
        // Rotation is left un-blocked.
        let t = crate::collide::sweep_max_fraction(
            (self.pose.x_mm, self.pose.y_mm, self.pose.theta_rad),
            (world_dx, world_dy),
            robot_shape,
            obstacles,
        );
        let eff_dx = world_dx * t;
        let eff_dy = world_dy * t;
        self.pose.x_mm += eff_dx;
        self.pose.y_mm += eff_dy;
        self.pose.theta_rad += da;

        // Synthesize encoder deltas matching the **actual** displacement —
        // if we hit a wall, the asserv sees smaller encoder deltas than
        // its PID expected.
        // Asserv quirk: `control_system::update_position` divides the
        // `encoders_to_position * motor_offsets` output by 1000 (see the
        // TODO in the asserv code). Mirror that scaling here.
        const ASSERV_XY_SCALE: f32 = 1000.0;
        // Map the displacement (sim body frame) back into the asserv
        // body frame: undo the rotation, then undo the mirror so the
        // encoders match the consigns the asserv sent.
        let dx_sim = dx_r * t;
        let dy_sim = dy_r * t;
        let dx_mid = ca * dx_sim + sa * dy_sim;
        let dy_mid = -sa * dx_sim + ca * dy_sim;
        let dx_asserv = mirror * dx_mid;
        let dy_asserv = dy_mid;
        let da_asserv = mirror * da;
        let body_for_enc = [
            dx_asserv * ASSERV_XY_SCALE,
            dy_asserv * ASSERV_XY_SCALE,
            da_asserv,
        ];
        let enc_delta = mat_vec3(self.enc_from_pos, body_for_enc);
        // The asserv fuses this delta with the commanded `va`; return
        // it in the *asserv* frame (mirrored) so the gyro agrees with
        // the PWM it just sent, otherwise the angular PID oscillates.
        (enc_delta, da_asserv)
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
    /// Obstacle collisions clamp the translation fraction; rotation is
    /// applied un-blocked.
    pub fn step(
        &mut self,
        consigns: [f32; 2],
        dt_s: f32,
        obstacles: &[crate::config::Obstacle],
        robot_shape: &crate::collide::RobotShape,
    ) -> [f32; 2] {
        let scale = self.max_wheel_speed_mm_s / 1000.0;
        let vl = consigns[0].clamp(-1000.0, 1000.0) * scale;
        let vr = consigns[1].clamp(-1000.0, 1000.0) * scale;

        let v_body = 0.5 * (vl + vr);
        let omega = (vr - vl) / self.wheel_base_mm;

        let c = self.pose.theta_rad.cos();
        let s = self.pose.theta_rad.sin();
        let world_dx = c * v_body * dt_s;
        let world_dy = s * v_body * dt_s;

        let t = crate::collide::sweep_max_fraction(
            (self.pose.x_mm, self.pose.y_mm, self.pose.theta_rad),
            (world_dx, world_dy),
            robot_shape,
            obstacles,
        );
        self.pose.x_mm += world_dx * t;
        self.pose.y_mm += world_dy * t;
        self.pose.theta_rad += omega * dt_s;

        let dl_mm = vl * dt_s * t;
        let dr_mm = vr * dt_s * t;
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
            0.0,
            false,
        );
        let shape = crate::collide::RobotShape::from_circle(1.0);
        // Integrate 1 second in 10ms ticks.
        for _ in 0..100 {
            state.step(consigns, 0.010, &[], &shape);
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
            0.0,
            false,
        );
        let shape = crate::collide::RobotShape::from_circle(1.0);
        for _ in 0..100 {
            state.step(consigns, 0.010, &[], &shape);
        }
        assert!((state.pose.theta_rad - 1.0).abs() < 0.01);
    }
}
