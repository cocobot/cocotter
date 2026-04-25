//! Host-only `MovementLowLevelHardware` that talks to the simulator.
//!
//! `set_motor_consigns` forwards the PWM values to the sim process and blocks
//! until it receives back the encoder + gyro deltas for the resulting step.
//! The deltas are cached and returned on the next `get_motor_offsets` /
//! `get_gyro_offset` call — this reproduces the 1-tick latency of the real
//! asserv on hardware.

use std::marker::PhantomData;
use std::sync::Arc;

use asserv::holonomic::conf::AsservHardware;
use asserv::maths::XYA;
use board_sabotter::SabotterBoard;
use sim_client::SimClient;
use sim_protocol::{Pose2D, SimMsgC2S};

pub struct MovementLowLevelHardware<B: SabotterBoard> {
    sim: Arc<SimClient>,
    cached_encoder_delta: [f32; 3],
    cached_gyro_delta: f32,
    _phantom: PhantomData<fn() -> B>,
}

impl<B: SabotterBoard> MovementLowLevelHardware<B> {
    pub fn new(_board: &mut B) -> Self {
        Self {
            sim: sim_client::global(),
            cached_encoder_delta: [0.0; 3],
            cached_gyro_delta: 0.0,
            _phantom: PhantomData,
        }
    }
}

impl<B: SabotterBoard> AsservHardware for MovementLowLevelHardware<B> {
    fn set_motors_break(&mut self, enable: bool) {
        let _ = self.sim.send(sim_protocol::SimMsgC2S::MotorsBreak { enable });
    }

    fn set_motor_consigns(&mut self, values: [f32; 3]) {
        let (enc, gyro) = self.sim.tick_holo(values);
        self.cached_encoder_delta = enc;
        self.cached_gyro_delta = gyro;
    }

    fn get_motor_offsets(&mut self) -> [f32; 3] {
        std::mem::replace(&mut self.cached_encoder_delta, [0.0; 3])
    }

    fn get_gyro_offset(&mut self) -> f32 {
        std::mem::replace(&mut self.cached_gyro_delta, 0.0)
    }

    fn teleport(&mut self, xya: XYA) {
        //let _ = self.sim.send(SimMsgC2S::Teleport {
        //    pose: Pose2D { x_mm: 2000.0 - xya.x, y_mm: 1500.0 + xya.y, theta_rad: xya.a },
        //});


        let _ = self.sim.send(SimMsgC2S::Teleport {
            pose: Pose2D { x_mm: 1500.0 + xya.y, y_mm: 2000.0 - xya.x, theta_rad: xya.a },
        });
    }
}
