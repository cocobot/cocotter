//! Host-only sim-backed differential asserv hardware.
//!
//! Consigns are forwarded to the simulator and the returned encoder deltas
//! are cached for the next `get_motor_offsets` call (1-tick latency, same
//! as the real hardware).

use std::marker::PhantomData;
use std::sync::Arc;

use asserv::differential::conf::AsservHardware;
use asserv::maths::XYA;
use board_pami::PamiBoard;
use sim_client::SimClient;
use sim_protocol::{Pose2D, SimMsgC2S};

pub struct PamiAsservHardware<B: PamiBoard> {
    sim: Arc<SimClient>,
    cached: [f32; 2],
    _phantom: PhantomData<fn() -> B>,
}

impl<B: PamiBoard> PamiAsservHardware<B> {
    pub fn new(_board: &mut B) -> Self {
        Self {
            sim: sim_client::global(),
            cached: [0.0, 0.0],
            _phantom: PhantomData,
        }
    }
}

impl<B: PamiBoard> AsservHardware for PamiAsservHardware<B> {
    fn emergency_stop_active(&mut self) -> bool {
        false
    }

    fn set_motor_consigns(&mut self, values: [f32; 2]) {
        let delta = self.sim.tick_diff(values);
        self.cached = delta;
    }

    fn get_motor_offsets(&mut self) -> [f32; 2] {
        std::mem::replace(&mut self.cached, [0.0, 0.0])
    }

    fn teleport(&mut self, xya: XYA) {
        let _ = self.sim.send(SimMsgC2S::Teleport {
            pose: Pose2D { x_mm: xya.x, y_mm: xya.y, theta_rad: xya.a },
        });
    }
}
