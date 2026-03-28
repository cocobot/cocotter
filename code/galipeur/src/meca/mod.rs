mod proxy;

pub use proxy::{ArmState, MecaProxy, MecaState, TranslationState};

use board_sabotter::SabotterBoard;


/// High-level meca interface — cloneable, passes to threads
pub struct Meca<B: SabotterBoard> {
    proxy: MecaProxy<B>,
}

impl<B: SabotterBoard> Meca<B> {
    pub fn new(can: B::Can) -> Self {
        let mut proxy = MecaProxy::new(can);
        proxy.init_rx();
        Self { proxy }
    }

    pub fn get_state(&self) -> MecaState {
        self.proxy.get_state()
    }

    // --- High-level algos ---

    //lower arm, activate pump, disable valve
    pub fn lower_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,true);
        let position = match arm {
            0 => 340,
            1 => 225,
            2 => 5,
            3 => 780,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn idle_arm_release(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 430,
            1 => 301,
            2 => 83,
            3 => 851,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, false, true);
    }

    pub fn idle_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 430,
            1 => 301,
            2 => 83,
            3 => 851,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn raise_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 736,
            1 => 602,
            2 => 367,
            3 => 927,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn raise_arm_release(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 736,
            1 => 602,
            2 => 367,
            3 => 927,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, false, true);
    }

    /// Lower arm, enable pump, raise arm
    pub fn grab(&self, module: u8, arm: u8) {
        self.proxy.set_pump(module, arm, true);
        self.proxy.set_valve(module, arm, false);
    }

    /// Open valve, disable pump
    pub fn release(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, true);
        self.proxy.set_pump(module, arm, false);
    }
}
