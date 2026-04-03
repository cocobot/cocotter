mod proxy;

pub use proxy::{MecaProxy, MecaState};

use crate::can::CanInterface;

/// High-level meca interface — cloneable, passes to threads
#[derive(Clone)]
pub struct Meca {
    proxy: MecaProxy,
}

impl Meca {
    pub fn new(can: &CanInterface) -> Self {
        Self {
            proxy: MecaProxy::new(can),
        }
    }

    /// Returns a proxy for direct state reads and low-level commands
    pub fn get_proxy(&self) -> MecaProxy {
        self.proxy.clone()
    }

    pub fn get_state(&self) -> MecaState {
        self.proxy.get_state()
    }

    // --- High-level algos ---
    pub fn no_torque_on_all(&self) {
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_torque(module, arm, false);
            }
        }
    }

    pub fn init(&self) {
        self.proxy.set_color_led_pwm(127);

        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_torque(module, arm, true);
                self.idle_arm_release(module, arm);
            }
        }
        self.proxy.set_stage2_torque(0, 0, true);
        self.proxy.set_stage2(0, 0, 150, 500);

        std::thread::sleep(std::time::Duration::from_secs(1));

        for module in 0..3 {
            for arm in 0..4 {
                self.raise_arm_release(module, arm);
            }
        }
        self.proxy.set_stage2(0, 0, 90, 500);
    }   

    //lower arm, activate pump, disable valve
    pub fn lower_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,true);
        let position = match arm {
            0 => 340,
            1 => 225,
            2 => 340,
            3 => 340,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn idle_arm_release(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 430,
            1 => 301,
            2 => 430,
            3 => 430,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, false, true);
    }

    pub fn idle_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 430,
            1 => 301,
            2 => 430,
            3 => 430,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn raise_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 736,
            1 => 602,
            2 => 736,
            3 => 736,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn raise_arm_release(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 736,
            1 => 602,
            2 => 736,
            3 => 736,
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
