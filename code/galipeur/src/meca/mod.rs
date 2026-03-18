mod proxy;

pub use proxy::{ArmState, MecaProxy, MecaState, TranslationState};

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

    pub fn lower_arm(&self, module: u8, arm: u8) {
        self.proxy.set_arm(module, arm, 400, 500, false, false);
    }

    pub fn raise_arm(&self, module: u8, arm: u8) {
        self.proxy.set_arm(module, arm, 500, 500, false, false);
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
