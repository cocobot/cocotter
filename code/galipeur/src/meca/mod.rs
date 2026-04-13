mod proxy;

use proxy::{MecaProxy, MecaState};
use board_sabotter::SabotterBoard;
use std::time::Duration;

use crate::can::GalipeurCan;


/// High-level meca interface — cloneable, passes to threads
pub struct Meca<B: SabotterBoard> {
    proxy: MecaProxy<B>,
}

impl<B: SabotterBoard> Meca<B> {
    pub fn new(can: GalipeurCan<B>) -> Self {
        let proxy = MecaProxy::new(can);
        Self { proxy }
    }

    pub fn get_state(&self) -> MecaState {
        self.proxy.get_state()
    }

    // --- High-level algos ---
    pub fn pre_init(&self) {
        if !self.proxy.ping(Duration::from_millis(3000)) {
            log::error!("Failed to ping Meca");
            return;
        }

        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_torque(module, arm, false);
            }
        }       
    }

    pub fn init(&self) {
        self.proxy.set_stage2_torque(0, 0, true);
        //self.stage2_transfer_open();


        std::thread::sleep(std::time::Duration::from_secs(1));

        for module in 0..3 {
            self.raise_arm_release(module);
        }        
    }   

    /// Infinite loop that requests and logs raw color sensor values for calibration.
    /// Call this instead of the normal match sequence to tune thresholds.
    #[allow(dead_code)]
    pub fn calibrate_color_sensors(&self) -> ! {
        log::info!("=== Color sensor calibration mode ===");       
        loop {
            // Request raw values from all sensors
            for module in 0..3u8 {
                for arm in 0..4u8 {
                    self.proxy.request_color_sensor_raw(module, arm);
                }
                break;
            }

            // Wait for responses
            std::thread::sleep(Duration::from_millis(200));

            // Read and log state
            let state = self.proxy.get_state();
            for (m, module) in state.color_raw.iter().enumerate() {
                for (a, raw) in module.iter().enumerate() {
                    // Also show the detected color from arm status
                    let arm_state = &state.arms[m][a];
                    log::info!(
                        "M{}A{}: C={:5} R={:5} G={:5} B={:5} | det={} hue={}",
                        m, a, raw.clear, raw.red, raw.green, raw.blue,
                        arm_state.color_detected, arm_state.hue
                    );
                }
                break;
            }
            log::info!("---");

            std::thread::sleep(Duration::from_millis(500));
        }
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

    pub fn raise_arm_release(&self, module: u8) {
        for arm in 0..=3 {
            self.proxy.set_torque(module,arm,true);
            let position = match arm {
                0 => 690,
                1 => 535,
                2 => 665,
                3 => 660,
                _ => 50,
            };
            self.proxy.set_arm(module, arm, position, 500, false, true);
        }
    }

    /// Lower arm, enable pump, raise arm
    pub fn grab(&self, module: u8) {
        for arm in 0..=3 {
            self.proxy.set_pump(module, arm, true);
            self.proxy.set_valve(module, arm, false);
        }
    }

    /// Open valve, disable pump
    pub fn release(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, true);
        self.proxy.set_pump(module, arm, false);
    }
}
