mod proxy;

pub use proxy::{MecaProxy, MecaState, ColorSensorRawState};

use std::thread;
use std::time::Duration;

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
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_torque(module, arm, true);
                self.idle_arm_release(module, arm);
            }
        }
        self.proxy.set_stage2_torque(0, 0, true);
        self.stage2_transfer_open();


        std::thread::sleep(std::time::Duration::from_secs(1));

        for module in 0..3 {
            for arm in 0..4 {
                self.raise_arm_release(module, arm);
            }
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
            thread::sleep(Duration::from_millis(200));

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

            thread::sleep(Duration::from_millis(500));
        }
    }

    //lower arm, activate pump, disable valve
    pub fn lower_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,true);
        let position = match arm {
            0 => 330,
            1 => 225,
            2 => 340,
            3 => 340,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 50, false, false);
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
        self.proxy.set_arm(module, arm, position, 50, false, true);
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
        self.proxy.set_arm(module, arm, position, 50, true, false);
    }

    pub fn raise_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            //0 => 736,
            //1 => 602,
            //2 => 736,
            //3 => 736,
            //_ => 50,

            0 => 761,
            1 => 625,
            2 => 746,
            3 => 746,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 50, true, false);
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
        self.proxy.set_arm(module, arm, position, 50, false, true);
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

    /// Close valve after release (call ~250ms after release)
    pub fn endrelease(&self, module: u8, arm: u8) {
        self.proxy.set_valve(module, arm, false);
    }

    pub fn stage2_transfer_open(&self) {
        self.proxy.set_stage2(0, 0, 820, 50);
        self.proxy.set_stage2(0, 1, 316, 50);
    }

    pub fn stage2_transfer_close(&self) {
        self.proxy.set_stage2(0, 0, 820, 50);
        self.proxy.set_stage2(0, 1, 654, 50);
    }

    pub fn stage2_keep(&self) {
        self.proxy.set_stage2(0, 0, 610, 50);
        self.proxy.set_stage2(0, 1, 654, 50);
    }

    pub fn wait_arm(&self, module: u8, arm: u8, seq: u64) -> bool {
        self.proxy.wait_arm(module, arm, seq, Duration::from_secs(3))
    }

    pub fn wait_arms(&self, module: u8, seq: u64) -> bool {
        self.proxy.wait_arms(module, seq, Duration::from_secs(3))
    }

    pub fn wait_translation(&self, module: u8, seq: u64) -> bool {
        self.proxy.wait_translation(module, seq, Duration::from_secs(3))
    }

    pub fn wait_stage2(&self, module: u8, servo: u8, seq: u64) -> bool {
        self.proxy.wait_stage2(module, servo, seq, Duration::from_secs(3))
    }

    pub fn wait_stage2s(&self, module: u8, seq: u64) -> bool {
        self.proxy.wait_stage2s(module, seq, Duration::from_secs(3))
    }

    pub fn spread_arms(&self, module: u8, spread: bool) -> u64 {
        let seq = self.proxy.translation_seq(module);
        if spread {
            self.proxy.set_translation(module, 640, 50);
        } else {
            self.proxy.set_translation(module, 910, 50);
        }
        seq
    }

    pub fn test_algo2(&self, module: u8) {
        self.stage2_keep();
        loop {
            self.proxy.set_color_led_pwm(255);
            log::info!("Spreading arms...");
            let seq = self.spread_arms(module, true);
            log::info!("Waiting for translation...");
            self.wait_translation(module, seq);

            log::info!("Lowering arms...");
            let seq = self.proxy.arms_seq(module);
            for arm in 0..4 {
                self.lower_arm_grab(module, arm);
            }
            log::info!("Waiting for arms to lower...");
            self.wait_arms(module, seq);
            log::info!("Done. Spreading arms...");

            let seq = self.spread_arms(module, false);
            self.wait_translation(module, seq);

            for arm in 0..4 {
                self.grab(module, arm);
            }
            thread::sleep(Duration::from_millis(250));
            //let seq = self.proxy.stage2s_seq(0);
            //self.stage2_transfer_open();
            //thread::sleep(Duration::from_millis(250));
            //self.wait_stage2s(0, seq);

            let seq = self.proxy.arms_seq(module);
            for arm in 0..4 {
                self.raise_arm_grab(module, arm);
            }
            self.wait_arms(module, seq);
            thread::sleep(Duration::from_millis(1000));

            let colors: Vec<bool> = self.proxy.get_state().arms[0].iter().map(|c| {
                // true = jaune (hue ~60), false = bleu (hue ~240)
                c.hue < 180
            }).collect();

            self.proxy.set_color_led_pwm(0);


            if !colors.get(0).copied().unwrap_or(false) {
                self.release(0, 0);
            }
            if !colors.get(3).copied().unwrap_or(false) {
                self.release(0, 3);
            }           
            thread::sleep(Duration::from_millis(250));
            if !colors.get(1).copied().unwrap_or(false) {
                self.release(0, 1);
            }
            if !colors.get(2).copied().unwrap_or(false) {
                self.release(0, 2);
            }
            thread::sleep(Duration::from_millis(250));
            let seq = self.proxy.arms_seq(module);
            for arm in 0..4 {
                if colors.get(arm).copied().unwrap_or(false) {
                    self.lower_arm_grab(module, arm as u8);
                }
            }
            self.wait_arms(module, seq);

            for arm in 0..4 {
                self.release(0, arm);
            }

            thread::sleep(Duration::from_millis(250));
            for arm in 0..4 {
                self.endrelease(module, arm);
            } 

            let seq = self.proxy.arms_seq(module);
            for arm in 0..4 {
                self.raise_arm_release(module, arm);
                self.endrelease(module, arm);
            }
            self.wait_arms(module, seq);
             thread::sleep(Duration::from_millis(5000));
        }

        let seq = self.proxy.stage2s_seq(0);
        self.stage2_transfer_close();
        self.wait_stage2s(0, seq);

        for arm in 1..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(250));
        for arm in 1..4 {
            self.endrelease(module, arm);
        }

        let seq = self.proxy.stage2s_seq(0);
        self.stage2_keep();
        self.wait_stage2s(0, seq);

        thread::sleep(Duration::from_millis(5000));


        //Second grab

        let seq = self.spread_arms(module, true);
        self.wait_translation(module, seq);

        let seq = self.proxy.arms_seq(module);
        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }
        self.wait_arms(module, seq);

        let seq = self.spread_arms(module, false);
        self.wait_translation(module, seq);

        for arm in 0..4 {
            self.grab(module, arm);
        }
        thread::sleep(Duration::from_millis(250));

        let seq = self.proxy.arms_seq(module);
        for arm in 0..4 {
            self.raise_arm_grab(module, arm);
        }
        self.wait_arms(module, seq);

        thread::sleep(Duration::from_millis(5000));

        for arm in 0..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(250));
        for arm in 0..4 {
            self.endrelease(module, arm);
        }

        thread::sleep(Duration::from_millis(5000));


        //stage 2 => stage 1
        let seq = self.proxy.stage2s_seq(0);
        self.stage2_transfer_close();
        self.wait_stage2s(0, seq);

        for arm in 0..4 {
            self.grab(module, arm);
        }
        thread::sleep(Duration::from_millis(250));

        let seq = self.proxy.stage2s_seq(0);
        self.stage2_transfer_open();
        self.wait_stage2s(0, seq);

        let seq = self.proxy.arms_seq(module);
        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }
        self.wait_arms(module, seq);

        for arm in 0..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(250));
        for arm in 0..4 {
            self.endrelease(module, arm);
        }

        }

        pub fn test_algo(&self, module: u8) {
        self.stage2_keep();
        loop {
            log::info!("Spreading arms...");
            let seq = self.spread_arms(module, true);
            log::info!("Waiting for translation...");
            self.wait_translation(module, seq);

            log::info!("Lowering arms...");
            let seq = self.proxy.arms_seq(module);
            for arm in 0..4 {
                self.lower_arm_grab(module, arm);
            }
            log::info!("Waiting for arms to lower...");
            self.wait_arms(module, seq);
            log::info!("Done. Spreading arms...");

            let seq = self.spread_arms(module, false);
            self.wait_translation(module, seq);


            for arm in 0..4 {
                self.grab(module, arm);
            }
            thread::sleep(Duration::from_millis(250));
            //let seq = self.proxy.stage2s_seq(0);
            //self.stage2_transfer_open();
            //thread::sleep(Duration::from_millis(250));
            //self.wait_stage2s(0, seq);

            let seq = self.proxy.arms_seq(module);
            for arm in 0..4 {
                self.raise_arm_grab(module, arm);
            }
            self.wait_arms(module, seq);

            thread::sleep(Duration::from_millis(2000));

            self.release(0, 0);
            self.release(0, 3);
            thread::sleep(Duration::from_millis(250));
            self.release(0, 1);
            self.release(0, 2);

            thread::sleep(Duration::from_millis(250));
            for arm in 0..4 {
                self.endrelease(module, arm);
            } 

             thread::sleep(Duration::from_millis(20000));
        }

        let seq = self.proxy.stage2s_seq(0);
        self.stage2_transfer_close();
        self.wait_stage2s(0, seq);

        for arm in 1..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(250));
        for arm in 1..4 {
            self.endrelease(module, arm);
        }

        let seq = self.proxy.stage2s_seq(0);
        self.stage2_keep();
        self.wait_stage2s(0, seq);

        thread::sleep(Duration::from_millis(5000));


        //Second grab

        let seq = self.spread_arms(module, true);
        self.wait_translation(module, seq);

        let seq = self.proxy.arms_seq(module);
        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }
        self.wait_arms(module, seq);

        let seq = self.spread_arms(module, false);
        self.wait_translation(module, seq);

        for arm in 0..4 {
            self.grab(module, arm);
        }
        thread::sleep(Duration::from_millis(250));

        let seq = self.proxy.arms_seq(module);
        for arm in 0..4 {
            self.raise_arm_grab(module, arm);
        }
        self.wait_arms(module, seq);

        thread::sleep(Duration::from_millis(5000));

        for arm in 0..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(250));
        for arm in 0..4 {
            self.endrelease(module, arm);
        }

        thread::sleep(Duration::from_millis(5000));


        //stage 2 => stage 1
        let seq = self.proxy.stage2s_seq(0);
        self.stage2_transfer_close();
        self.wait_stage2s(0, seq);

        for arm in 0..4 {
            self.grab(module, arm);
        }
        thread::sleep(Duration::from_millis(250));

        let seq = self.proxy.stage2s_seq(0);
        self.stage2_transfer_open();
        self.wait_stage2s(0, seq);

        let seq = self.proxy.arms_seq(module);
        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }
        self.wait_arms(module, seq);

        for arm in 0..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(250));
        for arm in 0..4 {
            self.endrelease(module, arm);
        }

        }
}
