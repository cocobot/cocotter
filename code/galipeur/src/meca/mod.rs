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
        for module in 0..3 {
            for arm in 0..4 {
                self.proxy.set_torque(module, arm, false);
            }
        }

        self.proxy.set_color_led_pwm(127);

        for module in 0..3 {
            for arm in 0..4 {
                // Clear existing color table (color_id=0 clears all)
                self.proxy.set_color_config(module, arm, 0, 0, 0, 0);

                // Color 1 = Blue — dummy CRGB thresholds
                // channel 0=Clear, 1=Red, 2=Green, 3=Blue
                self.proxy.set_color_config(module, arm, 1, 0, 2000, 5000);   // Clear
                self.proxy.set_color_config(module, arm, 1, 1, 200, 1000);    // Red (low)
                self.proxy.set_color_config(module, arm, 1, 2, 500, 1000);    // Green
                self.proxy.set_color_config(module, arm, 1, 3, 800, 2000);   // Blue (high)

                // Color 2 = Yellow — dummy CRGB thresholds
                self.proxy.set_color_config(module, arm, 2, 0, 10000, 20000);   // Clear (bright)
                self.proxy.set_color_config(module, arm, 2, 1, 5000, 10000);   // Red (high)
                self.proxy.set_color_config(module, arm, 2, 2, 3000, 10000);   // Green (high)
                self.proxy.set_color_config(module, arm, 2, 3, 1000, 3000);    // Blue (low)

                // Sensor config: integration_time=0xC0 (24 cycles ~58ms), gain=1 (4x)
                self.proxy.set_color_sensor_config(module, arm, 0xC0, 1);
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
        self.proxy.set_stage2(0, 0, 150, 500);

        std::thread::sleep(std::time::Duration::from_secs(1));

        for module in 0..3 {
            for arm in 0..4 {
                self.raise_arm_release(module, arm);
            }
        }
        self.proxy.set_stage2(0, 0, 90, 500);
    }   

    /// Infinite loop that requests and logs raw color sensor values for calibration.
    /// Call this instead of the normal match sequence to tune thresholds.
    #[allow(dead_code)]
    pub fn calibrate_color_sensors(&self, led_pwm: u8, integration_time: u8, gain: u8) -> ! {
        log::info!("=== Color sensor calibration mode ===");
        log::info!("LED PWM={}, integration_time=0x{:02X}, gain={}", led_pwm, integration_time, gain);
        log::info!("Place samples under sensors and read CRGB values.");

        self.proxy.set_color_led_pwm(led_pwm);

        for module in 0..3u8 {
            for arm in 0..4u8 {
                self.proxy.set_color_sensor_config(module, arm, integration_time, gain);
            }
        }

        loop {
            // Request raw values from all sensors
            for module in 0..3u8 {
                for arm in 0..4u8 {
                    self.proxy.request_color_sensor_raw(module, arm);
                }
            }

            // Wait for responses
            thread::sleep(Duration::from_millis(200));

            // Read and log state
            let state = self.proxy.get_state();
            for (m, module) in state.color_raw.iter().enumerate() {
                for (a, raw) in module.iter().enumerate() {
                    // Also show the detected color from arm status
                    let color_id = state.arms[m][a].color;
                    log::info!(
                        "M{}A{}: C={:5} R={:5} G={:5} B={:5} | detected={}",
                        m, a, raw.clear, raw.red, raw.green, raw.blue, color_id
                    );
                }
            }
            log::info!("---");

            thread::sleep(Duration::from_millis(500));
        }
    }

    //lower arm, activate pump, disable valve
    pub fn lower_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,true);
        let position = match arm {
            0 => 310,
            1 => 225,
            2 => 340,
            3 => 320,
            _ => 300,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn idle_arm_release(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 350,
            1 => 240,
            2 => 350,
            3 => 350,
            _ => 300,
        };
        self.proxy.set_arm(module, arm, position, 500, false, true);
    }

    pub fn idle_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 350,
            1 => 240,
            2 => 350,
            3 => 350,
            _ => 300,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn raise_arm_grab(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 690,
            1 => 535,
            2 => 665,
            3 => 660,
            _ => 300,
        };
        self.proxy.set_arm(module, arm, position, 500, true, false);
    }

    pub fn raise_arm_release(&self, module: u8, arm: u8) {
        self.proxy.set_torque(module,arm,false);
        let position = match arm {
            0 => 690,
            1 => 535,
            2 => 665,
            3 => 660,
            _ => 300,
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
