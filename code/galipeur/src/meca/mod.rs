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

        self.proxy.set_color_led_pwm(127);

        for module in 0..3 {
            for arm in 0..4 {      
                //Embassy stm32 FDCAN bug... do not burst config messages
                if !self.proxy.ping(Duration::from_millis(3000)) {
                    log::error!("Failed to ping Meca after disabling torque - check servo connections");
                    return;
                }
                thread::sleep(Duration::from_millis(100));
                        
                // Clear existing color table (color_id=0 clears all)
                self.proxy.set_color_config(module, arm, 0, 0, 0, 0);

                // Color 1 = Blue — dummy CRGB thresholds
                // channel 0=Clear, 1=Red, 2=Green, 3=Blue
                self.proxy.set_color_config(module, arm, 1, 0, 100, 500);   // Clear
                self.proxy.set_color_config(module, arm, 1, 1, 20, 150);    // Red (low)
                self.proxy.set_color_config(module, arm, 1, 2, 50, 250);    // Green
                self.proxy.set_color_config(module, arm, 1, 3, 200, 600);   // Blue (high)

                // Color 2 = Yellow — dummy CRGB thresholds
                self.proxy.set_color_config(module, arm, 2, 0, 200, 800);   // Clear (bright)
                self.proxy.set_color_config(module, arm, 2, 1, 150, 500);   // Red (high)
                self.proxy.set_color_config(module, arm, 2, 2, 150, 500);   // Green (high)
                self.proxy.set_color_config(module, arm, 2, 3, 20, 150);    // Blue (low)

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
            0 => 340,
            1 => 225,
            2 => 340,
            3 => 340,
            _ => 50,
        };
        self.proxy.set_arm(module, arm, position, 500, false, false);
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

    pub fn stage2_transfer_open(&self) {
        self.proxy.set_stage2(0, 0, 820, 500);
        self.proxy.set_stage2(0, 1, 316, 500);
    }

    pub fn stage2_transfer_close(&self) {
        self.proxy.set_stage2(0, 0, 820, 500);
        self.proxy.set_stage2(0, 1, 654, 500);
    }

    pub fn stage2_keep(&self) {
        self.proxy.set_stage2(0, 0, 610, 500);
        self.proxy.set_stage2(0, 1, 654, 500);
    }

    pub fn spread_arms(&self, module: u8, spread: bool) {
        if spread {
            self.proxy.set_translation(module, 640, 500);
        } else {
            self.proxy.set_translation(module, 910, 500);
        }        
    }

    pub fn test_algo(&self, module: u8) {
        self.spread_arms(module, true);

        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }

        thread::sleep(Duration::from_millis(1000));

        self.spread_arms(module, false);

        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.grab(module, arm);
        }
        self.stage2_transfer_open();

        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.raise_arm_grab(module, arm);
        }

        thread::sleep(Duration::from_millis(1000));

        self.stage2_transfer_close();

        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.release(module, arm);
        }

        thread::sleep(Duration::from_millis(1000));

        self.stage2_keep();        

        thread::sleep(Duration::from_millis(5000));


        //Second grab 

        self.spread_arms(module, true);
        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }
        thread::sleep(Duration::from_millis(1000));

        self.spread_arms(module, false);
        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.grab(module, arm);
        }
        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.raise_arm_grab(module, arm);
        }
        thread::sleep(Duration::from_millis(5000));

        for arm in 0..4 {
            self.release(module, arm);
        }        

        thread::sleep(Duration::from_millis(5000));


        //stage 2 => stage 1
        self.stage2_transfer_close();
        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.grab(module, arm);
        }
        thread::sleep(Duration::from_millis(1000));

        self.stage2_transfer_open();
        thread::sleep(Duration::from_millis(1000));

        for arm in 0..4 {
            self.lower_arm_grab(module, arm);
        }
        thread::sleep(Duration::from_millis(1000));
            
        for arm in 0..4 {
            self.release(module, arm);
        }
        thread::sleep(Duration::from_millis(1000));

       

        }
}
