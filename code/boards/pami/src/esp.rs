use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use embedded_hal_bus::i2c::MutexDevice;
use esp_idf_svc::{
    bt::{Ble, BtDriver},
    hal::{
        adc::{
            ADC1,
            attenuation,
            oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver},
        },
        gpio::{AnyInputPin, AnyOutputPin, Input, InputPin, Output, PinDriver, Gpio1},
        i2c::{I2cConfig, I2cDriver},
        ledc::{LedcDriver, LedcTimerDriver, Resolution, config::TimerConfig},
        pcnt::{Pcnt, PcntChannel, PcntChannelConfig, PcntControlMode, PcntCountMode, PcntEvent, PcntEventType, PcntDriver, PinIndex},
        peripheral::Peripheral,
        prelude::Peripherals,
        units::Hertz,
        sys::EspError,
    },
    nvs::EspDefaultNvsPartition,
};
use ssd1306::{
    I2CDisplayInterface, Ssd1306,
    mode::BufferedGraphicsMode,
    prelude::{
        DisplayConfig,
        DisplayRotation,
        DisplaySize128x64,
        I2CInterface as DisplayI2CInterface,
    },
};

use ble::{BleBuilder, RomePeripheral};
use tca6408::TCA6408;
use vlx::{DistanceData, VlxI2cDriver, VlxError, VlxSensor, ZoneAlarm, l5::VL53L5CX};
use crate::{Encoder, PamiBoard, PamiButtons, PamiMotor, PamiMotors, Vbatt};


pub type I2cType = MutexDevice<'static, I2cDriver<'static>>;
pub type PamiDisplay = Ssd1306<DisplayI2CInterface<I2cType>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;


pub struct EspPamiBoard<'d> {
    ble: Option<BtDriver<'static, Ble>>,
    buttons: Option<PamiButtons<I2cType>>,
    display: Option<PamiDisplay>,
    heartbeat_led: Option<PinDriver<'static, AnyOutputPin, Output>>,
    line_sensor: Option<TCA6408<I2cType>>,
    vbatt: Option<PamiVbatt>,
    vlx_sensor: Option<PamiVlxSensor>,
    motors: Option<PamiMotors<EspEncoder<'d>, LedcDriver<'static>>>,
    emergency_stop: Option<PinDriver<'static, AnyInputPin, Input>>,
}

impl<'d> PamiBoard for EspPamiBoard<'d> {
    type I2c = I2cType;
    type Led = PinDriver<'static, AnyOutputPin, Output>;
    type Display = PamiDisplay;
    type Vbatt = PamiVbatt;
    type Vlx = PamiVlxSensor;
    type MotorEncoder = EspEncoder<'d>;
    type MotorPwm = LedcDriver<'static>;

    fn init() -> Self {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();
        let nvs = EspDefaultNvsPartition::take().unwrap();

        let peripherals = Peripherals::take().unwrap();

        // Initialize digital output pins
        let heartbeat_led = PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio4)).unwrap();

        // Initialize the I2C bus
        let config = I2cConfig::new().baudrate(Hertz(100_000));
        let i2c_driver = Mutex::new(I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &config).unwrap());
        // Leak `i2c_driver` to get a static lifetime
        // The I2cDriver never be dropped the end of the program, but it's fine
        let i2c_driver_static = Box::leak(Box::new(i2c_driver));

        // VLX sensor
        let vlx_i2c_driver = VlxI2cDriver::register(MutexDevice::new(i2c_driver_static));
        let vlx_sensor = PamiVlxSensor {
            sensor: VL53L5CX::new(&vlx_i2c_driver, 0x31),
            enable: PinDriver::output(Into::<AnyOutputPin>::into(peripherals.pins.gpio3)).unwrap(),
        };

        // ADC
        let adc = Rc::new(AdcDriver::new(peripherals.adc1).unwrap());
        let adc_vbatt = AdcChannelDriver::new(
            adc,
            peripherals.pins.gpio1,
            &AdcChannelConfig { attenuation: attenuation::DB_11, ..Default::default() },
        ).unwrap();
        let vbatt = PamiVbatt(adc_vbatt);

        let ble = Some(BtDriver::new(peripherals.modem, Some(nvs.clone())).unwrap());

        let line_sensor = TCA6408::new(MutexDevice::new(i2c_driver_static), 0b010_0000);
        let buttons = PamiButtons(TCA6408::new(MutexDevice::new(i2c_driver_static), 0b010_0001));

        // Screen
        let display = {
            let interface = I2CDisplayInterface::new(MutexDevice::new(i2c_driver_static));
            let mut display  = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                .into_buffered_graphics_mode();
            display.init().unwrap();
            display
        };

        // Motors
        let motor_pwm = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(Hertz(25_000)).resolution(Resolution::Bits10),
        ).unwrap();
        let motors = PamiMotors {
            left: PamiMotor {
                encoder: EspEncoder::new(peripherals.pcnt0,  peripherals.pins.gpio39,  peripherals.pins.gpio40).unwrap(),
                pwm_forward: LedcDriver::new(peripherals.ledc.channel0, &motor_pwm, peripherals.pins.gpio21).unwrap(),
                pwm_backward: LedcDriver::new(peripherals.ledc.channel1, &motor_pwm, peripherals.pins.gpio14).unwrap(),
            },
            right: PamiMotor {
                encoder: EspEncoder::new(peripherals.pcnt1, peripherals.pins.gpio41, peripherals.pins.gpio42).unwrap(),
                pwm_forward: LedcDriver::new(peripherals.ledc.channel2, &motor_pwm, peripherals.pins.gpio12).unwrap(),
                pwm_backward: LedcDriver::new(peripherals.ledc.channel3, &motor_pwm, peripherals.pins.gpio13).unwrap(),
            },
        };

        let emergency_stop = PinDriver::input(Into::<AnyInputPin>::into(peripherals.pins.gpio15)).unwrap();

        Self {
            heartbeat_led: Some(heartbeat_led),
            ble,
            line_sensor: Some(line_sensor),
            buttons: Some(buttons),
            display: Some(display),
            vbatt: Some(vbatt),
            vlx_sensor: Some(vlx_sensor),
            motors: Some(motors),
            emergency_stop: Some(emergency_stop),
        }
    }

    fn restart() {
        esp_idf_svc::hal::reset::restart();
    }

    fn bt_mac_address(&self) -> [u8; 6] {
        let mut mac = [0; 6];
        let mac_type = esp_idf_svc::sys::esp_mac_type_t_ESP_MAC_BT;
        // SAFETY: MAC address is 6-byte long
        unsafe { esp_idf_svc::sys::esp_read_mac(mac.as_mut_ptr() as *mut _, mac_type) };
        mac
    }

    fn heartbeat_led(&mut self) -> Option<Self::Led> {
        self.heartbeat_led.take()
    }

    fn line_sensor(&mut self) -> Option<TCA6408<Self::I2c>> {
        self.line_sensor.take()
    }

    fn buttons(&mut self) -> Option<PamiButtons<Self::I2c>> {
        self.buttons.take()
    }

    fn display(&mut self) -> Option<Self::Display> {
        self.display.take()
    }

    fn vbatt(&mut self) -> Option<Self::Vbatt> {
        self.vbatt.take()
    }

    fn vlx_sensor(&mut self) -> Option<Self::Vlx> {
        self.vlx_sensor.take()
    }

    fn motors(&mut self) -> Option<PamiMotors<Self::MotorEncoder, Self::MotorPwm>> {
        self.motors.take()
    }

    fn emergency_stop(&mut self) -> Option<Box<dyn FnMut() -> bool>> {
        let pin = self.emergency_stop.take()?;
        let f = move || { pin.is_low() };
        Some(Box::new(f))
    }

    fn rome<F: Fn([u8; 6], u32) + Send + Sync +'static>(&mut self, device_name: String, passkey_notifier: F) -> Option<(Sender<Box<[u8]>>, Receiver<Box<[u8]>>)> {
        let ble = self.ble.take()?;
        // Note: for now, client is not used, so we can easily initialize both server and client
        // and drop the client. But if the client (and `.with_scanner()`) are needed,
        // another approach must be implemented. Maybe by changing the BLE API.
        let (ble_server, _ble_client) = BleBuilder::new(ble)
            .with_passkey_notifier(move |addr, key| { passkey_notifier(addr.into(), key) })
            .run();
        Some(RomePeripheral::run(ble_server, device_name))
    }
}


pub struct PamiVbatt(AdcChannelDriver<'static, Gpio1, Rc<AdcDriver<'static, ADC1>>>);

impl PamiVbatt {
    const fn raw_to_mv(raw: f32) -> f32 {
        const VBATT_RL_KOHMS: f32 = 91.0;
        const VBATT_RH_KOHMS: f32 = 91.0;
        const ADC_INPUT_IMP_KOHMS: f32 = 500.0;
        const RL_KOHMS: f32 = VBATT_RL_KOHMS * ADC_INPUT_IMP_KOHMS / (VBATT_RL_KOHMS + ADC_INPUT_IMP_KOHMS);
        raw * (1.0 + VBATT_RH_KOHMS/RL_KOHMS)
    }

    fn mv_to_percent(measure_mv: f32) -> u8 {
        // LiFePO4 voltage to percentage lookup table (voltage in mV, percentage points)
        const LIFEPO4_CURVE: [(f32, f32); 11] = [
            (3400.0, 100.0),
            (3350.0, 90.0),
            (3320.0, 80.0),
            (3300.0, 70.0),
            (3270.0, 60.0),
            (3260.0, 50.0),
            (3250.0, 40.0),
            (3220.0, 35.0),
            (3200.0, 20.0),
            (3000.0, 10.0),
            (2500.0, 0.0),
        ];

        match LIFEPO4_CURVE.iter().position(|(mv, _)| measure_mv >= *mv) {
            None => 0,
            Some(0) => 100,
            Some(i) => {
                let (mv0, pct0) = LIFEPO4_CURVE[i];
                let (mv1, pct1) = LIFEPO4_CURVE[i - 1];
                let pct = pct0 + (pct1 - pct0) * (measure_mv - mv0) / (mv1 - mv0);
                pct as u8
            }
        }
    }
}

impl Vbatt for PamiVbatt {
    fn read_vbatt(&mut self) -> (u16, u8) {
        let raw = self.0.read().unwrap() as f32;
        let mv = Self::raw_to_mv(raw);
        let pct = Self::mv_to_percent(mv);
        (mv as u16, pct)
    }
}


pub struct PamiVlxSensor {
    sensor: VL53L5CX,
    enable: PinDriver<'static, AnyOutputPin, Output>,
}

impl VlxSensor for PamiVlxSensor {
    fn init(&mut self) -> Result<(), VlxError> {
        // Disable then enable VLX
        self.enable.set_low().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));
        self.enable.set_high().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100));
        // Initialize sensor
        self.sensor.init()
    }

    fn get_distance(&mut self) -> Result<DistanceData, VlxError> {
        self.sensor.get_distance()
    }

    fn set_alarms(&mut self, alarms: &[ZoneAlarm]) -> Result<(), VlxError> {
        self.sensor.set_alarms(alarms)
    }
}


pub struct EspEncoder<'d> {
    unit: PcntDriver<'d>,
    approx_value: Arc<AtomicI32>,
}

impl<'d> EspEncoder<'d> {
    pub fn new<PCNT: Pcnt>(
        pcnt: impl Peripheral<P = PCNT> + 'd,
        pin_a: impl Peripheral<P = impl InputPin> + 'd,
        pin_b: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self, EspError> {
        const LOW_LIMIT: i16 = -100;
        const HIGH_LIMIT: i16 = 100;

        let mut unit = PcntDriver::new(
            pcnt,
            Some(pin_a),
            Some(pin_b),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
        )?;
        unit.channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;
        unit.channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )?;
        unit.set_filter_value(1023)?;
        unit.filter_enable()?;

        //TODO Use "overflow" operations to avoid this value
        let approx_value = Arc::new(AtomicI32::new(0));
        // unsafe interrupt code to catch the upper and lower limits from the encoder
        // and track the overflow in `value: Arc<AtomicI32>` - I plan to use this for
        // a wheeled robot's odomerty
        //TODO Remove `unsafe`
        unsafe {
            let approx_value = approx_value.clone();
            unit.subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                }
            })?;
        }
        unit.event_enable(PcntEvent::HighLimit)?;
        unit.event_enable(PcntEvent::LowLimit)?;
        unit.counter_pause()?;
        unit.counter_clear()?;
        unit.counter_resume()?;

        Ok(Self { unit, approx_value })
    }
}

impl Encoder<i32> for EspEncoder<'_> {
    type Error = EspError;

    fn get_value(&self) -> Result<i32, EspError> {
        //TODO check
        let value = self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i32;
        Ok(value)
    }
}
