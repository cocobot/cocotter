use esp_idf_svc::hal::{adc, delay::TickType, gpio::{AnyInputPin, Level, PinDriver}, i2c::{I2cConfig, I2cDriver}, ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver}, pcnt::PcntDriver, peripherals::Peripherals, spi, units::KiloHertz};

fn main()  {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    //init all the peripherals
    let peripherals = Peripherals::take().unwrap();
    let mut adc = adc::AdcDriver::new(peripherals.adc1, &adc::config::Config::new().calibration(true)).unwrap();
    let timer_motors = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &TimerConfig::default().frequency(KiloHertz(16).into()),
    ).unwrap();
    let mut i2c_driver = I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio18, peripherals.pins.gpio17, &I2cConfig::new().baudrate(KiloHertz(100).into())).unwrap();
    let spi_driver = spi::SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio48,
        peripherals.pins.gpio36,
        Some(peripherals.pins.gpio35),
        &spi::SpiDriverConfig::new(),
    ).unwrap();
    
    let mut battery_level_input: adc::AdcChannelDriver<{ adc::attenuation::NONE }, _> = adc::AdcChannelDriver::new(peripherals.pins.gpio1).unwrap();
    let start_robot_input = PinDriver::input(peripherals.pins.gpio2).unwrap();
    let mut led_run_output = PinDriver::output(peripherals.pins.gpio4).unwrap();
    let mut led_com_output = PinDriver::output(peripherals.pins.gpio5).unwrap();
    let contact_left_input = PinDriver::input(peripherals.pins.gpio6).unwrap();
    let conf3_input = PinDriver::input(peripherals.pins.gpio7).unwrap();
    let tof_irq_input = PinDriver::input(peripherals.pins.gpio8).unwrap();
    let mut current_right_motor_input: adc::AdcChannelDriver<{ adc::attenuation::NONE }, _> = adc::AdcChannelDriver::new(peripherals.pins.gpio9).unwrap();
    let mut current_left_motor_input: adc::AdcChannelDriver<{ adc::attenuation::NONE }, _> = adc::AdcChannelDriver::new(peripherals.pins.gpio10).unwrap();
    let io_irq_input = PinDriver::input(peripherals.pins.gpio11).unwrap();
    let mut right_motor_dir_output = PinDriver::output(peripherals.pins.gpio12).unwrap();
    let mut right_motor_pwm = LedcDriver::new(
        peripherals.ledc.channel0,
        &timer_motors,
        peripherals.pins.gpio13,
    ).unwrap();
    let mut left_motor_dir_output = PinDriver::output(peripherals.pins.gpio14).unwrap();
    let emergency_stop_input = PinDriver::input(peripherals.pins.gpio15).unwrap();
    let mut enable_front_tof  = PinDriver::output(peripherals.pins.gpio16).unwrap();
    let mut left_motor_pwm = LedcDriver::new(
        peripherals.ledc.channel1,
        &timer_motors,
        peripherals.pins.gpio21,
    ).unwrap();
    let mut gyro_spi = spi::SpiDeviceDriver::new(
        spi_driver, 
        Some(peripherals.pins.gpio37), 
        &spi::config::Config::new().baudrate(KiloHertz(100).into()).data_mode(spi::config::MODE_3)
    ).unwrap();
    let gyro_irq_input = PinDriver::input(peripherals.pins.gpio38).unwrap();
    let left_encoder = PcntDriver::new(
        peripherals.pcnt0,
        Some(peripherals.pins.gpio39),
        Some(peripherals.pins.gpio40),
        Option::<AnyInputPin>::None,
        Option::<AnyInputPin>::None,
    ).unwrap();
    let right_encoder = PcntDriver::new(
        peripherals.pcnt1,
        Some(peripherals.pins.gpio42),
        Some(peripherals.pins.gpio41),
        Option::<AnyInputPin>::None,
        Option::<AnyInputPin>::None,
    ).unwrap();
    //TODO QUAD ENC
    let contact_right_input = PinDriver::input(peripherals.pins.gpio47).unwrap();

    let mut loop_counter = 0;
    loop {
        log::info!("Batery ADC value: {}", adc.read(&mut battery_level_input).unwrap());
        log::info!("Start Robot input: {:?}", start_robot_input.get_level());
        led_run_output.set_level(if loop_counter % 3 == 0 { Level::High } else { Level::Low }).unwrap();
        led_com_output.set_level(if loop_counter % 3 == 1 { Level::Low } else { Level::High }).unwrap();
        log::info!("Contact left: {:?}", contact_left_input.get_level());
        log::info!("Conf3: {:?}", conf3_input.get_level());
        log::info!("TOF IRQ: {:?}", tof_irq_input.get_level());
        log::info!("Current right motor ADC value: {}", adc.read(&mut current_right_motor_input).unwrap());
        log::info!("Current left motor ADC value: {}", adc.read(&mut current_left_motor_input).unwrap());
        log::info!("IO IRQ: {:?}", io_irq_input.get_level());
        right_motor_dir_output.set_level(if (loop_counter / 2) % 2 == 0 { Level::High } else { Level::Low }).unwrap();
        right_motor_pwm.set_duty(if loop_counter % 2 == 0 { 500 } else { 1000 }).unwrap();
        left_motor_dir_output.set_level(if (loop_counter / 2) % 2 == 0 { Level::Low } else { Level::High }).unwrap();
        log::info!("Emergency stop: {:?}", emergency_stop_input.get_level());
        enable_front_tof.set_level(if loop_counter % 2 == 0 { Level::High } else { Level::Low }).unwrap();
        left_motor_pwm.set_duty(if loop_counter % 2 == 0 { 500 } else { 1000 }).unwrap();
        log::info!("Conact right: {:?}", contact_right_input.get_level());
        log::info!("Gyro IRQ: {:?}", gyro_irq_input.get_level());
        log::info!("Left encoder value: {}", left_encoder.get_counter_value().unwrap());
        log::info!("Right encoder value: {}", right_encoder.get_counter_value().unwrap());
        
        let mut buf = [0x0F, 0x00];
        gyro_spi.transfer_in_place(&mut buf).unwrap();
        log::info!("Gyro WHO_AM_I read: {:?}(expected {:?})", buf[1], 0x6C);

        for addr in 1..=127 {
            match i2c_driver.read(addr as u8, &mut [0], TickType::new_millis(5).into()) {
                Ok(_) => log::info!("I2C device found at address 0x{:02X}", addr),
                Err(_) => (),
            }
        }

        std::thread::sleep(std::time::Duration::from_secs(500));
        loop_counter += 1;
    }
}
