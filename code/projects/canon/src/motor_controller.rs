use canon_2019::hardware::{UDrivePin, VDrivePin, UEnablePin, VEnablePin, WDrivePin, WEnablePin, DebugUart};
use embassy_stm32::{usart::UartTx, peripherals::USART3, gpio::{Output, Pin}};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

use crate::hall_sensors::Phase;

const MOTOR_CONTROL_DEBUG_HALL : bool = true;

pub struct MotorDriveIO<DrivePin : Pin, EnablePin : Pin> {
    pub drive_pin: Output<'static, DrivePin>,
    pub enable_pin: Output<'static, EnablePin>,
}

pub struct MotorState {
    encoder : i32,    
    target_encoder : i32,
    phase: Phase,

    u_io: Option<MotorDriveIO<UDrivePin, UEnablePin>>,
    v_io: Option<MotorDriveIO<VDrivePin, VEnablePin>>,
    w_io: Option<MotorDriveIO<WDrivePin, WEnablePin>>,

    debug_uart : Option<UartTx<'static, DebugUart>>,
}

static INSTANCE: Mutex<ThreadModeRawMutex, MotorState> = Mutex::new(MotorState::new());

impl MotorState {
    const fn new() -> Self {
        Self {
            encoder: 0,     
            target_encoder: 0,       
            phase: Phase::U,
            debug_uart: None,
            u_io: None,
            v_io: None,
            w_io: None,
        }
    }

    pub fn set_debug_uart(&mut self, uart: UartTx<'static, USART3>) {
        self.debug_uart = Some(uart);
    }

    pub fn set_io(&mut self, u_io: MotorDriveIO<UDrivePin, UEnablePin>, v_io: MotorDriveIO<VDrivePin, VEnablePin>, w_io: MotorDriveIO<WDrivePin, WEnablePin>) {
        self.u_io = Some(u_io);
        self.v_io = Some(v_io);
        self.w_io = Some(w_io);
    }

    pub fn update_phase(&mut self, phase: Phase) {
        let mut delta = phase.as_isize() - self.phase.as_isize();
        self.phase = phase;

        if delta >= 3 {
            delta -= 6;
        }
        else if delta <= -3 {
            delta += 6;
        }

        //TODO: bug when changing direction ????

        self.encoder += delta as i32;
        self.update_motor_io();

        if MOTOR_CONTROL_DEBUG_HALL {
            if let Some(debug_uart) = &mut self.debug_uart {
                let _ = debug_uart.blocking_write(b"Hall: ");
                let _ = debug_uart.blocking_write(&self.phase.to_str().as_bytes());
                let _ = debug_uart.blocking_write(b"\r\n");
            }
        }
    }

    pub fn step_requested(&mut self, forward: bool) {
        if forward {
            self.target_encoder += 1;
        }
        else {
            self.target_encoder -= 1;
        }

        self.update_motor_io();
    }

    fn update_motor_io(&mut self) {
        let u_io = self.u_io.as_mut().unwrap();
        let v_io = self.v_io.as_mut().unwrap();
        let w_io = self.w_io.as_mut().unwrap();
        
        
        //TODO: implement control logic
        u_io.drive_pin.set_low();
        v_io.drive_pin.set_low();
        w_io.drive_pin.set_low();
                
        u_io.enable_pin.set_low();
        v_io.enable_pin.set_low();
        w_io.enable_pin.set_low();           
    }

    pub fn get_mutex() ->  &'static Mutex<ThreadModeRawMutex, MotorState> {
        &INSTANCE
    }
}