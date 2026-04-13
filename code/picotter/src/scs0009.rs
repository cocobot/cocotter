//! SCS0009 servo driver (half-duplex UART)
//!
//! Binary packet protocol: [0xFF, 0xFF, ID, LEN, INST, ...PARAMS, CHECKSUM]
//! - INST_PING (0x01): Check if servo is present
//! - INST_READ (0x02): Read register(s)
//! - INST_WRITE (0x03): Write register(s)

use embedded_io_async::{Read, Write};
use embassy_time::{with_timeout, Duration, Timer};

const HEADER: [u8; 2] = [0xFF, 0xFF];

// Instructions
const INST_PING: u8 = 0x01;
const INST_READ: u8 = 0x02;
const INST_WRITE: u8 = 0x03;

// SCS0009 registers
const REG_ID: u8 = 0x05;
const REG_TORQUE_ENABLE: u8 = 0x28;
const REG_GOAL_POSITION: u8 = 0x2A;
const REG_LOCK: u8 = 0x30;
const REG_PRESENT_POSITION: u8 = 0x38;
const REG_PRESENT_LOAD: u8 = 0x3C;

pub struct Scs0009<TX, RX> {
    tx: TX,
    rx: RX,
}

impl<TX: Write, RX: Read> Scs0009<TX, RX> {
    pub fn new(tx: TX, rx: RX) -> Self {
        Self { tx, rx }
    }

    /// Set servo ID via broadcast (0xFE)
    /// Call `unlock_eeprom()` before and `lock_eeprom()` after to persist
    pub async fn set_id(&mut self, new_id: u8) -> Result<(), ScsError> {
        self.write_byte(0xFE, REG_ID, new_id).await
    }

    pub async fn modify_id(&mut self, init_id: u8, new_id: u8) -> Result<(), ScsError> {
        self.write_byte(init_id, REG_ID, new_id).await
    }

    /// Change a specific servo's ID
    /// Call `unlock_eeprom()` before and `lock_eeprom()` after to persist
    pub async fn change_id(&mut self, old_id: u8, new_id: u8) -> Result<(), ScsError> {
        self.write_byte(old_id, REG_ID, new_id).await
    }

    /// Unlock EEPROM to allow parameter writes
    pub async fn unlock_eeprom(&mut self, id: u8) -> Result<(), ScsError> {
        self.write_byte(id, REG_LOCK, 0).await
    }

    /// Lock EEPROM to save parameters
    pub async fn lock_eeprom(&mut self, id: u8) -> Result<(), ScsError> {
        self.write_byte(id, REG_LOCK, 1).await
    }

    /// Ping a servo, returns true if present
    pub async fn ping(&mut self, id: u8) -> Result<bool, ScsError> {
        let len: u8 = 2;
        let checksum = Self::calc_checksum(&[id, len, INST_PING]);
        let packet = [HEADER[0], HEADER[1], id, len, INST_PING, checksum];
        self.send_packet(&packet).await?;

        match self.read_response().await {
            Ok(_) => Ok(true),
            Err(ScsError::Timeout) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Scan for servos in an ID range
    /// Returns an array of up to MAX_RESULTS found IDs
    pub async fn scan<const MAX_RESULTS: usize>(
        &mut self,
        start_id: u8,
        end_id: u8,
    ) -> ([u8; MAX_RESULTS], usize) {
        let mut found = [0u8; MAX_RESULTS];
        let mut count = 0;

        for id in start_id..=end_id {
            if count >= MAX_RESULTS {
                break;
            }
            if let Ok(true) = self.ping(id).await {
                found[count] = id;
                count += 1;
            }
        }

        (found, count)
    }

    /// Read current servo position
    /// Returns (position, servo_error_byte) — position is valid even if error != 0
    pub async fn read_position(&mut self, id: u8) -> Result<(u16, u8), ScsError> {
        self.read_word(id, REG_PRESENT_POSITION).await
    }

    /// Read current servo load (torque)
    /// Returns (load, servo_error_byte) — load is valid even if error != 0
    pub async fn read_load(&mut self, id: u8) -> Result<(u16, u8), ScsError> {
        self.read_word(id, REG_PRESENT_LOAD).await
    }

    /// Enable or disable torque
    pub async fn set_torque(&mut self, id: u8, enable: bool) -> Result<(), ScsError> {
        self.write_byte(id, REG_TORQUE_ENABLE, u8::from(enable))
            .await
    }

    /// Read a single byte from a register
    /// Returns (value, servo_error_byte)
    pub async fn read_byte(&mut self, id: u8, reg: u8) -> Result<(u8, u8), ScsError> {
        let len: u8 = 4;
        let count: u8 = 1;
        let checksum = Self::calc_checksum(&[id, len, INST_READ, reg, count]);
        let packet = [HEADER[0], HEADER[1], id, len, INST_READ, reg, count, checksum];
        self.send_packet(&packet).await?;

        let (val, err) = self.read_response_with_data(1).await?;
        Ok((val as u8, err))
    }

    /// Read a word (2 bytes) from a register
    async fn read_word(&mut self, id: u8, reg: u8) -> Result<(u16, u8), ScsError> {
        let len: u8 = 4;
        let count: u8 = 2;
        let checksum = Self::calc_checksum(&[id, len, INST_READ, reg, count]);
        let packet = [HEADER[0], HEADER[1], id, len, INST_READ, reg, count, checksum];
        self.send_packet(&packet).await?;

        self.read_response_with_data(2).await
    }

    /// Set servo position with time and speed
    /// - position: 0-1023
    /// - time: time in ms to reach position (0 = use speed)
    /// - speed: steps/sec (ignored if time > 0)
    pub async fn set_position(
        &mut self,
        id: u8,
        position: u16,
        time: u16,
        speed: u16,
    ) -> Result<(), ScsError> {
        let pos = position.min(1023);
        // Big-endian (high byte first)
        let pos_h = ((pos >> 8) & 0xFF) as u8;
        let pos_l = (pos & 0xFF) as u8;
        let time_h = ((time >> 8) & 0xFF) as u8;
        let time_l = (time & 0xFF) as u8;
        let speed_h = ((speed >> 8) & 0xFF) as u8;
        let speed_l = (speed & 0xFF) as u8;

        let len: u8 = 9; // 7 params + instruction + checksum
        let checksum = Self::calc_checksum(&[
            id, len, INST_WRITE, REG_GOAL_POSITION,
            pos_h, pos_l, time_h, time_l, speed_h, speed_l,
        ]);
        let packet = [
            HEADER[0], HEADER[1], id, len, INST_WRITE, REG_GOAL_POSITION,
            pos_h, pos_l, time_h, time_l, speed_h, speed_l, checksum,
        ];
        self.send_packet(&packet).await?;

        // Read response unless broadcast
        if id != 0xFE {
            self.read_response().await?;
        }
        Ok(())
    }

    /// Write a single byte to a register
    async fn write_byte(&mut self, id: u8, reg: u8, value: u8) -> Result<(), ScsError> {
        let len: u8 = 4;
        let checksum = Self::calc_checksum(&[id, len, INST_WRITE, reg, value]);
        let packet = [HEADER[0], HEADER[1], id, len, INST_WRITE, reg, value, checksum];
        self.send_packet(&packet).await?;

        if id != 0xFE {
            self.read_response().await?;
        }
        Ok(())
    }

    async fn send_packet(&mut self, packet: &[u8]) -> Result<(), ScsError> {
        self.flush_rx().await;
        self.tx.write_all(packet).await.map_err(|_| ScsError::TxError)?;
        self.tx.flush().await.map_err(|_| ScsError::TxError)?;

        // Wait for last byte to be transmitted (~10us per byte at 1Mbaud)
        Timer::after_micros(150).await;

        // Skip TX echo (half-duplex)
        self.skip_echo(packet.len()).await;
        Ok(())
    }

    /// Skip TX echo bytes in half-duplex mode
    async fn skip_echo(&mut self, len: usize) {
        let mut buf = [0u8; 32];
        let to_skip = len.min(buf.len());
        let mut skipped = 0;
        while skipped < to_skip {
            let result = with_timeout(
                Duration::from_millis(2),
                self.rx.read(&mut buf[..to_skip - skipped]),
            )
            .await;
            match result {
                Ok(Ok(n)) => skipped += n,
                _ => break,
            }
        }
    }

    /// Flush RX buffer
    async fn flush_rx(&mut self) {
        let mut buf = [0u8; 64];
        for _ in 0..3 {
            if with_timeout(Duration::from_millis(1), self.rx.read(&mut buf))
                .await
                .is_err()
            {
                break;
            }
        }
    }

    /// Read servo response with 50ms timeout
    async fn read_response(&mut self) -> Result<u8, ScsError> {
        let mut buf = [0u8; 6];
        let mut idx = 0;

        // Find header 0xFF 0xFF
        loop {
            let result = with_timeout(
                Duration::from_millis(50),
                self.rx.read(&mut buf[idx..idx + 1]),
            )
            .await;
            match result {
                Ok(Ok(1)) => {}
                Ok(Ok(_)) => continue,
                Ok(Err(_)) => return Err(ScsError::RxError),
                Err(_) => return Err(ScsError::Timeout),
            }

            if buf[idx] == 0xFF {
                idx += 1;
                if idx >= 2 {
                    break;
                }
            } else {
                idx = 0;
            }
        }

        // Read ID, len, error, checksum (4 bytes)
        let mut remaining = 4;
        while remaining > 0 {
            let start = 6 - remaining;
            let result = with_timeout(
                Duration::from_millis(10),
                self.rx.read(&mut buf[start..6]),
            )
            .await;
            match result {
                Ok(Ok(n)) => remaining -= n,
                Ok(Err(_)) => return Err(ScsError::RxError),
                Err(_) => return Err(ScsError::Timeout),
            }
        }

        let error = buf[4];
        if error != 0 {
            return Err(ScsError::ServoError(error));
        }

        Ok(error)
    }

    /// Read servo response with data (for READ commands)
    /// Returns (value, servo_error_byte) — value is always extracted even if error != 0
    async fn read_response_with_data(&mut self, data_len: usize) -> Result<(u16, u8), ScsError> {
        let mut buf = [0u8; 16];
        let mut idx = 0;

        // Find header 0xFF 0xFF
        loop {
            let result = with_timeout(
                Duration::from_millis(50),
                self.rx.read(&mut buf[idx..idx + 1]),
            )
            .await;
            match result {
                Ok(Ok(1)) => {}
                Ok(Ok(_)) => continue,
                Ok(Err(_)) => return Err(ScsError::RxError),
                Err(_) => return Err(ScsError::Timeout),
            }

            if buf[idx] == 0xFF {
                idx += 1;
                if idx >= 2 {
                    break;
                }
            } else {
                idx = 0;
            }
        }

        // Read: ID + LEN + ERROR + DATA + CHECKSUM
        let total_to_read = 2 + data_len + 1;
        let mut remaining = total_to_read;
        while remaining > 0 {
            let start = 2 + total_to_read - remaining;
            let result = with_timeout(
                Duration::from_millis(10),
                self.rx.read(&mut buf[start..2 + total_to_read]),
            )
            .await;
            match result {
                Ok(Ok(n)) => remaining = remaining.saturating_sub(n),
                Ok(Err(_)) => return Err(ScsError::RxError),
                Err(_) => return Err(ScsError::Timeout),
            }
        }

        let error = buf[4];

        let value = if data_len == 1 {
            buf[5] as u16
        } else {
            // Data is big-endian (high byte first)
            ((buf[5] as u16) << 8) | (buf[6] as u16)
        };
        Ok((value, error))
    }

    fn calc_checksum(data: &[u8]) -> u8 {
        let sum: u16 = data.iter().map(|&b| b as u16).sum();
        (!sum as u8) & 0xFF
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ScsError {
    TxError,
    RxError,
    Timeout,
    InvalidResponse,
    ServoError(u8),
}
