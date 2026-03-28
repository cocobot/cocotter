//! M703A laser distance module driver
//!
//! Protocol: UART 19200 baud, 8N1, full-duplex
//! Commands are single ASCII chars. Responses are ASCII strings terminated by \r\n.
//! Continuous mode: nCTRL pin LOW + send D/M/F command → continuous measurements.

use embedded_io_async::{Read, Write};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use rtt_target::rprintln;

// Commands
const CMD_LASER_ON: u8 = b'O';
const CMD_FAST_MEASURE: u8 = b'F';
const CMD_POWER_OFF: u8 = b'X';
const CMD_VERSION: u8 = b'V';

const READ_BYTE_TIMEOUT: Duration = Duration::from_millis(500);
const OK_RESPONSE_TIMEOUT: Duration = Duration::from_secs(1);
const LINE_BUF_SIZE: usize = 32;

#[derive(Debug, Clone, Copy, Default)]
pub struct LidarMeasurement {
    pub distance_mm: u32,
    pub signal_quality: u16,
    pub valid: bool,
}

impl LidarMeasurement {
    pub const fn new() -> Self {
        Self {
            distance_mm: 0,
            signal_quality: 0,
            valid: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum M703aError {
    TxError,
    RxError,
    Timeout,
    ParseError,
    ModuleError(u8),
}

pub struct M703a<TX, RX> {
    tx: TX,
    rx: RX,
    id: u8,
    last_measurement: LidarMeasurement,
    last_valid_tick: Instant,
}

impl<TX: Write, RX: Read> M703a<TX, RX> {
    pub fn new(id: u8, tx: TX, rx: RX) -> Self {
        Self {
            tx,
            rx,
            id,
            last_measurement: LidarMeasurement::new(),
            last_valid_tick: Instant::now(),
        }
    }

    pub fn last_measurement(&self) -> LidarMeasurement {
        self.last_measurement
    }

    pub fn is_stale(&self, timeout: Duration) -> bool {
        Instant::now().duration_since(self.last_valid_tick) > timeout
    }

    pub async fn send_command(&mut self, cmd: u8) -> Result<(), M703aError> {
        self.tx
            .write_all(&[cmd])
            .await
            .map_err(|_| M703aError::TxError)?;
        self.tx.flush().await.map_err(|_| M703aError::TxError)?;
        Ok(())
    }

    async fn flush_rx(&mut self) {
        let mut buf = [0u8; 1];
        loop {
            if with_timeout(Duration::from_millis(1), self.rx.read(&mut buf))
                .await
                .is_err()
            {
                break;
            }
        }
    }

    /// Wait for ",OK!" response with timeout
    async fn wait_ok(&mut self) -> Result<(), M703aError> {
        // We look for "OK!" in the stream
        let mut matched = 0u8; // 0='O', 1='K', 2='!'
        let pattern = b"OK!";
        let deadline = Instant::now() + OK_RESPONSE_TIMEOUT;

        loop {
            if Instant::now() > deadline {
                return Err(M703aError::Timeout);
            }
            let mut buf = [0u8; 1];
            let remaining = deadline.duration_since(Instant::now());
            match with_timeout(remaining, self.rx.read(&mut buf)).await {
                Ok(Ok(1)) => {
                    if buf[0] == pattern[matched as usize] {
                        matched += 1;
                        if matched == 3 {
                            return Ok(());
                        }
                    } else if buf[0] == pattern[0] {
                        matched = 1;
                    } else {
                        matched = 0;
                    }
                }
                Ok(Ok(_)) => continue,
                Ok(Err(_)) => return Err(M703aError::RxError),
                Err(_) => return Err(M703aError::Timeout),
            }
        }
    }

    /// Turn on laser. Call this before nCTRL goes low.
    pub async fn laser_on(&mut self) -> Result<(), M703aError> {
        self.flush_rx().await;
        self.send_command(CMD_LASER_ON).await?;
        self.wait_ok().await?;
        Timer::after_millis(50).await;
        Ok(())
    }

    /// Read module version. Send 'V', read response line, return as string.
    pub async fn read_version(&mut self) -> Result<[u8; LINE_BUF_SIZE], M703aError> {
        self.flush_rx().await;
        self.send_command(CMD_VERSION).await?;
        let mut buf = [0u8; LINE_BUF_SIZE];
        let len = self.read_line(&mut buf).await?;
        // Zero out the rest for clean printing
        for b in &mut buf[len..] {
            *b = 0;
        }
        Ok(buf)
    }

    /// Start fast continuous measurement. nCTRL must already be LOW.
    pub async fn start_continuous(&mut self) -> Result<(), M703aError> {
        self.send_command(CMD_FAST_MEASURE).await?;
        self.last_valid_tick = Instant::now();
        Ok(())
    }

    /// Power off the module, then laser on + start continuous again
    pub async fn reset(&mut self) {
        self.send_command(CMD_POWER_OFF).await.ok();
        Timer::after_millis(200).await;
        if let Err(e) = self.laser_on().await {
            rtt_target::rprintln!("Lidar {}: reset laser_on failed: {:?}", self.id, e);
            return;
        }
        if let Err(e) = self.start_continuous().await {
            rtt_target::rprintln!("Lidar {}: reset start_continuous failed: {:?}", self.id, e);
        }
    }

    /// Read bytes until \n, returns number of bytes written to buf (excluding \n)
    async fn read_line(&mut self, buf: &mut [u8]) -> Result<usize, M703aError> {
        let mut pos = 0;
        loop {
            if pos >= buf.len() {
                // Line too long, discard
                return Err(M703aError::ParseError);
            }
            let mut byte = [0u8; 1];
            match with_timeout(READ_BYTE_TIMEOUT, self.rx.read(&mut byte)).await {
                Ok(Ok(1)) => {
                    if byte[0] == b'\n' {
                        return Ok(pos);
                    }
                    buf[pos] = byte[0];
                    pos += 1;
                }
                Ok(Ok(_)) => continue,
                Ok(Err(_)) => return Err(M703aError::RxError),
                Err(_) => return Err(M703aError::Timeout),
            }
        }
    }

    /// Read one measurement from the continuous stream
    pub async fn read_measurement(&mut self) -> Result<LidarMeasurement, M703aError> {
        let mut buf = [0u8; LINE_BUF_SIZE];
        let len = self.read_line(&mut buf).await?;
        let m = parse_response(&buf[..len])?;
        self.last_measurement = m;
        self.last_valid_tick = Instant::now();
        Ok(m)
    }
}

/// Parse a M703A response line (without trailing \n).
/// Normal: " 1.494m,0054" or "12.345m,0079"
/// Error:  ":Er.05!"
fn parse_response(line: &[u8]) -> Result<LidarMeasurement, M703aError> {
    // Trim trailing \r
    let line = if line.last() == Some(&b'\r') {
        &line[..line.len() - 1]
    } else {
        line
    };

    rprintln!("Parsing line: {}", core::str::from_utf8(line).unwrap_or("<invalid utf8>"));

    // Trim leading spaces
    let line = trim_leading_spaces(line);

    if line.is_empty() {
        return Err(M703aError::ParseError);
    }

    // Strip leading ':' (M703A prefixes both measurements and errors with ':')
    let line = if line[0] == b':' {
        trim_leading_spaces(&line[1..])
    } else {
        line
    };

    if line.is_empty() {
        return Err(M703aError::ParseError);
    }

    // Error response: "ErXX!"
    if line.len() >= 2 && line[0] == b'E' && line[1] == b'r' {
        return parse_error(line);
    }

    // Find 'm' (end of distance) and ',' (separator)
    let m_pos = find_byte(line, b'm').ok_or(M703aError::ParseError)?;
    let comma_pos = find_byte(line, b',').ok_or(M703aError::ParseError)?;

    if comma_pos <= m_pos {
        return Err(M703aError::ParseError);
    }

    let distance_str = &line[..m_pos];
    let quality_str = &line[comma_pos + 1..];

    let distance_mm = parse_distance_mm(distance_str)?;
    let signal_quality = parse_u16_from_bytes(quality_str).ok_or(M703aError::ParseError)?;

    Ok(LidarMeasurement {
        distance_mm,
        signal_quality,
        valid: true,
    })
}

/// Parse "ErXX!" → ModuleError(XX)
fn parse_error(line: &[u8]) -> Result<LidarMeasurement, M703aError> {
    // Format: "ErXX!" - find digits between "Er" and "!"
    let bang_pos = find_byte(line, b'!').ok_or(M703aError::ParseError)?;
    if bang_pos <= 2 {
        return Err(M703aError::ParseError);
    }
    let code = parse_u16_from_bytes(&line[2..bang_pos]).ok_or(M703aError::ParseError)?;
    Err(M703aError::ModuleError(code as u8))
}

/// Parse "12.345" → 12345 mm, "0.848" → 848 mm
/// M703A always returns 3 decimal places, so frac_val is directly mm.
fn parse_distance_mm(s: &[u8]) -> Result<u32, M703aError> {
    let dot_pos = find_byte(s, b'.');
    match dot_pos {
        Some(dp) => {
            let int_part = parse_u32_from_bytes(&s[..dp]).ok_or(M703aError::ParseError)?;
            let frac_val = parse_u32_from_bytes(&s[dp + 1..]).ok_or(M703aError::ParseError)?;
            Ok(int_part * 1000 + frac_val)
        }
        None => {
            let meters = parse_u32_from_bytes(s).ok_or(M703aError::ParseError)?;
            Ok(meters * 1000)
        }
    }
}

fn parse_u32_from_bytes(bytes: &[u8]) -> Option<u32> {
    if bytes.is_empty() {
        return None;
    }
    let mut result: u32 = 0;
    for &b in bytes {
        if !b.is_ascii_digit() {
            return None;
        }
        result = result.checked_mul(10)?.checked_add((b - b'0') as u32)?;
    }
    Some(result)
}

fn parse_u16_from_bytes(bytes: &[u8]) -> Option<u16> {
    parse_u32_from_bytes(bytes).and_then(|v| u16::try_from(v).ok())
}

fn find_byte(s: &[u8], needle: u8) -> Option<usize> {
    s.iter().position(|&b| b == needle)
}

fn trim_leading_spaces(s: &[u8]) -> &[u8] {
    let start = s.iter().position(|&b| b != b' ').unwrap_or(s.len());
    &s[start..]
}
