//! ESP32 self-OTA handler using ESP-IDF OTA partition APIs

use esp_idf_svc::sys;

use crate::ota::OtaHandler;

/// OTA handler for updating the ESP32 itself using ESP-IDF OTA APIs.
///
/// Writes firmware to the next OTA partition, verifies CRC32, and
/// sets the boot partition on finish. Reboot triggers `esp_restart()`.
pub struct EspSelfOtaHandler {
    ota_handle: Option<sys::esp_ota_handle_t>,
    partition: *const sys::esp_partition_t,
    expected_size: u32,
    expected_crc: u32,
    written: u32,
    crc_hasher: crc32fast::Hasher,
}

// SAFETY: esp_ota_* APIs are thread-safe and the partition pointer is valid for 'static
unsafe impl Send for EspSelfOtaHandler {}

impl EspSelfOtaHandler {
    pub fn new() -> Self {
        Self {
            ota_handle: None,
            partition: std::ptr::null(),
            expected_size: 0,
            expected_crc: 0,
            written: 0,
            crc_hasher: crc32fast::Hasher::new(),
        }
    }

    fn cleanup(&mut self) {
        if let Some(handle) = self.ota_handle.take() {
            unsafe { sys::esp_ota_abort(handle) };
        }
        self.partition = std::ptr::null();
        self.written = 0;
        self.crc_hasher = crc32fast::Hasher::new();
    }
}

impl OtaHandler for EspSelfOtaHandler {
    fn start(&mut self, size: u32, crc: u32) -> u8 {
        // Abort any previous in-progress OTA
        self.cleanup();

        // Find next OTA partition
        let partition = unsafe {
            sys::esp_ota_get_next_update_partition(std::ptr::null())
        };
        if partition.is_null() {
            log::error!("ESP OTA: no next update partition found");
            return 2; // no_space
        }

        // Check partition size
        let part_size = unsafe { (*partition).size };
        if size as u32 > part_size {
            log::error!("ESP OTA: firmware {size} bytes exceeds partition {part_size} bytes");
            return 2; // no_space
        }

        // Begin OTA
        let mut handle: sys::esp_ota_handle_t = 0;
        let rc = unsafe {
            sys::esp_ota_begin(partition, size as usize, &mut handle)
        };
        if rc != 0 {
            log::error!("ESP OTA: esp_ota_begin failed: {rc}");
            return 1; // busy
        }

        self.ota_handle = Some(handle);
        self.partition = partition;
        self.expected_size = size;
        self.expected_crc = crc;
        self.written = 0;
        self.crc_hasher = crc32fast::Hasher::new();

        log::info!("ESP OTA: started, size={size}, crc=0x{crc:08x}");
        0 // ok
    }

    fn data(&mut self, offset: u32, data: &[u8]) -> u32 {
        let handle = match self.ota_handle {
            Some(h) => h,
            None => {
                log::error!("ESP OTA: data received but no OTA in progress");
                return 0;
            }
        };

        if offset != self.written {
            log::warn!("ESP OTA: expected offset {}, got {offset}", self.written);
            return self.written;
        }

        let rc = unsafe {
            sys::esp_ota_write(handle, data.as_ptr() as *const _, data.len())
        };
        if rc != 0 {
            log::error!("ESP OTA: esp_ota_write failed at offset {offset}: {rc}");
            return self.written;
        }

        self.crc_hasher.update(data);
        self.written += data.len() as u32;
        self.written
    }

    fn finish(&mut self) -> u8 {
        let handle = match self.ota_handle.take() {
            Some(h) => h,
            None => {
                log::error!("ESP OTA: finish but no OTA in progress");
                return 2; // size_mismatch
            }
        };

        // Check size
        if self.written != self.expected_size {
            log::error!(
                "ESP OTA: size mismatch: written={}, expected={}",
                self.written, self.expected_size
            );
            unsafe { sys::esp_ota_abort(handle) };
            return 2; // size_mismatch
        }

        // Check CRC
        let computed_crc = std::mem::replace(&mut self.crc_hasher, crc32fast::Hasher::new()).finalize();
        if computed_crc != self.expected_crc {
            log::error!(
                "ESP OTA: CRC mismatch: computed=0x{computed_crc:08x}, expected=0x{:08x}",
                self.expected_crc
            );
            unsafe { sys::esp_ota_abort(handle) };
            return 1; // crc_fail
        }

        // End OTA (validates image header internally)
        let rc = unsafe { sys::esp_ota_end(handle) };
        if rc != 0 {
            log::error!("ESP OTA: esp_ota_end failed: {rc}");
            return 1; // crc_fail (image validation failed)
        }

        // Set boot partition
        let rc = unsafe { sys::esp_ota_set_boot_partition(self.partition) };
        if rc != 0 {
            log::error!("ESP OTA: esp_ota_set_boot_partition failed: {rc}");
            return 1;
        }

        log::info!("ESP OTA: finished successfully, {} bytes, CRC OK", self.written);
        0 // ok
    }

    fn reboot(&mut self) {
        log::info!("ESP OTA: rebooting...");
        unsafe { sys::esp_restart() };
    }

    fn abort(&mut self) {
        log::info!("ESP OTA: aborting");
        self.cleanup();
    }
}
