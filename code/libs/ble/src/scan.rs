use esp_idf_svc::bt::BdAddr;
use esp_idf_svc::sys;


/// Scan result passed to BleClient scan handler
///
/// Structure is derived from `sys::esp_ble_gap_cb_param_t_ble_scan_result_evt_param`.
/// It does not expose all data from the C structure but can be extended if needed.
///
/// This structure will be removed when esp-idf provides a proper `BleGapEvent::ScanResult`.
pub struct BleScanResult {
    raw: sys::esp_ble_gap_cb_param_t_ble_scan_result_evt_param,
    pub addr: BdAddr,
}

impl BleScanResult {
    pub fn adv_data(&self) -> &[u8] {
        if self.raw.adv_data_len > 0 {
            // Safety: slice validity is ensured by the C interface
            unsafe {
                std::slice::from_raw_parts(self.raw.ble_adv.as_ptr(), self.raw.adv_data_len as usize)
            }
        } else {
            &[]
        }
    }

    /// Iterate on advertisement data structures
    ///
    /// Return slices for each structure, without length but with AD type
    pub fn iter_ad(&self) -> AdvDataIter {
        AdvDataIter { data: self.adv_data() }
    }

    /// Return advertised local name, if found
    pub fn local_name(&self) -> Option<&str> {
        for ad_data in self.iter_ad() {
            match ad_data {
                AdData::ShortenedLocalName(s) => return Some(s),
                AdData::CompleteLocalName(s) => return Some(s),
                _ => {}
            }
        }
        None
    }
}


/// Common AD data types
///
/// https://www.bluetooth.com/specifications/assigned-numbers/
pub enum AdData<'a> {
    Flags(u8),
    ShortenedLocalName(&'a str),
    CompleteLocalName(&'a str),
    Empty,
    Unknown(u8, &'a [u8]), 
    Invalid(u8, &'a [u8]),
}

impl<'a> From<&'a [u8]> for AdData<'a> {
    fn from(value: &'a [u8]) -> Self {
        if value.is_empty() {
            return Self::Empty;
        }
        match value[0] {
            0x01 => if value.len() == 1 {
                return Self::Flags(value[1]);
            }
            0x08 => if let Ok(s) = str::from_utf8(&value[1..]) {
                return Self::ShortenedLocalName(s);
            }
            0x09 => if let Ok(s) = str::from_utf8(&value[1..]) {
                return Self::CompleteLocalName(s);
            }
            _ => {
                return Self::Unknown(value[0], &value[1..]);
            }
        }
        Self::Invalid(value[0], &value[1..])
    }
}



pub struct AdvDataIter<'a> {
    data: &'a [u8],
}

impl<'a> Iterator for AdvDataIter<'a> {
    type Item = AdData<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.data.is_empty() {
            None
        } else {
            let len = self.data[0];
            let (head, tail) = self.data.split_at_checked(1 + len as usize)?;
            self.data = tail;
            Some(head[1..].into())
        }
    }
}


impl From<sys::esp_ble_gap_cb_param_t_ble_scan_result_evt_param> for BleScanResult {
    fn from(value: sys::esp_ble_gap_cb_param_t_ble_scan_result_evt_param) -> Self {
        let addr = BdAddr::from_bytes(value.bda);
        Self {
            raw: value,
            addr,
        }
    }
}


