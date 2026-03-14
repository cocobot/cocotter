use esp_idf_svc::bt::BdAddr;
use esp_idf_svc::bt::ble::gap::GapSearchResult;


/// Scan result passed to BleClient scan handler
///
/// Wrap the underlying esp-idf-svc structure to provide helpers.
/// It does not expose all data but can be extended if needed.
pub struct BleScanResult<'a>(pub GapSearchResult<'a>);

impl BleScanResult<'_> {
    pub fn addr(&self) -> BdAddr {
        self.0.bda
    }

    pub fn adv_data(&self) -> &[u8] {
        self.0.ble_adv.unwrap_or(&[])
    }

    /// Iterate on advertisement data structures
    ///
    /// Return slices for each structure, without length but with AD type
    pub fn iter_ad(&self) -> AdvDataIter<'_> {
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

