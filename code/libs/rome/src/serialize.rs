
pub trait Writer {
    fn write(&mut self, data: &[u8]);
}


impl Writer for Vec<u8> {
    fn write(&mut self, data: &[u8]) {
        self.extend_from_slice(data)
    }
}


pub trait Serialize {
    fn serialized_size(&self) -> usize;
    fn serialize<W: Writer>(&self, encoder: &mut W);
}

impl Serialize for u8 {
    fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

    fn serialize<W: Writer>(&self, encoder: &mut W) {
        encoder.write(&[*self])
    }
}

impl Serialize for u16 {
    fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

    fn serialize<W: Writer>(&self, encoder: &mut W) {
        encoder.write(&self.to_le_bytes())
    }
}

impl Serialize for u32 {
    fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

    fn serialize<W: Writer>(&self, encoder: &mut W) {
        encoder.write(&self.to_le_bytes())
    }
}

impl Serialize for f32 {
    fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

    fn serialize<W: Writer>(&self, encoder: &mut W) {
        encoder.write(&self.to_le_bytes())
    }
}

impl Serialize for bool {
    fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

    fn serialize<W: Writer>(&self, encoder: &mut W) {
        u8::from(*self).serialize(encoder)
    }
}

impl<T: Serialize, const N: usize> Serialize for [T; N] {
    fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

    fn serialize<W: Writer>(&self, encoder: &mut W) {
        for item in self {
            item.serialize(encoder);
        }
    }
}


