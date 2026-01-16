
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

macro_rules! impl_serialize_le_bytes {
    ($name:ident) => {
        impl Serialize for $name {
            fn serialized_size(&self) -> usize { core::mem::size_of::<Self>() }

            fn serialize<W: Writer>(&self, encoder: &mut W) {
                encoder.write(&self.to_le_bytes())
            }
        }
    }
}

impl_serialize_le_bytes!(u8);
impl_serialize_le_bytes!(i8);
impl_serialize_le_bytes!(u16);
impl_serialize_le_bytes!(i16);
impl_serialize_le_bytes!(u32);
impl_serialize_le_bytes!(i32);
impl_serialize_le_bytes!(f32);

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

