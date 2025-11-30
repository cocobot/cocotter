use core::mem::MaybeUninit;
use crate::DecodeError;


pub trait Reader {
    /// Read data, panic if there is not enough data to read
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError>;
}


impl Reader for &[u8] {
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError> {
        let (a, b) = self.split_at_checked(bytes.len()).ok_or(DecodeError::EndOfData)?;
        bytes.copy_from_slice(a);
        *self = b;
        Ok(())
    }
}


pub trait Deserialize: Sized {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError>;
}

impl Deserialize for u8 {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
        let mut buffer = [0u8; 1];
        reader.read(&mut buffer)?;
        Ok(buffer[0])
    }
}

impl Deserialize for u16 {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
        let mut buffer = [0u8; 2];
        reader.read(&mut buffer)?;
        Ok(Self::from_le_bytes(buffer))
    }
}

impl Deserialize for u32 {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
        let mut buffer = [0u8; 4];
        reader.read(&mut buffer)?;
        Ok(Self::from_le_bytes(buffer))
    }
}

impl Deserialize for f32 {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
        let mut buffer = [0u8; 4];
        reader.read(&mut buffer)?;
        Ok(Self::from_le_bytes(buffer))
    }
}

impl Deserialize for bool {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
        let mut buffer = [0u8; 1];
        reader.read(&mut buffer)?;
        Ok(buffer[0] != 0)
    }
}

impl<T: Deserialize, const N: usize> Deserialize for [T; N] {
    fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
        // Note: Use `try_from_fn()` when stabilized; see feature `array_try_from_fn`
        //core::array::try_from_fn(|_| T::deserialize(reader))

        // Drop already initialized items on exit
        struct Guard<'a, T> {
            data: &'a mut [MaybeUninit<T>],
            initialized: usize,
        }

        impl<T> Drop for Guard<'_, T> {
            fn drop(&mut self) {
                if self.initialized > 0 {
                    // SAFETY: slice contains only initialized items
                    unsafe {
                        let initialized_slice = self.data.get_unchecked_mut(..self.initialized);
                        core::ptr::drop_in_place(initialized_slice as *mut [MaybeUninit<T>] as *mut [T]);
                    }
                }
            }
        }

        let mut data = [const { MaybeUninit::uninit() }; N];
        let mut guard = Guard { data: &mut data, initialized: 0 };

        for i in 0..N {
            // SAFETY: `i < guard.data.len()`
            let item = unsafe { guard.data.get_unchecked_mut(i) };
            item.write(T::deserialize(reader)?);
            guard.initialized += 1;
        }

        // Everything has been initialized, don't drop from the guard
        core::mem::forget(guard);

        // Note: cannot use `mem::transmute` because of its limitations
        // SAFETY: all items have been initialized
        Ok(unsafe { data.as_ptr().cast::<[T; N]>().read() })
    }
}

