use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

const DEFAULT_TIMEOUT: Duration = Duration::from_secs(3);

struct WatchedInner<T> {
    value: T,
    generation: u64,
}

/// Thread-safe container that notifies waiters on update.
///
/// - `get()` returns the current value immediately.
/// - `wait_next()` blocks until the next `update()` call and returns the new value.
pub struct Watched<T> {
    inner: Arc<(Mutex<WatchedInner<T>>, Condvar)>,
}

impl<T> Clone for Watched<T> {
    fn clone(&self) -> Self {
        Self { inner: self.inner.clone() }
    }
}

impl<T: Default> Default for Watched<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

impl<T> Watched<T> {
    pub fn new(value: T) -> Self {
        Self {
            inner: Arc::new((
                Mutex::new(WatchedInner { value, generation: 0 }),
                Condvar::new(),
            )),
        }
    }

    /// Update the value and notify all waiters.
    pub fn update(&self, f: impl FnOnce(&mut T)) {
        let (lock, cvar) = &*self.inner;
        if let Ok(mut inner) = lock.lock() {
            f(&mut inner.value);
            inner.generation += 1;
            cvar.notify_all();
        }
    }

    /// Get the current value (copy).
    pub fn get(&self) -> T where T: Copy {
        let (lock, _) = &*self.inner;
        lock.lock().map(|inner| inner.value).unwrap()
    }

    /// Block until the next update (timeout 3s), then return the new value.
    /// Returns `None` on timeout.
    pub fn wait_next(&self) -> Option<T> where T: Copy {
        let (lock, cvar) = &*self.inner;
        let inner = lock.lock().ok()?;
        let cur_gen = inner.generation;
        let (inner, result) = cvar.wait_timeout_while(inner, DEFAULT_TIMEOUT, |i| i.generation == cur_gen).ok()?;
        if result.timed_out() {
            None
        } else {
            Some(inner.value)
        }
    }
}
