//! Shared memory between DanteBridge (the exe) and the ASIO driver (the DLL).
//! Implemented as a memory-mapped file in C:\Users\Public\DanteBridge\ so that
//! both processes see the same memory. Two rings of interleaved f32 frames:
//!   In  = Dante -> ASIO inputs   (written by the bridge, read by the driver)
//!   Out = ASIO outputs -> Dante  (written by the driver, read by the bridge)

use std::fs::OpenOptions;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

pub const CH: usize = 4; // channels in each direction
pub const FRAMES: usize = 32768; // frames per ring (power of two)
const HEADER: usize = 128;
const RING_BYTES: usize = FRAMES * CH * 4;
const TOTAL: usize = HEADER + 2 * RING_BYTES;

const OFF_IN_W: usize = 16;
const OFF_OUT_W: usize = 24;
const OFF_BRIDGE_BEAT: usize = 32;
const OFF_DRIVER_BEAT: usize = 40;

#[derive(Clone, Copy, PartialEq)]
pub enum Dir {
    In = 0,
    Out = 1,
}

pub struct Shm {
    _map: memmap2::MmapMut,
    base: *mut u8,
}

unsafe impl Send for Shm {}
unsafe impl Sync for Shm {}

fn file_path() -> PathBuf {
    let base = std::env::var_os("PUBLIC")
        .or_else(|| std::env::var_os("PROGRAMDATA"))
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    base.join("DanteBridge").join("asio_shm.bin")
}

impl Shm {
    pub fn open() -> Result<Shm, String> {
        let path = file_path();
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&path)
            .map_err(|e| format!("{}: {e}", path.display()))?;
        if file.metadata().map(|m| m.len()).unwrap_or(0) < TOTAL as u64 {
            file.set_len(TOTAL as u64).map_err(|e| format!("{}: {e}", path.display()))?;
        }
        let mut map = unsafe { memmap2::MmapMut::map_mut(&file) }
            .map_err(|e| format!("{}: {e}", path.display()))?;
        let base = map.as_mut_ptr();
        Ok(Shm { _map: map, base })
    }

    fn a64(&self, off: usize) -> &AtomicU64 {
        unsafe { &*(self.base.add(off) as *const AtomicU64) }
    }

    fn ring(&self, d: Dir) -> *const AtomicU32 {
        unsafe { self.base.add(HEADER + (d as usize) * RING_BYTES) as *const AtomicU32 }
    }

    fn off(d: Dir) -> usize {
        match d {
            Dir::In => OFF_IN_W,
            Dir::Out => OFF_OUT_W,
        }
    }

    pub fn write_pos(&self, d: Dir) -> u64 {
        self.a64(Self::off(d)).load(Ordering::Acquire)
    }

    pub fn set_write_pos(&self, d: Dir, v: u64) {
        self.a64(Self::off(d)).store(v, Ordering::Release);
    }

    /// data = interleaved frames (CH channels per frame)
    pub fn write_frames(&self, d: Dir, pos: u64, data: &[f32]) {
        let ring = self.ring(d);
        for (i, f) in data.chunks_exact(CH).enumerate() {
            let slot = (pos as usize).wrapping_add(i) & (FRAMES - 1);
            for c in 0..CH {
                unsafe { (*ring.add(slot * CH + c)).store(f[c].to_bits(), Ordering::Relaxed) };
            }
        }
    }

    pub fn read_frames(&self, d: Dir, pos: u64, out: &mut [f32]) {
        let ring = self.ring(d);
        for (i, f) in out.chunks_exact_mut(CH).enumerate() {
            let slot = (pos as usize).wrapping_add(i) & (FRAMES - 1);
            for c in 0..CH {
                f[c] = f32::from_bits(unsafe { (*ring.add(slot * CH + c)).load(Ordering::Relaxed) });
            }
        }
    }

    pub fn bump_bridge_beat(&self) {
        self.a64(OFF_BRIDGE_BEAT).fetch_add(1, Ordering::Relaxed);
    }

    pub fn bump_driver_beat(&self) {
        self.a64(OFF_DRIVER_BEAT).fetch_add(1, Ordering::Relaxed);
    }

    pub fn driver_beat(&self) -> u64 {
        self.a64(OFF_DRIVER_BEAT).load(Ordering::Relaxed)
    }

    /// called by the bridge when it starts
    pub fn reset(&self) {
        self.set_write_pos(Dir::In, 0);
        self.set_write_pos(Dir::Out, 0);
    }
}
