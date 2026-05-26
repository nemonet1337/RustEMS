//! Non-volatile storage for learned data.
//!
//! Provides persistence for LTFT cells and other learned parameters.

use crate::fuel::ltft::LtftCell;

/// Storage key for LTFT data.
pub const LTFT_STORAGE_KEY: u32 = 0x4C544654; // "LTFT"

/// Number of LTFT cells stored.
pub const LTFT_CELL_COUNT: usize = 16;

/// Storage error type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageError {
    /// Storage write failed.
    WriteFailed,
    /// Storage read failed.
    ReadFailed,
    /// Invalid data format.
    InvalidData,
    /// Storage not available.
    NotAvailable,
}

/// Storage interface for non-volatile data.
///
/// This is a trait that can be implemented for different storage backends
/// (flash, EEPROM, external memory, etc.).
pub trait Storage {
    /// Write data to storage.
    ///
    /// # Arguments
    /// * `key` - Storage key identifier
    /// * `data` - Data to write
    ///
    /// # Returns
    /// Ok on success, StorageError on failure.
    fn write(&mut self, key: u32, data: &[u8]) -> Result<(), StorageError>;

    /// Read data from storage.
    ///
    /// # Arguments
    /// * `key` - Storage key identifier
    /// * `buffer` - Buffer to read data into
    ///
    /// # Returns
    /// Number of bytes read on success, StorageError on failure.
    fn read(&self, key: u32, buffer: &mut [u8]) -> Result<usize, StorageError>;

    /// Erase data from storage.
    ///
    /// # Arguments
    /// * `key` - Storage key identifier
    ///
    /// # Returns
    /// Ok on success, StorageError on failure.
    fn erase(&mut self, key: u32) -> Result<(), StorageError>;
}

/// In-memory storage implementation for testing.
#[derive(Clone, Debug, Default)]
pub struct MemoryStorage {
    data: heapless::Vec<(u32, heapless::Vec<u8, 256>), 16>,
}

impl MemoryStorage {
    /// Create a new in-memory storage.
    pub fn new() -> Self {
        Self::default()
    }

    /// Clear all stored data.
    pub fn clear(&mut self) {
        self.data.clear();
    }
}

impl Storage for MemoryStorage {
    fn write(&mut self, key: u32, data: &[u8]) -> Result<(), StorageError> {
        let mut vec = heapless::Vec::new();
        for &byte in data {
            if vec.push(byte).is_err() {
                return Err(StorageError::WriteFailed);
            }
        }

        // Replace existing entry or add new one
        for entry in &mut self.data {
            if entry.0 == key {
                entry.1 = vec;
                return Ok(());
            }
        }

        if self.data.push((key, vec)).is_err() {
            Err(StorageError::WriteFailed)
        } else {
            Ok(())
        }
    }

    fn read(&self, key: u32, buffer: &mut [u8]) -> Result<usize, StorageError> {
        for entry in &self.data {
            if entry.0 == key {
                let len = entry.1.len().min(buffer.len());
                buffer[..len].copy_from_slice(&entry.1[..len]);
                return Ok(len);
            }
        }
        Err(StorageError::ReadFailed)
    }

    fn erase(&mut self, key: u32) -> Result<(), StorageError> {
        let mut idx = None;
        for (i, entry) in self.data.iter().enumerate() {
            if entry.0 == key {
                idx = Some(i);
                break;
            }
        }
        if let Some(i) = idx {
            self.data.remove(i);
            Ok(())
        } else {
            Err(StorageError::ReadFailed)
        }
    }
}

/// LTFT storage manager.
pub struct LtftStorage<S: Storage> {
    storage: S,
}

impl<S: Storage> LtftStorage<S> {
    /// Create a new LTFT storage manager.
    pub fn new(storage: S) -> Self {
        Self { storage }
    }

    /// Save LTFT cells to storage.
    ///
    /// # Arguments
    /// * `cells` - LTFT cells to save
    ///
    /// # Returns
    /// Ok on success, StorageError on failure.
    pub fn save_ltft(&mut self, cells: &[LtftCell; LTFT_CELL_COUNT]) -> Result<(), StorageError> {
        let mut buffer = [0u8; LTFT_CELL_COUNT * 12]; // 12 bytes per cell (trim: 4, count: 4, valid: 1, padding: 3)

        for (i, cell) in cells.iter().enumerate() {
            let offset = i * 12;
            let trim_bytes = cell.trim.to_le_bytes();
            let count_bytes = cell.sample_count.to_le_bytes();
            let valid_byte = u8::from(cell.valid);

            buffer[offset..offset + 4].copy_from_slice(&trim_bytes);
            buffer[offset + 4..offset + 8].copy_from_slice(&count_bytes);
            buffer[offset + 8] = valid_byte;
        }

        self.storage.write(LTFT_STORAGE_KEY, &buffer)
    }

    /// Load LTFT cells from storage.
    ///
    /// # Returns
    /// LTFT cells on success, StorageError on failure.
    pub fn load_ltft(&self) -> Result<[LtftCell; LTFT_CELL_COUNT], StorageError> {
        let mut buffer = [0u8; LTFT_CELL_COUNT * 12];
        let len = self.storage.read(LTFT_STORAGE_KEY, &mut buffer)?;

        if len != LTFT_CELL_COUNT * 12 {
            return Err(StorageError::InvalidData);
        }

        let mut cells = [LtftCell::default(); LTFT_CELL_COUNT];

        for i in 0..LTFT_CELL_COUNT {
            let offset = i * 12;
            let trim = f32::from_le_bytes(buffer[offset..offset + 4].try_into().unwrap());
            let sample_count = u32::from_le_bytes(buffer[offset + 4..offset + 8].try_into().unwrap());
            let valid = buffer[offset + 8] != 0;

            cells[i] = LtftCell {
                trim,
                sample_count,
                valid,
            };
        }

        Ok(cells)
    }

    /// Erase LTFT data from storage.
    ///
    /// # Returns
    /// Ok on success, StorageError on failure.
    pub fn erase_ltft(&mut self) -> Result<(), StorageError> {
        self.storage.erase(LTFT_STORAGE_KEY)
    }

    /// Get the underlying storage.
    pub fn storage(&mut self) -> &mut S {
        &mut self.storage
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fuel::ltft::LtftCell;

    #[test]
    fn memory_storage_write_read() {
        let mut storage = MemoryStorage::new();
        let data = [1u8, 2, 3, 4];

        storage.write(0x1234, &data).unwrap();
        let mut buffer = [0u8; 4];
        let len = storage.read(0x1234, &mut buffer).unwrap();

        assert_eq!(len, 4);
        assert_eq!(buffer, data);
    }

    #[test]
    fn memory_storage_erase() {
        let mut storage = MemoryStorage::new();
        let data = [1u8, 2, 3, 4];

        storage.write(0x1234, &data).unwrap();
        storage.erase(0x1234).unwrap();

        let mut buffer = [0u8; 4];
        assert!(storage.read(0x1234, &mut buffer).is_err());
    }

    #[test]
    fn ltft_storage_save_load() {
        let _storage = MemoryStorage::new();
        let mut ltft_storage = LtftStorage::new(_storage);

        let mut cells = [LtftCell::default(); LTFT_CELL_COUNT];
        cells[0].trim = 1.05;
        cells[0].sample_count = 100;
        cells[0].valid = true;

        ltft_storage.save_ltft(&cells).unwrap();
        let loaded = ltft_storage.load_ltft().unwrap();

        assert_eq!(loaded[0].trim, 1.05);
        assert_eq!(loaded[0].sample_count, 100);
        assert!(loaded[0].valid);
    }

    #[test]
    fn ltft_storage_erase() {
        let _storage = MemoryStorage::new();
        let mut ltft_storage = LtftStorage::new(_storage);

        let cells = [LtftCell::default(); LTFT_CELL_COUNT];
        ltft_storage.save_ltft(&cells).unwrap();
        ltft_storage.erase_ltft().unwrap();

        assert!(ltft_storage.load_ltft().is_err());
    }
}
