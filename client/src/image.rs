//! ECU configuration image (in-memory byte buffer).
//!
//! Mirrors the Java `ConfigurationImage` class.

/// In-memory snapshot of the ECU configuration (one page).
#[derive(Debug, Clone)]
pub struct ConfigImage {
    /// Raw bytes of the configuration page
    data: Vec<u8>,
    /// ECU firmware signature associated with this image
    pub signature: String,
}

impl ConfigImage {
    /// Create a new blank image of `size` bytes.
    pub fn new(size: usize, signature: impl Into<String>) -> Self {
        Self {
            data: vec![0u8; size],
            signature: signature.into(),
        }
    }

    /// Number of bytes in the image.
    pub fn size(&self) -> usize {
        self.data.len()
    }

    /// Read-only view of the raw bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    /// Mutable access to a byte range (used during chunked reads).
    pub fn write_range(&mut self, offset: usize, src: &[u8]) {
        let end = offset + src.len();
        self.data[offset..end].copy_from_slice(src);
    }

    /// Return a sub-slice of the image.
    pub fn range(&self, offset: usize, length: usize) -> &[u8] {
        &self.data[offset..offset + length]
    }

    /// Find the first differing byte range between `self` and `other`.
    ///
    /// Returns `Some((start, end))` (exclusive end) or `None` if identical.
    pub fn first_diff_range(&self, other: &ConfigImage) -> Option<(usize, usize)> {
        let len = self.data.len().min(other.data.len());
        let start = (0..len).find(|&i| self.data[i] != other.data[i])?;
        let end = (start..len)
            .rev()
            .find(|&i| self.data[i] != other.data[i])
            .map(|i| i + 1)
            .unwrap_or(start + 1);
        Some((start, end))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_and_read_range() {
        let mut img = ConfigImage::new(16, "test-sig");
        img.write_range(4, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(img.range(4, 3), &[0xAA, 0xBB, 0xCC]);
        assert_eq!(img.range(0, 4), &[0u8; 4]);
    }

    #[test]
    fn first_diff_range_identical() {
        let a = ConfigImage::new(8, "sig");
        let b = a.clone();
        assert_eq!(a.first_diff_range(&b), None);
    }

    #[test]
    fn first_diff_range_single_byte() {
        let mut a = ConfigImage::new(8, "sig");
        let mut b = ConfigImage::new(8, "sig");
        a.write_range(3, &[0xFF]);
        b.write_range(3, &[0x00]);
        assert_eq!(a.first_diff_range(&b), Some((3, 4)));
    }
}
