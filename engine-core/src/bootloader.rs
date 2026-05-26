//! Bootloader for firmware updates.
//!
//! Provides bootloader functionality for:
//! - Firmware update reception
//! - Flash programming
//! - Boot verification
//! - Rollback support

/// Bootloader state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootloaderState {
    /// Normal operation (booted into application).
    Normal,
    /// Bootloader active, waiting for firmware.
    Bootloader,
    /// Firmware reception in progress.
    Receiving,
    /// Flash programming in progress.
    Programming,
    /// Verification in progress.
    Verifying,
    /// Boot failed, rollback available.
    RollbackAvailable,
    /// Boot failed, no rollback available.
    BootFailed,
}

/// Bootloader command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootloaderCommand {
    /// Enter bootloader mode.
    EnterBootloader,
    /// Start firmware update.
    StartUpdate,
    /// Cancel firmware update.
    CancelUpdate,
    /// Verify current firmware.
    Verify,
    /// Rollback to previous firmware.
    Rollback,
    /// Boot to application.
    BootApplication,
}

/// Bootloader configuration.
#[derive(Clone, Copy, Debug)]
pub struct BootloaderConfig {
    /// Bootloader timeout (ms) before auto-booting to application.
    pub boot_timeout_ms: u32,
    /// Maximum firmware size (bytes).
    pub max_firmware_size: u32,
    /// Enable rollback support.
    pub enable_rollback: bool,
    /// CRC polynomial for verification.
    pub crc_polynomial: u32,
}

impl Default for BootloaderConfig {
    fn default() -> Self {
        Self {
            boot_timeout_ms: 5000, // 5 seconds
            max_firmware_size: 512 * 1024, // 512 KB
            enable_rollback: true,
            crc_polynomial: 0x04C11DB7, // CRC-32
        }
    }
}

/// Bootloader status.
#[derive(Clone, Copy, Debug)]
pub struct BootloaderStatus {
    /// Current state.
    pub state: BootloaderState,
    /// Firmware size received (bytes).
    pub firmware_size: u32,
    /// Firmware CRC (if calculated).
    pub firmware_crc: Option<u32>,
    /// Boot attempt count.
    pub boot_attempts: u32,
}

/// Bootloader.
pub struct Bootloader {
    cfg: BootloaderConfig,
    state: BootloaderState,
    boot_timer_ms: u32,
    firmware_size: u32,
    boot_attempts: u32,
}

impl Bootloader {
    /// Create a new bootloader with default configuration.
    pub fn new() -> Self {
        Self::with_config(BootloaderConfig::default())
    }

    /// Create a new bootloader with custom configuration.
    pub fn with_config(cfg: BootloaderConfig) -> Self {
        Self {
            cfg,
            state: BootloaderState::Bootloader,
            boot_timer_ms: cfg.boot_timeout_ms,
            firmware_size: 0,
            boot_attempts: 0,
        }
    }

    /// Process bootloader command.
    ///
    /// # Arguments
    /// * `cmd` - Command to process
    pub fn process_command(&mut self, cmd: BootloaderCommand) {
        match cmd {
            BootloaderCommand::EnterBootloader => {
                self.state = BootloaderState::Bootloader;
                self.boot_timer_ms = self.cfg.boot_timeout_ms;
            }
            BootloaderCommand::StartUpdate => {
                if self.state == BootloaderState::Bootloader {
                    self.state = BootloaderState::Receiving;
                    self.firmware_size = 0;
                }
            }
            BootloaderCommand::CancelUpdate => {
                if matches!(self.state, BootloaderState::Receiving | BootloaderState::Programming) {
                    self.state = BootloaderState::Bootloader;
                    self.firmware_size = 0;
                }
            }
            BootloaderCommand::Verify => {
                if self.state == BootloaderState::Programming {
                    self.state = BootloaderState::Verifying;
                }
            }
            BootloaderCommand::Rollback => {
                if self.cfg.enable_rollback {
                    self.state = BootloaderState::RollbackAvailable;
                }
            }
            BootloaderCommand::BootApplication => {
                self.state = BootloaderState::Normal;
                self.boot_attempts = 0;
            }
        }
    }

    /// Update bootloader state.
    ///
    /// # Arguments
    /// * `dt_ms` - Time since last update in milliseconds
    ///
    /// # Returns
    /// Current bootloader status.
    pub fn update(&mut self, dt_ms: u32) -> BootloaderStatus {
        // Auto-boot timer
        if self.state == BootloaderState::Bootloader {
            self.boot_timer_ms = self.boot_timer_ms.saturating_sub(dt_ms);

            if self.boot_timer_ms == 0 {
                self.state = BootloaderState::Normal;
            }
        }

        BootloaderStatus {
            state: self.state,
            firmware_size: self.firmware_size,
            firmware_crc: None,
            boot_attempts: self.boot_attempts,
        }
    }

    /// Receive firmware data chunk.
    ///
    /// # Arguments
    /// * `data` - Firmware data chunk
    ///
    /// # Returns
    /// `true` if data accepted, `false` if buffer full or error.
    pub fn receive_firmware(&mut self, data: &[u8]) -> bool {
        if self.state != BootloaderState::Receiving {
            return false;
        }

        if (self.firmware_size as usize + data.len()) > self.cfg.max_firmware_size as usize {
            return false;
        }

        self.firmware_size += data.len() as u32;
        true
    }

    /// Get current bootloader state.
    pub fn state(&self) -> BootloaderState {
        self.state
    }

    /// Check if bootloader is active.
    pub fn is_bootloader_active(&self) -> bool {
        self.state != BootloaderState::Normal
    }

    /// Check if rollback is available.
    pub fn is_rollback_available(&self) -> bool {
        self.cfg.enable_rollback && self.boot_attempts > 0
    }

    /// Get the configuration.
    pub fn config(&self) -> BootloaderConfig {
        self.cfg
    }

    /// Set the configuration.
    pub fn set_config(&mut self, cfg: BootloaderConfig) {
        self.cfg = cfg;
    }

    /// Increment boot attempt counter (called on boot failure).
    pub fn increment_boot_attempts(&mut self) {
        self.boot_attempts += 1;
    }
}

impl Default for Bootloader {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bootloader_auto_boots() {
        let mut bootloader = Bootloader::new();

        // Initially in bootloader mode
        assert_eq!(bootloader.state(), BootloaderState::Bootloader);

        // Wait for timeout
        for _ in 0..510 {
            bootloader.update(10);
        }

        // Should auto-boot to normal
        assert_eq!(bootloader.state(), BootloaderState::Normal);
    }

    #[test]
    fn enter_bootloader_command() {
        let mut bootloader = Bootloader::new();
        bootloader.process_command(BootloaderCommand::BootApplication);

        assert_eq!(bootloader.state(), BootloaderState::Normal);

        bootloader.process_command(BootloaderCommand::EnterBootloader);
        assert_eq!(bootloader.state(), BootloaderState::Bootloader);
    }

    #[test]
    fn start_update_sequence() {
        let mut bootloader = Bootloader::new();
        bootloader.process_command(BootloaderCommand::StartUpdate);

        assert_eq!(bootloader.state(), BootloaderState::Receiving);
    }

    #[test]
    fn cancel_update() {
        let mut bootloader = Bootloader::new();
        bootloader.process_command(BootloaderCommand::StartUpdate);
        bootloader.process_command(BootloaderCommand::CancelUpdate);

        assert_eq!(bootloader.state(), BootloaderState::Bootloader);
    }

    #[test]
    fn receive_firmware_data() {
        let mut bootloader = Bootloader::new();
        bootloader.process_command(BootloaderCommand::StartUpdate);

        let data = [0u8; 100];
        assert!(bootloader.receive_firmware(&data));

        let status = bootloader.update(10);
        assert_eq!(status.firmware_size, 100);
    }

    #[test]
    fn firmware_size_limit() {
        let mut bootloader = Bootloader::new();
        bootloader.process_command(BootloaderCommand::StartUpdate);

        // Try to receive more than max size
        let large_data = [0u8; 1024 * 1024]; // 1 MB
        assert!(!bootloader.receive_firmware(&large_data));
    }

    #[test]
    fn rollback_support() {
        let mut bootloader = Bootloader::new();
        bootloader.increment_boot_attempts();

        assert!(bootloader.is_rollback_available());

        bootloader.process_command(BootloaderCommand::Rollback);
        assert_eq!(bootloader.state(), BootloaderState::RollbackAvailable);
    }

    #[test]
    fn boot_attempt_counter() {
        let mut bootloader = Bootloader::new();
        assert_eq!(bootloader.boot_attempts, 0);

        bootloader.increment_boot_attempts();
        bootloader.increment_boot_attempts();

        assert_eq!(bootloader.boot_attempts, 2);
    }
}
