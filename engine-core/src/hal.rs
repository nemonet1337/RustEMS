//! HAL trait definitions — the interface between control logic and hardware.
//!
//! These traits are implemented by:
//! - `rusefi-hal-sim` for PC simulation
//! - `rusefi-hal-stm32f4` for STM32F407
//! - `rusefi-hal-stm32f7` for STM32F767

use crate::sensors::AdcChannel;

/// Callback type for timer-scheduled events.
pub type TimerCallback = fn();

/// Trigger input — reads crank and cam sensor pulse timestamps.
///
/// Implementations must capture hardware timer values on interrupt and store
/// them in a queue for the control loop to consume.
pub trait TriggerInput {
    /// Returns the timestamp (microseconds) of the most recent crank pulse,
    /// or `None` if no new pulse has arrived since the last call.
    fn read_crank_timestamp(&mut self) -> Option<u64>;

    /// Returns the timestamp (microseconds) of the most recent cam pulse,
    /// or `None` if no new pulse has arrived since the last call.
    fn read_cam_timestamp(&mut self) -> Option<u64>;
}

/// Ignition coil output — controls coil charge and fire events.
///
/// Cylinder numbering is 0-based internally.
pub trait IgnitionOutput {
    /// Begin charging the ignition coil for the given cylinder.
    ///
    /// Must be called `dwell_us` before [`coil_fire`].
    fn coil_charge(&mut self, cylinder: u8);

    /// Fire (discharge) the ignition coil for the given cylinder.
    ///
    /// Called at the calculated spark angle.
    fn coil_fire(&mut self, cylinder: u8);
}

/// ADC input — reads raw 12-bit ADC values from sensor channels.
pub trait AdcInput {
    /// Read the raw ADC value (0–4095) for the given channel.
    ///
    /// For STM32F407 with 3.3 V reference: `voltage = raw * 3.3 / 4096`.
    fn read_raw(&mut self, channel: AdcChannel) -> u16;
}

/// Fuel injector output — controls injector open/close events.
///
/// Only compiled when the `fuel-fi` feature is enabled.
#[cfg(feature = "fuel-fi")]
pub trait InjectorOutput {
    /// Open (energise) the injector for the given cylinder.
    fn open(&mut self, cylinder: u8);

    /// Close (de-energise) the injector for the given cylinder.
    fn close(&mut self, cylinder: u8);
}

/// System timer — provides microsecond timestamps and one-shot scheduling.
pub trait SystemTimer {
    /// Return the current time in microseconds since boot/reset.
    fn now_us(&self) -> u64;

    /// Schedule `callback` to be called approximately `delay_us` microseconds
    /// from now.
    ///
    /// Only one scheduled callback per timer instance is required for the
    /// initial implementation; implementations may queue multiple.
    fn schedule_us(&mut self, delay_us: u64, callback: TimerCallback);
}

// ─── CAN ─────────────────────────────────────────────────────────────────────

/// A single CAN 2.0B frame (standard or extended ID, up to 8 data bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    /// 11-bit standard ID (bits 0-10) or 29-bit extended ID.
    pub id: u32,
    /// `true` if `id` is a 29-bit extended identifier.
    pub is_extended: bool,
    /// Number of valid bytes in `data` (0–8).
    pub dlc: u8,
    /// Frame payload.
    pub data: [u8; 8],
}

impl CanFrame {
    /// Construct a standard-ID (11-bit) frame.
    pub fn standard(id: u16, data: &[u8]) -> Self {
        let dlc = data.len().min(8) as u8;
        let mut buf = [0u8; 8];
        buf[..dlc as usize].copy_from_slice(&data[..dlc as usize]);
        Self { id: id as u32, is_extended: false, dlc, data: buf }
    }

    /// Construct an extended-ID (29-bit) frame.
    pub fn extended(id: u32, data: &[u8]) -> Self {
        let dlc = data.len().min(8) as u8;
        let mut buf = [0u8; 8];
        buf[..dlc as usize].copy_from_slice(&data[..dlc as usize]);
        Self { id, is_extended: true, dlc, data: buf }
    }
}

/// CAN bus interface — non-blocking transmit and receive.
pub trait CanBus {
    /// Attempt to transmit a frame. Returns `true` if accepted by the
    /// hardware TX mailbox, `false` if all mailboxes are full.
    fn transmit(&mut self, frame: &CanFrame) -> bool;

    /// Return the oldest received frame, or `None` if the RX FIFO is empty.
    fn receive(&mut self) -> Option<CanFrame>;
}

// ─── UART ─────────────────────────────────────────────────────────────────────

/// UART / serial port interface — byte-level non-blocking I/O.
pub trait UartPort {
    /// Write up to `buf.len()` bytes. Returns the number of bytes actually
    /// written (may be 0 if the TX buffer is full).
    fn write_bytes(&mut self, buf: &[u8]) -> usize;

    /// Read up to `buf.len()` bytes. Returns the number of bytes read
    /// (may be 0 if no data is available).
    fn read_bytes(&mut self, buf: &mut [u8]) -> usize;
}

// ─── PWM outputs ─────────────────────────────────────────────────────────────

/// Generic PWM output — controls a single PWM channel by duty cycle.
///
/// Used for IAC valves, boost waste-gate solenoids, VVT cam-phaser
/// solenoids, and any other PWM-controlled actuator.
pub trait PwmOutput {
    /// Set the duty cycle (0.0 = fully off, 100.0 = fully on).
    fn set_duty(&mut self, duty_pct: f32);

    /// Return the current duty cycle.
    fn duty(&self) -> f32;
}

// ─── Relay / digital outputs ─────────────────────────────────────────────────

/// Binary relay / digital output — controls a single on/off channel.
///
/// Used for the fuel pump relay, cooling fan relay, AC clutch relay, etc.
pub trait RelayOutput {
    /// Energise (turn on) the relay.
    fn on(&mut self);

    /// De-energise (turn off) the relay.
    fn off(&mut self);

    /// Return `true` if the relay is currently energised.
    fn is_on(&self) -> bool;
}
