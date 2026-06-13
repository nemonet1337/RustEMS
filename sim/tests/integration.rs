//! Integration tests for rusefi-core control logic.
//!
//! These tests run on the host (std) and exercise the full signal path:
//!   trigger decode → ignition compute → fuel compute → HAL mock output
//!
//! HAL mocks are provided by `rusefi-hal-sim`.

use approx::assert_relative_eq;
use rusefi_core::{
    config::EngineConfig,
    ignition::{compute_ignition, tdc_angles_from_firing_order},
    sensors::SensorData,
    trigger::{
        missing_tooth::{MissingToothConfig, MissingToothDecoder},
        SyncEdge, SyncState, TriggerSignal,
    },
};

#[cfg(feature = "fuel-fi")]
use rusefi_core::fuel::{compute_injection, estimate_airmass_g};

use rusefi_core::hal::{CanBus, CanFrame, IgnitionOutput as IgnitionOutputTrait, UartPort};
use rusefi_hal_sim::ignition_sim::IgnitionEvent;
use rusefi_hal_sim::{SimCanBus, SimIgnitionOutput, SimUartPort};

// ─── helpers ─────────────────────────────────────────────────────────────────

fn make_36_1_decoder() -> MissingToothDecoder {
    MissingToothDecoder::new(MissingToothConfig {
        total_teeth: 36,
        missing_teeth: 1,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    })
}

fn feed_one_cycle(dec: &mut MissingToothDecoder, rpm: f32) -> u64 {
    // tooth interval in µs for 35 present teeth per 720° cycle
    let rev_us = 60_000_000.0 / rpm; // one full 360° rev in µs
    let tooth_us = (2.0 * rev_us / 36.0) as u64; // 720° / 36 teeth
    let gap_us = tooth_us * 2;

    let mut t = 0u64;
    for _ in 0..35 {
        t += tooth_us;
        let _ = dec.process(TriggerSignal::CrankRise, t);
    }
    // gap pulse
    t += gap_us;
    let _ = dec.process(TriggerSignal::CrankRise, t);
    t
}

// ─── trigger integration ─────────────────────────────────────────────────────

#[test]
fn trigger_sync_after_one_full_cycle() {
    let mut dec = make_36_1_decoder();
    feed_one_cycle(&mut dec, 1000.0);
    assert_eq!(dec.sync_state(), SyncState::CrankSynced);
}

#[test]
fn trigger_sync_counter_increments_each_cycle() {
    let mut dec = make_36_1_decoder();
    feed_one_cycle(&mut dec, 1000.0);
    assert_eq!(dec.sync_counter(), 0); // first sync: counter stays 0
    feed_one_cycle(&mut dec, 1000.0);
    assert_eq!(dec.sync_counter(), 1); // second sync: increments
    feed_one_cycle(&mut dec, 1000.0);
    assert_eq!(dec.sync_counter(), 2);
}

#[test]
fn trigger_rpm_estimate_plausible_at_1000rpm() {
    let mut dec = make_36_1_decoder();
    // tooth interval at 1000 rpm (720° / 36 teeth in 2 revs)
    let rpm_target = 1000.0_f32;
    let tooth_us = (2.0 * 60_000_000.0 / rpm_target / 36.0) as u64;

    // Feed enough teeth to achieve sync + stable RPM
    // (two full cycles worth of normal + gap pulses)
    let t = feed_one_cycle(&mut dec, rpm_target);
    let t2 = feed_one_cycle(&mut dec, rpm_target);

    // Feed two more normal teeth after the second cycle's gap so that
    // tooth_duration[1] holds a normal (not gap) interval.
    let t3 = t2 + tooth_us;
    let _ = dec.process(TriggerSignal::CrankRise, t3);
    let state = dec
        .process(TriggerSignal::CrankRise, t3 + tooth_us)
        .unwrap();

    let rpm = state
        .rpm
        .expect("rpm should be available after 2 full cycles");
    // Allow ±10 % tolerance (integer µs rounding)
    assert!(
        rpm > 900.0 && rpm < 1100.0,
        "expected ~1000 rpm, got rpm={rpm}"
    );
    let _ = t; // suppress unused warning
}

#[test]
fn trigger_60_2_config() {
    let mut dec = MissingToothDecoder::new(MissingToothConfig {
        total_teeth: 60,
        missing_teeth: 2,
        engine_cycle_deg: 720.0,
        sync_edge: SyncEdge::Rise,
    });

    let tooth_us = 1_000u64;
    let gap_us = tooth_us * 3; // 60-2: gap ratio = 3

    let mut t = 0u64;
    for _ in 0..58 {
        t += tooth_us;
        let _ = dec.process(TriggerSignal::CrankRise, t);
    }
    t += gap_us;
    let _ = dec.process(TriggerSignal::CrankRise, t);
    assert_eq!(dec.sync_state(), SyncState::CrankSynced);
}

// ─── ignition integration ────────────────────────────────────────────────────

#[test]
fn ignition_cranking_timing() {
    let cfg = EngineConfig::default_4cyl();
    let sensors = SensorData {
        rpm: Some(200.0),
        ..Default::default()
    };
    let out = compute_ignition(&cfg, &sensors, 0.0).expect("valid");
    // cranking_timing_deg = 5.0 (from default_4cyl config)
    assert_relative_eq!(out.advance_deg, 5.0, epsilon = 0.01);
}

#[test]
fn ignition_running_timing_from_table() {
    let cfg = EngineConfig::default_4cyl();
    let sensors = SensorData {
        rpm: Some(2000.0),
        load_pct: Some(50.0),
        ..Default::default()
    };
    let out = compute_ignition(&cfg, &sensors, 0.0).expect("valid");
    // ignition_table is flat 10.0 degrees
    assert_relative_eq!(out.advance_deg, 10.0, epsilon = 0.01);
}

#[test]
fn ignition_dwell_decreases_at_high_rpm() {
    let cfg = EngineConfig::default_4cyl();
    let low = SensorData {
        rpm: Some(1000.0),
        ..Default::default()
    };
    let high = SensorData {
        rpm: Some(6000.0),
        ..Default::default()
    };
    let dwell_low = compute_ignition(&cfg, &low, 0.0).expect("valid").dwell_ms;
    let dwell_high = compute_ignition(&cfg, &high, 0.0).expect("valid").dwell_ms;
    assert!(
        dwell_high < dwell_low,
        "dwell_low={dwell_low} dwell_high={dwell_high}"
    );
}

#[test]
fn ignition_tdc_angles_4cyl_firing_order() {
    let order = [0u8, 2, 3, 1];
    let angles = tdc_angles_from_firing_order(&order);
    assert_eq!(angles.len(), 4);
    assert_relative_eq!(angles[0], 0.0, epsilon = 0.01);
    assert_relative_eq!(angles[1], 180.0, epsilon = 0.01);
    assert_relative_eq!(angles[2], 360.0, epsilon = 0.01);
    assert_relative_eq!(angles[3], 540.0, epsilon = 0.01);
}

// ─── fuel integration (fuel-fi only) ─────────────────────────────────────────

#[cfg(feature = "fuel-fi")]
#[test]
fn fuel_pulse_positive_at_wot() {
    let cfg = EngineConfig::default_4cyl();
    let sensors = SensorData {
        rpm: Some(3000.0),
        load_pct: Some(100.0),
        battery_volts: Some(12.0),
        ..Default::default()
    };
    let airmass = estimate_airmass_g(101.325, cfg.displacement_cc_per_cyl, 0.85);
    let inj = compute_injection(&cfg, &sensors, airmass).expect("valid");
    assert!(inj.pulse_ms > 0.0, "pulse_ms={}", inj.pulse_ms);
    assert!(inj.fuel_mass_g > 0.0);
    assert_relative_eq!(inj.target_lambda, 1.0, epsilon = 0.01);
}

#[cfg(feature = "fuel-fi")]
#[test]
fn fuel_pulse_increases_with_airmass() {
    let cfg = EngineConfig::default_4cyl();
    let sensors = SensorData {
        rpm: Some(3000.0),
        load_pct: Some(50.0),
        ..Default::default()
    };

    let low_mass = estimate_airmass_g(50.0, cfg.displacement_cc_per_cyl, 0.80);
    let high_mass = estimate_airmass_g(101.0, cfg.displacement_cc_per_cyl, 0.80);

    let low_pulse = compute_injection(&cfg, &sensors, low_mass)
        .expect("valid")
        .pulse_ms;
    let high_pulse = compute_injection(&cfg, &sensors, high_mass)
        .expect("valid")
        .pulse_ms;
    assert!(high_pulse > low_pulse, "low={low_pulse} high={high_pulse}");
}

// ─── HAL mock: ignition output ───────────────────────────────────────────────

#[test]
fn sim_ignition_output_charge_and_fire() {
    let mut ign = SimIgnitionOutput::new();

    ign.coil_charge(0);
    ign.coil_charge(2);
    ign.coil_fire(0);

    // Check recorded event sequence
    assert_eq!(ign.events[0], IgnitionEvent::Charge(0));
    assert_eq!(ign.events[1], IgnitionEvent::Charge(2));
    assert_eq!(ign.events[2], IgnitionEvent::Fire(0));
    assert_eq!(ign.events.len(), 3);

    // fire_events() should yield only cylinder 0
    let fired: Vec<u8> = ign.fire_events().collect();
    assert_eq!(fired, vec![0u8]);
}

// ─── HAL mock: CAN bus ───────────────────────────────────────────────────────

#[test]
fn sim_can_loopback_roundtrip() {
    let mut can = SimCanBus::new();
    let frame = CanFrame::standard(0x7E8, &[0x01, 0x02, 0x03]);

    assert!(can.transmit(&frame));
    let rx = can.receive().expect("loopback frame available");
    assert_eq!(rx.id, 0x7E8);
    assert_eq!(rx.dlc, 3);
    assert_eq!(&rx.data[..3], &[0x01, 0x02, 0x03]);
}

#[test]
fn sim_can_extended_id() {
    let mut can = SimCanBus::new();
    let frame = CanFrame::extended(0x18DA_F110, &[0xAA, 0xBB]);
    can.transmit(&frame);
    let rx = can.receive().expect("frame");
    assert!(rx.is_extended);
    assert_eq!(rx.id, 0x18DA_F110);
}

#[test]
fn sim_can_inject_external_frame() {
    let mut can = SimCanBus::new();
    can.inject(CanFrame::standard(0x100, &[0xFF]));
    let rx = can.receive().expect("injected frame");
    assert_eq!(rx.id, 0x100);
    // No TX — inject does not count as a transmitted frame
    assert!(can.receive().is_none());
}

// ─── HAL mock: UART ─────────────────────────────────────────────────────────

#[test]
fn sim_uart_write_and_drain() {
    let mut uart = SimUartPort::new();
    let written = uart.write_bytes(b"hello");
    assert_eq!(written, 5);
    let drained = uart.drain_tx();
    assert_eq!(drained, b"hello");
}

#[test]
fn sim_uart_feed_and_read() {
    let mut uart = SimUartPort::new();
    uart.feed_rx(b"ok\n");
    let mut buf = [0u8; 8];
    let n = uart.read_bytes(&mut buf);
    assert_eq!(n, 3);
    assert_eq!(&buf[..3], b"ok\n");
}

#[test]
fn sim_uart_partial_read() {
    let mut uart = SimUartPort::new();
    uart.feed_rx(b"abcde");
    let mut buf = [0u8; 3];
    let n = uart.read_bytes(&mut buf);
    assert_eq!(n, 3);
    assert_eq!(&buf, b"abc");
    // Remaining 2 bytes still in buffer
    let mut buf2 = [0u8; 8];
    let n2 = uart.read_bytes(&mut buf2);
    assert_eq!(n2, 2);
    assert_eq!(&buf2[..2], b"de");
}
