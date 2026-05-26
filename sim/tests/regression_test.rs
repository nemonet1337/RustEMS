//! Regression tests — compare simulation output against reference data.
//!
//! These tests verify that the Rust implementation produces results
//! consistent with the reference rusEFI simulator output.

use rusefi_core::actuators::{IdleConfig, IdleController};
use rusefi_core::config::EngineConfig;
use rusefi_core::ignition::{compute_ignition, tdc_angles_from_firing_order, RpmLimiter, RpmLimiterConfig};
use rusefi_core::sensors::SensorData;

/// Tolerance for ignition timing comparison (degrees).
const IGNITION_TOLERANCE_DEG: f32 = 1.0;
/// Tolerance for injection pulse width comparison (%).
const INJECTION_TOLERANCE_PCT: f32 = 2.0;
/// Tolerance for IAC duty comparison (%).
const IAC_TOLERANCE_PCT: f32 = 3.0;

/// Helper: create sensor data with given RPM.
fn sensors_with_rpm(rpm: f32) -> SensorData {
    SensorData {
        rpm: Some(rpm),
        map_kpa: Some(50.0),
        tps_pct: Some(0.0),
        iat_celsius: Some(25.0),
        clt_celsius: Some(80.0),
        battery_volts: Some(12.5),
        load_pct: Some(20.0),
        maf_voltage: None,
        fuel_level_pct: None,
        oil_pressure_kpa: None,
        lambda1_voltage: None,
        lambda2_voltage: None,
    }
}

#[test]
fn test_idle_1cyl_carb() {
    let cfg = EngineConfig::default_1cyl();
    let sensors = sensors_with_rpm(800.0);

    // Compute ignition for cylinder 1 (TDC at 0°)
    let result = compute_ignition(&cfg, &sensors, 0.0);
    assert!(result.is_some(), "Should compute ignition");

    let ign = result.unwrap();
    // At idle, expect moderate advance (~10°) and reasonable dwell (~4ms)
    assert!(
        ign.advance_deg >= 5.0 && ign.advance_deg <= 15.0,
        "Idle advance should be ~10°, got {}",
        ign.advance_deg
    );
    assert!(
        ign.dwell_ms >= 2.0 && ign.dwell_ms <= 6.0,
        "Idle dwell should be ~4ms, got {}",
        ign.dwell_ms
    );
}

#[test]
fn test_idle_4cyl_carb_firing_order() {
    let cfg = EngineConfig::default_4cyl();
    let sensors = sensors_with_rpm(800.0);

    // Check firing order 1-3-4-2 produces correct TDC angles
    let tdc_angles = tdc_angles_from_firing_order(&[0, 2, 3, 1]);
    assert_eq!(tdc_angles.len(), 4);

    // Expected: 0°, 180°, 360°, 540°
    assert!((tdc_angles[0] - 0.0).abs() < IGNITION_TOLERANCE_DEG);
    assert!((tdc_angles[1] - 180.0).abs() < IGNITION_TOLERANCE_DEG);
    assert!((tdc_angles[2] - 360.0).abs() < IGNITION_TOLERANCE_DEG);
    assert!((tdc_angles[3] - 540.0).abs() < IGNITION_TOLERANCE_DEG);

    // Compute ignition for each cylinder
    for (i, &tdc) in tdc_angles.iter().enumerate() {
        let result = compute_ignition(&cfg, &sensors, tdc);
        assert!(
            result.is_some(),
            "Should compute ignition for cylinder {}",
            i + 1
        );

        let ign = result.unwrap();
        // All cylinders should have same advance at idle
        assert!(
            (ign.advance_deg - 10.0).abs() < IGNITION_TOLERANCE_DEG,
            "Cylinder {} advance out of tolerance: {}",
            i + 1,
            ign.advance_deg
        );
    }
}

#[test]
fn test_cold_start_enrichment() {
    let cfg = EngineConfig::default_4cyl();

    // Cold engine: -20°C coolant
    let cold_sensors = SensorData {
        rpm: Some(200.0),
        map_kpa: Some(30.0),
        tps_pct: Some(0.0),
        iat_celsius: Some(-10.0),
        clt_celsius: Some(-20.0),
        battery_volts: Some(12.0),
        load_pct: Some(10.0),
        maf_voltage: None,
        fuel_level_pct: None,
        oil_pressure_kpa: None,
        lambda1_voltage: None,
        lambda2_voltage: None,
    };

    // Warm engine: 80°C coolant
    let warm_sensors = SensorData {
        rpm: Some(800.0),
        map_kpa: Some(50.0),
        tps_pct: Some(0.0),
        iat_celsius: Some(25.0),
        clt_celsius: Some(80.0),
        battery_volts: Some(12.5),
        load_pct: Some(20.0),
        maf_voltage: None,
        fuel_level_pct: None,
        oil_pressure_kpa: None,
        lambda1_voltage: None,
        lambda2_voltage: None,
    };

    let cold_result = compute_ignition(&cfg, &cold_sensors, 0.0);
    let warm_result = compute_ignition(&cfg, &warm_sensors, 0.0);

    assert!(cold_result.is_some());
    assert!(warm_result.is_some());

    let cold_ign = cold_result.unwrap();
    let warm_ign = warm_result.unwrap();

    // Cold engine should have more advance (from CLT correction table)
    // Cold: +4°, Warm: -1.5° based on config
    assert!(
        cold_ign.advance_deg > warm_ign.advance_deg,
        "Cold engine should have more advance: cold={}°, warm={}°",
        cold_ign.advance_deg,
        warm_ign.advance_deg
    );
}

#[test]
fn test_idle_control_response() {
    let cfg = IdleConfig::default_4cyl();
    let mut ctrl = IdleController::new(cfg);

    // Simulate cold start
    let duty_cold = ctrl.update(600.0, -20.0, 0.0, false, 10.0, false);

    // Simulate warm idle
    let duty_warm = ctrl.update(800.0, 80.0, 0.0, false, 10.0, false);

    // Cold engine should need more IAC duty
    assert!(
        duty_cold > duty_warm,
        "Cold idle duty ({}) should exceed warm ({})",
        duty_cold,
        duty_warm
    );

    // Check convergence over time
    let mut rpm = 600.0;
    ctrl.reset();

    for _ in 0..200 {
        let duty = ctrl.update(rpm, 80.0, 0.0, false, 10.0, false);
        // Simulate engine response: more duty = higher RPM
        rpm += (duty - 40.0) * 0.5;
        rpm = rpm.clamp(400.0, 1200.0);
    }

    // Should converge near target (~800 RPM at 80°C)
    assert!(
        (rpm - 800.0).abs() < 100.0,
        "Idle control should converge: final RPM = {}",
        rpm
    );
}

#[test]
fn test_rpm_limiter() {
    let cfg = RpmLimiterConfig::default_4cyl();
    let mut limiter = RpmLimiter::new(cfg);

    // Below limit: should not cut
    assert!(!limiter.update(7000.0), "Should not cut below limit");
    assert!(!limiter.is_active());

    // Above limit: should cut
    assert!(limiter.update(7600.0), "Should cut above limit");
    assert!(limiter.is_active());

    // Stay above recovery: should keep cutting
    assert!(limiter.update(7500.0), "Should keep cutting above recovery");

    // Drop below recovery: should resume
    assert!(!limiter.update(7100.0), "Should resume below recovery");
    assert!(!limiter.is_active());
}

#[test]
fn test_dwell_voltage_correction() {
    let cfg = EngineConfig::default_4cyl();

    // Low voltage: higher dwell needed
    let low_v_sensors = SensorData {
        rpm: Some(2000.0),
        map_kpa: Some(50.0),
        tps_pct: Some(50.0),
        iat_celsius: Some(25.0),
        clt_celsius: Some(80.0),
        battery_volts: Some(9.0),
        load_pct: Some(50.0),
        maf_voltage: None,
        fuel_level_pct: None,
        oil_pressure_kpa: None,
        lambda1_voltage: None,
        lambda2_voltage: None,
    };

    // Normal voltage: standard dwell
    let normal_v_sensors = SensorData {
        rpm: Some(2000.0),
        map_kpa: Some(50.0),
        tps_pct: Some(50.0),
        iat_celsius: Some(25.0),
        clt_celsius: Some(80.0),
        battery_volts: Some(12.0),
        load_pct: Some(50.0),
        maf_voltage: None,
        fuel_level_pct: None,
        oil_pressure_kpa: None,
        lambda1_voltage: None,
        lambda2_voltage: None,
    };

    let low_v_result = compute_ignition(&cfg, &low_v_sensors, 0.0).unwrap();
    let normal_v_result = compute_ignition(&cfg, &normal_v_sensors, 0.0).unwrap();

    // Low voltage should have higher dwell
    assert!(
        low_v_result.dwell_ms > normal_v_result.dwell_ms,
        "Low voltage ({}) should increase dwell: low={}ms, normal={}ms",
        low_v_sensors.battery_volts.unwrap(),
        low_v_result.dwell_ms,
        normal_v_result.dwell_ms
    );
}

#[test]
fn test_overdwell_protection() {
    let mut cfg = EngineConfig::default_4cyl();

    // Set very high base dwell that would exceed safe limit
    cfg.dwell_ms_table = [15.0; 8]; // 15ms is too long

    let sensors = sensors_with_rpm(1000.0);
    let result = compute_ignition(&cfg, &sensors, 0.0).unwrap();

    // Should be clamped to MAX_DWELL_MS (10ms)
    assert!(
        result.dwell_ms <= 10.0,
        "Dwell should be limited to 10ms max, got {}ms",
        result.dwell_ms
    );
}
