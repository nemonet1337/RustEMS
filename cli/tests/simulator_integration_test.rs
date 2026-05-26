//! Integration tests that connect to a running rusEFI simulator.
//!
//! These tests are **ignored by default** because they require the rusEFI
//! simulator binary to be running and listening on TCP port 29001.
//!
//! To run manually:
//!   1. Start the simulator: `simulator/build/rusefi_simulator`
//!   2. Run: `cargo test -p rusefi-cli -- --ignored`

use rusefi_client::EcuClient;
use rusefi_protocol::transport::tcp;

const SIMULATOR_HOST: &str = "127.0.0.1";
const SIMULATOR_PORT: u16 = 29001;

/// Returns true if the simulator TCP port appears to be open.
async fn simulator_available() -> bool {
    tokio::net::TcpStream::connect((SIMULATOR_HOST, SIMULATOR_PORT))
        .await
        .is_ok()
}

#[tokio::test]
#[ignore = "Requires running rusEFI simulator on port 29001"]
async fn test_simulator_hello() {
    if !simulator_available().await {
        eprintln!("SKIP: simulator not reachable at {SIMULATOR_HOST}:{SIMULATOR_PORT}");
        return;
    }

    let stream = tcp::connect(SIMULATOR_HOST, SIMULATOR_PORT)
        .await
        .expect("TCP connect failed");

    let mut client = EcuClient::new(stream);
    let sig = client.hello().await.expect("Hello command failed");

    println!("Simulator signature: {sig}");
    assert!(!sig.is_empty(), "Signature must not be empty");
    // rusEFI simulator signatures contain "rusEFI" or a version string
    assert!(
        sig.contains("rusEFI") || sig.contains("20"),
        "Unexpected signature: {sig}"
    );
}

#[tokio::test]
#[ignore = "Requires running rusEFI simulator on port 29001"]
async fn test_simulator_firmware_version() {
    if !simulator_available().await {
        eprintln!("SKIP: simulator not reachable");
        return;
    }

    let stream = tcp::connect(SIMULATOR_HOST, SIMULATOR_PORT)
        .await
        .expect("TCP connect failed");

    let mut client = EcuClient::new(stream);
    let version = client
        .get_firmware_version()
        .await
        .expect("GetFirmwareVersion failed");

    println!("Firmware version: {version}");
    assert!(!version.is_empty());
}

#[tokio::test]
#[ignore = "Requires running rusEFI simulator on port 29001"]
async fn test_simulator_read_image_small() {
    if !simulator_available().await {
        eprintln!("SKIP: simulator not reachable");
        return;
    }

    let stream = tcp::connect(SIMULATOR_HOST, SIMULATOR_PORT)
        .await
        .expect("TCP connect failed");

    let mut client = EcuClient::new(stream).with_blocking_factor(128);
    let sig = client.hello().await.expect("Hello failed");

    // Read just the first 256 bytes as a smoke test
    // (real page size is ~20 000+ bytes and needs INI to know exactly)
    let image = client
        .read_image(256, &sig)
        .await
        .expect("read_image failed");

    assert_eq!(image.size(), 256);
    println!("Read {} bytes OK", image.size());
}
