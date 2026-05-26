//! Integration tests for rusefi-codegen
//!
//! These tests verify that the Rust tool produces output identical to the Java tool.

use std::fs;
use std::path::Path;
use std::process::Command;

/// Path to the real rusefi_config.txt
const CONFIG_TXT_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../firmware/integration/rusefi_config.txt");

/// Test that we can parse the real rusefi_config.txt
#[test]
fn test_parse_real_config() {
    let config_path = Path::new(CONFIG_TXT_PATH);
    
    // Skip if file doesn't exist (e.g., in CI without full repo)
    if !config_path.exists() {
        eprintln!("Skipping: {} not found", CONFIG_TXT_PATH);
        return;
    }
    
    let content = fs::read_to_string(config_path)
        .expect("Failed to read rusefi_config.txt");
    
    // Parse should succeed
    let result = rusefi_codegen::parser::parse_document(&content);
    
    match result {
        Ok(lines) => {
            println!("Successfully parsed {} lines", lines.len());
            // Count non-empty, non-comment lines
            let significant = lines.iter()
                .filter(|l| !matches!(l, rusefi_codegen::parser::ConfigLine::Empty | rusefi_codegen::parser::ConfigLine::Comment(_)))
                .count();
            println!("Significant lines: {}", significant);
            assert!(significant > 0, "Expected at least some significant lines");
        }
        Err(e) => {
            panic!("Parse failed: {}", e);
        }
    }
}

/// Test CLI help works
#[test]
fn test_cli_help() {
    let output = Command::new("cargo")
        .args(["run", "--bin", "rusefi-codegen", "--", "--help"])
        .current_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/.."))
        .output()
        .expect("Failed to run cargo run -- --help");
    
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    
    // Check that help mentions key arguments
    let combined = format!("{}{}", stdout, stderr);
    assert!(
        combined.contains("--definition") || combined.contains("definition"),
        "Help should mention --definition\nGot: {}", combined
    );
}

/// Test with a minimal config file
#[test]
fn test_minimal_config() {
    use tempfile::TempDir;
    
    let temp_dir = TempDir::new().unwrap();
    let config_path = temp_dir.path().join("test_config.txt");
    
    let minimal_config = r#"
! Test config file
#define TEST_VALUE 42
#define TEST_STRING "hello"

struct test_struct
uint8_t field1;First field;"units",1,0,0,255,0
uint16_t field2;Second field;"ms",1,0,0,1000,0
end_struct
"#;
    
    fs::write(&config_path, minimal_config).unwrap();
    
    // Parse the minimal config
    let content = fs::read_to_string(&config_path).unwrap();
    let result = rusefi_codegen::parser::parse_document(&content);
    
    assert!(result.is_ok(), "Failed to parse minimal config: {:?}", result);
    
    let lines = result.unwrap();
    
    // Should have parsed defines and fields
    let defines: Vec<_> = lines.iter()
        .filter_map(|l| match l {
            rusefi_codegen::parser::ConfigLine::Define(k, v) => Some((k.clone(), v.clone())),
            _ => None,
        })
        .collect();
    
    assert_eq!(defines.len(), 2, "Expected 2 defines");
    // Define names are stored as-is from parse; check case-insensitively
    assert!(defines.iter().any(|(k, _)| k.to_lowercase() == "test_value"));
    
    let fields: Vec<_> = lines.iter()
        .filter(|l| matches!(l, rusefi_codegen::parser::ConfigLine::Field(_)))
        .collect();
    
    assert_eq!(fields.len(), 2, "Expected 2 fields");
}

/// Test VariableRegistry with real-world expressions
#[test]
fn test_registry_expressions() {
    use rusefi_codegen::registry::VariableRegistry;
    
    let mut registry = VariableRegistry::new();
    
    // Register common rusEFI defines
    registry.register_numeric("BLOCKING_FACTOR", 1024);
    registry.register_numeric("FUEL_RPM_COUNT", 16);
    registry.register_numeric("FUEL_LOAD_COUNT", 16);
    
    // Test array size calculation
    let expr = "@@FUEL_RPM_COUNT@@ * @@FUEL_LOAD_COUNT@@";
    let expanded = registry.apply_variables(expr);
    assert_eq!(expanded, "16 * 16");
    
    // Test simple evaluation
    let result = registry.evaluate("16 * 16").unwrap();
    assert_eq!(result, 256);
}

/// Compare output with reference (when available)
#[test]
#[ignore = "Requires Java tool output for comparison - run manually"]
fn test_compare_with_java_output() {
    use similar::TextDiff;
    use std::path::PathBuf;
    
    // This test requires:
    // 1. Java tool has been run: java -jar config_definition-all.jar ...
    // 2. Output saved to: tmp/java-output/
    // 3. Rust tool output saved to: tmp/rust-output/
    
    let java_output = PathBuf::from("tmp/java-output/engine_configuration_generated_structures.h");
    let rust_output = PathBuf::from("tmp/rust-output/engine_configuration_generated_structures.h");
    
    if !java_output.exists() || !rust_output.exists() {
        eprintln!("Skipping comparison test - output files not found");
        return;
    }
    
    let java_content = fs::read_to_string(&java_output).unwrap();
    let rust_content = fs::read_to_string(&rust_output).unwrap();
    
    let diff = TextDiff::from_lines(&java_content, &rust_content);
    
    let mut differences = 0;
    for change in diff.iter_all_changes() {
        match change.tag() {
            similar::ChangeTag::Delete | similar::ChangeTag::Insert => {
                differences += 1;
                eprintln!("{}", change);
            }
            _ => {}
        }
    }
    
    assert_eq!(differences, 0, "Found {} differences between Java and Rust output", differences);
}
