//! rusEFI Configuration Code Generator Library
//!
//! Rust port of the Java configuration_definition toolset.
//! Generates C headers and TunerStudio INI files from rusefi_config.txt.

// Public API items will be wired up to main.rs in a later step.
// Until then, suppress dead_code warnings for this library crate.
#![allow(dead_code)]

pub mod generator;
pub mod model;
pub mod parser;
pub mod registry;

use anyhow::{Context, Result};
use std::path::Path;

use generator::CHeaderGenerator;
use model::{ConfigDocument, ConfigStructure, TypeRegistry};
use parser::{ConfigLine, parse_document};
use registry::VariableRegistry;

/// Options for a single code generation run
pub struct GenerateOptions<'a> {
    /// Primary definition file paths (prepend + main)
    pub definition_files: Vec<&'a Path>,
    /// C header output paths (one file per path)
    pub c_destinations: Vec<&'a Path>,
    /// TunerStudio INI output directory
    pub ts_destination: Option<&'a Path>,
    /// Whether to output `#define` values in the C header
    pub with_c_defines: bool,
    /// Whether to zero-initialize struct members
    pub initialize_to_zero: bool,
}

/// Run the code generation process
///
/// Reads all definition files in order, processes `#define` directives into a
/// `VariableRegistry`, builds the struct hierarchy, and writes each requested
/// output file.
pub fn generate(opts: &GenerateOptions<'_>) -> Result<()> {
    let mut registry = VariableRegistry::new();
    let mut type_registry = TypeRegistry::new();
    let mut document = ConfigDocument::new();

    // Read and parse every definition file in order
    for path in &opts.definition_files {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Cannot read {:?}", path))?;

        let lines = parse_document(&content)
            .map_err(|e| anyhow::anyhow!("{}", e))?;

        process_lines(lines, &mut registry, &mut type_registry, &mut document)?;
    }

    // --- C header output ---
    for dest in &opts.c_destinations {
        let mut c_gen = CHeaderGenerator::new(registry.clone());
        c_gen.set_zero_init(opts.initialize_to_zero);
        c_gen.set_with_defines(opts.with_c_defines);

        if let Some(file_name) = dest.file_name().and_then(|n| n.to_str()) {
            c_gen.start_file(file_name);
        }
        for structure in &document.structures {
            c_gen.handle_struct(structure, &type_registry)?;
        }

        if let Some(parent) = dest.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("Cannot create directory {:?}", parent))?;
        }
        c_gen.end_file(dest)?;
    }

    Ok(())
}

/// Process a parsed line list: register defines, build struct stack
fn process_lines(
    lines: Vec<ConfigLine>,
    registry: &mut VariableRegistry,
    _type_registry: &mut TypeRegistry,
    document: &mut ConfigDocument,
) -> Result<()> {
    let mut struct_stack: Vec<ConfigStructure> = Vec::new();

    for line in lines {
        match line {
            ConfigLine::Define(name, value) => {
                // Apply existing variables to the value before storing
                let expanded = registry.apply_variables(&value);
                registry.register(name, expanded);
            }
            ConfigLine::StructStart { name, with_prefix, .. } => {
                struct_stack.push(ConfigStructure::new(&name, with_prefix));
            }
            ConfigLine::StructEnd => {
                if let Some(finished) = struct_stack.pop() {
                    if struct_stack.is_empty() {
                        document.structures.push(finished);
                    } else if let Some(parent) = struct_stack.last_mut() {
                        // Nested struct — add as inline field (future work)
                        let _ = parent;
                    }
                }
            }
            ConfigLine::Field(field) => {
                if let Some(current) = struct_stack.last_mut() {
                    current.add_field(field);
                } else {
                    // Top-level field outside any struct
                    document.top_level_fields.push(field);
                }
            }
            ConfigLine::BitField { name, true_label, false_label, comment } => {
                use model::ConfigField;
                if let Some(current) = struct_stack.last_mut() {
                    let f = ConfigField::new_bit_field(
                        &name,
                        true_label,
                        false_label,
                        comment,
                    );
                    current.add_bit_field(f);
                }
            }
            ConfigLine::CustomType(ct) => {
                _type_registry.register_custom(&ct.name, ct.size);
            }
            // Include / SplitLines are resolved at a higher level (CLI)
            _ => {}
        }
    }

    Ok(())
}

/// Version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
