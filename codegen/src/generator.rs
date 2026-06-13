//! Code generators for C headers and TunerStudio INI files

use crate::model::{ConfigField, ConfigStructure, TsInfo, TypeRegistry};
use anyhow::{anyhow, Result};
use std::path::Path;
use tracing::info;

/// C header file generator
pub struct CHeaderGenerator {
    output: String,
    registry: crate::registry::VariableRegistry,
    need_zero_init: bool,
    with_defines: bool,
}

impl CHeaderGenerator {
    /// Create a new C header generator
    pub fn new(registry: crate::registry::VariableRegistry) -> Self {
        Self {
            output: String::new(),
            registry,
            need_zero_init: false,
            with_defines: true,
        }
    }

    /// Set zero initialization flag
    pub fn set_zero_init(&mut self, value: bool) {
        self.need_zero_init = value;
    }

    /// Set whether to include #defines in output
    pub fn set_with_defines(&mut self, value: bool) {
        self.with_defines = value;
    }

    /// Start a new file
    pub fn start_file(&mut self, source_file: &str) {
        self.output.push_str("//\n");
        self.output.push_str(&format!(
            "// Generated automatically from {}\n",
            source_file
        ));
        self.output.push_str("// by rusefi-codegen\n");
        self.output.push_str("//\n\n");
        self.output.push_str("#pragma once\n");
        self.output.push_str("#include \"rusefi_types.h\"\n\n");
    }

    /// End the file and write output
    pub fn end_file(&mut self, path: &Path) -> Result<()> {
        if self.with_defines {
            self.output.push_str("\n// Defines section\n");
            for key in self.registry.keys() {
                if let Some(value) = self.registry.get(key) {
                    self.output
                        .push_str(&format!("#define {} {}\n", key.to_uppercase(), value));
                }
            }
        }

        self.output.push_str("\n// end\n");

        std::fs::write(path, &self.output)?;
        info!("Wrote C header: {:?}", path);
        Ok(())
    }

    /// Handle a structure end (write struct definition)
    pub fn handle_struct(
        &mut self,
        structure: &ConfigStructure,
        types: &TypeRegistry,
    ) -> Result<()> {
        let struct_name = &structure.name;

        // Struct comment
        if let Some(comment) = &structure.comment {
            self.output.push_str(&format!("// {}\n", comment));
        }

        // Struct definition
        self.output.push_str("#pragma pack(push, 1)\n");
        self.output
            .push_str(&format!("struct {} {{\n", struct_name));

        // Track offset for @OFFSET@ substitution
        let mut offset = 0;

        for field in &structure.fields {
            self.write_field(field, &mut offset, types)?;
        }

        self.output.push_str("};\n");
        self.output.push_str("#pragma pack(pop)\n\n");

        // Register struct size
        let size = structure.total_size(types);
        self.registry
            .register(format!("{}_size", struct_name), size.to_string());

        Ok(())
    }

    fn write_field(
        &mut self,
        field: &ConfigField,
        offset: &mut usize,
        types: &TypeRegistry,
    ) -> Result<()> {
        // Add field comment
        if let Some(comment) = &field.comment {
            self.output.push_str(&format!("    // {}\n", comment));
        }

        // Field type and name
        let type_name = &field.type_name;
        let field_name = &field.name;

        // Handle arrays
        if field.is_array() {
            let dims: Vec<String> = field.array_sizes.iter().map(|s| s.to_string()).collect();
            self.output.push_str(&format!(
                "    {} {}[{}];\n",
                type_name,
                field_name,
                dims.join("][")
            ));
        } else {
            self.output
                .push_str(&format!("    {} {};\n", type_name, field_name));
        }

        // Update offset
        *offset += field.total_size(types);

        Ok(())
    }
}

/// TunerStudio INI generator
pub struct TsIniGenerator {
    constants: String,
    registry: crate::registry::VariableRegistry,
    total_offset: usize,
}

impl TsIniGenerator {
    /// Create a new TS INI generator
    pub fn new(registry: crate::registry::VariableRegistry) -> Self {
        Self {
            constants: String::new(),
            registry,
            total_offset: 0,
        }
    }

    /// Start the constants section
    pub fn start_constants(&mut self, page: usize) {
        self.constants.push_str(&format!("page = {}\n", page));
    }

    /// Handle a field for TS output
    pub fn handle_field(&mut self, field: &ConfigField, types: &TypeRegistry) -> Result<()> {
        // Skip bit fields for now (handled separately)
        if field.is_bit_field {
            return Ok(());
        }

        // Skip directive fields
        if field.is_directive {
            return Ok(());
        }

        let ts_type = types
            .to_ts_type(&field.type_name)
            .ok_or_else(|| anyhow!("Unknown type for TS: {}", field.type_name))?;

        // Handle arrays - expand for TS
        if field.is_array() {
            self.handle_array_field(field, ts_type, types)?;
        } else {
            self.handle_scalar_field(field, ts_type)?;
        }

        Ok(())
    }

    fn handle_scalar_field(&mut self, field: &ConfigField, ts_type: &str) -> Result<()> {
        let name = &field.name;
        let offset = self.total_offset;

        self.constants
            .push_str(&format!("    {} = {}, {}, ", name, ts_type, offset));

        if let Some(ts_info) = &field.ts_info {
            self.write_ts_info(ts_info);
        } else {
            self.constants.push_str("\"units\", 1, 0, 1, 100, 0\n");
        }

        self.total_offset += 4; // Approximate - needs proper size tracking

        Ok(())
    }

    fn handle_array_field(
        &mut self,
        field: &ConfigField,
        ts_type: &str,
        types: &TypeRegistry,
    ) -> Result<()> {
        let base_name = &field.name;
        let element_size = field.type_size(types);
        let total_elements = field.total_array_size();

        // For 2D arrays, we need to flatten for TS
        for i in 0..total_elements {
            let field_name = format!("{}{}", base_name, i + 1);
            let offset = self.total_offset + (i * element_size);

            self.constants
                .push_str(&format!("    {} = {}, {}, ", field_name, ts_type, offset));

            if let Some(ts_info) = &field.ts_info {
                self.write_ts_info(ts_info);
            } else {
                self.constants.push_str("\"units\", 1, 0, 0, 100, 0\n");
            }
        }

        self.total_offset += element_size * total_elements;

        Ok(())
    }

    fn write_ts_info(&mut self, info: &TsInfo) {
        self.constants.push_str(&format!(
            "\"{}\", {}, {}, {}, {}, {}\n",
            info.units, info.scale, info.offset, info.min, info.max, info.digits
        ));
    }

    /// Write the generated INI content
    pub fn write_to_file(&self, path: &Path, template_content: Option<&str>) -> Result<()> {
        let output = if let Some(template) = template_content {
            self.merge_with_template(template)
        } else {
            self.generate_standalone()
        };

        std::fs::write(path, output)?;
        info!("Wrote TS INI: {:?}", path);
        Ok(())
    }

    fn generate_standalone(&self) -> String {
        let mut output = String::new();
        output.push_str("; Generated by rusefi-codegen\n\n");
        output.push_str("[Constants]\n");
        output.push_str(&self.constants);
        output
    }

    fn merge_with_template(&self, template: &str) -> String {
        // Find CONFIG_DEFINITION_START and CONFIG_DEFINITION_END markers
        let start_marker = "CONFIG_DEFINITION_START";
        let end_marker = "CONFIG_DEFINITION_END";

        let start_pos = template
            .find(start_marker)
            .map(|p| template[..p].rfind('\n').map(|n| n + 1).unwrap_or(0));
        let end_pos = template.find(end_marker).map(|p| p + end_marker.len());

        if let (Some(start), Some(end)) = (start_pos, end_pos) {
            let prefix = &template[..start];
            let suffix = &template[end..];

            format!(
                "{}; {}\n{}\n; {}\n{}",
                prefix, start_marker, self.constants, end_marker, suffix
            )
        } else {
            // No markers found, append at end
            format!("{}\n\n[Constants]\n{}", template, self.constants)
        }
    }

    /// Get total size of all processed fields
    pub fn total_size(&self) -> usize {
        self.total_offset
    }
}

/// Java VariableRegistry generator
pub struct JavaRegistryGenerator;

impl JavaRegistryGenerator {
    /// Generate Java VariableRegistryValues class
    pub fn generate(
        registry: &crate::registry::VariableRegistry,
        output_path: &Path,
        class_name: &str,
    ) -> Result<()> {
        let mut output = String::new();

        output.push_str("// Generated by rusefi-codegen\n\n");
        output.push_str("package com.rusefi.config.generated;\n\n");
        output.push_str(&format!("public class {} {{\n", class_name));

        for key in registry.keys() {
            if let Some(value) = registry.get(key) {
                // Try to determine if it's a string or numeric constant
                if value.parse::<i64>().is_ok() {
                    output.push_str(&format!(
                        "    public static final int {} = {};\n",
                        key.to_uppercase(),
                        value
                    ));
                } else {
                    output.push_str(&format!(
                        "    public static final String {} = \"{}\";\n",
                        key.to_uppercase(),
                        value
                    ));
                }
            }
        }

        output.push_str("}\n");

        std::fs::write(output_path, output)?;
        info!("Wrote Java registry: {:?}", output_path);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ConfigField;
    use crate::registry::VariableRegistry;

    #[test]
    fn test_c_header_generation() {
        let registry = VariableRegistry::new();
        let mut generator = CHeaderGenerator::new(registry);

        generator.start_file("test.txt");

        let types = TypeRegistry::new();
        let mut structure = ConfigStructure::new("test_struct", true);
        structure.add_field(ConfigField::new("field1", "uint8_t"));
        structure.add_field(ConfigField::new("field2", "float"));

        generator.handle_struct(&structure, &types).unwrap();

        assert!(generator.output.contains("struct test_struct"));
        assert!(generator.output.contains("uint8_t field1"));
        assert!(generator.output.contains("float field2"));
    }

    #[test]
    fn test_ts_generator() {
        let registry = VariableRegistry::new();
        let mut generator = TsIniGenerator::new(registry);

        generator.start_constants(1);

        let types = TypeRegistry::new();
        let mut field = ConfigField::new("testField", "uint16_t");
        field.ts_info = Some(TsInfo {
            units: "ms".to_string(),
            scale: 1.0,
            offset: 0.0,
            min: 0.0,
            max: 1000.0,
            digits: 0,
        });

        generator.handle_field(&field, &types).unwrap();

        assert!(generator.constants.contains("testField"));
        assert!(generator.constants.contains("U16"));
        assert!(generator.constants.contains("ms"));
    }
}
