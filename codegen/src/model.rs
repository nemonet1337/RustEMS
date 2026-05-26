//! Core data models for configuration code generation
//! 
//! This module defines the AST representation of parsed config files
//! and type system definitions.

use std::collections::HashMap;

/// A configuration field definition
#[derive(Debug, Clone, PartialEq)]
pub struct ConfigField {
    /// Field name (identifier)
    pub name: String,
    /// C type name
    pub type_name: String,
    /// Optional comment text
    pub comment: Option<String>,
    /// Array dimensions (empty for scalar)
    pub array_sizes: Vec<usize>,
    /// TunerStudio metadata
    pub ts_info: Option<TsInfo>,
    /// Whether this is a bit field (boolean packed in 32-bit word)
    pub is_bit_field: bool,
    /// Whether this is a preprocessor directive
    pub is_directive: bool,
    /// True value label for bit fields
    pub true_label: Option<String>,
    /// False value label for bit fields  
    pub false_label: Option<String>,
    /// Has autoscale flag
    pub has_autoscale: bool,
    /// Iterate flag (for TS output expansion)
    pub is_iterate: bool,
    /// Source iterate field name
    pub from_iterate: Option<String>,
    /// Iterate index
    pub iterate_index: Option<usize>,
}

impl ConfigField {
    /// Create a new scalar field
    pub fn new(name: impl Into<String>, type_name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            type_name: type_name.into(),
            comment: None,
            array_sizes: Vec::new(),
            ts_info: None,
            is_bit_field: false,
            is_directive: false,
            true_label: None,
            false_label: None,
            has_autoscale: false,
            is_iterate: false,
            from_iterate: None,
            iterate_index: None,
        }
    }

    /// Create a new bit field
    pub fn new_bit_field(
        name: impl Into<String>,
        true_label: Option<String>,
        false_label: Option<String>,
        comment: Option<String>,
    ) -> Self {
        Self {
            name: name.into(),
            type_name: "boolean".to_string(),
            comment,
            array_sizes: Vec::new(),
            ts_info: None,
            is_bit_field: true,
            is_directive: false,
            true_label,
            false_label,
            has_autoscale: false,
            is_iterate: false,
            from_iterate: None,
            iterate_index: None,
        }
    }

    /// Check if this field is an array
    pub fn is_array(&self) -> bool {
        !self.array_sizes.is_empty()
    }

    /// Get the total number of elements in an array
    pub fn total_array_size(&self) -> usize {
        self.array_sizes.iter().product()
    }

    /// Get the C type size in bytes
    pub fn type_size(&self, types: &TypeRegistry) -> usize {
        types.get_size(&self.type_name)
    }

    /// Get the total size including arrays
    pub fn total_size(&self, types: &TypeRegistry) -> usize {
        self.type_size(types) * self.total_array_size()
    }
}

/// TunerStudio field metadata
#[derive(Debug, Clone, PartialEq)]
pub struct TsInfo {
    /// Display units (e.g., "ms", "kPa", "%")
    pub units: String,
    /// Scale factor for conversion
    pub scale: f64,
    /// Offset for conversion
    pub offset: f64,
    /// Minimum value
    pub min: f64,
    /// Maximum value
    pub max: f64,
    /// Number of digits for display
    pub digits: u32,
}

impl TsInfo {
    /// Parse TS info from string: "units",scale,offset,min,max,digits
    pub fn parse(s: &str) -> Result<Self, String> {
        // Remove surrounding quotes from units if present
        let parts: Vec<&str> = s.split(',').collect();
        if parts.len() != 6 {
            return Err(format!("Expected 6 components, got {}", parts.len()));
        }

        let units = parts[0].trim().trim_matches('"').to_string();
        let scale = parts[1].trim().parse().map_err(|_| "Invalid scale")?;
        let offset = parts[2].trim().parse().map_err(|_| "Invalid offset")?;
        let min = parts[3].trim().parse().map_err(|_| "Invalid min")?;
        let max = parts[4].trim().parse().map_err(|_| "Invalid max")?;
        let digits = parts[5].trim().parse().map_err(|_| "Invalid digits")?;

        Ok(Self {
            units,
            scale,
            offset,
            min,
            max,
            digits,
        })
    }
}

/// A configuration structure definition
#[derive(Debug, Clone, PartialEq)]
pub struct ConfigStructure {
    /// Struct name
    pub name: String,
    /// Optional comment
    pub comment: Option<String>,
    /// Whether to add prefix to field names
    pub with_prefix: bool,
    /// Parent struct name (for nested structs)
    pub parent: Option<String>,
    /// Fields in this struct
    pub fields: Vec<ConfigField>,
    /// Bit fields pending packing
    pub pending_bits: Vec<ConfigField>,
}

impl ConfigStructure {
    /// Create a new struct
    pub fn new(name: impl Into<String>, with_prefix: bool) -> Self {
        Self {
            name: name.into(),
            comment: None,
            with_prefix,
            parent: None,
            fields: Vec::new(),
            pending_bits: Vec::new(),
        }
    }

    /// Add a field to this struct
    pub fn add_field(&mut self, field: ConfigField) {
        self.fields.push(field);
    }

    /// Add a bit field (accumulates until alignment needed)
    pub fn add_bit_field(&mut self, field: ConfigField) {
        self.pending_bits.push(field);
    }

    /// Flush pending bit fields into the main field list
    pub fn flush_bits(&mut self) {
        if !self.pending_bits.is_empty() {
            // TODO: Create a packed bit field representation
            self.fields.append(&mut self.pending_bits);
        }
    }

    /// Calculate total struct size
    pub fn total_size(&self, types: &TypeRegistry) -> usize {
        let mut size = 0;
        for field in &self.fields {
            size += field.total_size(types);
        }
        // Align to 4 bytes
        (size + 3) & !3
    }
}

/// Type registry for looking up type sizes and properties
#[derive(Debug, Default)]
pub struct TypeRegistry {
    /// Primitive type sizes
    primitives: HashMap<String, usize>,
    /// Custom types (from 'custom' directive)
    custom_types: HashMap<String, usize>,
    /// Struct definitions
    structs: HashMap<String, usize>,
}

impl TypeRegistry {
    /// Create a new registry with primitive types pre-registered
    pub fn new() -> Self {
        let mut registry = Self::default();
        
        // Register primitive C types
        registry.primitives.insert("int8_t".to_string(), 1);
        registry.primitives.insert("uint8_t".to_string(), 1);
        registry.primitives.insert("int16_t".to_string(), 2);
        registry.primitives.insert("uint16_t".to_string(), 2);
        registry.primitives.insert("int32_t".to_string(), 4);
        registry.primitives.insert("int".to_string(), 4);
        registry.primitives.insert("uint32_t".to_string(), 4);
        registry.primitives.insert("float".to_string(), 4);
        registry.primitives.insert("boolean".to_string(), 4);

        registry
    }

    /// Get the size of a type in bytes
    pub fn get_size(&self, type_name: &str) -> usize {
        if let Some(&size) = self.primitives.get(type_name) {
            return size;
        }
        if let Some(&size) = self.custom_types.get(type_name) {
            return size;
        }
        if let Some(&size) = self.structs.get(type_name) {
            return size;
        }
        // Default to 4 for unknown types (with warning in actual implementation)
        4
    }

    /// Register a custom type
    pub fn register_custom(&mut self, name: impl Into<String>, size: usize) {
        self.custom_types.insert(name.into(), size);
    }

    /// Register a struct type
    pub fn register_struct(&mut self, name: impl Into<String>, size: usize) {
        self.structs.insert(name.into(), size);
    }

    /// Check if a type is a primitive
    pub fn is_primitive(&self, type_name: &str) -> bool {
        self.primitives.contains_key(type_name)
    }

    /// Check if a type is a struct
    pub fn is_struct(&self, type_name: &str) -> bool {
        self.structs.contains_key(type_name)
    }

    /// Get min/max values for numeric types
    pub fn get_range(&self, type_name: &str) -> Option<(i64, i64)> {
        match type_name {
            "int8_t" => Some((i8::MIN as i64, i8::MAX as i64)),
            "uint8_t" => Some((0, u8::MAX as i64)),
            "int16_t" => Some((i16::MIN as i64, i16::MAX as i64)),
            "uint16_t" => Some((0, u16::MAX as i64)),
            "int32_t" | "int" => Some((i32::MIN as i64, i32::MAX as i64)),
            "uint32_t" => Some((0, u32::MAX as i64)),
            _ => None,
        }
    }

    /// Convert C type to TunerStudio type code
    pub fn to_ts_type(&self, type_name: &str) -> Option<&'static str> {
        match type_name {
            "float" => Some("F32"),
            "uint32_t" => Some("U32"),
            "int32_t" | "int" => Some("S32"),
            "int16_t" => Some("S16"),
            "uint16_t" => Some("U16"),
            "int8_t" => Some("S08"),
            "uint8_t" => Some("U08"),
            _ => None,
        }
    }
}

/// Parsed document (entire config file)
#[derive(Debug, Default)]
pub struct ConfigDocument {
    /// Top-level defines
    pub defines: HashMap<String, String>,
    /// All structures
    pub structures: Vec<ConfigStructure>,
    /// Top-level fields outside any struct
    pub top_level_fields: Vec<ConfigField>,
    /// Custom type definitions
    pub custom_types: Vec<CustomTypeDef>,
}

impl ConfigDocument {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Custom type definition from 'custom' directive
#[derive(Debug, Clone, PartialEq)]
pub struct CustomTypeDef {
    /// Type name
    pub name: String,
    /// Size in bytes
    pub size: usize,
    /// TunerStudio line definition
    pub ts_line: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_type_registry_primitives() {
        let reg = TypeRegistry::new();
        assert_eq!(reg.get_size("uint8_t"), 1);
        assert_eq!(reg.get_size("int16_t"), 2);
        assert_eq!(reg.get_size("float"), 4);
    }

    #[test]
    fn test_ts_info_parse() {
        let info = TsInfo::parse("\"ms\",1,0,-10,10,2").unwrap();
        assert_eq!(info.units, "ms");
        assert_eq!(info.scale, 1.0);
        assert_eq!(info.min, -10.0);
        assert_eq!(info.max, 10.0);
        assert_eq!(info.digits, 2);
    }

    #[test]
    fn test_config_field_array() {
        let mut field = ConfigField::new("test", "uint8_t");
        field.array_sizes = vec![4, 8];
        assert!(field.is_array());
        assert_eq!(field.total_array_size(), 32);
    }
}
