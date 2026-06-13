//! Parser for rusefi_config.txt files
//!
//! Implements recursive descent parsing for the config file format.

use crate::model::{ConfigField, CustomTypeDef, TsInfo};
use nom::{
    branch::alt,
    bytes::complete::{tag, take_while, take_while1},
    character::complete::{char, digit1, not_line_ending, space0, space1},
    combinator::{opt, recognize},
    sequence::{preceded, tuple},
    IResult, Parser as _,
};

/// A line in the config file
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigLine {
    Empty,
    Comment(String),
    Define(String, String),
    Include(String),
    SplitLines(String),
    StructStart {
        name: String,
        with_prefix: bool,
        comment: Option<String>,
    },
    StructEnd,
    BitField {
        name: String,
        true_label: Option<String>,
        false_label: Option<String>,
        comment: Option<String>,
    },
    CustomType(CustomTypeDef),
    Field(ConfigField),
}

/// Parse a full config file
pub fn parse_document(input: &str) -> Result<Vec<ConfigLine>, String> {
    let lines: Vec<&str> = input.lines().collect();
    let mut results = Vec::new();

    for (idx, line) in lines.iter().enumerate() {
        let line_num = idx + 1;
        let trimmed = line.trim();

        if trimmed.is_empty() {
            results.push(ConfigLine::Empty);
            continue;
        }

        match parse_line(trimmed) {
            Ok((_, parsed)) => results.push(parsed),
            Err(e) => {
                return Err(format!(
                    "Parse error at line {}: {}\n  Line content: {}",
                    line_num, e, line
                ));
            }
        }
    }

    Ok(results)
}

/// Parse a single line
fn parse_line(input: &str) -> IResult<&str, ConfigLine> {
    alt((
        parse_define,
        parse_comment,
        parse_include,
        parse_split_lines,
        parse_struct_no_prefix,
        parse_struct_end,
        parse_struct,
        parse_bit,
        parse_custom,
        parse_field,
    ))
    .parse(input)
}

/// Parse a comment line (starting with !, or //)
/// Note: '#' lines are NOT comments - they may be #define directives
fn parse_comment(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = alt((tag("!"), tag("//"))).parse(input)?;
    let (rest, content) = not_line_ending.parse(rest)?;
    Ok((rest, ConfigLine::Comment(content.to_string())))
}

/// Parse a #define directive
fn parse_define(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("#define").parse(input)?;
    let (rest, _) = space1.parse(rest)?;
    let (rest, name) = identifier.parse(rest)?;
    let (rest, value) = opt(preceded(
        space1,
        take_while(|c: char| !c.is_ascii_control()),
    ))
    .parse(rest)?;

    let value_str = value.map(|s| s.trim().to_string()).unwrap_or_default();
    Ok((rest, ConfigLine::Define(name, value_str)))
}

/// Parse include_file directive
fn parse_include(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("include_file").parse(input)?;
    let (rest, _) = space1.parse(rest)?;
    let (rest, path) = take_while(|c: char| !c.is_ascii_control()).parse(rest)?;
    Ok((rest, ConfigLine::Include(path.trim().to_string())))
}

/// Parse split_lines directive
fn parse_split_lines(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("split_lines").parse(input)?;
    // No space required: split_lines@@VAR@@ or split_lines path
    let (rest, _) = space0.parse(rest)?;
    let (rest, template) = take_while(|c: char| !c.is_ascii_control()).parse(rest)?;
    Ok((rest, ConfigLine::SplitLines(template.trim().to_string())))
}

/// Parse struct start
fn parse_struct(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("struct ").parse(input)?;
    let (rest, name) = identifier.parse(rest)?;
    let (rest, comment) = opt(preceded(
        space1,
        take_while(|c: char| !c.is_ascii_control()),
    ))
    .parse(rest)?;

    Ok((
        rest,
        ConfigLine::StructStart {
            name,
            with_prefix: true,
            comment: comment.map(|s| s.trim().to_string()),
        },
    ))
}

/// Parse struct_no_prefix start
fn parse_struct_no_prefix(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("struct_no_prefix ").parse(input)?;
    let (rest, name) = identifier.parse(rest)?;
    let (rest, comment) = opt(preceded(
        space1,
        take_while(|c: char| !c.is_ascii_control()),
    ))
    .parse(rest)?;

    Ok((
        rest,
        ConfigLine::StructStart {
            name,
            with_prefix: false,
            comment: comment.map(|s| s.trim().to_string()),
        },
    ))
}

/// Parse end_struct
fn parse_struct_end(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("end_struct").parse(input)?;
    Ok((rest, ConfigLine::StructEnd))
}

/// Parse bit field declaration
fn parse_bit(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("bit").parse(input)?;
    let (rest, _) = space1.parse(rest)?;

    // Parse bit_name[,true_value,false_value];comment
    let (rest, bit_spec) = take_while(|c: char| c != ';' && !c.is_ascii_control()).parse(rest)?;
    let (rest, comment) = opt(preceded(char(';'), not_line_ending)).parse(rest)?;

    let bit_spec = bit_spec.trim();
    let parts: Vec<&str> = bit_spec.split(',').collect();

    let name = parts.first().unwrap_or(&"").trim().to_string();
    let true_label = parts.get(1).map(|s| s.trim().trim_matches('"').to_string());
    let false_label = parts.get(2).map(|s| s.trim().trim_matches('"').to_string());

    Ok((
        rest,
        ConfigLine::BitField {
            name,
            true_label,
            false_label,
            comment: comment.map(|s| s.to_string()),
        },
    ))
}

/// Parse custom type definition
fn parse_custom(input: &str) -> IResult<&str, ConfigLine> {
    let (rest, _) = tag("custom").parse(input)?;
    let (rest, _) = space1.parse(rest)?;
    let (rest, name) = identifier.parse(rest)?;
    let (rest, _) = space1.parse(rest)?;
    let (rest, size_str) = digit1.parse(rest)?;
    let (rest, _) = space1.parse(rest)?;
    let (rest, ts_line) = take_while(|c: char| !c.is_ascii_control()).parse(rest)?;

    let size = size_str.parse::<usize>().unwrap_or(1);

    Ok((
        rest,
        ConfigLine::CustomType(CustomTypeDef {
            name,
            size,
            ts_line: ts_line.trim().to_string(),
        }),
    ))
}

/// Parse a field definition
/// Handles both:
///   type name[ARRAY];comment
///   type[ARRAY] name;comment  (iterate form)
fn parse_field(input: &str) -> IResult<&str, ConfigLine> {
    // Parse type name
    let (rest, type_name) = identifier.parse(input)?;

    // Check if array spec is attached directly to type name (no space before '[')
    let (rest, type_array_spec) = opt(parse_array_spec).parse(rest)?;

    let (rest, _) = space1.parse(rest)?;

    // Parse field name
    let (rest, field_name) = identifier.parse(rest)?;

    // Parse optional array spec after field name
    let (rest, name_array_spec) = opt(parse_array_spec).parse(rest)?;

    // Use whichever array spec was found
    let array_spec = type_array_spec.or(name_array_spec);

    // Parse optional semicolon and comment
    let (rest, maybe_comment) = opt(preceded(
        char(';'),
        take_while(|c: char| c != ';' && !c.is_ascii_control()),
    ))
    .parse(rest)?;

    // Parse optional TS info
    let (rest, ts_info) = opt(preceded(
        char(';'),
        take_while(|c: char| !c.is_ascii_control()),
    ))
    .parse(rest)?;

    let comment = maybe_comment.map(|s: &str| s.trim().to_string());

    let mut field = ConfigField::new(field_name, type_name);
    field.comment = comment;

    if let Some((sizes, is_iterate)) = array_spec {
        field.array_sizes = sizes;
        field.is_iterate = is_iterate;
    }

    if let Some(ts_str) = ts_info {
        let ts_str = ts_str.trim();
        if !ts_str.is_empty() {
            if let Ok(ts) = TsInfo::parse(ts_str) {
                field.ts_info = Some(ts);
            }
        }
    }

    Ok((rest, ConfigLine::Field(field)))
}

/// Parse array specification [N], [N x M], [VARNAME], or [VARNAME iterate]
/// Returns (dimensions_as_usize_vec, is_iterate)
fn parse_array_spec(input: &str) -> IResult<&str, (Vec<usize>, bool)> {
    let (rest, _) = char('[').parse(input)?;
    // Content is everything up to ']'
    let (rest, inner) = take_while(|c: char| c != ']').parse(rest)?;
    let (rest, _) = char(']').parse(rest)?;

    let inner = inner.trim();
    let is_iterate = inner.ends_with(" iterate");
    let size_part = if is_iterate {
        inner[..inner.len() - " iterate".len()].trim()
    } else {
        inner
    };

    // Split on 'x' to get dimensions
    let dims: Vec<usize> = size_part
        .split(['x', 'X'])
        .map(|s| s.trim().parse::<usize>().unwrap_or(0)) // 0 = variable size (resolved later)
        .collect();

    Ok((rest, (dims, is_iterate)))
}

/// Parse an identifier (plain or @@TEMPLATE@@ form)
fn identifier(input: &str) -> IResult<&str, String> {
    // Try @@TEMPLATE_VAR@@ form first
    if let Ok((rest, id)) = template_identifier(input) {
        return Ok((rest, id));
    }
    // Regular identifier: starts with letter/underscore
    let (rest, id) = recognize(tuple((
        take_while1(|c: char| c.is_alphabetic() || c == '_'),
        take_while(|c: char| c.is_alphanumeric() || c == '_'),
    )))
    .parse(input)?;
    Ok((rest, id.to_string()))
}

/// Parse a @@TEMPLATE_VAR@@ style identifier
fn template_identifier(input: &str) -> IResult<&str, String> {
    let (rest, _) = tag("@@").parse(input)?;
    let (rest, name) = take_while1(|c: char| c.is_alphanumeric() || c == '_').parse(rest)?;
    let (rest, _) = tag("@@").parse(rest)?;
    Ok((rest, format!("@@{}@@", name)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_comment() {
        let (_, line) = parse_comment("! This is a comment").unwrap();
        assert!(matches!(line, ConfigLine::Comment(_)));
    }

    #[test]
    fn test_parse_define() {
        let (_, line) = parse_define("#define FLASH_DATA_VERSION 260407").unwrap();
        if let ConfigLine::Define(name, value) = line {
            assert_eq!(name, "FLASH_DATA_VERSION");
            assert_eq!(value, "260407");
        } else {
            panic!("Expected Define");
        }
    }

    #[test]
    fn test_parse_struct() {
        let (_, line) = parse_struct("struct engine_configuration_s").unwrap();
        if let ConfigLine::StructStart {
            name, with_prefix, ..
        } = line
        {
            assert_eq!(name, "engine_configuration_s");
            assert!(with_prefix);
        } else {
            panic!("Expected StructStart");
        }
    }

    #[test]
    fn test_parse_field_simple() {
        let (_, line) =
            parse_field("uint8_t cylindersCount;Number of Cylinders;\"count\",1,0,1,12,0").unwrap();
        if let ConfigLine::Field(field) = line {
            assert_eq!(field.name, "cylindersCount");
            assert_eq!(field.type_name, "uint8_t");
            assert!(field.ts_info.is_some());
        } else {
            panic!("Expected Field");
        }
    }

    #[test]
    fn test_parse_field_array() {
        let (_, line) = parse_field("float veTable[16 x 16];VE table;\"%\",1,0,0,255,0").unwrap();
        if let ConfigLine::Field(field) = line {
            assert_eq!(field.name, "veTable");
            assert_eq!(field.array_sizes, vec![16, 16]);
        } else {
            panic!("Expected Field");
        }
    }

    #[test]
    fn test_parse_bit() {
        let (_, line) = parse_bit("bit isEnabled;Enable this feature").unwrap();
        if let ConfigLine::BitField { name, .. } = line {
            assert_eq!(name, "isEnabled");
        } else {
            panic!("Expected BitField");
        }
    }

    #[test]
    fn test_parse_custom() {
        let (_, line) = parse_custom(
            "custom can_baudrate_e 1 bits, U08, @OFFSET@, [0:1], @@can_baudrate_e_enum@@",
        )
        .unwrap();
        if let ConfigLine::CustomType(ct) = line {
            assert_eq!(ct.name, "can_baudrate_e");
            assert_eq!(ct.size, 1);
        } else {
            panic!("Expected CustomType");
        }
    }
}
