//! Variable registry for storing and substituting #define values
//!
//! Similar to the Java VariableRegistry class.

use regex::Regex;
use std::collections::HashMap;
use tracing::debug;

/// Variable registry for #define storage and substitution
#[derive(Debug, Default, Clone)]
pub struct VariableRegistry {
    /// Variable storage (case-insensitive lookup)
    variables: HashMap<String, String>,
    /// Integer value cache for numeric variables
    int_values: HashMap<String, i64>,
}

impl VariableRegistry {
    pub const TEMPLATE_TAG: &'static str = "@@";
    pub const TEMPLATE_QUITE_OPEN: &'static str = "@#";
    pub const TEMPLATE_QUITE_CLOSE: &'static str = "#@";
    pub const MULT_TOKEN: char = '*';

    /// Create a new empty registry
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a variable with its value
    pub fn register(&mut self, name: impl Into<String>, value: impl Into<String>) {
        let name = name.into();
        let value = value.into();
        debug!("Registering variable: {} = {}", name, value);
        self.variables.insert(name.to_lowercase(), value);
    }

    /// Register a numeric variable (also caches as int)
    pub fn register_numeric(&mut self, name: impl Into<String>, value: i64) {
        let name = name.into();
        self.int_values.insert(name.to_lowercase(), value);
        self.variables
            .insert(name.to_lowercase(), value.to_string());
    }

    /// Get a variable value (case-insensitive)
    pub fn get(&self, name: &str) -> Option<&str> {
        self.variables.get(&name.to_lowercase()).map(|s| s.as_str())
    }

    /// Check if a variable exists
    pub fn contains(&self, name: &str) -> bool {
        self.variables.contains_key(&name.to_lowercase())
    }

    /// Get a numeric value if available
    pub fn get_numeric(&self, name: &str) -> Option<i64> {
        if let Some(&val) = self.int_values.get(&name.to_lowercase()) {
            return Some(val);
        }
        // Try parsing from string value
        self.get(name).and_then(|s| s.parse().ok())
    }

    /// Apply variable substitution to a string
    /// Replaces @@VAR@@ with the variable value
    pub fn apply_variables(&self, input: &str) -> String {
        self.apply_template(input, Self::TEMPLATE_TAG, Self::TEMPLATE_TAG)
    }

    /// Apply template substitution with custom delimiters
    fn apply_template(&self, input: &str, open: &str, close: &str) -> String {
        let pattern = format!(
            "{}([A-Za-z_][A-Za-z0-9_]*){}",
            regex::escape(open),
            regex::escape(close)
        );
        let Ok(re) = Regex::new(&pattern) else {
            // Pattern is always valid (built from regex::escape), so this branch is unreachable
            return input.to_string();
        };

        re.replace_all(input, |caps: &regex::Captures| {
            let var_name = &caps[1];
            self.get(var_name).unwrap_or(&caps[0]).to_string()
        })
        .to_string()
    }

    /// Apply quiet template substitution (@#expr#@)
    /// Similar to apply_variables but with different delimiters
    pub fn apply_quiet_template(&self, input: &str) -> String {
        self.apply_template(input, Self::TEMPLATE_QUITE_OPEN, Self::TEMPLATE_QUITE_CLOSE)
    }

    /// Process a #define line and register it
    pub fn process_define_line(&mut self, line: &str) -> Result<(), String> {
        let line = line.trim();
        if !line.starts_with("#define") {
            return Err("Not a #define line".to_string());
        }

        let rest = line[7..].trim();
        let parts: Vec<&str> = rest.splitn(2, |c: char| c.is_ascii_whitespace()).collect();

        let name = parts[0].trim();
        if name.is_empty() {
            return Err("Empty define name".to_string());
        }

        let value = parts.get(1).map(|s| s.trim()).unwrap_or("");

        // Try to parse as numeric
        if let Ok(num) = value.parse::<i64>() {
            self.register_numeric(name, num);
        } else {
            self.register(name, value);
        }

        Ok(())
    }

    /// Read prepend file and register all #defines
    pub fn read_prepend_file(&mut self, path: &std::path::Path) -> Result<(), String> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;

        for (line_num, line) in content.lines().enumerate() {
            let trimmed = line.trim();
            if trimmed.starts_with("#define") {
                if let Err(e) = self.process_define_line(trimmed) {
                    return Err(format!("{}:{}: {}", path.display(), line_num + 1, e));
                }
            }
        }

        Ok(())
    }

    /// Get all variable names
    pub fn keys(&self) -> impl Iterator<Item = &String> {
        self.variables.keys()
    }

    /// Evaluate an expression with variables
    /// Supports basic arithmetic: +, -, *, /
    pub fn evaluate(&self, expr: &str) -> Result<i64, String> {
        // Simple expression evaluator - supports variables and basic ops
        let expr = self.apply_variables(expr);
        self.eval_expr(&expr)
    }

    fn eval_expr(&self, expr: &str) -> Result<i64, String> {
        // Very basic parser for expressions like "16 * 16" or "1024"
        let expr = expr.trim();

        // Try direct parse first
        if let Ok(val) = expr.parse::<i64>() {
            return Ok(val);
        }

        // Handle simple binary expressions
        for op in ["*", "/", "+", "-"] {
            if let Some(pos) = expr.find(op) {
                let left = expr[..pos].trim();
                let right = expr[pos + op.len()..].trim();

                let left_val = self.eval_expr(left)?;
                let right_val = self.eval_expr(right)?;

                return match op {
                    "*" => Ok(left_val * right_val),
                    "/" => Ok(left_val / right_val),
                    "+" => Ok(left_val + right_val),
                    "-" => Ok(left_val - right_val),
                    _ => Err(format!("Unknown operator: {}", op)),
                };
            }
        }

        Err(format!("Cannot evaluate expression: {}", expr))
    }

    /// Unquote a string (remove surrounding quotes)
    pub fn unquote(s: &str) -> String {
        let s = s.trim();
        if let (Some(first), Some(last)) = (s.chars().next(), s.chars().last()) {
            if s.len() >= 2 && first == last && (first == '"' || first == '\'') {
                return s[1..s.len() - 1].to_string();
            }
        }
        s.to_string()
    }

    /// Quote a string (add surrounding double quotes)
    pub fn quote(s: &str) -> String {
        format!("\"{}\"", s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_get() {
        let mut reg = VariableRegistry::new();
        reg.register("TEST_VAR", "test_value");
        assert_eq!(reg.get("TEST_VAR"), Some("test_value"));
        assert_eq!(reg.get("test_var"), Some("test_value")); // case-insensitive
    }

    #[test]
    fn test_apply_variables() {
        let mut reg = VariableRegistry::new();
        reg.register("SIZE", "16");
        reg.register("NAME", "test");

        let result = reg.apply_variables("Field @@NAME@@ has size @@SIZE@@");
        assert_eq!(result, "Field test has size 16");
    }

    #[test]
    fn test_process_define() {
        let mut reg = VariableRegistry::new();
        reg.process_define_line("#define BLOCKING_FACTOR 1024")
            .unwrap();
        assert_eq!(reg.get("BLOCKING_FACTOR"), Some("1024"));
        assert_eq!(reg.get_numeric("BLOCKING_FACTOR"), Some(1024));
    }

    #[test]
    fn test_evaluate_expression() {
        let mut reg = VariableRegistry::new();
        reg.register_numeric("A", 10);
        reg.register_numeric("B", 5);

        assert_eq!(reg.evaluate("16 * 16"), Ok(256));
        assert_eq!(reg.evaluate("10 + 5"), Ok(15));
    }

    #[test]
    fn test_unquote() {
        assert_eq!(VariableRegistry::unquote("\"hello\""), "hello");
        assert_eq!(VariableRegistry::unquote("'hello'"), "hello");
        assert_eq!(VariableRegistry::unquote("hello"), "hello");
    }
}
