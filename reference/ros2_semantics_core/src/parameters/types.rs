/// A ROS-style parameter value.
///
/// # Semantics
/// This enum represents the value payload for a ROS 2 parameter, matching the
/// canonical ROS parameter type set.
///
/// Notes:
/// - [`Value::NotSet`] is the **required observable representation** for unknown
///   parameters in some ROS-facing APIs (see Repo A `docs/spec/parameters.md`).
/// - This crate does not implement deletion by setting `NotSet` unless explicitly
///   supported by the store implementation.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    ByteArray(Vec<u8>),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl Value {
    pub fn ty(&self) -> Type {
        match self {
            Value::NotSet => Type::NotSet,
            Value::Bool(_) => Type::Bool,
            Value::Integer(_) => Type::Integer,
            Value::Double(_) => Type::Double,
            Value::String(_) => Type::String,
            Value::ByteArray(_) => Type::ByteArray,
            Value::BoolArray(_) => Type::BoolArray,
            Value::IntegerArray(_) => Type::IntegerArray,
            Value::DoubleArray(_) => Type::DoubleArray,
            Value::StringArray(_) => Type::StringArray,
        }
    }
}

/// A ROS-style parameter type.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Type {
    NotSet,
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
}

/// Parameter descriptor flags and metadata.
///
/// This is a deliberately minimal model intended to preserve semantic behaviour,
/// not mirror every upstream field.
#[derive(Debug, Clone, PartialEq)]
pub struct Descriptor {
    pub read_only: bool,
    pub dynamic_typing: bool,
    pub additional_constraints: String,
    pub description: String,
}

impl Default for Descriptor {
    fn default() -> Self {
        Self {
            read_only: false,
            dynamic_typing: false,
            additional_constraints: String::new(),
            description: String::new(),
        }
    }
}

/// A named parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: Value,
}

/// A parameter + its descriptor/type, as returned by describe operations.
#[derive(Debug, Clone, PartialEq)]
pub struct DescribedParameter {
    pub name: String,
    pub ty: Type,
    pub descriptor: Descriptor,
}

/// Result of a single set attempt (policy outcome, not an error).
#[derive(Debug, Clone, PartialEq)]
pub struct SetResult {
    pub success: bool,
    pub reason: Option<String>,
}

impl SetResult {
    pub fn ok() -> Self {
        Self {
            success: true,
            reason: None,
        }
    }
    pub fn err(reason: impl Into<String>) -> Self {
        Self {
            success: false,
            reason: Some(reason.into()),
        }
    }
}

/// Event record describing accepted changes for adapter emission.
///
/// Rejected updates MUST NOT produce change records (Repo A global spec).
#[derive(Debug, Default, Clone, PartialEq)]
pub struct EventRecord {
    pub new_parameters: Vec<Parameter>,
    pub changed_parameters: Vec<Parameter>,
    pub deleted_parameters: Vec<Parameter>,
}

impl EventRecord {
    pub fn merge(&mut self, other: EventRecord) {
        self.new_parameters.extend(other.new_parameters);
        self.changed_parameters.extend(other.changed_parameters);
        self.deleted_parameters.extend(other.deleted_parameters);
    }
}

/// Result of a list operation.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct ListResult {
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}
