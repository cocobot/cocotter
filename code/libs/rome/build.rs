use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::collections::{HashMap, HashSet};
use yaml_rust::{YamlLoader, Yaml};


enum ParamType<'a> {
    Name(&'a str),
    Choice(Vec<&'a str>),
}

enum Parameters<'a> {
    None,
    Positional(Vec<ParamType<'a>>),
    Named(Vec<(&'a str, ParamType<'a>)>),
}

struct Message<'a> {
    id: u8,
    name: &'a str,
    parameters: Parameters<'a>,
}


fn main() {
    let yaml_file = "rome_messages.yaml";
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", yaml_file);

    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let out_path = out_dir.join("rome_messages.rs");

    let doc = load_yaml_doc(yaml_file);
    let messages = parse_message_doc(&doc);
    generate_bindings(&messages, out_path);
}


fn parse_message_doc(doc: &Yaml) -> Vec<Message<'_>> {
    let hash = if let Yaml::Hash(hash) = doc {
        hash
    } else {
        panic!("Invalid document: top level element must be an object");
    };


    let mut messages = Vec::new();
    let mut ids_in_use = HashMap::new();
    let mut names_in_use = HashSet::new();
    for (group_id, items) in hash {
        let mut current_id = if let Yaml::Integer(id) = group_id {
            if *id <= 0 || *id > u8::MAX as i64 {
                panic!("Invalid message group ID: value must be a valid non-zero 8-bit value, got {id:?}");
            }
            *id as u8
        } else {
            panic!("Invalid message group ID: key must be an integer, got {group_id:?}");
        };
        let items = if let Yaml::Hash(hash) = items {
            hash
        } else {
            panic!("Invalid message group: value must an object");
        };

        for (message_name, parameters_decl) in items {
            let message_name = if let Yaml::String(name) = message_name {
                name.as_str()
            } else {
                panic!("Invalid message name: must be a string, got {message_name:?}");
            };
            if let Some(old_name) = ids_in_use.insert(current_id, message_name) {
                panic!("Duplicate message ID {current_id}, used by {old_name} and {message_name}");
            }
            if !names_in_use.insert(message_name) {
                panic!("Duplicate message name: {message_name}");
            }
            let parameters = match parameters_decl {
                Yaml::Null => {
                    Parameters::None
                }
                Yaml::Array(items) => {
                    let items = items.iter().map(parse_type_name).collect();
                    Parameters::Positional(items)
                }
                Yaml::Hash(items) => {
                    // Note: assume YAML is correct and there is no duplicate parameter name
                    let items = items.iter().map(|(name, value)| {
                        let param_name = if let Yaml::String(name) = name {
                            name.as_str()
                        } else {
                            panic!("Invalid parameter name: must be a string, got {name:?}");
                        };
                        let param_type = parse_type_name(value);
                        (param_name, param_type)
                    }).collect();
                    Parameters::Named(items)
                }
                _ => panic!("Invalid messsage declaration: value must be an array or object"),
            };
            messages.push(Message {
                id: current_id,
                name: message_name,
                parameters,
            });
            current_id += 1;
        }
    }

    messages
}

fn parse_type_name(yaml: &Yaml) -> ParamType<'_> {
    match yaml {
        Yaml::String(name) => ParamType::Name(name.as_str()),
        Yaml::Array(items) => ParamType::Choice(items.iter().map(|v| {
            if let Yaml::String(s) = v {
                s.as_str()
            } else {
                panic!("Invalid choice value: must be a string, got {v:?}")
            }
        }).collect()),
        _ => panic!("Invalid parameter type: {yaml:?}"),
    }
}


fn capitalize(s: &str) -> String {
    let mut result = String::with_capacity(s.len());
    let mut upper = true;
    for c in s.chars() {
        if c == '_' {
            upper = true;
        } else if upper {
            result.push(c.to_ascii_uppercase());
            upper = false;
        } else {
            result.push(c);
        }
    }
    result
}


fn generate_bindings<P: AsRef<Path>>(messages: &[Message], path: P) {
    let f = File::create(path).unwrap();
    let mut writer = BufWriter::new(f);

    writeln!(writer, "// This file is generated").unwrap();
    writeln!(writer, "use crate::DecodeError;").unwrap();
    writeln!(writer, "use crate::deserialize::{{Deserialize, Reader}};").unwrap();
    writeln!(writer, "use crate::serialize::{{Serialize, Writer}};").unwrap();
    writeln!(writer, "\n").unwrap();

    // #[derive(Debug)]
    // pub enum Message {
    //     Empty,
    //     SomeValues([u16; 3], params::SomeValuesParam1),
    //     Coordinates {
    //         x: f32,
    //         y: f32,
    //     },
    // }
    writeln!(writer, "#[derive(Debug)]").unwrap();
    writeln!(writer, "pub enum Message {{").unwrap();
    for message in messages {
        match &message.parameters {
            Parameters::None => {
                writeln!(writer, "    {},", message.name).unwrap();
            }
            Parameters::Positional(params) => {
                let values = params.iter().enumerate().map(|(i, typ)| {
                    let suffix = if params.len() == 1 { ParamChoiceSuffix::None } else { ParamChoiceSuffix::Index(i) };
                    format_rust_type(typ, message.name, suffix)
                }).collect::<Vec<_>>().join(", ");
                writeln!(writer, "    {}({}),", message.name, values).unwrap();
            }
            Parameters::Named(params) => {
                writeln!(writer, "    {} {{", message.name).unwrap();
                for (name, typ) in params {
                    let type_name = format_rust_type(typ, message.name, ParamChoiceSuffix::Name(name));
                    writeln!(writer, "        {name}: {type_name},").unwrap();
                }
                writeln!(writer, "    }},").unwrap();
            }
        }
    }
    writeln!(writer, "}}\n").unwrap();

    // #[repr(u8)]
    // pub enum MessageId {
    //     Empty = 20
    //     SomeValues = 21,
    //     Coordinates = 22,
    // }
    writeln!(writer, "#[repr(u8)]").unwrap();
    writeln!(writer, "pub enum MessageId {{").unwrap();
    for message in messages {
        writeln!(writer, "    {} = {},", message.name, message.id).unwrap();
    }
    writeln!(writer, "}}\n").unwrap();

    // impl Message {
    //     pub(crate) fn deserialize_with_id<R: Reader>(id: u8, reader: &mut R) -> Result<Self, DecodeError> {
    //         match id {
    //             20 => Ok(Self::Empty),
    //             21 => Ok(Self::SomeValues(
    //                 <[u16; 3]>::deserialize(reader)?,
    //                 SomeValuesParam1::deserialize(reader)?,
    //             )),
    //             22 => Ok(Self::Coordinates {
    //                 x: f32::deserialize(reader)?,
    //                 y: f32::deserialize(reader)?,
    //             }),
    //             id => Err(DecodeError::UnknownMessage(id)),
    //         }
    //     }
    //
    //     pub fn message_id(&self) -> u8 {
    //         match self {
    //             Self::Empty => 20,
    //             Self::SomeValues => 21,
    //             Self::Coordinates => 22,
    //         }
    //     }
    // }
    writeln!(writer, "impl Message {{").unwrap();
    writeln!(writer, "    pub(crate) fn deserialize_with_id<R: Reader>(id: u8, reader: &mut R) -> Result<Self, DecodeError> {{").unwrap();
    writeln!(writer, "        match id {{").unwrap();
    for message in messages {
        match &message.parameters {
            Parameters::None => {
                writeln!(writer, "            {} => Ok(Self::{}),", message.id, message.name).unwrap();
            }
            Parameters::Positional(params) => {
                writeln!(writer, "            {} => Ok(Self::{}(", message.id, message.name).unwrap();
                for (i, typ) in params.iter().enumerate() {
                    let suffix = if params.len() == 1 { ParamChoiceSuffix::None } else { ParamChoiceSuffix::Index(i) };
                    let type_name = format_rust_type(typ, message.name, suffix);
                    writeln!(writer, "                {}::deserialize(reader)?,", enclosed_rust_type(&type_name)).unwrap();
                }
                writeln!(writer, "            )),").unwrap();
            }
            Parameters::Named(params) => {
                writeln!(writer, "            {} => Ok(Self::{} {{", message.id, message.name).unwrap();
                for (name, typ) in params {
                    let type_name = format_rust_type(typ, message.name, ParamChoiceSuffix::Name(name));
                    writeln!(writer, "                {}: {}::deserialize(reader)?,", name, enclosed_rust_type(&type_name)).unwrap();
                }
                writeln!(writer, "            }}),").unwrap();
            }
        }
    }
    writeln!(writer, "            id => Err(DecodeError::UnknownMessage(id)),").unwrap();
    writeln!(writer, "        }}").unwrap();
    writeln!(writer, "    }}\n").unwrap();
    writeln!(writer, "    pub fn message_id(&self) -> u8 {{").unwrap();
    writeln!(writer, "        match self {{").unwrap();
    for message in messages {
        writeln!(writer, "            {} => {},", destructured_parameters_ignored(message), message.id).unwrap();
    }
    writeln!(writer, "      }}").unwrap();
    writeln!(writer, "    }}").unwrap();
    writeln!(writer, "}}\n").unwrap();

    // impl Serialize for Message {
    //     fn serialized_size(&self) -> usize {
    //         (match self {
    //             Self::Empty => 0,
    //             Self::SomeValues(v0, v1) => {
    //                 v0.serialized_size() +
    //                 v1.serialized_size()
    //             }
    //             Self::Coordinates { x, y } => {
    //                 x.serialized_size() +
    //                 y.serialized_size()
    //             }
    //         }) + 1
    //     }
    //
    //     fn serialize<W: Writer>(&self, writer: &mut W) {
    //         match self {
    //             Self::Empty => {
    //                 20u8.serialize(writer);
    //             },
    //             Self::SomeValues(v0, v1) => {
    //                 21u8.serialize(writer);
    //                 v0.serialize(writer);
    //                 v1.serialize(writer);
    //             }
    //             Self::Coordinates { x, y } => {
    //                 22u8.serialize(writer);
    //                 x.serialize(writer);
    //                 y.serialize(writer);
    //             }
    //         }
    //     }
    // }
    writeln!(writer, "impl Serialize for Message {{").unwrap();
    writeln!(writer, "    fn serialized_size(&self) -> usize {{").unwrap();
    writeln!(writer, "        (match self {{").unwrap();
    for message in messages {
        match &message.parameters {
            Parameters::None => {
                writeln!(writer, "            {} => 0,", destructured_parameters(message)).unwrap();
            }
            Parameters::Positional(params) => {
                writeln!(writer, "            {} => {{", destructured_parameters(message)).unwrap();
                for i in 0..params.len() {
                    writeln!(writer, "                v{i}.serialized_size(){}", if i == params.len() - 1 { "" } else { " +" }).unwrap();
                }
                writeln!(writer, "            }}").unwrap();
            }
            Parameters::Named(params) => {
                writeln!(writer, "            {} => {{", destructured_parameters(message)).unwrap();
                for (i, (name, _)) in params.iter().enumerate() {
                    writeln!(writer, "                {name}.serialized_size(){}", if i == params.len() - 1 { "" } else { " +" }).unwrap();
                }
                writeln!(writer, "            }}").unwrap();
            }
        }
    }
    writeln!(writer, "        }}) + 1").unwrap();
    writeln!(writer, "    }}\n").unwrap();

    writeln!(writer, r#"    fn serialize<W: Writer>(&self, writer: &mut W) {{"#).unwrap();
    writeln!(writer, r#"        match self {{"#).unwrap();
    for message in messages {
        writeln!(writer, "            {} => {{", destructured_parameters(message)).unwrap();
        writeln!(writer, "                {}u8.serialize(writer);", message.id).unwrap();
        match &message.parameters {
            Parameters::None => {}
            Parameters::Positional(params) => {
                for i in 0..params.len() {
                    writeln!(writer, "                v{i}.serialize(writer);").unwrap();
                }
            }
            Parameters::Named(params) => {
                for (name, _) in params {
                    writeln!(writer, "                {name}.serialize(writer);").unwrap();
                }
            }
        }
        writeln!(writer, "            }}").unwrap();
    }
    writeln!(writer, "        }}").unwrap();
    writeln!(writer, "    }}").unwrap();
    writeln!(writer, "}}\n\n").unwrap();

    // Parameter types are gathered in `params` module
    writeln!(writer, "pub mod params {{").unwrap();
    writeln!(writer, "    use super::*;").unwrap();

    // #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    // #[repr(u8)]
    // pub enum MessageParam {
    //     Choice = 0,
    //     Alternative = 1,
    // }
    //
    // impl Deserialize for MessageParam {
    //     fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {
    //         match u8:deserialize(reader)? {
    //             0 => Ok(Self::Choice),
    //             1 => Ok(Self::Alternative),
    //             n => Err(DecodeError::BadChoiceValue(n)),
    //         }
    //     }
    // }
    //
    // impl Serialize for MessageParam {
    //     fn serialized_size(&self) -> usize { core::mem::size_of::<u8>() }
    //     fn serialize<W: Writer>(&self, encoder: &mut W) { (*self as u8).serialize(encoder) }
    // }
    let mut rust_choices = vec![];
    for message in messages {
        match &message.parameters {
            Parameters::None => {},
            Parameters::Positional(params) => {
                for (i, typ) in params.iter().enumerate() {
                    if let ParamType::Choice(choices) = typ {
                        let suffix = if params.len() == 1 { ParamChoiceSuffix::None } else { ParamChoiceSuffix::Index(i) };
                        let type_name = format_rust_choice_type(message.name, suffix);
                        rust_choices.push((type_name, choices));
                    }
                }
            }
            Parameters::Named(params) => {
                for (name, typ) in params {
                    if let ParamType::Choice(choices) = typ {
                        let type_name = format_rust_choice_type(message.name, ParamChoiceSuffix::Name(name));
                        rust_choices.push((type_name, choices));
                    }
                }
            }
        }
    }
    for (name, choices) in &rust_choices {
        writeln!(writer).unwrap();
        writeln!(writer, "    #[derive(Clone, Copy, PartialEq, Eq, Debug)]").unwrap();
        writeln!(writer, "    #[repr(u8)]").unwrap();
        writeln!(writer, "    pub enum {} {{", name).unwrap();
        for (i, choice) in choices.iter().enumerate() {
            writeln!(writer, "        {} = {i},", capitalize(choice)).unwrap();
        }
        writeln!(writer, "    }}\n").unwrap();

        writeln!(writer, "    impl Deserialize for {name} {{").unwrap();
        writeln!(writer, "        fn deserialize<R: Reader>(reader: &mut R) -> Result<Self, DecodeError> {{").unwrap();
        writeln!(writer, "            match u8::deserialize(reader)? {{").unwrap();
        for (i, choice) in choices.iter().enumerate() {
            writeln!(writer, "                {i} => Ok(Self::{}),", capitalize(choice)).unwrap();
        }
        writeln!(writer, "                n => Err(DecodeError::BadChoiceValue(n)),").unwrap();
        writeln!(writer, "            }}").unwrap();
        writeln!(writer, "        }}").unwrap();
        writeln!(writer, "    }}\n").unwrap();

        writeln!(writer, "    impl Serialize for {name} {{").unwrap();
        writeln!(writer, "        fn serialized_size(&self) -> usize {{ core::mem::size_of::<u8>() }}").unwrap();
        writeln!(writer, "        fn serialize<W: Writer>(&self, encoder: &mut W) {{ (*self as u8).serialize(encoder) }}").unwrap();
        writeln!(writer, "    }}").unwrap();
    }

    writeln!(writer, "}}").unwrap();  // End of `params` module
}

enum ParamChoiceSuffix<'a> {
    None,
    Index(usize),
    Name(&'a str),
}

fn format_rust_type(typ: &ParamType<'_>, message_name: &str, suffix: ParamChoiceSuffix) -> String {
    match typ {
        ParamType::Name(s) => s.to_string(),
        ParamType::Choice(_) => format!("params::{}", format_rust_choice_type(message_name, suffix)),
    }
}

fn format_rust_choice_type(message_name: &str, suffix: ParamChoiceSuffix) -> String {
    match suffix {
        ParamChoiceSuffix::None => format!("{message_name}Param"),
        ParamChoiceSuffix::Index(i) => format!("{message_name}Param{i}"),
        ParamChoiceSuffix::Name(s) => format!("{message_name}{}", capitalize(s)),
    }
}

fn enclosed_rust_type(name: &str) -> String {
    if name.chars().next().map(|c| c.is_ascii_alphabetic()).unwrap_or(false) {
        name.into()
    } else {
        format!("<{name}>")
    }
}

fn destructured_parameters(message: &Message<'_>) -> String {
    match &message.parameters {
        Parameters::None => {
            format!("Self::{}", message.name)
        }
        Parameters::Positional(params) => {
            let names = (0..params.len())
                .map(|i| format!("v{i}"))
                .collect::<Vec<_>>()
                .join(", ");
            format!("Self::{}({})", message.name, names)
        }
        Parameters::Named(params) => {
            let names = params.iter()
                .map(|(name, _)| *name)
                .collect::<Vec<_>>()
                .join(", ");
            format!("Self::{} {{ {} }}", message.name, names)
        }
    }
}

fn destructured_parameters_ignored(message: &Message<'_>) -> String {
    match &message.parameters {
        Parameters::None => {
            format!("Self::{}", message.name)
        }
        Parameters::Positional(params) => {
            let names = (0..params.len()).map(|_| "_").collect::<Vec<_>>().join(", ");
            format!("Self::{}({})", message.name, names)
        }
        Parameters::Named(_params) => {
            format!("Self::{} {{ .. }}", message.name)
        }
    }
}

fn load_yaml_doc<P: AsRef<Path>>(path: P) -> Yaml {
    let mut contents = String::new();
    let mut file = File::open(path).unwrap();
    file.read_to_string(&mut contents).unwrap();

    let mut docs = YamlLoader::load_from_str(&contents).unwrap();
    if docs.len() != 1 {
        panic!("Unexpected document count in YAML files, expect 1 document, got {}", docs.len());
    }
    docs.pop().unwrap()
}
