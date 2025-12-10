use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::collections::{HashMap, HashSet};
use yaml_rust::{YamlLoader, Yaml};


type ParamType<'a> = &'a str;

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
                        let type_name = parse_type_name(value);
                        (param_name, type_name)
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

fn parse_type_name(yaml: &Yaml) -> &str {
    if let Yaml::String(name) = yaml {
        name.as_str()
    } else {
        panic!("Invalid parameter type: must be a string, got {yaml:?}");
    }
}


fn generate_bindings<P: AsRef<Path>>(messages: &[Message], path: P) {
    let f = File::create(path).unwrap();
    let mut writer = BufWriter::new(f);

    writeln!(writer, "// This file is generated").unwrap();
    writeln!(writer, "use crate::DecodeError;").unwrap();
    writeln!(writer, "use crate::deserialize::{{Deserialize, Reader}};").unwrap();
    writeln!(writer, "use crate::serialize::{{Serialize, Writer}};").unwrap();
    writeln!(writer).unwrap();

    // #[derive(Debug)]
    // pub enum Message {
    //     Empty,
    //     SomeValues([u16; 3]),
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
                let values = params.join(", ");
                writeln!(writer, "    {}({}),", message.name, values).unwrap();
            }
            Parameters::Named(params) => {
                writeln!(writer, "    {} {{", message.name).unwrap();
                for (name, typ) in params {
                    writeln!(writer, "        {name}: {typ},").unwrap();
                }
                writeln!(writer, "    }},").unwrap();
            }
        }
    }
    write!(writer, "}}\n\n").unwrap();

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
    write!(writer, "}}\n\n").unwrap();

    // impl Message {
    //     pub(crate) fn deserialize_with_id<R: Reader>(id: u8, reader: &mut R) -> Result<Self, DecodeError> {
    //         match id {
    //             20 => Ok(Self::Empty),
    //             21 => Ok(Self::SomeValues(
    //                 <[u16; 3]>::deserialize(reader)?,
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
                for typ in params {
                    writeln!(writer, "                {}::deserialize(reader)?,", enclosed_rust_type(typ)).unwrap();
                }
                writeln!(writer, "            )),").unwrap();
            }
            Parameters::Named(params) => {
                writeln!(writer, "            {} => Ok(Self::{} {{", message.id, message.name).unwrap();
                for (name, typ) in params {
                    writeln!(writer, "                {}: {}::deserialize(reader)?,", name, enclosed_rust_type(typ)).unwrap();
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
    //             Self::SomeValues(v0) => {
    //                 v0.serialized_size()
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
    //             Self::SomeValues(v0) => {
    //                 21u8.serialize(writer);
    //                 v0.serialize(writer);
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

    writeln!(writer, "}}\n").unwrap();
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
