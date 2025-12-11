import io
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Self
import yaml


ParamType = (str, Any)
Parameters = tuple[ParamType] | dict[str, ParamType] | None


class Encoder:
    """Encode binary values from data"""

    def __init__(self):
        self.buffer = bytearray()

    def write_type(self, typ: ParamType, data: Any) -> None:
        cat, val = typ
        match cat:
            case "fmt":
                return self.write_pack(val, data)
            case "array":
                for v in data:
                    self.write_type(val[0], v)
            case _:
                raise ValueError("Unexpected type: {typ!r}")

    def write_pack(self, fmt: str, data: Any) -> Any:
        self.buffer += struct.pack(fmt, data)


class Decoder:
    """Decode binary values from data"""

    def __init__(self, data: bytes):
        self.io = io.BytesIO(data)

    def eof(self) -> bool:
        return self.io.tell() >= len(self.io.getbuffer())

    def read_type(self, typ: ParamType) -> Any:
        cat, val = typ
        match cat:
            case "fmt":
                return self.read_unpack(val)
            case "array":
                return [self.read_type(val[0]) for _ in range(val[1])]
            case _:
                raise ValueError("Unexpected type: {typ!r}")

    def read_unpack(self, fmt: str) -> Any:
        data = self.io.read(struct.calcsize(fmt))
        return struct.unpack(fmt, data)[0]


@dataclass
class Message:
    """Message declaration"""
    id: int
    name: str
    params: Parameters

    def __class_getitem__(cls, key: str | int) -> Self:
        """Get a message declaration, by name or ID"""
        if isinstance(key, int):
            return messages[key]
        elif isinstance(key, str):
            return messages_by_name[key]
        else:
            raise KeyError(key)

    @staticmethod
    def decode(data: bytes) -> "Frame":
        """Decode a message frame"""

        if not data:
            raise ValueError("Empty data, no message ID")
        decoder = Decoder(data)
        message_id = decoder.io.read(1)[0]
        message = messages.get(message_id)
        if not message:
            raise KeyError(f"Unknown message ID: {message_id}")
        frame = message.decode_payload(decoder)
        if not decoder.eof():
            raise ValueError(f"Undecoded data after message with ID {message_id}")
        return frame


class MessageEmpty(Message):
    """Message without parameters"""

    def __call__(self) -> "Frame":
        return Frame(self, None)

    def decode_payload(self, decoder: Decoder) -> "Frame":
        return Frame(self, None)


class MessagePositional(Message):
    """Message with positional parameters"""

    def __call__(self, *args) -> "Frame":
        return Frame(self, args)

    def decode_payload(self, decoder: Decoder) -> "Frame":
        return Frame(self, tuple(decoder.read_type(typ) for typ in self.params))


class MessageNamed(Message):
    """Message with named parameters"""
    def __call__(self, **kwargs) -> "Frame":
        return Frame(self, kwargs)

    def decode_payload(self, decoder: Decoder) -> "Frame":
        return Frame(self, {k: decoder.read_type(typ) for k, typ in self.params.items()})


@dataclass
class Frame:
    """Instance of a message, with specific values"""
    message: Message
    # Parameter valeus
    params: Parameters

    def __init__(self, message: Message, params: Parameters):
        if type(params) is not type(message.params):
            raise ValueError("Provided parameters type does not match message declaration")
        match params:
            case tuple(params):
                if len(params) != len(message.params):
                    raise ValueError("Provided parameters count does not match message declaration")
            case dict(params):
                if set(params) != set(message.params):
                    raise ValueError("Provided parameter names does not match message declaration")
        # Note: values are not checked at this stage
        self.message = message
        self.params = params

    def __str__(self) -> str:
        match self.params:
            case None:
                params_str = "-"
            case tuple(params):
                params_str = ", ".join(str(v) for v in params)
            case dict(params):
                params_str = ", ".join(f"{k}={v}" for k, v in params.items())
        return f"<Frame {self.message.name!r} {params_str}>"

    def encode(self) -> bytes:
        encoder = Encoder()
        encoder.buffer.append(self.message.id)
        match self.params:
            case None:
                pass  # Nothing encode
            case tuple(params):
                for typ, v in zip(self.message.params, params, strict=True):
                    encoder.write_type(typ, v)
            case dict(params):
                for name in self.message.params:
                    encoder.write_type(self.message.params[name], params[name])
        return bytes(encoder.buffer)


# Registered messages, indexed by ID
messages: dict[int, Message] = {}
# Registered messages, indexed by name
messages_by_name: dict[str, Message] = {}


def load_messages(path: Path | str) -> list[Message]:
    """Load message declarations from a YAML file"""

    with open(path) as f:
        doc = yaml.safe_load(f)

    if not isinstance(doc, dict):
        raise ValueError("Invalid document: top level element must be an object")

    declarations: dict[int, Message] = {}
    names_in_use: set[str] = set()
    for group_id, group_items in doc.items():
        if not isinstance(group_id, int):
            raise ValueError(f"Invalid message group ID: key must be an integer, got {group_id!r}")
        if group_id <= 0 or group_id > 0xff:
            raise ValueError(f"Invalid message group ID: value must be a valid non-zero 8-bit value, got {group_id!r}")
        if not isinstance(group_items, dict):
            raise ValueError("Invalid message group: value must an object")
        current_id = group_id

        for message_name, parameters_decl in group_items.items():
            if not isinstance(message_name, str):
                raise ValueError("Invalid message name: must be a string, got {message_name!r}")
            if current_id in declarations:
                raise ValueError(f"Duplicate message ID {current_id}, used by {declarations[current_id].name} and {message_name}")
            if message_name in names_in_use:
                raise ValueError(f"Duplicate message name: {message_name}")

            match parameters_decl:
                case None:
                    message = MessageEmpty(current_id, message_name, None)
                case list(items):
                    message = MessagePositional(current_id, message_name, tuple(_parse_type_name(v) for v in items))
                case dict(items):
                    # Note: assume YAML is correct and there is no duplicate parameter name
                    params = {}
                    for k, v in items.items():
                        if not isinstance(k, str):
                            raise ValueError(f"Invalid parameter name: must be a string, got {k!r}")
                        params[k] = _parse_type_name(v)
                    message = MessageNamed(current_id, message_name, params)
                case _:
                    raise ValueError("Invalid messsage declaration: value must be an array or object")
            declarations[current_id] = message
            names_in_use.add(message_name)
            current_id += 1

    return list(declarations.values())


def register_messages(path: Path | str, append: bool = True) -> None:
    """Load and register message declarations from a YAML file"""

    global messages
    global messages_by_name

    declarations = load_messages(path)
    if append is False:
        messages.clear()
        messages_by_name.clear()
    else:
        for decl in declarations:
            if decl.id in messages:
                raise ValueError(f"Message with ID {decl.id} already registered with name {messages[decl.id].name}")
            if decl.name in messages_by_name:
                raise ValueError(f"Message with name {decl.name} already registered with id {messages_by_name[decl.name].id}")
    messages |= {decl.id: decl for decl in declarations}
    messages_by_name |= {decl.name: decl for decl in declarations}


def _parse_type_name(value) -> ParamType:
    if not isinstance(value, str):
        raise ValueError(f"Invalid parameter type: must be a string, got {value!r}")
    match value:
        case "bool":
            return ("fmt", "<?")
        case "u8":
            return ("fmt", "<B")
        case "u16":
            return ("fmt", "<H")
        case "u32":
            return ("fmt", "<L")
        case "f32":
            return ("fmt", "<f")
    if m := re.match(r"^\[(.*); (\d+)\]$", value):
        return ("array", (_parse_type_name(m.group(1)), int(m.group(2))))
    raise ValueError("Unsupported type: {value!r}")

