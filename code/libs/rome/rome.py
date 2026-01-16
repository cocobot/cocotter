import io
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Self, TextIO
import yaml

DEFAULT_MESSAGES_PATH = Path(__file__).parent / "rome_messages.yaml"

ParamType = (str, Any)
Parameters = tuple[ParamType] | dict[str, ParamType] | None
Arguments = tuple[Any] | dict[str, Any] | None


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
            case "choice":
                return self.write_pack("<B", val.index(data))
            case _:
                raise ValueError(f"Unexpected type: {typ!r}")

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
            case "choice":
                return val[self.read_unpack("<B")]
            case _:
                raise ValueError(f"Unexpected type: {typ!r}")

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
    # Parameter values
    args: Arguments

    def __init__(self, message: Message, args: Arguments):
        if type(args) is not type(message.params):
            raise ValueError("Provided arguments type does not match message declaration")
        match args:
            case tuple(args):
                if len(args) != len(message.params):
                    raise ValueError("Provided arguments count does not match message declaration")
            case dict(args):
                if set(args) != set(message.params):
                    raise ValueError("Provided argument names does not match message declaration")
        # Note: values are not checked at this stage
        self.message = message
        self.args = args

    def __str__(self) -> str:
        match self.args:
            case None:
                args_str = "-"
            case tuple(args):
                args_str = ", ".join(str(v) for v in args)
            case dict(args):
                args_str = ", ".join(f"{k}={v}" for k, v in args.items())
        return f"<Frame {self.message.name!r} {args_str}>"

    def encode(self) -> bytes:
        encoder = Encoder()
        encoder.buffer.append(self.message.id)
        match self.args:
            case None:
                pass  # Nothing encode
            case tuple(args):
                for typ, v in zip(self.message.params, args, strict=True):
                    encoder.write_type(typ, v)
            case dict(args):
                for name in self.message.params:
                    encoder.write_type(self.message.params[name], args[name])
        return bytes(encoder.buffer)


# Registered messages, indexed by ID
messages: dict[int, Message] = {}
# Registered messages, indexed by name
messages_by_name: dict[str, Message] = {}


def load_messages(source: Path | str | TextIO) -> list[Message]:
    """Load message declarations from a YAML file or file object"""

    if isinstance(source, (Path, str)):
        with open(source) as f:
            doc = yaml.safe_load(f)
    else:
        doc = yaml.safe_load(source)

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
                raise ValueError(f"Invalid message name: must be a string, got {message_name!r}")
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


def register_messages(declarations: Path | str | list[Message], append: bool = True) -> None:
    """Register message declarations from list, or loaded from a YAML file"""

    global messages
    global messages_by_name

    if not isinstance(declarations, list):
        declarations = load_messages(declarations)
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

def register_default_messages() -> None:
    """Register default messages"""
    return register_messages(DEFAULT_MESSAGES_PATH)


def _parse_type_name(value) -> ParamType:
    if isinstance(value, str):
        match value:
            case "bool":
                return ("fmt", "<?")
            case "u8":
                return ("fmt", "<B")
            case "i8":
                return ("fmt", "<b")
            case "u16":
                return ("fmt", "<H")
            case "i16":
                return ("fmt", "<h")
            case "u32":
                return ("fmt", "<L")
            case "i32":
                return ("fmt", "<l")
            case "f32":
                return ("fmt", "<f")
        if m := re.match(r"^\[(.*); (\d+)\]$", value):
            return ("array", (_parse_type_name(m.group(1)), int(m.group(2))))
    elif isinstance(value, list):
        # Note: nested choices are not supported, but accepted here
        if all(isinstance(v, str) for v in value):
            return ("choice", value)
    raise ValueError(f"Unsupported type: {value!r}")

