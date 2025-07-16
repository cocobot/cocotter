# TCA6408 GPIO Expander Driver

A `no_std` Rust driver for the TCA6408 I2C GPIO expander.

## Features

- 8-bit GPIO expander with I2C interface
- Configure pins as inputs or outputs
- Read input pin states
- Set output pin states
- Polarity inversion support
- Individual pin control
- Bitfield operations for efficient pin manipulation

## Usage

```rust
use tca6408::{TCA6408, GpioPins};

// Create a new TCA6408 instance
let mut gpio_expander = TCA6408::new(i2c, 0x20);

// Initialize with default configuration
gpio_expander.init().unwrap();

// Configure pin 0 as output, others as inputs
gpio_expander.set_pin_mode(0, false).unwrap();

// Set pin 0 high
gpio_expander.set_output_pin(0, true).unwrap();

// Read input pin 1
let pin1_state = gpio_expander.read_input_pin(1).unwrap();

// Read all inputs at once
let inputs = gpio_expander.read_inputs().unwrap();
if inputs.get_pin(2) {
    // Pin 2 is high
}

// Set multiple outputs using bitfield
let mut outputs = GpioPins::new();
outputs.set_p0(true);  // Pin 0 high
outputs.set_p1(false); // Pin 1 low
outputs.set_p7(true);  // Pin 7 high
gpio_expander.write_outputs(outputs).unwrap();
```

## Register Map

| Register | Address | Description |
|----------|---------|-------------|
| Input | 0x00 | Read GPIO pin states |
| Output | 0x01 | Set GPIO pin states for outputs |
| Polarity Inversion | 0x02 | Invert input polarity |
| Configuration | 0x03 | Configure pins as inputs (1) or outputs (0) |

## Default Configuration

After `init()`:
- All pins configured as inputs
- Normal polarity (non-inverted)
- All outputs set to low

## I2C Address

The TCA6408 supports addresses from 0x20 to 0x27 depending on the A0, A1, A2 pins:

| A2 | A1 | A0 | Address |
|----|----|----|---------|
| 0  | 0  | 0  | 0x20    |
| 0  | 0  | 1  | 0x21    |
| 0  | 1  | 0  | 0x22    |
| 0  | 1  | 1  | 0x23    |
| 1  | 0  | 0  | 0x24    |
| 1  | 0  | 1  | 0x25    |
| 1  | 1  | 0  | 0x26    |
| 1  | 1  | 1  | 0x27    |