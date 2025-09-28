# Build Setup Instructions

## Prerequisites

### Fedora/Linux Dependencies

Install the following system dependencies first:

```bash
# Essential build tools
sudo dnf install perl gcc gcc-c++ openssl-devel

# Multi-lib support for cross-compilation
sudo dnf install gcc-multilib
```

### Rust Installation

If Rust is not already installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

## ESP32 Development Environment Setup

### 1. Install espup

```bash
cargo install espup
```

### 2. Install ESP toolchain

```bash
espup install
```

### 3. Set ESP as default Rust toolchain

```bash
rustup default esp
```

## Build Commands

Once the environment is set up, you can build and flash the robots:

### For PAMI robot

```bash
# Build, flash and monitor on real hardware
cargo rpami

# Run simulator with robot ID 0
timeout 10 cargo spami -- 0
```

### For Galipeur robot

```bash
# Build, flash and monitor on real hardware
cargo rgalipeur

# Run simulator
timeout 10 cargo sgalipeur
```

### Additional Commands

```bash
# Monitor serial output only
cargo monitor

# Clean build artifacts
cargo clean
```

## Troubleshooting

- **openssl-sys errors**: Make sure `openssl-devel` is installed
- **Perl not found**: Install `perl` package
- **gcc multilib errors**: Install `gcc-multilib` or equivalent for your distro
- **ESP toolchain issues**: Re-run `espup install` and ensure `~/.cargo/env` is sourced

## Notes

- The simulator never exits on its own, always use `timeout` when running simulated robots
- Default serial port is `/dev/ttyACM0` (configurable in `.cargo/config.toml`)
- Cargo aliases (rpami, rgalipeur, spami, sgalipeur) are defined in `.cargo/config.toml`
- For detailed project structure and development workflow, see `CLAUDE.md`