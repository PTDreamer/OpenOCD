# OpenOCD Development Guide
## STMicroelectronics Automotive MCU Fork

This document provides a comprehensive guide to understanding the OpenOCD codebase architecture, with particular focus on the FT2232 JTAG interface implementation.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Directory Structure](#directory-structure)
3. [JTAG Subsystem](#jtag-subsystem)
4. [FT2232 Interface Deep Dive](#ft2232-interface-deep-dive)
5. [Target System](#target-system)
6. [Flash Drivers](#flash-drivers)
7. [TCL Configuration System](#tcl-configuration-system)
8. [Adding a New Target Device](#adding-a-new-target-device)
9. [Debugging Tips](#debugging-tips)

---

## Architecture Overview

OpenOCD is structured in layers:

```
┌─────────────────────────────────────────────────────────────────┐
│                        TCL Scripts                               │
│              (interface/*.cfg, target/*.cfg)                     │
├─────────────────────────────────────────────────────────────────┤
│                      Command Layer                               │
│           (TCL command registration and execution)               │
├─────────────────────────────────────────────────────────────────┤
│                       Server Layer                               │
│              (GDB server, Telnet, TCL server)                    │
├─────────────────────────────────────────────────────────────────┤
│                       Target Layer                               │
│      (CPU-specific: ARM, PowerPC, MIPS, RISC-V, etc.)           │
├─────────────────────────────────────────────────────────────────┤
│                     Transport Layer                              │
│                    (JTAG, SWD, SWIM)                            │
├─────────────────────────────────────────────────────────────────┤
│                    JTAG/SWD Core                                 │
│        (TAP management, command queue, state machine)           │
├─────────────────────────────────────────────────────────────────┤
│                   Interface Drivers                              │
│        (FT2232/FTDI, ST-Link, J-Link, etc.)                     │
├─────────────────────────────────────────────────────────────────┤
│                      USB Layer                                   │
│                      (libusb)                                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
openocd_st/
├── src/
│   ├── openocd.c              # Main entry point
│   ├── flash/
│   │   └── nor/
│   │       └── spc56x.c       # SPC56x flash driver
│   ├── jtag/
│   │   ├── adapter.c          # Adapter configuration
│   │   ├── core.c             # JTAG core subsystem
│   │   ├── interface.c        # TAP state machine
│   │   ├── jtag.h             # JTAG API definitions
│   │   └── drivers/
│   │       ├── ftdi.c         # FT2232 high-level driver
│   │       ├── mpsse.c        # MPSSE protocol layer
│   │       └── mpsse.h        # MPSSE API
│   ├── target/
│   │   ├── target.c           # Target management
│   │   ├── powerpc56.c        # PowerPC e200z0 target
│   │   └── ...
│   ├── server/                # GDB/Telnet servers
│   ├── transport/             # JTAG/SWD transport
│   └── helper/                # Utilities
├── tcl/
│   ├── interface/
│   │   └── ftdi/             # FT2232 adapter configs
│   └── target/
│       ├── spc56.cfg         # Common SPC56 procedures
│       ├── spc563m.cfg       # SPC563M target config
│       └── spc564B.cfg       # SPC564B target config
└── doc/
```

---

## JTAG Subsystem

### TAP State Machine

JTAG uses a 16-state state machine controlled by the TMS signal. OpenOCD tracks the current state and computes TMS sequences to transition between states.

**Key states (from `src/jtag/jtag.h`):**

```c
typedef enum tap_state {
    TAP_RESET   = 0x0f,   // Test-Logic-Reset
    TAP_IDLE    = 0x0c,   // Run-Test/Idle
    TAP_DRSHIFT = 0x02,   // Shift-DR (data register)
    TAP_IRSHIFT = 0x0a,   // Shift-IR (instruction register)
    TAP_DRPAUSE = 0x03,   // Pause-DR
    TAP_IRPAUSE = 0xb,    // Pause-IR
    // ... more states
} tap_state_t;
```

### TAP Management (core.c)

The JTAG core manages a linked list of TAPs (Test Access Ports):

```c
// Global TAP list head
static struct jtag_tap *__jtag_all_taps;

// Each TAP has:
struct jtag_tap {
    const char *chip;           // Chip name (e.g., "spc563m")
    const char *tapname;        // TAP name (e.g., "tap")
    int ir_length;              // IR length (e.g., 5 bits)
    uint32_t idcode;            // JTAG IDCODE (e.g., 0x0ae01041)
    uint8_t *cur_instr;         // Current IR value
    bool enabled;               // TAP enabled state
    // ...
};
```

### Command Queue

Operations are queued and executed in batches for efficiency:

```c
// Queue a JTAG scan
int jtag_add_dr_scan(struct jtag_tap *tap, 
                     int num_fields, 
                     const struct scan_field *fields, 
                     tap_state_t state);

// Execute the queue
int jtag_execute_queue(void);
```

---

## FT2232 Interface Deep Dive

The FT2232 family of chips (FT2232C, FT2232H, FT4232H, FT232H) provide USB-to-JTAG/SWD functionality through FTDI's MPSSE (Multi-Protocol Synchronous Serial Engine).

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      ftdi.c (High-Level)                         │
│  - JTAG command execution (scan, reset, statemove)              │
│  - SWD protocol support                                          │
│  - Signal management (SRST, TRST, LED)                          │
├─────────────────────────────────────────────────────────────────┤
│                      mpsse.c (Protocol Layer)                    │
│  - MPSSE command encoding                                        │
│  - USB bulk transfers via libusb                                 │
│  - Buffered read/write with flush                               │
├─────────────────────────────────────────────────────────────────┤
│                        libusb                                    │
│  - USB device enumeration                                        │
│  - Bulk data transfer                                            │
│  - Control transfers (reset, bitmode)                           │
└─────────────────────────────────────────────────────────────────┘
```

### MPSSE Mode

The MPSSE engine allows the FT chip to act as a synchronous serial interface. OpenOCD configures it for JTAG:

```c
// From mpsse.c - Enable MPSSE mode
err = libusb_control_transfer(ctx->usb_dev,
    FTDI_DEVICE_OUT_REQTYPE,
    SIO_SET_BITMODE_REQUEST,
    0x0b | (BITMODE_MPSSE << 8),  // Enable MPSSE
    ctx->index,
    NULL, 0, ctx->usb_write_timeout);
```

### JTAG Mode Configuration

```c
// From ftdi.c - JTAG mode settings
#define JTAG_MODE (LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT)

// Bits meanings:
// LSB_FIRST (0x08): Shift data LSB first
// POS_EDGE_IN (0x04): Sample input on positive clock edge  
// NEG_EDGE_OUT (0x01): Output data on negative clock edge
```

### Key FT2232 Functions

#### 1. Initialization (`ftdi_initialize`)

```c
static int ftdi_initialize(void)
{
    // Open the MPSSE context
    mpsse_ctx = mpsse_open(ftdi_vid, ftdi_pid, 
                           ftdi_device_desc,
                           adapter_get_required_serial(), 
                           adapter_usb_get_location(), 
                           ftdi_channel);
    
    // Set initial GPIO state
    mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
    mpsse_set_data_bits_high_byte(mpsse_ctx, output >> 8, direction >> 8);
    
    // Configure clock frequency
    freq = mpsse_set_frequency(mpsse_ctx, adapter_get_speed_khz() * 1000);
    
    return mpsse_flush(mpsse_ctx);
}
```

#### 2. Command Execution (`ftdi_execute_queue`)

```c
static int ftdi_execute_queue(void)
{
    // Iterate through JTAG command queue
    for (struct jtag_command *cmd = jtag_command_queue; cmd; cmd = cmd->next) {
        ftdi_execute_command(cmd);  // Encode each command
    }
    
    // Flush all commands to USB
    return mpsse_flush(mpsse_ctx);
}
```

#### 3. Data Scanning (`ftdi_execute_scan`)

```c
static void ftdi_execute_scan(struct jtag_command *cmd)
{
    // Move to appropriate shift state (DR or IR)
    move_to_state(type == SCAN_OUT ? TAP_DRSHIFT : TAP_IRSHIFT);
    
    // For each field in the scan
    for (int i = 0; i < cmd->cmd.scan->num_fields; i++) {
        struct scan_field *field = &cmd->cmd.scan->fields[i];
        
        // Clock data out (TDI) and/or in (TDO)
        mpsse_clock_data(mpsse_ctx,
                        field->out_value, 0,    // Data to send
                        field->in_value, 0,     // Buffer for received data
                        field->num_bits,        // Number of bits
                        JTAG_MODE);
    }
}
```

#### 4. State Transitions (`move_to_state`)

```c
static void move_to_state(tap_state_t goal_state)
{
    tap_state_t start_state = tap_get_state();
    
    // Get TMS bit sequence to transition
    uint8_t tms_bits;
    int tms_count = tap_get_tms_path(start_state, goal_state);
    tms_bits = tap_get_tms_path_bits(start_state, goal_state);
    
    // Clock out TMS sequence
    mpsse_clock_tms_cs_out(mpsse_ctx, &tms_bits, 0, 
                           tms_count, false, JTAG_MODE);
    
    tap_set_state(goal_state);
}
```

### MPSSE Protocol Details

The MPSSE uses specific command bytes to control operations:

| Command | Opcode | Description |
|---------|--------|-------------|
| Clock Data Out (LSB) | 0x19 | Clock out bytes, TDI, LSB first |
| Clock Data In (LSB) | 0x29 | Clock in bytes, TDO, LSB first |
| Clock Data In/Out (LSB) | 0x39 | Clock in and out bytes simultaneously |
| Clock TMS | 0x4B | Clock TMS bits for state transitions |
| Set Low Byte | 0x80 | Set ADBUS[7:0] direction and value |
| Set High Byte | 0x82 | Set ACBUS[7:0] direction and value |
| Read Low Byte | 0x81 | Read ADBUS[7:0] |
| Read High Byte | 0x83 | Read ACBUS[7:0] |
| Set Divisor | 0x86 | Set clock divisor |

#### Example: Clock Data Out

```c
void mpsse_clock_data_out(struct mpsse_ctx *ctx, 
                          const uint8_t *out, 
                          unsigned out_offset,
                          unsigned length, 
                          uint8_t mode)
{
    // For byte transfers
    buffer_write_byte(ctx, mode | 0x10);           // Command: clock out
    buffer_write_byte(ctx, (length - 1) & 0xff);   // Length low byte
    buffer_write_byte(ctx, (length - 1) >> 8);     // Length high byte
    buffer_write(ctx, out, out_offset, length * 8); // Data
}
```

### Signal Management

The FT2232 GPIO pins are mapped to JTAG signals:

```c
// From ftdi.c
struct signal {
    const char *name;      // Signal name (e.g., "nSRST")
    uint16_t data_mask;    // GPIO bit mask for output
    uint16_t input_mask;   // GPIO bit mask for input
    uint16_t oe_mask;      // Output enable mask
    bool invert_data;      // Invert logic level
    // ...
};

static void ftdi_set_signal(const struct signal *s, char value)
{
    switch (value) {
        case '0': output &= ~s->data_mask; break;
        case '1': output |= s->data_mask; break;
        case 'z': direction &= ~s->data_mask; break;
    }
    mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
}
```

### USB Communication

MPSSE uses bulk transfers for efficiency:

```c
// From mpsse.c
static int mpsse_flush(struct mpsse_ctx *ctx)
{
    // Send all buffered commands
    int transferred;
    libusb_bulk_transfer(ctx->usb_dev, ctx->out_ep,
                         ctx->write_buffer, ctx->write_count,
                         &transferred, ctx->usb_write_timeout);
    
    // Read responses
    while (ctx->read_count > 0) {
        libusb_bulk_transfer(ctx->usb_dev, ctx->in_ep,
                             ctx->read_chunk, ctx->read_chunk_size,
                             &transferred, ctx->usb_read_timeout);
        // Process received data...
    }
}
```

### FT2232 Chip Types

```c
// From mpsse.c - Chip detection based on USB descriptor
switch (desc.bcdDevice) {
    case 0x500: ctx->type = TYPE_FT2232C; break;  // 6 MHz base clock
    case 0x700: ctx->type = TYPE_FT2232H; break;  // 30 MHz base clock (High-speed)
    case 0x800: ctx->type = TYPE_FT4232H; break;  // 30 MHz, 4-channel
    case 0x900: ctx->type = TYPE_FT232H; break;   // 30 MHz, single channel
}
```

---

## Target System

### Target Types

OpenOCD supports various CPU architectures via target types:

```c
// From target.c
static struct target_type *target_types[] = {
    &arm7tdmi_target,
    &cortexm_target,
    &cortexa_target,
    &aarch64_target,
    &powerpc56_target,  // Used for SPC56x
    // ... more
};
```

### PowerPC56 Target (for SPC56x)

The SPC56x family uses the e200z0 PowerPC core with OnCE (On-Chip Emulation) debug:

```c
// From powerpc56.c
struct target_type powerpc56_target = {
    .name = "powerpc56",
    .poll = powerpc56_poll,
    .halt = powerpc56_halt,
    .resume = powerpc56_resume,
    .step = powerpc56_step,
    .read_memory = powerpc56_read_memory,
    .write_memory = powerpc56_write_memory,
    // ...
};
```

### OnCE Debug Registers

```c
// OnCE register addresses
#define E200Zxx_ONCE_CPUSCR  0x10  // CPU Scan Register
#define E200Zxx_ONCE_OCR     0x12  // OnCE Control Register
#define E200Zxx_ONCE_IAC1    0x20  // Instruction Address Compare 1
#define E200Zxx_ONCE_DAC1    0x24  // Data Address Compare 1
#define E200Zxx_ONCE_DBSR    0x30  // Debug Status Register
#define E200Zxx_ONCE_DBCR0   0x31  // Debug Control Register 0
```

---

## Flash Drivers

### SPC56x Flash Driver

The `spc56x.c` flash driver is generic and auto-detects flash geometry:

```c
// From flash/nor/spc56x.c
static int spc56x_probe(struct flash_bank *bank)
{
    // Read MIDR to identify chip
    target_read_u32(target, MIDR_ADDR, &midr);
    
    // Auto-configure based on detected variant
    // The driver handles all SPC56x variants automatically
}

static int spc56x_erase(struct flash_bank *bank, unsigned first, unsigned last)
{
    // Use PowerPC assembly loader for flash operations
}
```

---

## TCL Configuration System

### Interface Configuration

Example FT2232 interface config (`tcl/interface/ftdi/olimex-arm-usb-ocd.cfg`):

```tcl
adapter driver ftdi
ftdi device_desc "Olimex OpenOCD JTAG"
ftdi vid_pid 0x15ba 0x0003

# GPIO initialization: value and direction
ftdi layout_init 0x0c08 0x0f1b

# Signal definitions
ftdi layout_signal nSRST -oe 0x0200
ftdi layout_signal nTRST -data 0x0100 -noe 0x0400
ftdi layout_signal LED -data 0x0800
```

### Target Configuration

Example SPC563M config (`tcl/target/spc563m.cfg`):

```tcl
# Include common SPC56 procedures
source [find target/spc56.cfg]

set _CHIPNAME   spc563m
set _TAPID      0x0ae01041

# Create JTAG TAP
jtag newtap $_CHIPNAME tap \
    -irlen 5 \
    -ircapture 0x1 \
    -irmask 0x1f \
    -expected-id $_TAPID

# Create target
target create $_CHIPNAME.cpu powerpc56 \
    -endian big \
    -chain-position $_CHIPNAME.tap

# Flash bank
flash bank $_CHIPNAME.0.flash spc56x 0x0 0xF00000 0 0 $_CHIPNAME.cpu
```

---

## Adding a New Target Device

### Step 1: Create Target Configuration

Create `tcl/target/your_chip.cfg`:

```tcl
source [find target/spc56.cfg]

set _CHIPNAME   your_chip
set _TAPID      0x????????  # Get from datasheet (JTAG IDCODE)

jtag newtap $_CHIPNAME tap \
    -irlen 5 \
    -expected-id $_TAPID

target create $_CHIPNAME.cpu powerpc56 \
    -endian big \
    -chain-position $_CHIPNAME.tap

# Configure flash (size from datasheet)
flash bank $_CHIPNAME.0.flash spc56x 0x0 SIZE 0 0 $_CHIPNAME.cpu
```

### Step 2: Key Values to Configure

| Parameter | Where to Find |
|-----------|---------------|
| TAPID | Datasheet "Device Identification" or "JTAG" section |
| Flash Size | Datasheet "Memory Map" section |
| RAM Size | Datasheet "Memory Map" section |
| Work Area Address | RAM base address |
| Endianness | Typically "big" for PowerPC |

### Step 3: Test the Configuration

```bash
openocd -f interface/ftdi/your_adapter.cfg -f target/your_chip.cfg
```

---

## Debugging Tips

### Enable Debug Output

```bash
# Maximum verbosity
openocd -d3 -f interface.cfg -f target.cfg

# JTAG I/O tracing
openocd -d4 -f interface.cfg -f target.cfg
```

### Common TCL Debug Commands

```tcl
# Scan IR
irscan spc563m.tap 0x02

# Scan DR
drscan spc563m.tap 32 0x0

# Read TAPID
jtag cget spc563m.tap -idcode

# Manual state transitions
pathmove RESET IDLE

# Check TAP state
jtag tapisenabled spc563m.tap
```

### GDB Connection

```bash
# Connect GDB
arm-none-eabi-gdb -ex "target remote :3333" your_binary.elf
```

---

## Building OpenOCD

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt install build-essential libtool pkg-config \
                 libusb-1.0-0-dev libftdi1-dev

# Build
./bootstrap
./configure --enable-ftdi
make
sudo make install

# Run (with your interface and target)
openocd -f tcl/interface/ftdi/olimex-arm-usb-ocd.cfg \
        -f tcl/target/spc563m.cfg
```

---

## Quick Reference

### FT2232 Pin Mapping (Typical)

| MPSSE Pin | JTAG Signal |
|-----------|-------------|
| ADBUS0 | TCK |
| ADBUS1 | TDI |
| ADBUS2 | TDO |
| ADBUS3 | TMS |
| ADBUS4-7 | GPIO (nTRST, nSRST, etc.) |
| ACBUS0-7 | Additional GPIO |

### Key Source Files

| File | Purpose |
|------|---------|
| `src/jtag/drivers/ftdi.c` | FT2232 JTAG/SWD driver |
| `src/jtag/drivers/mpsse.c` | MPSSE USB protocol |
| `src/jtag/core.c` | JTAG TAP management |
| `src/jtag/interface.c` | TAP state machine |
| `src/target/powerpc56.c` | SPC56x target support |
| `src/flash/nor/spc56x.c` | SPC56x flash driver |

---

## Appendix: Data Flow Example

**Reading a memory address via JTAG:**

```
User: "mdw 0x40000000"
         │
         ▼
   ┌───────────────┐
   │  TCL Command  │ (server/tcl_server.c)
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │ Target Layer  │ powerpc56_read_memory()
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │ OnCE/Nexus    │ Read via debug registers
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │  JTAG Core    │ jtag_add_dr_scan()
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │  FTDI Driver  │ ftdi_execute_scan()
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │    MPSSE      │ mpsse_clock_data()
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │    libusb     │ USB bulk transfer
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │   FT2232 HW   │ TCK/TDI/TDO/TMS signals
   └───────────────┘
         │
         ▼
   ┌───────────────┐
   │  Target MCU   │ JTAG TAP controller
   └───────────────┘
```

---

*Document generated for OpenOCD ST-Automotive fork*
*Last updated: 2024*
