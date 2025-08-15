# AGENTS.md – Modbus Converter IP Core (Codex Build Guide)

## Overview
This project implements a **parameterizable Modbus Converter IP Core** in Verilog that bridges a Linux host (running OpenPLC runtime on Raspberry Pi) to GPIO I/O and external Modbus RTU/ASCII devices.

The design is modular, synthesis-ready for FPGA/ASIC, and follows a fully synchronous reset style with an APB CSR slave interface.

---

## Agent Roles

### 1. **CSR Agent** (`csr_block.v`)
- Implements AMBA 3 APB-compliant CSR slave
- Handles register access for:
  - REG00: Digital Output Control (%QX)
  - REG01: Digital Input Status (%IX)
  - REG02: Timer/Counter
  - REG03: Modbus message buffer
- Parameterizable register widths
- Ensures synchronous read/write timing

---

### 2. **UART Agents**
#### **UART RX** (`uart_rx.v`)
- Receives Modbus RTU/ASCII frames
- Detects framing boundaries:
  - RTU: 3.5 char silent interval
  - ASCII: colon start, CRLF end
- Performs CRC16/LRC checking

#### **UART TX** (`uart_tx.v`)
- Transmits Modbus response frames
- Handles timing gaps for RTU/ASCII mode
- Works with Modbus Controller for response scheduling

---

### 3. **UART Bridge Agent** (`uart_bridge.v`)
- Interfaces UART RX/TX with Modbus Controller
- Buffers incoming/outgoing frames
- Passes validated frames to Modbus FSM

---

### 4. **GPIO Agents**
#### **GPIO Input** (`gpio_input.v`)
- Reads digital inputs from hardware pins
- Maps to Modbus %IX registers

#### **GPIO Output** (`gpio_output.v`)
- Writes digital outputs from Modbus %QX commands
- Latches outputs on Modbus write function codes (05, 0F, 06, 10)

---

### 5. **Modbus Controller Agent** (`modbus_controller.v`)
- FSM for Modbus protocol handling
- Supports function codes:
  - 01: Read Coils
  - 02: Read Discrete Inputs
  - 03: Read Holding Registers
  - 04: Read Input Registers
  - 05: Write Single Coil
  - 06: Write Single Register
  - 0F: Write Multiple Coils
  - 10: Write Multiple Registers
- Configurable Master/Slave mode
- Interfaces with GPIO and CSR

---

## Simulation Agent
- Self-checking testbench for:
  - RTU and ASCII mode frames
  - Valid CRC16 and LRC verification
  - GPIO loopback
  - Timing checks
- Automatic pass/fail scoreboard

---

## Deliverables
- **RTL:** Verilog `.v` source files
- **Testbench:** Self-checking `.v` files
- **Docs:** PDF with block diagrams, FSM diagrams, register map, timing
- **Constraints:** FPGA synthesis constraints
- **Scripts:** ModelSim/Xcelium simulation scripts

---

## Notes
- All modules are **parameterized** for reuse.
- Only synchronous resets are used.
- APB interface is AMBA 3 compliant.

---

## Coding Style Guidelines (Based on `csr.v` Reference)

All RTL modules in this project should strictly follow the coding conventions used in `csr.v`:

1. **Indentation & Formatting**
   - Use consistent indentation (spaces, matching `csr.v` style).
   - Align parameters, port lists, and signal declarations for readability.

2. **Module Declaration**
   - Parameter list on top, then port list.
   - Parameters use uppercase names with default values.
   - All ports explicitly declared with direction, type, and width.

3. **Naming Conventions**
   - Lowercase with underscores for signals and instances.
   - Uppercase for parameters, constants, and state encodings.
   - `_i` suffix for inputs, `_o` suffix for outputs, `_n` for active-low signals.

4. **Comments**
   - Use block comments for module descriptions.
   - Use inline comments for important lines of code or signal meaning.

5. **Reset Style**
   - Synchronous reset only, active-high by default.
   - All registers initialized in reset block.

6. **Coding Practices**
   - Use `localparam` for state machine encodings.
   - Avoid hard-coded values; use parameters where possible.
   - No inferred latches; all sequential logic inside `always @(posedge clk)`.

This ensures all modules (`uart_rx.v`, `uart_tx.v`, `uart_bridge.v`, `gpio_input.v`, `gpio_output.v`, `modbus_controller.v`) remain consistent, maintainable, and synthesis-friendly.

---

## Simulation and Linting Flow

For this Modbus Converter IP Core project, both **Verilator** and **Icarus Verilog (iverilog)** are used in a complementary workflow:

1. **Run Verilator lint** early to catch width mismatches, unused signals, and synthesis issues before simulation. (`timescale only needed in testbench)
2. **Use iverilog** to run functional simulations, especially for protocol testbenches (RTU/ASCII) and GPIO/UART verification.
3. Maintain a single **file list** (`.f` file or Makefile variable) so both tools compile the exact same RTL set.
4. Optionally run **Verilator simulation** for large-scale or long-run tests where speed is critical, while iverilog is preferred for mixed behavioral testbenches.
5. Always check protocol compliance, frame timing, and GPIO mapping consistency in both simulators.

This hybrid approach ensures rapid feedback, robust verification, and synthesis-friendly RTL.
