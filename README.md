# Pipelined-MIPS-Processor
Repository Structure:
├── src/                # Verilog source files
│   ├── alu.v
│   ├── reg_file.v
│   ├── data_mem.v
│   ├── alu_control.v
│   ├── forwarding_unit.v
│   ├── hazard_unit.v
│   └── ...
├── testbench/          # Testbenches for unit and integration testing
│   ├── single_cycle_tb.v
│   └── pipelined_tb.v
├── README.md

This processor implements the standard 5-stage MIPS pipeline:

IF (Instruction Fetch) – Fetches the instruction from memory.
ID (Instruction Decode) – Decodes instruction, reads register file.
EX (Execute) – Performs ALU operations, branch target calculation.
MEM (Memory Access) – Accesses data memory (for loads/stores).
WB (Write Back) – Writes result back to the register file.

Features Implemented:
Single-cycle baseline MIPS processor
Register File, ALU, Data Memory modules
ALU Control logic 
Control Unit
Fully pipelined datapath with:
- Forwarding Unit to handle data hazards
- Hazard Detection Unit to stall on load-use dependencies
Instruction and data memory separated for testing


