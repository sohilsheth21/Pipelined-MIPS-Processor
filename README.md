# Pipelined-MIPS-Processor

This repository contains the RTL implementation of a 32-bit MIPS processor written in Verilog as part of the ECEN651 course project at Texas A&M. The design is the classic 5-stage pipeline model and includes mechanisms to handle control and data hazards, enabling efficient instruction throughput.

---

## 🔧 Project Overview

- **Architecture**: 5-stage pipelined MIPS processor  
- **Language**: Verilog HDL  
- **Design Tool**: Xilinx Vivado (for synthesis and waveform tracing)

---

## Features

- **Five Pipeline Stages**
  - **IF (Instruction Fetch)** – Fetch instruction from instruction memory
  - **ID (Instruction Decode)** – Decode and read register operands
  - **EX (Execute)** – ALU computation and effective address calculation
  - **MEM (Memory Access)** – Load/store to data memory
  - **WB (Write Back)** – Result written back to register file

- **Data Forwarding Unit**
  - Handles RAW hazards by forwarding from MEM and WB stages to EX stage

- **Hazard Detection Unit**
  - Stalls pipeline for load-use hazards
  - Inserts noops for control flow instructions (branches, jumps)

- **Separate Instruction and Data Memory**

- **Modular Components**
  - Program Counter (PC)
  - Register File
  - ALU and ALU Control
  - Pipeline registers between stages
  - Main Controller

## 📌 Notes

- First implemented the single cycle processor and then moved to the pipelined processor design.
- All modules were simulated and tested individually before integration.
- The processor correctly handles data/control hazards and forwarding logic.
---



