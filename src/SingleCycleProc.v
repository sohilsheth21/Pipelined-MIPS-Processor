`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Design Name: Single Cycle Processor
// Module Name: SingleCycleProc
// Project Name: Pipelined MIPS processor
//////////////////////////////////////////////////////////////////////////////////
module SingleCycleProc(CLK, Reset_L, startPC, dMemOut);
input CLK, Reset_L;
input [31:0] startPC;
output wire [31:0] dMemOut;

reg [31:0] PC;
wire [4:0] mux1;
wire [31:0] add1;
wire [31:0] op1;
wire[31:0] mux2;
wire[31:0]mux3;
wire[31:0]mux4; 
wire[31:0]mux5;
wire[31:0]sign_extend;
wire [27:0] left_shift;

always @(negedge CLK) begin
if (Reset_L ==0)
PC=startPC;
else
PC = mux5;
end

//instruction cycle
wire [31:0] instruction_out;

wire RegDst,ALU_Src,MemToReg,RegWrite,MemRead,MemWrite,Branch,Jump,SignExtend; //SingleCycleControl
wire [3:0] ALU_Op;

//register file
wire [31:0] Bus_A;
wire [31:0] Bus_B;

//ALU
wire [31:0]ALU_res; 
wire ZERO;

//alu control
wire [3:0] ALU_control; 

//memory
wire [31:0] data_mem_out;

//***INSTANTIATING MODULES***

InstructionMemory M1(.Data(instruction_out),.Address(PC)); //instantite intruction memory
SingleCycleControl M2(.RegDst(RegDst), .ALUSrc(ALU_Src), .MemToReg(MemToReg), .RegWrite(RegWrite), .MemRead(MemRead),.MemWrite(MemWrite), .Branch(Branch), .Jump(Jump), .SignExtend(SignExtend),.ALUOp(ALU_Op), .Opcode(instruction_out[31:26]));
RegisterFile M3(.RA(instruction_out[25:21]),.RB(instruction_out[20:16]), .RW(mux1),.BusW(mux3),.RegWr(RegWrite),.Clk(CLK),.BusA(Bus_A),.BusB(Bus_B));//instantiate register file
ALUControl M5(.ALUCtrl(ALU_control),.ALUop(ALU_Op), .FuncCode(instruction_out[5:0]));//instantiate ALU control
ALU M4(.BusW(ALU_res),.Zero(ZERO),.BusA(Bus_A), .BusB(mux2),.ALUCtrl(ALU_control));//instantiate ALU
DataMemory M6(.Address(ALU_res[5:0]),.WriteData(Bus_B),.MemoryRead(MemRead),.MemoryWrite(MemWrite),.Clock(CLK),.ReadData(data_mem_out)); //instantiate memory module

assign mux1 = RegDst ? instruction_out[15:11] : instruction_out[20:16] ; 	//MUX1 to register
assign mux2 = ALU_Src ? sign_extend:Bus_B; 					//MUX2
assign mux3 = MemToReg ? dMemOut : ALU_res; 					//MUX3
assign sign_extend = SignExtend ? {{16{instruction_out[15]}},instruction_out[15:0]} :  {{16'b0},instruction_out[15:0]}; //sign extend block
assign op1 = Branch & ZERO; 							//AND Gate;
assign add1 = PC + 4'b0100; 							//PC+4[31-28]
assign left_shift = instruction_out[25:0]<<2;
assign jump_addr = {add1[31:28],left_shift}; 					//jump address[31-0]
assign ALU_add_res = add1 + sign_extend<<2;
assign mux4 = op1 ? ALU_add_res : add1; 					//MUX4
assign mux5 = Jump ? jump_addr : mux4; 						//MUX5

assign dMemOut = data_mem_out;
endmodule



