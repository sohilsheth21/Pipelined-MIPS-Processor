`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Design Name: Data Memory 
// Module Name: DataMemory
// Project Name: Pipelined MIPS Proc
//////////////////////////////////////////////////////////////////////////////////

module DataMemory(address, write_data, read_data, mem_write, mem_read,clk);

input [5:0] address;
input [31:0] write_data;
input mem_read, mem_write;
input clk;


output [31:0] read_data;
reg [31:0] read_data;

reg [31:0] data_mem [63:0]; //256 bytes mem capacity

//read at posedge of clk

always @ (posedge clk)
begin

if (mem_read == 1'b1) begin
read_data = data_mem [address];
end
end

always @ (negedge clk)
begin

if (mem_write == 1) begin
data_mem[address] = write_data;
end
end

endmodule
