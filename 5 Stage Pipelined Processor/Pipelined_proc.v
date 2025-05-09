`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Design Name: Pipelined Processor
// Module Name: PipelinedProc
// Project Name: Pipelined MIPS Proc
//////////////////////////////////////////////////////////////////////////////////

//Defining 2:1 muxes
module mux21(input [31:0]zero , input [31:0]one, input select, output [31:0]out);
	assign out = ((select == 0) ? zero : one);
endmodule

module mux21_5bit(input [4:0]zero , input [4:0]one, input select, output [4:0]out);
	assign out = ((select == 0) ? zero : one);
endmodule


//Defining 3:1 MUX
module mux31(input [31:0]zero , input [31:0]one, input [31:0]two, input [1:0]select, output reg [31:0]out);
	always @(*) begin
		case(select)
			2'b00 : out <= zero;
			2'b01 : out <= one;
			2'b10 : out <= two;
			default: out<=32'bx;
		endcase
	end
endmodule

//Defining 4:1 MUX
module mux41(input [31:0]zero , input [31:0]one, input [31:0]two, input [31:0]three, input [1:0]select, output reg [31:0]out);
	always @(*) begin
		case(select)
			2'b00 : out <= zero;
			2'b01 : out <= one;
			2'b10 : out <= two;
			2'b11 : out <= three;
		endcase
	end
endmodule


//Defining sign-extendeing hardware
module sign_extend(input [15:0]in, input signExtend, output reg [31:0]out);
	always @(*) begin
	   if (signExtend == 0) begin
	       out<={{16{1'b0}},in[15:0]};
	   end
	   else begin
	       out <= {{16{in[15]}},in[15:0]};
	   end
	end
endmodule


module PipelinedProc(input CLK, Reset_L, 
					input [31:0]startPC, 
					output [31:0]dMemOut
					);

//==================================================================REGISTERS AND WIRE DECLARATION=================================================
	//Stage 1 IF_STAGE
	reg [31:0]program_counter; //program_counter
	wire [31:0]interim_counter, next_counter; // program_counter + 4
	wire [31:0] IF_instruction;
	 reg [31:0]IF_ID_PC;
     reg [31:0]IF_ID_Instruction;
	
	
	//Stage 2 ID_STAGE
	wire [31:0] ID_Reg_A, ID_Reg_B, ID_Sign_Extended, jump_address, ID_zero_Extended;
	reg [31:0] ID_EX_Reg_A, ID_EX_Reg_B ;
	reg [1:0]ID_EX_AluOpCtrl_A, ID_EX_AluOpCtrl_B;
	wire [27:0]interim_jump;

	reg [31:0] ID_EX_PC;
(* keep = "true" *)  reg [31:0] ID_EX_Sign_Extended, ID_EX_Instruction;
	
	reg [31:0]ID_EX_shift; wire [31:0]id_shift;
	reg ID_EX_Use_Shmt;
	wire IF_write, PC_write, bubble;
    wire [1:0]addrSel;
		
	wire ID_RegDst, ID_ALUSrc, ID_MemToReg, ID_RegWrite, ID_MemRead, ID_MemWrite, ID_Branch, ID_Jump, ID_SignExtend, ID_UseShmt,ID_UseImmed;
	reg  ID_EX_RegDst, ID_EX_RegWrite, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_SignExtend_control;

(* keep = "true" *) reg ID_EX_MemToReg;
	
	wire [3:0]ID_ALUOp;
	reg  [3:0]ID_EX_ALUOp;
	
	wire ID_DataMemForwardCtrl_EX, ID_DataMemForwardCtrl_MEM;
	reg  ID_EX_DataMemForwardCtrl_EX, ID_EX_DataMemForwardCtrl_MEM;
		
	wire [1:0] ID_AluOpCtrl_A, ID_AluOpCtrl_B;
	
	wire [31:0]ID_Instruction, ID_PC, signExtend_output;
	
	//Stage 3 EXE
	wire EX_RegDst, EX_ALUSrc,  EX_RegWrite, EX_MemRead, EX_MemWrite, EX_Branch, EX_Jump, EX_SignExtend;
	wire [4:0] EX_UseShmt;
	reg  EX_MEM_RegDst, EX_MEM_RegWrite, EX_MEM_MemRead, EX_MEM_MemWrite;
(* keep = "true" *) reg EX_MEM_MemToReg;
	wire EX_MemToReg;
	
	wire  [3:0]EX_ALUOp, EX_ALUCtrl;
	
	wire [31:0] advanced_shift;
	
	wire [31:0] EX_Reg_A, EX_Reg_B, EX_Sign_Extended, EX_Instruction, EX_PC, EX_zero_Extended, EX_sign_input_B;
	
	wire  [1:0] EX_AluOpCtrl_A, EX_AluOpCtrl_B;
	
	wire  EX_DataMemForwardCtrl_EX, EX_DataMemForwardCtrl_MEM;
	reg   EX_MEM_DataMemForwardCtrl_MEM;
	reg  [4:0]EX_MEM_Rw;
	wire [4:0]EX_Rw;
	wire [31:0] EX_Datain, EX_ALU_A, EX_ALU_B, EX_ALU_Output, interim_branch, branch_target;
	wire EX_ALU_Zero;
	reg  [31:0]EX_MEM_ALU_Output, EX_MEM_Datain;
	
	
	//Stage 4 MEM
	wire  MEM_MemToReg, MEM_RegWrite, MEM_MemRead, MEM_MemWrite, MEM_DataMemForwardCtrl_MEM;
	reg   MEM_WB_RegWrite; 
	reg MEM_WB_MemToReg;
	reg [31:0]MEM_WB_Dataout, MEM_WB_ALU_Output;
	
	wire [31:0]MEM_ALU_Output, MEM_Datain, MEM_Dataout, MEM_RAM_WriteData;
	wire [4:0]MEM_Rw;
	reg  [4:0]MEM_WB_Rw;
	
	//Stage 5
	wire WB_RegWrite, WB_MemToReg;
	wire [31:0]WB_Dataout, WB_ALU_Output, WB_Output;
	wire [4:0]WB_Rw;
	
//==============================================================================LOGIC DESCRIPTION=============================================================	
	//*********Stage 1 logic - PC AND Instruction Memory***********
	//pc logic
	always @(negedge CLK, negedge Reset_L) 
						begin//LOGIC FOR PC_write TO BE WRITTEN
	                         if (Reset_L == 0) program_counter <= startPC;
							 else if (!PC_write) program_counter <= program_counter;
                             else program_counter <= next_counter; // update program_counter
						end 
	
	//program_counter adder (Fill the space below)
	assign interim_counter = program_counter + 32'd4;
    
	//instantiating the instruction memory
   	InstructionMemory m1 (.Data(IF_instruction), .Address(program_counter));
	
	//Input MUX to PC
	mux31 m2(.zero(interim_counter) ,.one(jump_address), .two(branch_target), .select(addrSel), .out(next_counter));
	
	//IF_ID Interface flip flops
	always @(negedge CLK or negedge Reset_L) begin
	if(!Reset_L)
	begin
	IF_ID_PC <= 32'b0;
	IF_ID_Instruction <= 32'b0;
	end
	else begin
	if(IF_write==0)
	begin
	IF_ID_Instruction<=IF_ID_Instruction;
	IF_ID_PC <= IF_ID_PC;
	end
	else begin
	IF_ID_Instruction<=IF_instruction;
	IF_ID_PC <=interim_counter;
	end
	end
	end

	//*********Stage 2 logic***********
	
	assign ID_Instruction = IF_ID_Instruction;
	assign ID_PC = IF_ID_PC;
	
	//JUMP address calculation
	assign interim_jump =ID_Instruction[25:0]<<2; 
	assign jump_address = {ID_PC[31:28],interim_jump}; 
		
	//Instantiating registerfile
	registerfile m3(.Ra(ID_Instruction[25:21]),.Rb(ID_Instruction[20:16]),.Rw(WB_Rw),.Bw(WB_Output),.clk(CLK),.Regwr(WB_RegWrite),.Ba(ID_Reg_A),.Bb(ID_Reg_B),.reset(Reset_L));
					   		
	//Instantiating control unit
	SingleCycleControl m4(ID_RegDst, ID_MemToReg, ID_RegWrite, ID_MemRead, ID_MemWrite, ID_Branch, ID_Jump, ID_SignExtend, ID_ALUOp, ID_Instruction[31:26] , ID_UseShmt, ID_Instruction[5:0],bubble, ID_UseImmed);
											
	//Instantiating Forwarding Unit
    	ForwardingUnit m5 (.UseShamt(ID_UseShmt) , .UseImmed(ID_UseImmed) , .ID_Rs 	(ID_Instruction[25:21]) , .ID_Rt(ID_Instruction[20:16]) ,.EX_Rw(EX_Rw) , .MEM_Rw(MEM_Rw), .EX_RegWrite(EX_RegWrite) , .MEM_RegWrite(MEM_RegWrite) , .AluOpCtrlA(ID_AluOpCtrl_A) , .AluOpCtrlB(ID_AluOpCtrl_B) , .DataMemForwardCtrl_EX(ID_DataMemForwardCtrl_EX),.DataMemForwardCtrl_MEM (ID_DataMemForwardCtrl_MEM));

	//Instantiating Sign Extend Hardware
	sign_extend m6 (.in(ID_Instruction[15:0]),.signExtend(ID_SignExtend), .out(signExtend_output));

    	//Instantiate HazardDetection
    	HazardUnit m7(IF_write, PC_write, bubble, addrSel,ID_Jump,ID_Branch,EX_ALU_Zero,EX_MemRead,ID_Instruction[25:21],ID_Instruction[20:16],EX_Instruction[20:16],ID_UseShmt,ID_UseImmed,CLK,Reset_L);
   
   
   	//ID_EX Interface flip flops
    always @(negedge CLK, negedge Reset_L)
    begin
    if(!Reset_L || bubble)begin
   ID_EX_Reg_A <= 32'b0;
   ID_EX_Reg_B <= 32'b0;
   ID_EX_PC <= 32'b0;
   ID_EX_Sign_Extended <= 32'b0;
   ID_EX_SignExtend_control <= 1'b0;
   ID_EX_RegDst <= 1'b0;
   ID_EX_RegWrite <= 1'b0;
   ID_EX_MemRead <= 1'b0;
   ID_EX_MemWrite <= 1'b0;
   ID_EX_MemToReg <= 1'b0;
   ID_EX_ALUOp <= 4'b0;
   ID_EX_DataMemForwardCtrl_EX <=1'b0;
   ID_EX_DataMemForwardCtrl_MEM <= 1'b0;
   ID_EX_AluOpCtrl_A <= 2'b0;
   ID_EX_AluOpCtrl_B <= 2'b0;
   ID_EX_shift <= 32'b0;
   ID_EX_Instruction <= 32'b0;
   ID_EX_Use_Shmt<=1'b0;
    end
    else
    begin
   ID_EX_Reg_A <= ID_Reg_A;
   ID_EX_Reg_B <= ID_Reg_B;
   ID_EX_PC <= ID_PC;
   ID_EX_Sign_Extended <= signExtend_output;
   ID_EX_SignExtend_control <= ID_SignExtend;
   ID_EX_RegDst <= ID_RegDst;
   ID_EX_RegWrite <= ID_RegWrite;
   ID_EX_MemRead <= ID_MemRead;
   ID_EX_MemWrite <= ID_MemWrite;
   ID_EX_MemToReg <= ID_MemToReg;
   ID_EX_ALUOp <= ID_ALUOp;
   ID_EX_DataMemForwardCtrl_EX <=ID_DataMemForwardCtrl_EX;
   ID_EX_DataMemForwardCtrl_MEM <= ID_DataMemForwardCtrl_MEM;
   ID_EX_AluOpCtrl_A <= ID_AluOpCtrl_A;
   ID_EX_AluOpCtrl_B <= ID_AluOpCtrl_B;
   ID_EX_shift <= id_shift;
   ID_EX_Instruction <= ID_Instruction[31:0];
   ID_EX_Use_Shmt<=ID_UseShmt;
    
    end
	end


	//********Stage 3 logic*******
	
	//Rs Rt
	assign EX_Reg_A = ID_EX_Reg_A;
	assign EX_Reg_B = ID_EX_Reg_B;
							
	//SignExtend
	assign EX_SignExtend = ID_EX_SignExtend_control;
	assign EX_Sign_Extended = ID_EX_Sign_Extended;
							
	//Instruction[25:0]
    	assign EX_Instruction = ID_EX_Instruction;
							
	//PC+4 Next Instruction
    	assign EX_PC = ID_EX_PC;
							
	//EX
    	assign EX_RegDst = ID_EX_RegDst;
    	assign EX_RegWrite = ID_EX_RegWrite;
    	assign EX_MemRead = ID_EX_MemRead;
    	assign EX_MemWrite = ID_EX_MemWrite;
    	assign EX_MemToReg = ID_EX_MemToReg;
    
    	assign EX_UseShmt = ID_EX_Use_Shmt;
	//MEM
	assign EX_ALUOp = ID_EX_ALUOp;
							
	//WB
	assign advanced_shift = ID_EX_shift;
							
	//Forwarding Unit 
	assign EX_DataMemForwardCtrl_EX = ID_EX_DataMemForwardCtrl_EX;
	assign EX_DataMemForwardCtrl_MEM = ID_EX_DataMemForwardCtrl_MEM;
	assign EX_AluOpCtrl_A = ID_EX_AluOpCtrl_A;
	assign EX_AluOpCtrl_B = ID_EX_AluOpCtrl_B;
	//Instantiating ALU control

    	ALUControl m9(.ALUCtrl(EX_ALUCtrl),.ALUop(EX_ALUOp),.FuncCode(EX_Instruction[5:0]));
			
	//ALU Source Muxes
	mux41 m10(.zero({27'b0,EX_Instruction[10:6]}) , .one(WB_Output), .two(MEM_ALU_Output), .three(EX_Reg_A), .select(EX_AluOpCtrl_A), .out(EX_ALU_A));
    	mux41 m11(.zero(EX_Sign_Extended) , .one(WB_Output), .two(MEM_ALU_Output), .three(EX_Reg_B), .select(EX_AluOpCtrl_B), .out(EX_ALU_B));

	//Instantiating ALU
		
	ALU m17(.BusA(EX_ALU_A),.BusB(EX_ALU_B),.ALUCtrl(EX_ALUCtrl),.shmt(EX_Instruction[10:6]), .BusW(EX_ALU_Output),.Zero(EX_ALU_Zero));
	
	//calculating branch_address
	assign interim_branch = EX_PC + (EX_Sign_Extended[30:0]<<2);
	assign branch_target = interim_branch;
	
	//mux after alu
	mux21 m12(.zero(EX_Reg_B) ,.one(WB_Output),.select(EX_DataMemForwardCtrl_EX),.out(EX_Datain));
	mux21_5bit m13(.zero(EX_Instruction[20:16]),.one(EX_Instruction[15:11]),.select(EX_RegDst),.out(EX_Rw));
	always @(negedge CLK, negedge Reset_L)
    begin
    if(!Reset_L)begin
    EX_MEM_MemToReg <= 1'b0;
    EX_MEM_DataMemForwardCtrl_MEM <= 1'b0;
    EX_MEM_Rw <= 5'b0;
    EX_MEM_ALU_Output <= 32'b0;
    EX_MEM_Datain <= 32'b0;
    EX_MEM_RegWrite <= 1'b0; 
    EX_MEM_MemRead <= 1'b0;
    EX_MEM_MemWrite <= 1'b0;
    EX_MEM_RegDst <= 1'b0;
    end
    else
    begin
    EX_MEM_MemToReg <= EX_MemToReg;
    EX_MEM_DataMemForwardCtrl_MEM <= EX_DataMemForwardCtrl_MEM;
    EX_MEM_Rw <= EX_Rw;
    EX_MEM_ALU_Output <= EX_ALU_Output;
    EX_MEM_Datain <= EX_Datain;
    EX_MEM_RegWrite <= EX_RegWrite; 
    EX_MEM_MemRead <= EX_MemRead;
    EX_MEM_MemWrite <= EX_MemWrite;
    EX_MEM_RegDst <= EX_RegDst;
    end
    end
       
	
	
	//**********Stage 4 logic*********************


	assign MEM_MemToReg = EX_MEM_MemToReg;
	assign MEM_RegWrite = EX_MEM_RegWrite;
	assign MEM_MemRead = EX_MEM_MemRead;
	assign MEM_MemWrite = EX_MEM_MemWrite;
	assign MEM_DataMemForwardCtrl_MEM = EX_MEM_DataMemForwardCtrl_MEM;
	assign MEM_ALU_Output = EX_MEM_ALU_Output;
	assign MEM_Datain = EX_MEM_Datain;
	assign MEM_Rw = EX_MEM_Rw;

    	mux21 m14(.zero(MEM_Datain) ,.one(WB_Output),.select(MEM_DataMemForwardCtrl_MEM),.out(MEM_RAM_WriteData));

	//Instantiating random access memory
    	DataMemory m15(.address(MEM_ALU_Output[7:2]), .write_data(MEM_RAM_WriteData), .read_data(MEM_Dataout), .mem_write(MEM_MemWrite), .mem_read(MEM_MemRead),.clk(CLK));
    always @(negedge CLK, negedge Reset_L)
    begin
    if(!Reset_L)begin
    MEM_WB_MemToReg <= 1'b0;
    MEM_WB_RegWrite <= 1'b0;
    MEM_WB_Dataout <= 32'b0;
    MEM_WB_ALU_Output <= 32'b0;
    MEM_WB_Rw <= 5'b0;
    end
    else
    begin
    MEM_WB_MemToReg <= MEM_MemToReg;
    MEM_WB_RegWrite <= MEM_RegWrite;
    MEM_WB_Dataout <= MEM_Dataout;
    MEM_WB_ALU_Output <= MEM_ALU_Output;
    MEM_WB_Rw <= MEM_Rw;
    end
    end   

    
	//*********Stage 5 logic****************************


    assign WB_RegWrite = MEM_WB_RegWrite;
    assign WB_MemToReg = MEM_WB_MemToReg;
    assign WB_Dataout = MEM_WB_Dataout;
    assign WB_ALU_Output = MEM_WB_ALU_Output;
    assign WB_Rw = MEM_WB_Rw; 
    
    mux21 m16(.one(WB_Dataout) ,.zero(WB_ALU_Output),.select(WB_MemToReg),.out(WB_Output));
	assign dMemOut = WB_Dataout;
	
endmodule


