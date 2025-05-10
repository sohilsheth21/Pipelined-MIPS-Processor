`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Design Name: Hazard Unit
// Module Name: HazardUnit
// Project Name: Pipelined MIPS Proc
//////////////////////////////////////////////////////////////////////////////////

module HazardUnit(	IF_write,
				    PC_write,
					bubble,
					addrSel,
					Jump,
					Branch,
					ALUZero,
					memReadEX,
					currRs,
					currRt,
					prevRt,
					UseShamt,
					UseImmed,
					Clk,
					Rst);

	// Outputs
	output reg IF_write, PC_write, bubble;
	output reg [1:0] addrSel;
	
	// Inputs
	input Jump, Branch, ALUZero, memReadEX, Clk, Rst;
	input UseShamt, UseImmed;
	input [4:0] currRs;
	input [4:0] currRt;
	input [4:0] prevRt;
	
	// State definition for FSM
	parameter NoHazard_state = 3'b000;	// No hazard - Normal state
	parameter Jump_state = 3'b001;		// Jump
	parameter Branch_0_state = 3'b010;	// Branch Encountered
	parameter Branch_1_state = 3'b011;	// Branch Taken
	
	// Internal signal - LdHazard - HIGH when hazard exists
	reg LdHazard;
	
	// Internal states - FSM_state, FSM_nxt_state
	reg [2:0] FSM_state;
	reg [2:0] FSM_nxt_state;
	
	
	// FSM state register
	always @(negedge Clk) begin
		if( Rst == 0 )
			FSM_state <= 0;
		else
			FSM_state <= FSM_nxt_state;
	end
	
	// Load Hazard Flowchart logic
    always @(*)begin
      if(prevRt==0)  
        LdHazard <= 0;  //no hazard when prev RT is 0
      else if(memReadEX==1)
        case({UseShamt,UseImmed})
            2'b00:
                if((prevRt==currRs) || (prevRt==currRt)) 
                    LdHazard <= 1;   //current Rt or Rs is same as prev Rt
                else
                    LdHazard <= 0;  

            2'b10:
                if(prevRt==currRs)
                    LdHazard <= 1;   //current Rs for shamt is same as prev Rt
                else
                    LdHazard <= 0;

            2'b01:
                if(prevRt==currRs)
                    LdHazard <= 1;   //current Rs for Immediate is same as prev Rt
                else
                    LdHazard <= 0;

            default:
                LdHazard <= 0;
        endcase
    end

	// FSM next state and output logic
	always @(*) begin	// Combinatory logic	
		case( FSM_state )
			
			NoHazard_state: begin	// Prioritize jump
				
				if( Jump == 1'b1 ) begin
					// Unconditional return to no hazard state
					IF_write = 1'b0;
					PC_write = 1'b1;
					bubble = 1'b0;
					addrSel = 2'b01;
					FSM_nxt_state = Jump_state;
				end
				
				else if( LdHazard == 1'b1 ) begin
					// If hazard is detected, stop fetching new IF and stop incrementing PC and writing into PC 
					IF_write = 1'b0;
					PC_write = 1'b0;
					bubble = 1'b1;
					addrSel = 2'b00;
					FSM_nxt_state = NoHazard_state;
				end
				
				else if (Branch == 1'b1)begin
					// Go to branch zero and check ALU zero flag
					IF_write = 1'b0;
					PC_write = 1'b0;
					bubble = 1'b0;
					addrSel = 2'b00;
					FSM_nxt_state = Branch_0_state;
				end
				
				else begin
					// Normal state
					IF_write = 1'b1;
					PC_write = 1'b1;
					bubble = 1'b0;
					addrSel = 2'b00;
					FSM_nxt_state = NoHazard_state;
				end
			end
				
			Jump_state: begin
				// Unconditional return to no hazard state
				// Stop execution until jump is resolved completely
				IF_write = 1'b1;
				PC_write = 1'b1;
				bubble = 1'b1;
				addrSel = 2'b00;
				FSM_nxt_state = NoHazard_state;
			end
			
			Branch_0_state: begin
				if( ALUZero == 1'b0 ) begin
					IF_write = 1'b1;
					PC_write = 1'b1;
					bubble = 1'b1;
					addrSel = 2'b00;
					FSM_nxt_state = NoHazard_state;
				end
				else if( ALUZero == 1'b1 ) begin
					IF_write = 1'b0;
					PC_write = 1'b1;
					bubble = 1'b1;
					addrSel = 2'b10;
					FSM_nxt_state = Branch_1_state;
				end
				
			end
			
			Branch_1_state: begin
				// Unconditional return to no hazard state
				IF_write = 1'b1;
				PC_write = 1'b1;
				bubble = 1'b1;
				addrSel = 2'b00;
				FSM_nxt_state = NoHazard_state;
			end
			
			default: begin
				FSM_nxt_state = NoHazard_state;
				PC_write = 1'bx;
				IF_write = 1'bx;
				bubble  = 1'bx;
				addrSel = 2'bxx;
			end
		endcase
	end
endmodule