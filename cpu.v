timescale 1ns / 1ps

// TOP MODULE 
module top(PCOut, CurrentInstr, AluInA, AluInB, AluResult, RegFileWriteData, zero, clk);
input clk;
output zero;
output wire [5:0] PCOut;
output wire [31:0] CurrentInstr, AluInA, AluInB, AluResult, RegFileWriteData;
wire RegDst, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
wire [1:0] ALUOp;
wire [3:0] opc; 
wire [4:0] wr;
wire [7:0] PCtoIM;
wire [31:0] sign_ext, ALUB_wire, MEMtoWB;
assign PCOut = PCtoIM[7:2]; 

pc_count PC(
.program(PCtoIM), 
.clk(clk)
);

inst_mem IM(
.read_addr(PCtoIM[7:2]),
.instruct(CurrentInstr),
.clk(clk)
);

main_control MC(
.RegDst(RegDst), 
.ALUSrc(ALUSrc), 
.MemtoReg(MemtoReg), 
.RegWrite(RegWrite), 
.MemRead(MemRead), 
.MemWrite(MemWrite), 
.ALUOp(ALUOp), 
.instr(CurrentInstr[31:26]),
.clk(clk) 
);

mux5 M5(
.min0(CurrentInstr[20:16]), 
.min1(CurrentInstr[15:11]), 
.sel(RegDst), 
.mout(wr)
);

regs R(
.rr1(CurrentInstr[25:21]),
.rr2(CurrentInstr[20:16]), 
.wr(wr), 
.wd(RegFileWriteData), 
.rd1(AluInA), 
.rd2(ALUB_wire), 
.write_en(RegWrite),
.clk(clk)
);

sign_extend SE(
.in(CurrentInstr[15:0]), 
.out(sign_ext),
.clk(clk)
);

mux32 M32(
.min0(ALUB_wire), 
.min1(sign_ext), 
.sel(ALUSrc), 
.mout(AluInB)
);

ALUcontrol AC(
.ALUOp(ALUOp),
.funct(CurrentInstr[5:0]), 
.opc(opc),
.clk(clk)
);

ALU A(
.A(AluInA),
.B(AluInB), 
.ALU_res(AluResult), 
.opc(opc), 
.zero(zero),
.clk(clk)
);

data_memory DM(
.address(AluResult[7:0]), 
.write_data(ALUB_wire), 
.read_data(MEMtoWB), 
.MemWrite(MemWrite), 
.MemRead(MemRead),
.clk(clk)
);

mux32 N32(
.min0(AluResult), 
.min1(MEMtoWB), 
.sel(MemtoReg), 
.mout(RegFileWriteData)
);
endmodule 






// BIG MUX
module mux32 (min0, min1, sel, mout);

input [31:0] min0, min1; 
input sel;
output [31:0] mout;
assign mout = (sel)? min1:min0;
endmodule

// SMALL MUX
module mux5 (min0, min1, sel, mout);
input [4:0] min0, min1; 
input sel;
output [4:0] mout;
assign mout = (sel)? min1:min0;
endmodule

// SIGN-EXTEND
module sign_extend(in, out, clk);
input clk;
input [15:0] in;
output reg [31:0] out;
always@(posedge clk)
begin
out [15:0] <= in[15:0];
out [31:16] <= in[15];
end
endmodule

// PROGRAM COUNTER
module pc_count(program, clk);
output reg [7:0] program;
input clk;
initial program = 8'b00000000;
always@(posedge clk)
 begin
	program = program + 8'b00000100;
end
endmodule

// INSTRUCTION MEMORY
module inst_mem(read_addr, instruct, clk);
input clk;
input [5:0] read_addr;
output reg [31:0] instruct;
reg [31:0] memo [63:0];
initial begin
  $readmemb("test_prog1.txt", memo);
end
always@(posedge clk)
begin
instruct = memo[read_addr];
end
endmodule

// REGISTER FILE 
module regs(rr1, rr2, wr, wd, rd1, rd2, write_en, clk);
integer cnt;
input clk, write_en; 
input [4:0] rr1, rr2, wr;
input [31:0] wd;
output[31:0] rd1, rd2;
reg [31:0] register[31:0];
initial
begin
for (cnt=0; cnt<32; cnt = cnt+1) 
register[cnt] = cnt;
end
always@(posedge clk)
begin
if (write_en) register[wr] = wd;
end
assign rd1 = rr1 ? register[rr1] : 0;
assign rd2 = rr2 ? register[rr2] : 0;
endmodule

//ALU
module ALU(A, B, ALU_res, opc, zero, clk);
input clk;
input [31:0] A, B;
input [3:0] opc;
output zero;
output reg[31:0] ALU_res;
reg [31:0] temp;
assign zero = (0==ALU_res);
always@(posedge clk) begin
case(opc)
	4'b0000: temp = A & B;
	4'b0001: temp = A | B;
	4'b0010: temp = A + B;
	4'b0110: temp = A - B;
	4'b0111: begin if (A < B) temp = 32'b00000000000000000000000000000001; else temp = 32'b0; end
	4'b1100: temp = ~(A | B);
	4'b1101: temp = ~(A & B);
	default: temp = 32'b0;
endcase
ALU_res = temp;
end 
endmodule 

// DATA MEMORY
module data_memory(address, write_data, read_data, MemWrite, MemRead, clk); 
input [7:0] address;
input MemWrite, MemRead, clk;
input [31:0] write_data; 
output[31:0] read_data;
reg [31:0] ram[0:255];
initial begin $readmemb("datamem.txt", ram, 0, 255); end
always@(posedge clk)
   begin
      if (MemWrite) begin ram[address] = write_data; end 
   end
assign read_data = MemWrite ? write_data : ram[address][31:0];
endmodule

// ALU CONTROL UNIT
module ALUcontrol(ALUOp, funct, opc, clk);
input clk;
input [5:0] funct;
input [1:0] ALUOp;
output reg [3:0] opc;
reg [3:0] temp;
always@(posedge clk)
case(funct/*[3:0]*/)
	4'b0000: temp <= 4'b0010; // add
	4'b0010: temp <= 4'b0110; // sub
	4'b0100: temp <= 4'b0000; // and
	4'b0101: temp <= 4'b0001; // or 
	4'b0111: temp <= 4'b1100; // nor
	4'b1010: temp <= 4'b0111; // slt
	4'b0110: temp <= 4'b1101; // nand 
	default: temp <= 4'b0000; // undefined 
endcase
always@(posedge clk)
begin
case(ALUOp)
	2'b00: opc = 4'b0010; // add
	2'b10: opc = temp; // above options
	default: opc = 4'b0000; 
endcase
end 
endmodule

// MAIN CONTROL UNIT
module main_control(RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, ALUOp, instr, clk);
input clk;
input [5:0] instr;
output reg [1:0] ALUOp;
output reg RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite;
always@(posedge clk)
begin
// inital values
ALUOp = 2'b10;
RegDst = 1'b1;
ALUSrc = 1'b0;
MemtoReg = 1'b0;
RegWrite = 1'b1;
MemRead = 1'b0;
MemWrite = 1'b0;
case(instr)
   6'b100011: begin ALUOp=2'b00; RegDst=1'b0; ALUSrc=1'b1; MemtoReg=1'b1; RegWrite=1'b1; MemRead=1'b1; MemWrite=1'b0; end//lw
   6'b101011: begin ALUOp=2'b00; RegDst=1'bx; ALUSrc=1'b1; MemtoReg=1'bx; RegWrite=1'b0; MemRead=1'b0; MemWrite=1'b1; end//sw
   6'b001000: begin ALUOp=2'b00; RegDst=1'b0; ALUSrc=1'b1; MemtoReg=1'b0; RegWrite=1'b1; MemRead=1'b0; MemWrite=1'b0; end//addi
   6'b000000: begin ALUOp=2'b10; RegDst=1'b1; ALUSrc=1'b0; MemtoReg=1'b0; RegWrite=1'b1; MemRead=1'b0; MemWrite=1'b0; end//R-Format                      
endcase
end
endmodule
