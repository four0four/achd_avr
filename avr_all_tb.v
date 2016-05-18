`timescale 10ns/1ns

module cpu_tb;

	reg rst = 0;
	reg clk = 0;

	wire [15:0] jmp_k;
	wire [7:0] rrdo, rddo, rddi;
	wire [7:0] S;
	wire [2:0] pcsrc;
	wire [15:0] progaddr;
	wire [15:0] progdata;
	wire [15:0] pc;

	wire [7:0] data;
	wire [15:0] dataaddr;
	wire dwrite;

	reg [15:0] pmem [511:0];

	wire [15:0] inst;

	wire stall;

	initial begin
		$dumpfile("full_test.vcd");
		$dumpvars(0, c);
		$dumpvars(0, f);
		#0 rst = 1;
		#10 rst = 0;
		#8000 $finish;
	end

	always #5 clk = !clk;

	avr_cpu c(
			.CLK(clk),
			.RST(rst),
			.stall(cpu_stall),
			.instr(inst),
			.p_addr(),
			.d_addr(dataaddr),
			.data_write(dwrite),
			.data(data),
			.S_reg(S),
			.pc_select(pcsrc),
			.pc_jmp(jmp_k),
			.cur_pc(pc),
			.Rr_do(rrdo),
			.Rd_do(rddo),
			.Rd_di(rddi)
	);

	avr_fetch f(
		.CLK(clk),
		.RST(rst),
		.stall(cpu_stall),
		.cur_instr(inst),
		.prog_addr(progaddr),
		.current_pc(pc),
		.jmp(jmp_k),
		.prog_data(progdata),
		.pc_src(pcsrc)
	);                  

	program_memory m(
		.CLK(clk),
		.RST(rst),
		.addr(progaddr[8:0]),
		.data(progdata),
		.enable(1'b1)
	);

	data_memory d(
		.CLK(clk),
		.RST(rst),
		.addr(dataaddr[10:0]),
		.write_en(dwrite),
		.data(data)
	);

endmodule


module program_memory(
	input wire [8:0] addr,
	input wire enable,
	input wire CLK,
	input wire RST,
	output reg [15:0] data);

	reg [15:0] rom [511:0];

	initial begin
		$readmemh("testcases/add_loop_testcase.hex", rom, 0, 511);
		data = 16'd0;
	end

	always @ (posedge CLK) begin
		if(enable) begin
			data <= {rom[addr][7:0], rom[addr][15:8]};
		end
		else begin
			data <= 16'bz;
		end

		if(RST) begin
			data <= 16'b0;
		end
	end

endmodule


module data_memory(
	input wire [10:0] addr,
	input wire write_en,
	input wire CLK,
	input wire RST,
	inout wire [7:0] data);

	reg [7:0] ram [2047:0];
	reg [7:0] data_r; 

	assign data = write_en ? 8'bz : data_r;

	integer i;
	initial begin
		for (i = 0; i < 2048; i = i + 1) begin
			ram[i] = 8'b0;
		end
	end

	always @ (posedge CLK) begin
		data_r <= ram[addr];
		if(write_en) begin
			ram[addr] <= data;
		end

		if(RST) begin
			data_r <= 16'b0;
		end
	end                   
endmodule
