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

	reg [15:0] pmem [511:0];

	wire [15:0] inst;

  wire stall;

	initial begin
		$dumpfile("full_test.vcd");
		$dumpvars(0, c);
		$dumpvars(0, f);
		#0 rst = 1;
		#10 rst = 0;
		#700 $finish;
	end

	always #5 clk = !clk;

	avr_cpu c(
			.CLK(clk),
			.RST(rst),
			.stall(cpu_stall),
			.instr(inst),
			.p_addr(),
			.d_addr(),
			.S_reg(S),
			.pc_select(pcsrc),
			.pc_jmp(jmp_k),
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


endmodule


module program_memory(
  input wire [8:0] addr,
  input wire enable,
  input wire CLK,
  input wire RST,
  output reg [15:0] data);

  reg [15:0] rom [511:0];

  initial begin
		$readmemh("program_test.hex", rom, 0, 511);
    data = 16'd0;
  end

  always @ (posedge CLK) begin
    if(enable) begin
      data <= rom[addr];
    end
    else begin
      data <= 16'bz;
    end
    if(RST) begin
      data <= 16'b0;
    end
  end
endmodule
