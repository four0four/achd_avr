`timescale 1ns/1ns

module fetch_tb;

  reg rst = 0;
  reg clk = 0;
  reg [15:0] pmem [511:0];
  reg [2:0] pcsrc = 3'b010;
  reg [15:0] jmp_k = 16'b0;

  wire [15:0] inst;
  wire [15:0] addr;


  initial begin
    $readmemh("fetchtest.hex", pmem);
		$dumpfile("fetch_test.vcd");
    $dumpvars(0, f);
    #5 rst = 1;
    #10 rst = 0;
    #5 pcsrc = 3'b000;
    #15 pcsrc = 3'b010;
    #15 pcsrc = 3'b000;
    #10 pcsrc = 3'b011;
    #50 pcsrc = 3'b101;
    #0 jmp_k = 16'h50;
    #10 pcsrc = 3'b100;
    #0 jmp_k = 16'h05;
    #10 $finish;
  end

  always #5 clk = !clk;


  avr_fetch f(
    .CLK(clk),
    .RST(rst),
    .cur_instr(inst),
    .prog_addr(addr),
    .jmp(jmp_k),
    .prog_data(pmem[addr]),
    .pc_src(pcsrc)
  );

endmodule

