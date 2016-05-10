`timescale 1ns/1ns

module cpu_tb;

  reg rst = 0;
  reg clk = 0;
  wire [7:0] rrdo, rddo, rddi;
  wire [7:0] S;
  //reg [15:0] pmem [511:0];

  reg [15:0] inst;

  initial begin
    //$readmemh("fetchtest.hex", pmem);
		$dumpfile("cpu_test.vcd");
    $dumpvars(0, c);
    #5 rst = 1;
    #5 rst = 0;
    #0 inst = 16'b0101000010100001;
    #10 inst = 16'b0101000010100010;
    #10 inst = 16'b0101000010100000;
    #10 inst = 16'b0; // NOP
		#10 inst = 16'b1110100000000000; // ldi 0x80 16
		#10 inst = 16'b1110000000010001; // ldi 0x01 17
		#10 inst = 16'b0000111100000001; // add 
    #5 $finish;
  end

  always #5 clk = !clk;

  avr_cpu c(
      .CLK(clk),
      .RST(rst),
      .instr(inst),
      .p_addr(),
      .d_addr(),
      .S_reg(S),
      .Rr_do(rrdo),
      .Rd_do(rddo),
      .Rd_di(rddi)
  );

endmodule

