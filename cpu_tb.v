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
		$dumpfile("testcases/cpu_test.vcd");
		$dumpvars(0, c);
		#0 rst = 1;
		#10 rst = 0;
		#0	inst = 16'b1110000010100100; // ldi r26, 0x04
		#10 inst = 16'b0101000010100001; // subi r26, 0x01
		#10 inst = 16'b0101000010100010; // subi r26, 0x02
		#10 inst = 16'b0101000010100000; // subi r26, 0x00
		#10 inst = 16'b0; // NOP
		#10 inst = 16'b1110100000000000; // ldi r16, 0x80
		#10 inst = 16'b1110000000010001; // ldi r17, 0x01
		#10 inst = 16'b0000111100000001; // add r16, r17
		#10 inst = 16'b0101000011000001; // subi r28, 0x01
		#10 inst = 16'b0001111111001010; // adc r28, r26
		#10 inst = 16'b1001010111000010; // swap r28
		#10 inst = 16'b0001111111001010; // adc r28, r26
		#10 inst = 16'b0; // NOP
		#10 $finish;
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
