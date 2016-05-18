module avr_cpu (
	input wire CLK,
	input wire RST,
	// currently held instruction
	// feeds combinational shit
	input wire [15:0] instr,
	// so close to not needing this
	// mostly exists just so we can push/pop into it
	input wire [15:0] cur_pc,

	inout wire [7:0]  data,


	// non-register memories
	// pad with 0s to fit
	output reg [15:0] p_addr,
	output reg [15:0] d_addr,
	output reg data_write,
	
	output reg [2:0] pc_select,
	output reg [15:0] pc_jmp,

	output reg [7:0] S_reg,

	output reg stall,


	// debugging
	output [7:0] Rr_do, 
	output [7:0] Rd_do,
	output [7:0] Rd_di

);

	// implemented:
	// direct single register
	// direct double register
	// direct I/O
	// direct data
	// relative program flow control (RJMP etc, implemented!)

	// direct data w/displacement 
	// indirect data
	// indirect data w/pre-decrement, or post-increment
	// program mem addressing (may not do this)
	// direct program flow control (JMP/CALL)
	// indirect program flow control (IJMP etc, not implemented)

	// slicing up instruction fields:
	wire immediate = instr[14];

	// reg addrs
	wire [4:0] Rr_addr = {instr[9],instr[3:0]};
	// if we're using immediate addressing, can only access upper 16 regs
	wire [4:0] Rd_addr = (immediate == 1'b1) ? {1'b1, instr[7:4]} : instr[8:4];
	wire [5:0] io_mem_addr = {instr[10:9], instr[3:0]};

	// immediates
	wire [7:0] K_8bit  = {instr[11:8], instr[3:0]};
	wire [11:0] K_12bit = instr[11:0];

	// reg file outputs at current addrs
	reg [7:0] Rr_do;
	reg [7:0] Rd_do;

	// dest reg data-in register (to-write)
	reg [7:0] Rd_di;

	// write pass/inhibit
	reg reg_write;

	// register file
	reg [7:0] reg_file [0:25];
	reg [15:0] reg_X;
	reg [15:0] reg_Y;
	reg [15:0] reg_Z;
	reg [15:0] reg_SP; // we want this?

	// multicycle state machine
	reg [3:0] holdstate, next_holdstate;

	// ret PC restore
	reg [15:0] pc_restore;

	// data memory control
	reg [7:0] data_out;
	assign data = (data_write == 1'b1) ? data_out : 8'bz;

	// IO mem control
	reg [7:0] io_mem_out;
	wire [7:0] io_mem_in = Rd_do;
	reg io_mem_write;
	reg [7:0] gpior [2:0];


	// SREG - half of these probably won't be needed
	// interrupt enable		I
	// bit copy storage		T
	// half-carry flag		H
	// sign bit (S=N^V)		S
	// 2's comp overflow	V
	// negative flag		N
	// zero flag			Z
	// carry flag			C
	reg I, T, H, S, V, N, Z, C;

	genvar i;
	generate
	for(i=0; i<26; i = i + 1) begin
	always @(posedge CLK) begin	// reset cond
		if(RST) begin
			reg_file[i] <= 8'b0;
		end
	end
	end
	endgenerate

	always @(posedge CLK) begin	
		if(RST) begin			// reset cond
			reg_X <= 16'b0;
			reg_Y <= 16'b0;
			reg_Z <= 16'b0;
			reg_SP <= 16'b0;
			{I, T, H, S, V, N, Z, C} <= 8'b0;
			S_reg <= 8'b0;
			holdstate <= 4'b0;
			next_holdstate <= 4'b0;
			stall <= 0;
			data_write <= 0;
			pc_restore <= 0;
		end
		else S_reg <= {I, T, H, S, V, N, Z, C};
	end

	always @ (posedge CLK) begin
		if (reg_write == 1'b1) begin
			if (Rd_addr < 5'd26) reg_file[Rd_addr] = Rd_di;
			// handle partial reg_{X,Y,Z} writing
			else begin
				case(Rd_addr)
					5'd26: reg_X[7:0]  <= Rd_di;
					5'd27: reg_X[15:8] <= Rd_di;
					5'd28: reg_Y[7:0]  <= Rd_di;
					5'd29: reg_Y[15:8] <= Rd_di;
					5'd30: reg_Z[7:0]  <= Rd_di;
					5'd31: reg_Z[15:8] <= Rd_di;
				endcase
			end
		end
	end

	always @(*) begin
		// handle partial reg_{X,Y,Z} loading
		// Rr
		if (Rr_addr < 5'd26) Rr_do = reg_file[Rr_addr];
		else begin
			case(Rr_addr)
				5'd26: Rr_do = reg_X[7:0];
				5'd27: Rr_do = reg_X[15:8];
				5'd28: Rr_do = reg_Y[7:0];
				5'd29: Rr_do = reg_Y[15:8];
				5'd30: Rr_do = reg_Z[7:0];
				5'd31: Rr_do = reg_Z[15:8];
			endcase
		end
		// Rd
		if (Rd_addr < 5'd26) Rd_do = reg_file[Rd_addr];
		else begin
			case(Rd_addr)
				5'd26: Rd_do = reg_X[7:0];
				5'd27: Rd_do = reg_X[15:8];
				5'd28: Rd_do = reg_Y[7:0];
				5'd29: Rd_do = reg_Y[15:8];
				5'd30: Rd_do = reg_Z[7:0];
				5'd31: Rd_do = reg_Z[15:8];
			endcase
		end
	end

	// multi-cycle sequential nonsense
	always @ (posedge CLK) begin
		casex(instr)
			16'b1001010100001000: begin	// RET
				if(holdstate == 3'h1) pc_restore[7:0] <= data;
				if(holdstate == 3'h2) pc_restore[15:8] <= data;
				if(holdstate == 4'h3) reg_SP <= reg_SP + 16'h2;
			end
			16'b1101xxxxxxxxxxxx: begin // RCALL
				if(holdstate == 4'h2) reg_SP <= reg_SP - 16'h2;
			end
			16'b1001000xxxxx1111: begin // POP
				if(holdstate == 4'b0) reg_SP <= reg_SP + 16'h1;
			end
			16'b1001001xxxxx1111: begin // PUSH
				if(holdstate == 4'b0) reg_SP <= reg_SP - 16'h1;
			end
		endcase
		holdstate <= next_holdstate;
	end

	// multicycle instruction combinatorial
	always @ (*) begin

		if(RST) pc_select = 3'b000;
		else pc_select = 3'b010; // PC = PC + 1

		data_write 	= 1'b0;
		data_out 		= 8'bz;
		d_addr		  = 16'bz;

		casex(instr) 
			16'b1101111010101101: begin // lol
				stall = 1'b1;
			end
			16'b1100xxxxxxxxxxxx: begin	  // RJMP
				pc_select	 	= 3'b100; 			// PC += K
				pc_jmp		 	= {{4{K_12bit[11]}}, K_12bit};
				case(holdstate) 
					4'h0: begin
						stall = 1'b1;
						next_holdstate = 4'h1;
					end
					4'h1: begin
						stall = 1'b0;
						next_holdstate = 4'h0;
						pc_select = 3'b010;
					end  
				endcase
			end // /RJMP

			16'b1101xxxxxxxxxxxx: begin // RCALL
				pc_jmp		 	= {{4{K_12bit[11]}}, K_12bit};
				case(holdstate)
					4'h0: begin
						next_holdstate = 4'h1;

						// stash pc[15:8]
						stall 			= 1'b1;
						d_addr			= reg_SP;
						data_out 		= cur_pc[15:8];
						data_write	= 1'b1;
						pc_select 	= 3'b001;				// Hold PC
					end
					4'h1: begin
						next_holdstate = 4'h2;

						// stash pc[7:0]
						stall 			= 1'b1;
						d_addr			= reg_SP - 16'b1;
						data_out 		= cur_pc[7:0];
						data_write	= 1'b1;
						pc_select		= 3'b100;				// PC += K
					end
					4'h2: begin
						stall = 1'b0;
						data_write	= 1'b1;
						next_holdstate = 4'h0;
						pc_select		= 3'b010;				// PC += 1, allow pipeline to catch up
					end
				endcase
			end // /RCALL

			16'b1001010100001000: begin	// RET
				pc_select		= 3'b010;
				pc_jmp			= pc_restore;
				io_mem_write = 1'b0;
				case(holdstate)
					4'h0: begin
						stall = 1'b1;
						next_holdstate = 4'h1;
						d_addr = reg_SP + 16'h1;
					end
					4'h1: begin
						stall = 1'b1;
						next_holdstate = 4'h2;
						d_addr = reg_SP + 16'h2;
					end
					4'h2: begin
						stall = 1'b1;
						next_holdstate = 4'h3;
						pc_select = 3'b101;
					end
					4'h3: begin
						stall = 1'b0;
						pc_select = 3'b010;
						next_holdstate = 4'h0;
					end
				endcase
			end // /RET

			16'b1001000xxxxx1111: begin // POP
				case(holdstate)
					// may be worth latching something here?
					// I don't think it matters right now with a pipeline this simple
					4'h0: begin
						stall = 1'b1;

						next_holdstate = 4'h1;

						d_addr = reg_SP + 1;
						pc_select = 3'b001;
					end
					4'h1: begin
						pc_select = 3'b010;
						stall = 1'b0;
						next_holdstate = 4'h0;
					end
				endcase
			end // /POP

			16'b1001001xxxxx1111: begin // PUSH
				case(holdstate)
					4'h0: begin
						// stall 1 cycle for memory
						// here we do NOT flush
						stall = 1'b1;

						next_holdstate = 4'h1;
						pc_select = 3'b001;

						// set up data memory things
						d_addr = reg_SP;
						data_write = 1'b1;
						data_out = Rd_do;
					end
					4'h1: begin
						pc_select = 3'b010;
						// done, clean up
						stall = 1'b0;
						next_holdstate = 4'h0;
						data_write = 1'b0;
					end
				endcase
			end // /PUSH

		endcase // casex(instr)

	end // always

	// instruction decoder && ALU - can split this up into something like write/flags/PC src but w/e
	always @ (*) begin
		H = S_reg[5];
		S = S_reg[4];
		V = S_reg[3];
		N = S_reg[2];
		Z = S_reg[1];
		C = S_reg[0];

		// default to write-out
		reg_write = 1'b1;
		Rd_di = 8'bz; // hi-z for now, maybe some pattern later
		io_mem_write = 1'b0;
		reg_SP 			= reg_SP;

		casex(instr)
			// chuck things that don't write back here
			16'b0000000000000000,       // NOP
			16'b1001001xxxxx1111,       // PUSH
			16'b1101xxxxxxxxxxxx,				// RCALL
			16'b1001010100001000,				// RET
			16'b1101xxxxxxxxxxxx: begin // RJMP
				reg_write = 1'b0;
			end
			16'b000x11xxxxxxxxxx: begin // ADD, ADC - bit 12 indicates carry
				Rd_di = Rd_do + Rr_do + ((instr[12] == 1'b1) ? C : 1'b0);
				H = (Rd_do[3] & Rr_do[3]) | (Rr_do[3] & ~Rd_do[3]) | (~Rd_do[3] & Rd_do[3]);
				V = (Rd_do[7] & Rr_do[7] & ~Rd_di[7]) | (~Rd_do[7] & Rr_do[7] & Rd_di[7]);
				N = Rd_di[7];
				S = V ^ N;
				Z = (Rd_di == 8'b0);
				C = (Rd_do[7] & Rr_do[7]) | (Rr_do[7] & ~Rd_di[7]) | (~Rd_di[7] & Rd_do[7]);
			end
			16'b000110xxxxxxxxxx: begin // SUB
				Rd_di = Rd_do - Rr_do;
				H = (~Rd_do[3] & Rr_do[3]) | (Rr_do[3] & Rd_di[3]) | (Rd_di[3] & ~Rd_do[3]);
				V = (Rd_do[7] & ~Rr_do[7] & ~Rd_di[7]) | (~Rd_do[7] & Rr_do[7] & Rd_di[7]);
				N = Rd_di[7];
				S = N ^ V;
				Z = (Rd_di == 8'b0);
				C = (~Rd_do[7] & Rr_do[7]) | (Rr_do[7] & Rd_di[7]) | (Rd_di[7] & ~Rd_do[7]);
			end
			16'b0101xxxxxxxxxxxx: begin // SUBI
				Rd_di = Rd_do - K_8bit;
				H = (~Rd_do[3] & K_8bit[3]) | (Rd_di[3] & K_8bit[3]) | (Rd_di[3] & ~Rd_do[3]);
				V = (Rd_do[7] & ~K_8bit[7] & ~Rd_di[7]) | (~Rd_do[7] & K_8bit[7] & Rd_di[7]);
				N = Rd_di[7];
				S = V ^ N;
				Z = (Rd_di == 8'b0);
				C = (~Rd_di[7] & K_8bit[7]) | (K_8bit[7] & Rd_di[7]) | (Rd_di[7] & ~Rd_do[7]);
			end
			16'b10110xxxxxxxxxxx: begin // IN
				Rd_di = io_mem_out;
			end
			16'b10111xxxxxxxxxxx: begin // OUT
				io_mem_write = 1'b1;  
				reg_write = 1'b0;
			end
			16'b1001010xxxxx0010: begin // SWAP
				Rd_di = {Rd_do[3:0], Rd_do[7:4]};
			end
			16'b1110xxxxxxxxxxxx: begin // LDI
				Rd_di = K_8bit;
			end
			16'b1001000xxxxx1111: begin // POP
				Rd_di = data;
			end
		endcase // casex(instr)
	end // always

	// IO memory 
	always @(*) begin 
		io_mem_out = 8'bz; // maybe a tell-tale pattern?
		case(io_mem_addr) 
			8'h3F:  io_mem_out = S_reg;
			8'h3E:  io_mem_out = reg_SP[15:8];
			8'h3D:  io_mem_out = reg_SP[7:0];

			8'h15:  io_mem_out = gpior[2];
			8'h14:  io_mem_out = gpior[1];
			8'h13:  io_mem_out = gpior[0];
		endcase
	end

	always @(posedge CLK) begin

		// write-back
		if (io_mem_write == 1'b1) begin
			case(io_mem_addr)
				8'h3F:  S_reg <= io_mem_in;
				8'h3E:  reg_SP[15:8] <= io_mem_in;
				8'h3D:  reg_SP[7:0] <= io_mem_in;

				8'h15:  gpior[2] <= io_mem_in;
				8'h14:  gpior[1] <= io_mem_in;
				8'h13:  gpior[0] <= io_mem_in;
			endcase
		end
	end

endmodule


module avr_fetch(
	input wire CLK,
	input wire RST,
	input wire stall,

	output reg [15:0] cur_instr,
	output wire [15:0] prog_addr,
	output wire [15:0] current_pc,

	input wire [15:0] jmp,
	input wire [15:0] prog_data,
	input wire [2:0] pc_src

);

	reg [15:0] PC_reg;
	reg [15:0] PC_next;

	reg [15:0] instr_last;

	assign prog_addr = PC_next;

	assign current_pc = PC_reg;

	always @ (posedge CLK) begin
		// reset logic
		if(RST) begin
			PC_reg <= 16'b0;
			PC_next <= 16'b0;
			cur_instr <= 16'b0; // NOP
		end

		else begin
			PC_reg <= PC_next;
			if (!stall) begin
				cur_instr <= prog_data;
			end
		end

	end

	// maybe move this, maybe remove it
	always @ (*) begin
		case(pc_src)
			3'b000: PC_next = 16'b0;		// reset cond
			3'b001: PC_next = PC_reg;		// hold/multi-cycle
			3'b010: PC_next = PC_reg + 1;	// normal
			3'b011: PC_next = PC_reg + 2;	// 32 bit instruction
			3'b100: PC_next = PC_reg + jmp;	// rel jump
			3'b101: PC_next = jmp;			// absolute jump
			// bug:
			3'b110: PC_next = 16'hFFFF;
			3'b111: PC_next = 16'hFFFF;
		endcase
	end

endmodule
