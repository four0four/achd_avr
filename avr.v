module avr_cpu (
	input wire CLK,
	input wire RST,

	// currently held instruction
	input wire [15:0] instr,
	// so close to not needing this
	// mostly exists just so we can push/pop into it
	input wire [15:0] cur_pc,

	// data memory
	input wire [7:0]  data_in,
	output reg [7:0] data_out,
	output reg [15:0] d_addr,
	output reg data_write,
	
	// fetch stage control
	output reg [2:0] pc_select,
	output reg [15:0] pc_jmp,
	output reg stall,

	// debugging
	output reg [7:0] S_reg,
	output reg [7:0] Rr_do, 
	output reg [7:0] Rd_do,
	output reg [7:0] Rd_di
);

	// slicing up instruction fields:
	wire 				immediate = instr[14];

	// reg addrs
	wire [4:0] 	Rr_addr = {instr[9],instr[3:0]};
	// if we're using immediate addressing, can only access upper 16 regs
	wire [4:0] 	Rd_addr = (immediate == 1'b1) ? {1'b1, instr[7:4]} : instr[8:4];
	reg [5:0] 	io_mem_addr; //= {instr[10:9], instr[3:0]};

	// immediates
	wire [7:0] 	K_8bit  = {instr[11:8], instr[3:0]};
	wire [11:0] K_12bit = instr[11:0];

	// branch conditions and offsets
	wire [6:0] 	K_branch = instr[9:3];
	wire [2:0] 	branch_cond = instr[2:0];
	wire			 	branch_negate = instr[10];

	// write pass/inhibit
	reg 				reg_write;

	// register file
	reg [7:0] 	reg_file [0:25];
	reg [15:0] 	reg_X;
	reg [15:0] 	reg_Y;
	reg [15:0] 	reg_Z;
	reg [15:0] 	reg_SP; // we want this?

	// multicycle state machine
	reg [3:0] 	holdstate, next_holdstate;

	// ret PC restore
	reg [15:0] 	pc_restore;

	// IO mem control
	reg [7:0] 	io_mem_out;
	wire [7:0] 	io_mem_in = Rd_do;
	reg 				io_mem_write;
	reg [7:0] 	gpior [2:0];

	// branch eqn stuff
	reg 				branch_result;

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

	// initial/simulation reg file clear
	genvar i;
	generate
			for(i=0; i<26; i = i + 1) begin : make_quartus_happy
			initial begin
				if(RST) begin
					reg_file[i] = 8'b0;
				end
			end
		end
	endgenerate

	always @(*) begin
		Rd_do = 8'bz; // hi-z for now, maybe some pattern later
		Rr_do = 8'bz; // hi-z for now, maybe some pattern later
		// handle partial reg_{X,Y,Z} loading
		// Rr
		if (Rr_addr < 5'd26) Rr_do = reg_file[Rr_addr];
		else begin
			case(Rr_addr)
				default: Rr_do = 8'bz;
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
				default: Rd_do = 8'bz;
				5'd26: Rd_do = reg_X[7:0];
				5'd27: Rd_do = reg_X[15:8];
				5'd28: Rd_do = reg_Y[7:0];
				5'd29: Rd_do = reg_Y[15:8];
				5'd30: Rd_do = reg_Z[7:0];
				5'd31: Rd_do = reg_Z[15:8];
			endcase
		end
		// IO memory space
		case(io_mem_addr) 
			default:	io_mem_out = 8'bz; // maybe a recognizable pattern
			8'h3F:  io_mem_out = S_reg;
			8'h3E:  io_mem_out = reg_SP[15:8];
			8'h3D:  io_mem_out = reg_SP[7:0];

			8'h15:  io_mem_out = gpior[2];
			8'h14:  io_mem_out = gpior[1];
			8'h13:  io_mem_out = gpior[0];
		endcase
	end

	// multi-cycle sequential & write-back
	always @ (posedge CLK) begin
		casex(instr)
			16'b1001010100001000: begin	// RET
				if(holdstate == 3'h1) pc_restore[7:0] <= data_in;
				if(holdstate == 3'h2) pc_restore[15:8] <= data_in;
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

		if(RST) begin			
			S_reg <= 8'h0;
			S_reg <= 8'h0;
			reg_X <= 16'b0;
			reg_Y <= 16'b0;
			reg_Z <= 16'b0;
			reg_SP <= 16'h0;
			pc_restore <= 0;
			gpior[0] <= 8'h0;
			gpior[1] <= 8'h0;
			gpior[2] <= 8'h0;
			holdstate <= 4'h0;
		end
		else begin
			S_reg <= {I, T, H, S, V, N, Z, C};
			holdstate <= next_holdstate;

			//  IO mem write back
			if (io_mem_write == 1'b1) begin
				case(io_mem_addr)
					8'h3F:  S_reg <= io_mem_in;
					8'h3E:  reg_SP[15:8] <= io_mem_in;
					8'h3D:  reg_SP[7:0] <= io_mem_in;

					8'h15:  gpior[2] <= io_mem_in;
					8'h14:  gpior[1] <= io_mem_in;
					8'h13:  gpior[0] <= io_mem_in;
				endcase
			end	// io_mem_write writeback
  
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
			end	// reg_write writeback
		end	// not in reset 
	end

	// multicycle instruction combinatorial
	always @ (*) begin

		if(RST) begin
			next_holdstate 	= 4'b0;
			pc_select 			= 3'b000;
			data_write 			= 1'b0;
			stall 					= 0;
		end
		else pc_select 	= 3'b010; // PC = PC + 1

		data_write 			= 1'b0;
		data_out 				= 8'bz;
		stall 					= 0;
		d_addr 					= reg_SP;
		next_holdstate 	= 4'h0;
		pc_jmp		 			= {{4{K_12bit[11]}}, K_12bit};
		io_mem_addr 		= {instr[10:9], instr[3:0]};

		casex(instr) 
			default: stall = 1'b0;				// just chug through illegal things
			16'b1111xxxxxxxxxxxx: begin		// BR*
				case(holdstate)
					4'h0: begin
						if (branch_negate ^ branch_result) begin	// either 1 or negated - good to branch
							pc_select = 3'b100;
							pc_jmp 		= {{9{K_branch[6]}}, K_branch};
							stall 		= 1'b1;
							next_holdstate = 1'h1;
						end
						else begin
							stall = 1'b0;
							next_holdstate = 1'h0;
						end
					end
					4'h1: begin
						stall = 1'b0;
					end
					default: next_holdstate = 4'h0;
				endcase
			end
			16'b1100xxxxxxxxxxxx: begin	  // RJMP
				case(holdstate) 
					4'h0: begin
						stall = 1'b1;
						pc_select	 	= 3'b100; 			// PC += K
						next_holdstate = 4'h1;
					end
					4'h1: begin
						stall = 1'b0;
						next_holdstate = 4'h0;
						pc_select = 3'b010;
					end  
					default: next_holdstate = 4'h0;
				endcase
			end // /RJMP

			16'b1101xxxxxxxxxxxx: begin // RCALL
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
						stall 					= 1'b0;
						data_write			= 1'b0;
						next_holdstate 	= 4'h0;
						pc_select				= 3'b010;				// PC += 1, allow pipeline to catch up
					end
					default: next_holdstate = 4'h0;
				endcase
			end // /RCALL

			16'b1001010100001000: begin	// RET
				pc_jmp			= pc_restore;
				case(holdstate)
					4'h0: begin
						stall 	= 1'b1;
						pc_select		= 3'b010;
						next_holdstate = 4'h1;
						d_addr 	= reg_SP + 16'h1;
					end
					4'h1: begin
						stall 	= 1'b1;
						pc_select = 3'b101;
						next_holdstate = 4'h2;
						d_addr 	= reg_SP + 16'h2;
					end
					4'h2: begin
						stall = 1'b1;
						pc_select = 3'b101;
						next_holdstate = 4'h3;
					end
					4'h3: begin
						stall = 1'b0;
						pc_select = 3'b010;
						next_holdstate = 4'h0;
					end
					default: next_holdstate = 4'h0;
				endcase
			end // /RET

			16'b1001000xxxxx1111: begin // POP
				case(holdstate)
					4'h0: begin
						stall = 1'b1;
						pc_select = 3'b001;
						next_holdstate = 4'h1;
						d_addr = reg_SP + 16'h1;
					end
					4'h1: begin
						stall = 1'b0;
						pc_select = 3'b010;
						next_holdstate = 4'h0;
					end
					default: next_holdstate = 4'h0;
				endcase
			end // /POP

			16'b1001001xxxxx1111: begin // PUSH
				case(holdstate)
					4'h0: begin
						// stall 1 cycle for memory
						// here we do NOT flush
						stall = 1'b1;
						pc_select = 3'b001;
						next_holdstate = 4'h1;
						// set up data memory things
						d_addr = reg_SP;
						data_out = Rd_do;
						data_write = 1'b1;
					end
					4'h1: begin
						// done, clean up
						stall = 1'b0;
						data_write = 1'b0;
						pc_select = 3'b010;
						next_holdstate = 4'h0;
					end
					default: next_holdstate = 4'h0;
				endcase
			end // /PUSH
		endcase // casex(instr)

	end // always

	// instruction decoder && ALU - can split this up into something like write/flags/PC src but w/e
	always @ (*) begin
		if(RST) begin
			{I, T, H, S, V, N, Z, C} = 8'b0;
			reg_write 		= 1'b0;
			Rd_di 				= 8'bz;
			io_mem_write 	= 1'b0;
			branch_result = 1'b0;
		end

		else begin
			I = S_reg[7];
			T = S_reg[6];
			H = S_reg[5];
			S = S_reg[4];
			V = S_reg[3];
			N = S_reg[2];
			Z = S_reg[1];
			C = S_reg[0];

			// default to write-out
			reg_write 		= 1'b1;
			Rd_di 				= 8'bz; // hi-z for now, maybe some pattern later
			io_mem_write 	= 1'b0;
			branch_result = 1'b0;
		end

		casex(instr)
			// chuck things that don't write back here
			16'b0000000000000000,       // NOP
			16'b1001001xxxxx1111,       // PUSH
			16'b1101xxxxxxxxxxxx,				// RCALL
			16'b1001010100001000,				// RET
			16'b1100xxxxxxxxxxxx: begin // RJMP
				reg_write = 1'b0;
			end
			default: reg_write = 1'b0;	// don't let unimplemented things write out
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
				Rd_di = data_in;
			end
			16'b000101xxxxxxxxxx: begin // CP
				// don't actually write out
				reg_write = 1'b0;
				Rd_di = Rd_do - Rr_do;
				H = (~Rd_do[3] & Rr_do[3]) | (Rr_do[3] & Rd_di[3]) | (Rd_di[3] & ~Rd_do[3]);
				V = (Rd_do[7] & ~Rr_do[7] & ~Rd_di[7] & Rd_do[7]& Rr_do[7] & Rd_di[7]);
				N = Rd_di[7];
				S = N ^ V;
				Z = (Rd_di == 8'b0);
				C = (~Rd_do[7] & Rr_do[7]) | (Rr_do[7] & Rd_di[7]) | (Rd_di[7] & ~Rd_do[7]);
			end
			16'b0011xxxxxxxxxxxx: begin // CPI
				// don't actually write out
				reg_write = 1'b0;
				Rd_di = Rd_do - K_8bit;
				H = (~Rd_do[3] & K_8bit[3]) | (Rd_di[3] & K_8bit[3]) | (Rd_di[3] & ~Rd_do[3]);
				V = (Rd_do[7] & ~K_8bit[7] & ~Rd_di[7]) | (~Rd_do[7] & K_8bit[7] & Rd_di[7]);
				N = Rd_di[7];
				S = N ^ V;
				Z = (Rd_di == 8'b0);
				C = (~Rd_di[7] & K_8bit[7]) | (K_8bit[7] & Rd_di[7]) | (Rd_di[7] & ~Rd_do[7]);
			end
			16'b000010xxxxxxxxxx: begin // SBC
				Rd_di = Rd_do - Rr_do - ((instr[12] == 1'b1) ? C : 1'b0);
				H = (Rd_do[3] & Rr_do[3]) | (Rr_do[3] & Rd_di[3]) | (Rd_di[3] & ~Rd_do[3]);
				V = (Rd_do[7] & ~Rr_do[7] & Rd_di[7]) | (~Rd_do[7] & Rr_do[7] & Rd_di[7]);
				N = Rd_di[7];
				S = V ^ N;
				Z = (Rd_di == 8'b0);
				C = (Rd_do[7] & Rr_do[7]) | (Rr_do[7] & ~Rd_di[7]) | (~Rd_di[7] & Rd_do[7]);
			end
			16'b0100xxxxxxxxxxxx: begin // SBCI
				Rd_di = Rd_do - K_8bit - ((instr[12] == 1'b1) ? C : 1'b0);
				H = (~Rd_do[3] & K_8bit[3]) | (Rd_di[3] & K_8bit[3]) | (Rd_di[3] & ~Rd_do[3]);
				V = (Rd_do[7] & ~K_8bit[7] & ~Rd_di[7]) | (~Rd_do[7] & K_8bit[7] & Rd_di[7]);
				N = Rd_di[7];
				S = V ^ N;
				Z = (Rd_di == 8'b0); 
				C = (~Rd_do[7] & K_8bit[7]) | (K_8bit[7] & Rd_di[7]) | (Rd_di[7] & ~Rd_do[7]);
			end
			16'b001010xxxxxxxxxx: begin // OR
				Rd_di = Rd_do | Rr_do;
				N = Rd_di[7];
				V = 1'b0;
				S = V ^ N;
				Z = (Rd_di == 8'b0);	
			end
			16'b0110xxxxxxxxxxxx: begin // ORI
				Rd_di = Rd_do | K_8bit;
				N = Rd_di[7];
				V = 1'b0;
				S = V ^ N;
				Z = (Rd_di == 8'b0);	
			end
			16'b001000xxxxxxxxxx: begin // AND
				Rd_di = Rd_do & Rr_do;
				Z = (Rd_di == 8'b0);
				V = 1'b0;
				N = Rd_di[7];
				S = V ^ N;
			end
			16'b0111xxxxxxxxxxxx: begin // ANDI
				Rd_di = Rd_do & K_8bit;
				Z = (Rd_di == 8'b0);
				V = 1'b0;
				N = Rd_di[7];
				S = V ^ N;
			end
			16'b1111xxxxxxxxxxxx: begin	// BR*
				case(branch_cond)
					3'b000: branch_result = (C == 1'b1);
					3'b001: branch_result = (Z == 1'b1);
					3'b010: branch_result = (N == 1'b1);
					3'b011: branch_result = (V == 1'b1);
					3'b100: branch_result = (S == 1'b1);
					3'b101: branch_result = (H == 1'b1);
					3'b110: branch_result = (T == 1'b1);
					3'b111: branch_result = (I == 1'b1);
				endcase
				reg_write = 1'b0;
			end
		endcase // casex(instr)
	end // always

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

	assign prog_addr = PC_next;

	assign current_pc = PC_reg;

	always @ (posedge CLK) begin
		// reset logic
		if(RST) begin
			PC_reg <= 16'b0;
			cur_instr <= 16'b0; // NOP
		end

		else begin
			PC_reg <= PC_next;
			if (!stall) begin
				cur_instr[7:0] <= prog_data[15:8];
				cur_instr[15:8] <= prog_data[7:0];
			end
		end

	end

	always @ (*) begin
		if(RST) PC_next = 16'h0;
		else begin
			case(pc_src)
				3'b000: PC_next = 16'h0;		// reset cond
				3'b001: PC_next = PC_reg;		// hold/multi-cycle
				3'b010: PC_next = PC_reg + 16'h1;	// normal
				3'b011: PC_next = PC_reg + 16'h2;	// 32 bit instruction
				3'b100: PC_next = PC_reg + jmp;	// rel jump
				3'b101: PC_next = jmp;			// absolute jump

				default: PC_next = 16'hFFFF;
			endcase
		end
	end

endmodule
