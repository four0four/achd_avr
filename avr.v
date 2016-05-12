module avr_cpu (
  input wire CLK,
  input wire RST,
  // currently held instruction
  // feeds combinational shit
  input wire [15:0] instr,

  // non-register memories
  // pad with 0s to fit
  output wire [15:0] p_addr,
  output wire [15:0] d_addr,

  output reg [7:0] S_reg,


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

  // immediates
  wire [7:0] K_8bit  = {instr[11:8], instr[3:0]};
  wire signed [11:0] K_12bit = instr[11:0];


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

  // SREG - half of these probably won't be needed
  // interrupt enable   I
  // bit copy storage   T
  // half-carry flag    H
  // sign bit (S=N^V)   S
  // 2's comp overflow  V 
  // negative flag      N
  // zero flag          Z
  // carry flag         C
  reg I, T, H, S, V, N, Z, C;

  genvar i;
  generate
  for(i=0; i<26; i = i + 1) begin
    always @(posedge CLK) begin // reset cond
      if(RST) begin          
        reg_file[i] <= 8'b0;
      end
    end
  end
  endgenerate


	always @(posedge CLK) begin 
		if(RST) begin              // reset cond
			reg_X <= 16'b0;
			reg_Y <= 16'b0;
			reg_Z <= 16'b0;
			{I, T, H, S, V, N, Z, C} <= 8'b0;
			S_reg <= 8'b0;
		end
		else S_reg <= {I, T, H, S, V, N, Z, C};
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

  always @ (posedge CLK) begin // negedge? lower freq, but we may actually work
		if (reg_write == 1'b1) begin
			if (Rd_addr < 5'd26) reg_file[Rd_addr] = Rd_di;
			// handle partial reg_{X,Y,Z} writing
			else begin
				case(Rd_addr)
					5'd26: reg_X[7:0]  = Rd_di;
					5'd27: reg_X[15:8] = Rd_di;
					5'd28: reg_Y[7:0]  = Rd_di;
					5'd29: reg_Y[15:8] = Rd_di;
					5'd30: reg_Z[7:0]  = Rd_di;
					5'd31: reg_Z[15:8] = Rd_di;
				endcase
			end
		end
  end

  // instruction decoder && ALU - can split this up into something like write/flags/PC src but w/e
  always @ (*) begin
		H = S_reg[5];
		S = S_reg[4];
		V = S_reg[3];
		N = S_reg[2];
		Z = S_reg[1];
		C = S_reg[0];
		reg_write = 1'b1;
    casex(instr)
			16'b0000000000000000: begin // NOP
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
			16'b1001010xxxxx0010: begin // SWAP
				Rd_di = {Rd_do[3:0], Rd_do[7:4]};
			end
			16'b1110xxxxxxxxxxxx: begin // LDI
				Rd_di = K_8bit;
			end
    endcase
  end

endmodule


module avr_fetch(
  input wire CLK,
  input wire RST,

  output reg [15:0] cur_instr,
  output wire [15:0] prog_addr,

  input wire [15:0] jmp,
  input wire [15:0] prog_data,
  input wire [2:0] pc_src

);

  reg [15:0] PC_reg;
  reg [15:0] PC_next;

  assign prog_addr = PC_next;

  // reset logic
  always @ (posedge CLK) begin
    if(RST) begin
      PC_reg <= 16'b0;
      PC_next <= 16'b0;
      cur_instr <= 16'b0; // NOP
    end
  end

  // maybe move this, maybe remove it
  always @ (*) begin
    case(pc_src)
      3'b000: PC_next = 16'b0;        // reset cond
      3'b001: PC_next = PC_reg;       // hold/multi-cycle
      3'b010: PC_next = PC_reg + 1;   // normal
      3'b011: PC_next = PC_reg + 2;   // 32 bit instruction
      3'b100: PC_next = PC_reg + jmp; // rel jump
      3'b101: PC_next = jmp;          // absolute jump
      // bug:
      3'b110: PC_next = 16'hFFFF;    
      3'b111: PC_next = 16'hFFFF;
    endcase
  end


  always @ (posedge CLK) begin
    PC_reg <= PC_next;  
    cur_instr <= prog_data;
  end

endmodule
