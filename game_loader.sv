// Module reads bytes and writes to proper address in ram.
// Done is asserted when the whole game is loaded.
// This parses iNES headers too.
module game_loader (
	input		      clk,
	input		      reset,
	input       [7:0] indata,
	input		      indata_clk,

	output reg [21:0] mem_addr,
	output      [7:0] mem_data,
	output            mem_write,
	output     [31:0] mapper_flags,
	output reg        done,
    output reg        error
);
    enum {START, READ, DONE, ERROR} state;
	reg [ 7:0] prgsize;
	reg [ 3:0] ctr;
	reg [ 7:0] ines[0:15];	// 16 bytes of iNES header
	reg [21:0] bytes_left;
	
	assign mem_data = indata;
	assign mem_write = (bytes_left != 0) && (READ == 1 || DONE == 2) && indata_clk;
	
    // Size to read
	wire [2:0] prg_size =
                ines[4] <= 1 ? 0 :
				ines[4] <= 2 ? 1 : 
				ines[4] <= 4 ? 2 : 
				ines[4] <= 8 ? 3 : 
				ines[4] <= 16 ? 4 : 
				ines[4] <= 32 ? 5 : 
				ines[4] <= 64 ? 6 : 7;

	wire [2:0] chr_size =
                ines[5] <= 1 ? 0 : 
				ines[5] <= 2 ? 1 : 
				ines[5] <= 4 ? 2 : 
				ines[5] <= 8 ? 3 : 
				ines[5] <= 16 ? 4 : 
				ines[5] <= 32 ? 5 : 
				ines[5] <= 64 ? 6 : 7;
	
	wire has_chr_ram = (ines[5] == 0);
	assign mapper_flags = {16'b0, has_chr_ram, ines[6][0], chr_size, prg_size, ines[7][7:4], ines[6][7:4]};

	always_ff @(posedge clk) begin
		if (reset) begin
            error <= 1;

			state <= START;
			done <= 0;
			ctr <= 0;
			mem_addr <= 0; // Address for PRG
		end else begin
			case(state)
            START: begin 
                // Read 16 bytes of ines header
                if (indata_clk) begin
                    ctr <= ctr + 1;
                    ines[ctr] <= indata;
                    bytes_left <= {ines[4], 14'b0};

                    // Read 16 bytes
                    if (ctr == 4'b1111) begin
                        state <= (ines[0] == 8'h4E) // N
                              && (ines[1] == 8'h45) // E
                              && (ines[2] == 8'h53) // S
                              && (ines[3] == 8'h1A) // EOF
                              && !ines[6][2] // No 512-byte trainer at $7000-$71FF
                              && !ines[6][3] // Not alternative nametable layout
                              ? READ : ERROR; // If all those are satisfied proceed
                    end
				end
            end
			READ, DONE: begin
                // Read the next |bytes_left| bytes into |mem_addr|
				if (bytes_left != 0) begin
					if (indata_clk) begin
						bytes_left <= bytes_left - 1;
						mem_addr <= mem_addr + 1;
					end
				end else if (state == READ) begin
                    // After the reading of the program is done,
                    // start reading the chr
					state <= DONE;
					mem_addr <= 22'b10_0000_0000_0000_0000_0000; // Address for CHR
					bytes_left <= {1'b0, ines[5], 13'b0};
				end else if (state == DONE) begin
                    done <= 1;
				end
            end
            ERROR: begin
                error <= 0;
            end
			endcase
		end
	end
endmodule
