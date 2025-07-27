`default_nettype none
module top
#(
  parameter use_external_nes_joypad=0,
  parameter C_audio=1, // 0: direct MSB 4->4 bit DAC, 1: 16->1 bit PWM-DAC, 2: 16->4 bit PWM-DAC
  parameter C_usb_speed=1,  // 0:6 MHz USB1.0, 1:48 MHz USB1.1, -1: USB disabled,
  // xbox360 : C_report_bytes=20, C_report_bytes_strict=1
  // darfon  : C_report_bytes= 8, C_report_bytes_strict=1
  parameter C_report_bytes=20, // 8:usual joystick, 20:xbox360
  parameter C_report_bytes_strict=1, // 0:when report length is variable/unknown
  parameter C_autofire_hz=10, // joystick trigger and bumper
  parameter c_ddr    =  1  // 0:SDR 1:DDR
)
(
  input  clk,
  output flash_csn,
  output flash_mosi,
  output flash_wpn,
  output flash_holdn,
  input  flash_miso,

  output [4:0]  led,

  output sdram_csn,       // chip select
  output sdram_clk,       // clock to SDRAM
  output sdram_cke,       // clock enable to SDRAM
  output sdram_rasn,      // SDRAM RAS
  output sdram_casn,      // SDRAM CAS
  output sdram_wen,       // SDRAM write-enable
  output [12:0] sdram_a,  // SDRAM address bus
  output  [1:0] sdram_ba, // SDRAM bank-address
  output  [1:0] sdram_dqm,// byte select
  inout  [15:0] sdram_d,  // data bus to/from SDRAM

  input   [1:0] btn,

  // input  usb_fpga_dp,
  inout  usb_fpga_bd_dp,
  inout  usb_fpga_bd_dn,
  output usb_fpga_pu_dp,
  output usb_fpga_pu_dn,

  // DVI out
  output  [3:0] gpdi_dp,
  // GPIO for audio
  input [27:0]  gpio
);
  wire  btn_start =  0;//btn[0];
  wire  btn_a     =  0;//btn[1];
  wire  user_reset;
  debouncer reset_btn(.clk(clk), .btn(~btn[1] | ~btn[0]), .sig(user_reset));

  wire [3:0] dvi_usb_clocks;
  wire dvi_clock_locked;
  ecp5pll
  #(
      .in_hz( 50*1000000),
    .out0_hz(125*1000000),
    .out1_hz( 48*1000000), .out1_tol_hz(100000),
    .out2_hz(  6*1000000), .out2_tol_hz( 10000),
    .out3_hz( 25*1000000)
  )
  clk_dvi_usb_inst
  (
    .clk_i(clk),
    .clk_o(dvi_usb_clocks),
    .locked(dvi_clock_locked)
  );
  wire clk_shift = dvi_usb_clocks[0];
  wire clk_48MHz = dvi_usb_clocks[1]; // USB1.1
  wire clk_6MHz  = dvi_usb_clocks[2]; // USB1.0
  wire clk_pixel = dvi_usb_clocks[3];

    wire S_phasedir;
    wire S_phasestep;
    wire S_phaseloadreg;

    wire [7:0] S_phase;
    btn_ecp5pll_phase
    #(
      .c_debounce_bits(16)
    )
    btn_ecp5pll_phase_inst
    (
      .clk(clk),
      .inc(~btn[1]),
      .dec(~btn[0]),
      .phase(S_phase),
      .phasedir(S_phasedir),
      .phasestep(S_phasestep),
      .phaseloadreg(S_phaseloadreg)
    );

  wire [3:0] sys_sdram_clocks;
  wire sys_sdram_clocks_locked;
  ecp5pll
  #(
      .in_hz( 50*1000000),
    .out0_hz(   85714285), // 4*25*6/7 MHz
    .out1_hz(   85714285), .out1_deg(070), // 0-45 ok, 90 fail, 150-359 ok
    .out2_hz(   21428571), //   25*6/7 MHz
    .out3_hz(   21428571)  // not used
  )
  clk_sdram_sys_inst
  (
    .clk_i(clk),
    .clk_o(sys_sdram_clocks),
    .locked(sys_sdram_clocks_locked),
    //.phasesel(2'd1), // select out1
    //.phasedir(S_phasedir),
    //.phasestep(S_phasestep),
    //.phaseloadreg(S_phaseloadreg)
  );
  wire clock_sdram_core = sys_sdram_clocks[0];
  wire clock_sdram_chip = sys_sdram_clocks[1];
  wire clock            = sys_sdram_clocks[2]; // NES system clock
  reg clock_locked;
  always @(posedge clock)
    clock_locked <= sys_sdram_clocks_locked;

  // assign led[4] = user_reset;
  assign led[3:0] = sdram_a[3:0];

  // prevent crosstalk at flash unused lines
  assign flash_wpn = 1;
  assign flash_holdn = 1;
  wire flash_sck;
  wire tristate = 1'b0;
  USRMCLK u1 (.USRMCLKI(flash_sck), .USRMCLKTS(tristate));

  wire btn_reset;
  reg [23:0] R_reset = 24'hFFFFFF;
  always @(posedge clock)
  begin
    if(clock_locked == 0 || dvi_clock_locked == 0 || btn_reset)
      R_reset <= 24'hFFFFFF;
    else
      if(R_reset[23])
        R_reset <= R_reset-1;
  end

  wire  [8:0] cycle;
  wire  [8:0] scanline;
  wire  [5:0] color;
  wire [15:0] sample;

  wire memory_write;
  wire [21:0] memory_addr_cpu, memory_addr_ppu;
  wire memory_read_cpu, memory_read_ppu;
  wire memory_write_cpu, memory_write_ppu;
  wire  [7:0] memory_din_cpu, memory_din_ppu;
  wire  [7:0] memory_dout_cpu, memory_dout_ppu;
  wire [31:0] mapper_flags;
  
  wire load_done;
  wire  [7:0] flash_loader_data_out;
  wire [21:0] game_loader_address;
  reg  [21:0] load_address_reg;
  wire  [7:0] game_loader_mem;
  wire flash_loader_data_ready;
  wire loader_write;

  wire clk_usb;  // 6 MHz USB1.0 or 48 MHz USB1.1
  generate if (C_usb_speed == 0) begin: G_low_speed
      assign clk_usb = clk_6MHz;
  end
  endgenerate
  generate if (C_usb_speed == 1) begin: G_full_speed
      assign clk_usb = clk_48MHz;
  end
  endgenerate

  assign usb_fpga_pu_dp = 1'b0;
  assign usb_fpga_pu_dn = 1'b0;
  // assign usb_fpga_otg_id = 1'b1;
  // assign n_extrst = 1'b1;

  wire [C_report_bytes*8-1:0] S_report;
  wire S_report_valid;
  wire [8:0] usb_buttons;

  // Generate USB stuff
  generate if (C_usb_speed >= 0) begin
  usbh_host_hid #(
    .C_usb_speed(C_usb_speed), // '0':Low-speed '1':Full-speed
    .C_report_length(C_report_bytes),
    .C_report_length_strict(C_report_bytes_strict)
  ) us2_hid_host_inst (
    .clk(clk_usb), // 6 MHz for low-speed USB1.0 device or 48 MHz for full-speed USB1.1 device
    .bus_reset(~dvi_clock_locked),
    .led(), // debug output
    //.usb_dif(usb_fpga_dp),
    .usb_dif(usb_fpga_bd_dp), // for trellis < 2020-03-08
    .usb_dp(usb_fpga_bd_dp),
    .usb_dn(usb_fpga_bd_dn),
    .hid_report(S_report),
    .hid_valid(S_report_valid)
  );

  usbh_report_decoder #(
    .c_autofire_hz(C_autofire_hz)
  ) usbh_report_decoder_inst (
    .i_clk(clk_usb),
    .i_report(S_report),
    .i_report_valid(S_report_valid),
    .o_btn(usb_buttons)
  );
    // assign led = usb_buttons[7:0] | gpio[7:0];
  end
  //else
    // assign led = {1'b0,btn};
  endgenerate

  assign btn_reset = usb_buttons[8] | user_reset;
  wire sys_reset;

  flash_loader flash_load_i (
    .clock(clock),
    .reset(sys_reset),
    .load_write_data(flash_loader_data_out),
    .data_valid(flash_loader_data_ready),

    // Flash load interface
    .flash_csn(flash_csn),
    .flash_sck(flash_sck),
    .flash_mosi(flash_mosi),
    .flash_miso(flash_miso)
  );
  assign sys_reset = R_reset[23];

  game_loader game_loader_i (
    .clk(clock),
    .reset(sys_reset),
    .indata(flash_loader_data_out),
    .indata_clk(flash_loader_data_ready),
    .mem_addr(game_loader_address),
    .mem_data(game_loader_mem),
    .mem_write(loader_write),
    .mapper_flags(mapper_flags),
    .done(load_done),
    .error(led[4]) // The last led will turn on if error loading game
  );

  reg loader_write_triggered = 1'b0;
  reg [7:0] loader_write_data_mem;
  reg [21:0] loader_addr_mem;
  reg loader_write_mem = 1'b0;

  wire [1:0] nes_ce;

  // loader_write -> clock when data available
  always @(posedge clock) begin
    if(loader_write) begin
      loader_write_triggered	<= 1'b1;
      loader_addr_mem		<= game_loader_address;
      loader_write_data_mem	<= game_loader_mem;
    end

    if(nes_ce == 3) begin
      loader_write_mem <= loader_write_triggered;
      if(loader_write_triggered)
        loader_write_triggered <= 1'b0;
    end
  end

  // reset after download
  reg [7:0] download_reset_cnt;
  wire download_reset = download_reset_cnt != 0;
  always @(posedge clock) begin
    if(!load_done)
      download_reset_cnt <= 8'd255;
    else if(load_done && download_reset_cnt != 0)
      download_reset_cnt <= download_reset_cnt - 8'd1;
  end

  // hold machine in reset until first download starts
  reg downloading = 0;
  reg init_reset = 1;
  always @(posedge clock) begin
    if (!downloading && flash_loader_data_ready) downloading <= 1'b1;
    if(downloading) init_reset <= 1'b0;
  end

// `define SDRAM_MIST

`ifndef SDRAM_MIST
  wire [15:0] sd_data_in;
  wire [15:0] sd_data_out;
  assign sdram_d = (!load_done ? !loader_write_mem : !memory_write) ? 16'hzzzz : sd_data_out;
  assign sd_data_in = sdram_d;
`endif

  // Always enable clock
  assign sdram_cke = 1'b1;
  assign sdram_clk = clock_sdram_chip; // Our shifted clock signal

  sdram sdram_i (
`ifdef SDRAM_MIST
   .SDRAM_DQ(sdram_d),
`else
   .sd_data_in(sd_data_in),
   .sd_data_out(sd_data_out),
`endif
   .sd_addr(sdram_a),
   .sd_dqm({sdram_dqm[1], sdram_dqm[0]}),
   .sd_cs(sdram_csn),
   .sd_ba(sdram_ba),
   .sd_we(sdram_wen),
   .sd_ras(sdram_rasn),
   .sd_cas(sdram_casn),
   // system interface
   .clk(clock_sdram_core),
   .clkref(nes_ce[1]),
`ifdef SDRAM_MIST
   .init_n(clock_locked),
`else
   .init(!clock_locked),
   .we_out(memory_write),
`endif

   // cpu/chipset interface
   .addrA     	   (!load_done ? {3'b000, loader_addr_mem} : {3'b000, memory_addr_cpu}),
   .addrB          ({3'b000, memory_addr_ppu} ),

   .weA            (!load_done ?  loader_write_mem : memory_write_cpu),
   .weB            (memory_write_ppu),

   .dinA           (!load_done ? loader_write_data_mem : memory_dout_cpu),
   .dinB           (memory_dout_ppu),

   .oeA            (memory_read_cpu),
   .doutA          (memory_din_cpu ),

   .oeB            (memory_read_ppu),
   .doutB          (memory_din_ppu )
  );

  reg reset_nes = 0;
  always @(posedge clock)
    reset_nes <= (!load_done || init_reset || download_reset || sys_reset);

  // select button is not functional
  // as we don't have any onboard buttons left on the board
  wire btn_select = 1'b0;

  reg last_joypad_clock;
  reg [7:0] joypad_bits;
  reg [1:0] buttons;

  reg [1:0] R_buttons;
  wire  joy_data, joy_strobe, joy_clock;
  generate
    if(use_external_nes_joypad)
    begin
      //assign gp[0] = 1'bz;
      assign joy_data = gp[0];
      //assign gp[1] = joy_strobe;
      //assign gp[2] = joy_clock;
      always @(posedge clock)
      begin
        if (joy_strobe || (!joy_clock && last_joypad_clock) )
          joypad_bits[0] <= !joy_data;
        last_joypad_clock <= joy_clock;
      end
    end
    else // use_external_nes_joypad == 0: control using USB or onboard buttons
    begin
      always @(posedge clock)
      begin
        R_buttons <= {btn_start, btn_a};
        if (joy_strobe)
          joypad_bits <= R_buttons | usb_buttons[7:0] | gpio[7:0];
        else
        begin
          if (!joy_clock && last_joypad_clock)
            joypad_bits <= {1'b0, joypad_bits[7:1]};
        end
        last_joypad_clock <= joy_clock;
      end
    end
  endgenerate

  wire [31:0] dbgadr;
  wire [2:0] dbgctr;

  NES nes_i (
   .clk(clock),
   .reset_nes(reset_nes),
   .sys_type(2'b00),
   .nes_div(nes_ce),
   .mapper_flags(mapper_flags),
   .sample(sample),
   .color(color),
   .joypad_strobe(joy_strobe),
   .joypad_clock(joy_clock),
   .joypad_data({3'b0, joypad_bits[0]}),
   .audio_channels(5'b11111),  // enable all channels
   .cpumem_addr(memory_addr_cpu),
   .cpumem_read(memory_read_cpu),
   .cpumem_din(memory_din_cpu),
   .cpumem_write(memory_write_cpu),
   .cpumem_dout(memory_dout_cpu),
   .ppumem_addr(memory_addr_ppu),
   .ppumem_read(memory_read_ppu),
   .ppumem_write(memory_write_ppu),
   .ppumem_din(memory_din_ppu),
   .ppumem_dout(memory_dout_ppu),
   .cycle(cycle),
   .scanline(scanline),
   .int_audio(1),
   .ext_audio(1)
  );

  wire blank;
  wire [7:0] r;
  wire [7:0] g;
  wire [7:0] b;
  wire vga_vs;
  wire vga_hs;

  vga vga_i (
    .I_CLK(clock),
    .I_CLK_VGA(clk_pixel),
    .I_COLOR(color),
    .I_HCNT(cycle),
    .I_VCNT(scanline),
    .O_HSYNC(vga_hs),
    .O_VSYNC(vga_vs),
    .O_BLANK(blank),
    .O_RED(r),
    .O_GREEN(g),
    .O_BLUE(b)
  );

  // VGA to digital video converter
  wire [1:0] tmds_clock, tmds_red, tmds_green, tmds_blue;
  vga2dvid #(
    .c_ddr(c_ddr?1'b1:1'b0),
    .c_shift_clock_synchronizer(1'b0)
  ) vga2dvid_instance (
    .clk_pixel(clk_pixel),
    .clk_shift(clk_shift),
    .in_red(r),
    .in_green(g),
    .in_blue(b),
    .in_hsync(vga_hs),
    .in_vsync(vga_vs),
    .in_blank(blank),
    .out_clock(tmds_clock),
    .out_red  (tmds_red  ),
    .out_green(tmds_green),
    .out_blue (tmds_blue )
  );

  generate
    if(c_ddr) begin
      // vendor specific DDR modules
      // convert SDR 2-bit input to DDR clocked 1-bit output (single-ended)
      // onboard GPDI
      ODDRX1F ddr0_clock (.D0(tmds_clock[0]), .D1(tmds_clock[1]), .Q(gpdi_dp[3]), .SCLK(clk_shift), .RST(0));
      ODDRX1F ddr0_red   (.D0(tmds_red  [0]), .D1(tmds_red  [1]), .Q(gpdi_dp[2]), .SCLK(clk_shift), .RST(0));
      ODDRX1F ddr0_green (.D0(tmds_green[0]), .D1(tmds_green[1]), .Q(gpdi_dp[1]), .SCLK(clk_shift), .RST(0));
      ODDRX1F ddr0_blue  (.D0(tmds_blue [0]), .D1(tmds_blue [1]), .Q(gpdi_dp[0]), .SCLK(clk_shift), .RST(0));
    end else begin
      assign gpdi_dp[3] = tmds_clock[0];
      assign gpdi_dp[2] = tmds_red  [0];
      assign gpdi_dp[1] = tmds_green[0];
      assign gpdi_dp[0] = tmds_blue [0];
    end
  endgenerate
endmodule

// BTN debouncer and signal generator for
// dynamic phase shift control
module btn_ecp5pll_phase
#(
  parameter c_debounce_bits = 16
)
(
  input  clk,         // 1-100 MHz
  input  inc, dec,    // BTNs
  output [7:0] phase, // counter for display
  output phasedir, phasestep, phaseloadreg
);
  reg [c_debounce_bits-1:0] R_debounce;
  reg  [1:0] R_btn, R_btn_prev;
  reg        R_new;
  always @(posedge clk)
  begin
    if(R_debounce[c_debounce_bits-1])
    begin
      if(R_btn != R_btn_prev)
      begin
        R_debounce <= 0;
        R_new <= 1;
      end
      R_btn[1] <= inc;
      R_btn[0] <= dec;
      R_btn_prev <= R_btn;
    end
    else
    begin
      R_debounce <= R_debounce + 1;
      R_new <= 0;
    end
  end
  wire S_inc = R_btn[1];
  wire S_dec = R_btn[0];

  reg R_phasedir, R_phasestep, R_phasestep_early;
  always @(posedge clk)
  begin
    if(R_new)
    begin
      R_phasedir        <= S_inc | S_dec ? S_inc : R_phasedir;
      R_phasestep_early <= S_inc | S_dec;
    end
    // phasedir must be stable 5ns before phasestep is pulsed
    R_phasestep <= R_phasestep_early;
  end
  assign phasedir     = R_phasedir;
  assign phasestep    = R_phasestep;
  assign phaseloadreg = 1'b0; // TODO support it
  
  reg [7:0] R_phase = 0;
  always @(posedge clk)
  begin
    if(R_phasestep_early == 1 && R_phasestep == 0)
      R_phase <= R_phasedir ? R_phase + 1 : R_phase - 1;
  end
  assign phase = R_phase; // for display
endmodule

module debouncer #(
    parameter N = 7
) (
    input  clk,
    input  btn,
    output sig
);
    logic [N-1:0] buffer;
    assign sig = buffer[N-1];

    always_ff @(posedge clk) begin
        if (!buffer[N-1] && btn) begin
            buffer <= buffer + 1;
        end else if (!btn) begin
            buffer <= 0;
        end
    end
endmodule
