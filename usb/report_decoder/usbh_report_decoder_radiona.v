// specific report decoder
// that converts radiona console USB joystick
// HID report to NES 8-bit button state

// For this core to work properly,
// set HID to accept strict report of exactly 8 bytes in top.v
// C_report_length=8, C_report_length_strict=1
// or maybe workaround:
// press central silver button toggles upper left LED marked "1"
// joystick will work reliably when upper left LED marked "1" is OFF
// if upper left LED marked "1" is ON, fake keypresses will be randomly generated

module usbh_report_decoder
#(
  parameter c_clk_hz=6000000,
  parameter c_autofire_hz=10
)
(
  input  wire        i_clk, // same as USB core clock domain
  input  wire [63:0] i_report,
  input  wire        i_report_valid,
  output reg   [8:0] o_btn
);

  localparam c_autofire_bits = $clog2(c_clk_hz/c_autofire_hz)-1;
  reg [c_autofire_bits-1:0] R_autofire;
  always @(posedge i_clk)
    R_autofire <= R_autofire + 1;

  wire [3:0] S_hat = i_report[43:40];
  reg  [3:0] R_hat_udlr;
  always @(posedge i_clk)
    R_hat_udlr <= S_hat == 4'b0000 ? 4'b1000 : // up
                  S_hat == 4'b0001 ? 4'b1001 : // up+right
                  S_hat == 4'b0010 ? 4'b0001 : // right
                  S_hat == 4'b0011 ? 4'b0101 : // down+right
                  S_hat == 4'b0100 ? 4'b0100 : // down
                  S_hat == 4'b0101 ? 4'b0110 : // down+left
                  S_hat == 4'b0110 ? 4'b0010 : // left
                  S_hat == 4'b0111 ? 4'b1010 : // up+left
                                     4'b0000 ; // 4'b1111 when not pressed

  // darfon/dragonrise joystick report decoder
  wire usbjoyl_l     =  (i_report[ 7:6 ] == 2'b00 ? 1'b1 : 1'b0);
  wire usbjoyl_r     =  (i_report[ 7:6 ] == 2'b11 ? 1'b1 : 1'b0);
  wire usbjoyl_u     =  (i_report[15:14] == 2'b00 ? 1'b1 : 1'b0);
  wire usbjoyl_d     =  (i_report[15:14] == 2'b11 ? 1'b1 : 1'b0);
  //wire usbjoyl_btn   =   i_report[54];
  //wire usbjoyr_l     =  (i_report[31:30] == 2'b00 ? 1'b1 : 1'b0);
  //wire usbjoyr_r     =  (i_report[31:30] == 2'b11 ? 1'b1 : 1'b0);
  //wire usbjoyr_u     =  (i_report[39:38] == 2'b00 ? 1'b1 : 1'b0);
  //wire usbjoyr_d     =  (i_report[39:38] == 2'b11 ? 1'b1 : 1'b0);
  //wire usbjoyr_btn   =   i_report[55];
  wire usbjoy_coin   =   i_report[44];
  wire usbjoy_play1  =   i_report[45];
  wire usbjoy_play2  =   i_report[46];
  wire usbjoy_red1   =   i_report[47];
  wire usbjoy_red2   =   i_report[48];
  wire usbjoy_red3   =   i_report[49];
  wire usbjoy_red4   =   i_report[50];
  wire usbjoy_blue1  =   i_report[51];
  wire usbjoy_blue2  =   i_report[52];
  wire usbjoy_blue3  =   i_report[53];
  wire usbjoy_blue4  =   i_report[54];
  
  wire usbjoy_a      =   usbjoy_blue1 | usbjoy_blue3;
  wire autofire_a    = ((usbjoy_red1  | usbjoy_red3) & R_autofire[c_autofire_bits-1]);
  wire usbjoy_b      =   usbjoy_blue2 | usbjoy_blue4;
  wire autofire_b    = ((usbjoy_red2  | usbjoy_red4) & R_autofire[c_autofire_bits-1]);
  wire usbjoy_start  =   usbjoy_play1;
  wire usbjoy_select =   usbjoy_play2;
  wire usbjoy_reset  =   usbjoy_coin;
  wire menu          =   usbjoy_red1 & usbjoy_red2 & usbjoy_red3 & usbjoy_red4;

  reg [8:0] R_btn;
  always @(posedge i_clk)
  begin
    o_btn <= R_btn | {7'b000000, autofire_b, autofire_a};
    if(i_report_valid)
      R_btn <=
      {
        usbjoy_reset,
        usbjoyl_r|menu,
        usbjoyl_l|menu,
        usbjoyl_d|menu,
        usbjoyl_u|menu,
        usbjoy_start, usbjoy_select, usbjoy_b, usbjoy_a
      };
  end

endmodule
