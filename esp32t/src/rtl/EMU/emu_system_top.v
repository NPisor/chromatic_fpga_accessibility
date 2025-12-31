// emu_system_top.v

module emu_system_top
(
    input               hclk,
    input               pclk,
    input               gclk,
    input               reset_n,
    input               POWER_GOOD,
    
    input               customPaletteEna,
    input       [63:0]  paletteBGIn,
    input       [63:0]  paletteOBJ0In,
    input       [63:0]  paletteOBJ1In,
    input               paletteOff,
    input       [2:0]   gbc_color_temp,
    output              gbc_mode,
    output      [63:0]  gpd,
    
    input               BTN_NODIAGONAL,
    input               BTN_A,
    input               BTN_B,
    input               BTN_DPAD_DOWN,
    input               BTN_DPAD_LEFT,
    input               BTN_DPAD_RIGHT,
    input               BTN_DPAD_UP,
    input               BTN_MENU,
    input               BTN_SEL,
    input               BTN_START,
    input               MENU_CLOSED,
    output              boot_rom_enabled,
    output  [15:0]      CART_A,
    output              CART_CLK,
    output              CART_CS,
    inout   [7:0]       CART_D,
    output              CART_RD,
    inout               CART_RST,
    output              CART_WR,
    output              CART_DATA_DIR_E,

    input               IR_RX,
    output              IR_LED,

    inout               LINK_CLK,
    input               LINK_IN,
    output              LINK_OUT,

    output [15:0]       left,
    output [15:0]       right,

    output lcd_on_int,
    output lcd_off_overwrite,

    input               gg_reset,
    input               gg_en,
    input  [128:0]      gg_code,

    // Peek/poke interface from system_monitor (gClk domain)
    input               peek_req,
    input               peek_we,
    input  [15:0]       peek_addr,
    input  [7:0]        peek_wdata,
    output              peek_busy,
    output              peek_data_valid,
    output [7:0]        peek_data,
    output [1:0]        peek_status,

    output reg [31:0]   game_hash,
    output reg          game_hash_valid,
    
    input               LCD_INIT_DONE,
    output              gb_lcd_clkena,
    output [14:0]       gb_lcd_data,
    output [1:0]        gb_lcd_mode,
    output              gb_lcd_on,
    output              gb_lcd_vsync
    
);

    parameter SRSIZE = 15;

    reg [SRSIZE-1:0] BTN_DPAD_DOWN_sr;
    reg [SRSIZE-1:0] BTN_DPAD_UP_sr;
    reg [SRSIZE-1:0] BTN_DPAD_LEFT_sr;
    reg [SRSIZE-1:0] BTN_DPAD_RIGHT_sr;

    reg [SRSIZE-1:0] BTN_START_sr;
    reg [SRSIZE-1:0] BTN_SEL_sr;
    reg [SRSIZE-1:0] BTN_B_sr;
    reg [SRSIZE-1:0] BTN_A_sr;

    reg BTN_DPAD_DOWN_filtered;
    reg BTN_DPAD_UP_filtered;
    reg BTN_DPAD_LEFT_filtered;
    reg BTN_DPAD_RIGHT_filtered;
    reg BTN_START_filtered;
    reg BTN_SEL_filtered;
    reg BTN_B_filtered;
    reg BTN_A_filtered;
    
    reg BTN_DPAD_DOWN_filtered_dir;
    reg BTN_DPAD_UP_filtered_dir;
    reg BTN_DPAD_LEFT_filtered_dir;
    reg BTN_DPAD_RIGHT_filtered_dir;

    always@(posedge pclk)
    begin
        BTN_DPAD_DOWN_sr <= {BTN_DPAD_DOWN_sr[SRSIZE-2:0], BTN_DPAD_DOWN&~BTN_MENU&MENU_CLOSED};
        BTN_DPAD_UP_sr <= {BTN_DPAD_UP_sr[SRSIZE-2:0], BTN_DPAD_UP&~BTN_MENU&MENU_CLOSED};
        BTN_DPAD_LEFT_sr <= {BTN_DPAD_LEFT_sr[SRSIZE-2:0], BTN_DPAD_LEFT&~BTN_MENU&MENU_CLOSED};
        BTN_DPAD_RIGHT_sr <= {BTN_DPAD_RIGHT_sr[SRSIZE-2:0], BTN_DPAD_RIGHT&~BTN_MENU&MENU_CLOSED};
        BTN_START_sr <= {BTN_START_sr [SRSIZE-2:0], BTN_START&~BTN_MENU&MENU_CLOSED};
        BTN_SEL_sr <= {BTN_SEL_sr [SRSIZE-2:0], BTN_SEL&~BTN_MENU&MENU_CLOSED};
        BTN_A_sr <= {BTN_A_sr [SRSIZE-2:0], BTN_A&~BTN_MENU&MENU_CLOSED};
        BTN_B_sr <= {BTN_B_sr [SRSIZE-2:0], BTN_B&~BTN_MENU&MENU_CLOSED};

        BTN_DPAD_DOWN_filtered <= &BTN_DPAD_DOWN_sr[SRSIZE-1:1];
        BTN_DPAD_UP_filtered <= &BTN_DPAD_UP_sr[SRSIZE-1:1];
        BTN_DPAD_LEFT_filtered <= &BTN_DPAD_LEFT_sr[SRSIZE-1:1];
        BTN_DPAD_RIGHT_filtered <= &BTN_DPAD_RIGHT_sr[SRSIZE-1:1];
        BTN_START_filtered <= &BTN_START_sr[SRSIZE-1:1];
        BTN_SEL_filtered <= &BTN_SEL_sr[SRSIZE-1:1];
        BTN_A_filtered <= &BTN_A_sr[SRSIZE-1:1];
        BTN_B_filtered <= &BTN_B_sr[SRSIZE-1:1];
        
        if (BTN_NODIAGONAL) begin
            BTN_DPAD_DOWN_filtered_dir  <= BTN_DPAD_DOWN_filtered  & ~BTN_DPAD_UP_filtered & ~BTN_DPAD_LEFT_filtered_dir & ~BTN_DPAD_RIGHT_filtered_dir;
            BTN_DPAD_UP_filtered_dir    <= BTN_DPAD_UP_filtered    & ~BTN_DPAD_DOWN_filtered & ~BTN_DPAD_LEFT_filtered_dir & ~BTN_DPAD_RIGHT_filtered_dir;
            BTN_DPAD_LEFT_filtered_dir  <= BTN_DPAD_LEFT_filtered  & ~BTN_DPAD_RIGHT_filtered & ~BTN_DPAD_UP_filtered_dir & ~BTN_DPAD_DOWN_filtered_dir;
            BTN_DPAD_RIGHT_filtered_dir <= BTN_DPAD_RIGHT_filtered & ~BTN_DPAD_LEFT_filtered & ~BTN_DPAD_UP_filtered_dir & ~BTN_DPAD_DOWN_filtered_dir;
        end else begin
            BTN_DPAD_DOWN_filtered_dir  <= BTN_DPAD_DOWN_filtered  & ~BTN_DPAD_UP_filtered;
            BTN_DPAD_UP_filtered_dir    <= BTN_DPAD_UP_filtered    & ~BTN_DPAD_DOWN_filtered;
            BTN_DPAD_LEFT_filtered_dir  <= BTN_DPAD_LEFT_filtered  & ~BTN_DPAD_RIGHT_filtered;
            BTN_DPAD_RIGHT_filtered_dir <= BTN_DPAD_RIGHT_filtered & ~BTN_DPAD_LEFT_filtered;
        end
        
    end

    wire [3:0] btn_key = {
        BTN_DPAD_DOWN_filtered_dir, 
        BTN_DPAD_UP_filtered_dir,
        BTN_DPAD_LEFT_filtered_dir, 
        BTN_DPAD_RIGHT_filtered_dir
    }; 
     
    wire [3:0] dpad_key = {
        BTN_START_filtered,
        BTN_SEL_filtered,
        BTN_B_filtered,
        BTN_A_filtered
    };

    reg [3:0] joy_din;
    wire [1:0] joy_p54;
    always@(posedge hclk)
    begin
        case(joy_p54)
            2'b00: joy_din <= (~btn_key) & (~dpad_key); 
            2'b01: joy_din <= ~dpad_key;
            2'b10: joy_din <= ~btn_key; 
            2'b11: joy_din <= 4'hF;
        endcase
    end

    wire wr;
    wire rd;
    wire [7:0] CART_DOUT;
    wire nCS;

    wire [7:0]  CART_DIN; 
    assign CART_DIN = CART_D;

    wire cpu_speed;
    wire cpu_halt;
    wire cpu_stop;
    
    wire [7:0] CART_DIN_r1;
    wire [15:0] a;
    wire [2:0] TSTATEo;
    wire sleep_savestate;

    speedcontrol u_speedcontrol
    (
        .clk_sys     (hclk),
        .pause       (sleep_savestate),
        .speedup     (),
        .cart_act    (rd | wr),
        .DMA_on      (),
        .ce          (ce),
        .ce_2x       (ce_2x),
        .refresh     (),
        .ff_on       ()
    );

    wire sel_cram = a[15:13] == 3'b101;           // 8k cart ram at $a000
    wire cart_oe = (rd & ~a[15]) | (sel_cram & rd);

    assign CART_RST = 1'bZ;

    reg gbreset;
    reg gbreset_ungated;
    reg CART_RST_r1;
    reg CART_RST_r2;
    reg ce_2x_r1;
    always@(posedge hclk)
        ce_2x_r1 <= ce_2x;
        
    always@(posedge hclk or negedge reset_n)
    begin
        if(~reset_n)
        begin
            gbreset <= 1'd1;
            gbreset_ungated <= 1'd1;
        end
        else
        begin
            CART_RST_r1 <= CART_RST;
            CART_RST_r2 <= CART_RST_r1;
            gbreset_ungated <= ~LCD_INIT_DONE ? 1'b1 : ~CART_RST_r2;
            if(~ce_2x_r1 & ce_2x & ce)
                gbreset <= gbreset_ungated;
                
            if (MENU_CLOSED & BTN_MENU & BTN_A & BTN_B & BTN_START & BTN_SEL) gbreset <= 1'd1;
            
            if (~POWER_GOOD) gbreset <= 1'b1;
        end
    end
    
    wire DMA_on;
    wire hdma_active;

    // Synchronize cheat control from system_monitor (gClk) into hclk domain
    reg        gg_reset_meta;
    reg        gg_reset_hclk;
    reg        gg_en_meta;
    reg        gg_en_hclk;
    reg [1:0]   gg_code_toggle_sync;
    reg [127:0] gg_code_payload_meta;
    reg [127:0] gg_code_payload_hclk;
    reg         gg_code_pulse_hclk;
    reg [128:0] gg_code_hclk;

    // Peek CDC: gclk (system_monitor) -> hclk -> GB
    reg  peek_req_meta;
    reg  peek_req_sync;
    reg  peek_req_sync_d;
    reg  peek_req_pulse;
    reg  peek_we_hclk;
    reg [15:0] peek_addr_hclk;
    reg [7:0]  peek_wdata_hclk;
    wire       peek_req_hclk   = peek_req_pulse;
    wire       peek_busy_hclk;
    wire       peek_data_valid_hclk;
    wire [7:0] peek_data_hclk;
    wire [1:0] peek_status_hclk;
    always @(posedge hclk or negedge reset_n) begin
        if(~reset_n) begin
            peek_req_meta        <= 1'b0;
            peek_req_sync        <= 1'b0;
            peek_req_sync_d      <= 1'b0;
            peek_req_pulse       <= 1'b0;
            peek_we_hclk         <= 1'b0;
            peek_addr_hclk       <= 16'd0;
            peek_wdata_hclk      <= 8'd0;
            gg_reset_meta        <= 1'b0;
            gg_reset_hclk        <= 1'b0;
            gg_en_meta           <= 1'b0;
            gg_en_hclk           <= 1'b0;
            gg_code_toggle_sync  <= 2'b00;
            gg_code_payload_meta <= 128'd0;
            gg_code_payload_hclk <= 128'd0;
            gg_code_pulse_hclk   <= 1'b0;
            gg_code_hclk         <= 129'd0;
        end else begin
            // two-flop sync for reset/enable
            gg_reset_meta  <= gg_reset;
            gg_reset_hclk  <= gg_reset_meta;
            gg_en_meta     <= gg_en;
            gg_en_hclk     <= gg_en_meta;

            // peek request sync and one-shot
            peek_req_meta   <= peek_req;
            peek_req_sync   <= peek_req_meta;
            peek_req_sync_d <= peek_req_sync;
            peek_req_pulse  <= 1'b0;
            if (peek_req_sync & ~peek_req_sync_d) begin
                peek_req_pulse  <= 1'b1;
                peek_we_hclk    <= peek_we;
                peek_addr_hclk  <= peek_addr;
                peek_wdata_hclk <= peek_wdata;
            end

            // shallow sync for payload bus to reduce metastability window
            gg_code_payload_meta <= gg_code[127:0];
            gg_code_payload_hclk <= gg_code_payload_meta;

            // toggle-based strobe sync on bit 128
            gg_code_toggle_sync <= {gg_code_toggle_sync[0], gg_code[128]};
            if (gg_code_toggle_sync[1] ^ gg_code_toggle_sync[0]) begin
                // payload already sampled into gg_code_payload_hclk above
                gg_code_pulse_hclk   <= 1'b1;           // one-cycle pulse for CODES edge detect
            end else begin
                gg_code_pulse_hclk   <= 1'b0;
            end
            gg_code_hclk <= {gg_code_pulse_hclk, gg_code_payload_hclk};
        end
    end

    assign peek_busy       = peek_busy_hclk;
    assign peek_data_valid = peek_data_valid_hclk;
    assign peek_data       = peek_data_hclk;
    assign peek_status     = peek_status_hclk;

    // Peek/poke is not implemented in the current GB core; respond immediately with an
    // unsupported status so system_monitor can complete the transaction.
    assign peek_busy_hclk        = 1'b0;
    assign peek_data_valid_hclk  = peek_req_hclk;
    assign peek_data_hclk        = 8'h00;
    assign peek_status_hclk      = 2'b11;
    cart u_cart
    (
       .hclk            (hclk           ),
       .pclk            (pclk           ),
       .ce              (ce             ),
       .ce_2x           (ce_2x          ),
       .gbreset         (gbreset        ),
       .cpu_speed       (cpu_speed      ),
       .cpu_halt        (cpu_halt       ),
       .cpu_stop        (cpu_stop       ),
       .wr              (wr             ),
       .rd              (rd             ),
       .a               (a              ),
       .CART_DOUT       (CART_DOUT      ),
       .nCS             (nCS            ),
       .TSTATEo         (TSTATEo        ),
       .DMA_on          (DMA_on         ),
       .hdma_active     (hdma_active    ),
                                        
       .CART_A          (CART_A         ),
       .CART_CLK        (CART_CLK       ),
       .CART_CS         (CART_CS        ),
       .CART_D          (CART_D         ),
       .CART_RD         (CART_RD        ),
       .CART_WR         (CART_WR        ),
       .CART_DATA_DIR_E (CART_DATA_DIR_E),
       .CART_DIN_r1     (CART_DIN_r1    )
    );

    wire sc_int_clock2;
    wire serial_clk_in;
    wire serial_clk_out;
    wire serial_data_out;
    
    reg LINK_IN_r1;
    reg LINK_CLK_r1;
    reg serial_clk_out_r1;
    reg sc_int_clock2_r1;
    reg serial_data_out_r1;
    always@(posedge hclk)
    begin
        LINK_IN_r1         <= LINK_IN;
        LINK_CLK_r1        <= LINK_CLK;
        serial_clk_out_r1  <= serial_clk_out;
        sc_int_clock2_r1   <= sc_int_clock2;
        serial_data_out_r1 <= serial_data_out;
    end

    assign LINK_CLK = sc_int_clock2_r1 ? serial_clk_out_r1 : 1'bZ;
    assign serial_clk_in = LINK_CLK_r1;

    wire serial_data_in = LINK_IN_r1;
    assign LINK_OUT = serial_data_out_r1;

    wire [63:0] SAVE_out_Din  ;
    wire [63:0] SAVE_out_Dout ;
    wire [25:0] SAVE_out_Adr  ;
    wire SAVE_out_rnw  ;                    
    wire SAVE_out_ena  ;                                    
    wire SAVE_out_done ; 

    // Synchronize gbreset into pclk for hash generation
    reg gbreset_meta_pclk;
    reg gbreset_sync_pclk;
    always @(posedge pclk or negedge reset_n) begin
        if (~reset_n) begin
            gbreset_meta_pclk <= 1'b1;
            gbreset_sync_pclk <= 1'b1;
        end else begin
            gbreset_meta_pclk <= gbreset;
            gbreset_sync_pclk <= gbreset_meta_pclk;
        end
    end

    localparam [31:0] FNV_OFFSET = 32'h811C9DC5;
    localparam [31:0] FNV_PRIME  = 32'h01000193;

    reg [15:0] header_seen;
    wire       header_addr   = (a[15:8] == 8'h01) && (a[7:0] >= 8'h34) && (a[7:0] <= 8'h43);
    wire [4:0] header_offset = a[7:0] - 8'h34;

    always @(posedge pclk or negedge reset_n) begin
        if (~reset_n) begin
            game_hash       <= FNV_OFFSET;
            header_seen     <= 16'd0;
            game_hash_valid <= 1'b0;
        end else begin
            if (gbreset_sync_pclk) begin
                game_hash       <= FNV_OFFSET;
                header_seen     <= 16'd0;
                game_hash_valid <= 1'b0;
            end else if (header_addr && rd && ~wr && ~game_hash_valid && ~header_seen[header_offset]) begin
                // Build a 32-bit FNV-1a hash across the 16 title bytes (0x0134-0x0143).
                header_seen[header_offset] <= 1'b1;
                game_hash <= (game_hash ^ CART_DIN_r1) * FNV_PRIME;

                if (&(header_seen | (16'd1 << header_offset))) begin
                    game_hash_valid <= 1'b1;
                end
            end
        end
    end
    
    reg ss_load = 1'b0;
    reg gbreset_1 = 1'b0;
    
    // synthesis translate_off
    always@(posedge hclk)
    begin
      gbreset_1 <= gbreset;
      ss_load   <= 1'b0;
      if (gbreset_1 && ~gbreset) ss_load <= 1'b1;
    end
    // synthesis translate_on

    wire [15:0] snd_l;  
    wire [15:0] snd_r;  

    gb u_gb(
        .reset(gbreset),

        .clk_sys(hclk),
        .ce(ce),
        .ce_2x(ce_2x),

        .isGBC(1'd1),
        .real_cgb_boot(1'd0),
        .customPaletteEna(customPaletteEna),
        .paletteBGIn(paletteBGIn),
        .paletteOBJ0In(paletteOBJ0In),
        .paletteOBJ1In(paletteOBJ1In),
        .paletteOff(paletteOff),
        .gbc_color_temp(gbc_color_temp),
        .gbc_mode(gbc_mode),
        .gpd(gpd),

        // cartridge interface
        // can adress up to 1MB ROM
        .ext_bus_addr(a[14:0]),
        .ext_bus_a15(a[15]),
        .cart_rd(rd),
        .cart_wr(wr),
        .cart_do(CART_DIN_r1),
        .cart_di(CART_DOUT),
        .cart_oe(cart_oe),
        .TSTATEo(TSTATEo),
        .TSTATE1(TSTATE1),
        .TSTATE2(TSTATE2),
        .TSTATE3(TSTATE3),
        .TSTATE4(TSTATE4),
        // WRAM or Cart RAM CS
        .nCS(nCS),

        .cgb_boot_download(1'd0),
        .dmg_boot_download(1'd0),
        .ioctl_wr(1'd0),
        .ioctl_addr(25'd0),
        .ioctl_dout(16'd0),

        // Bootrom features
        .boot_rom_enabled(boot_rom_enabled),

        .boot_gba_en(1'd0),
        .fast_boot_en(1'd0),
        // audio
        .audio_l(snd_l),
        .audio_r(snd_r),

        // Megaduck?
        .megaduck(1'd0),

        .IR_RX(IR_RX),
        .IR_LED(IR_LED),

        // lcd interface
        .lcd_clkena(gb_lcd_clkena),
        .lcd_data(gb_lcd_data),
        //      .lcd_data_gb(),
        .lcd_mode(gb_lcd_mode),
        .lcd_on(gb_lcd_on),
        .lcd_vsync(gb_lcd_vsync),

        .joy_p54(joy_p54),
        .joy_din(joy_din),

        .lcd_on_int(lcd_on_int),
        .lcd_off_overwrite(lcd_off_overwrite),

        .speed(cpu_speed),   //GBC
        .cpu_stop(cpu_stop),
        .cpu_halt(cpu_halt),
        .DMA_on(DMA_on),
        .hdma_active(hdma_active),
        .gg_reset(gg_reset_hclk),
        .gg_en(gg_en_hclk),
        .gg_code(gg_code_hclk),
        .gg_available(),

        //serial port
        .sc_int_clock2(sc_int_clock2),
        .serial_clk_in(serial_clk_in),
        .serial_clk_out(serial_clk_out),
        .serial_data_in(serial_data_in),
        .serial_data_out(serial_data_out),

        // savestates
        .increaseSSHeaderCount(1'd0),
        .cart_ram_size(8'd0),
        .save_state(1'd0),
        .load_state(ss_load),
        .savestate_number(2'd0),
        .sleep_savestate(sleep_savestate),

        .SaveStateExt_Din(), 
        .SaveStateExt_Adr(), 
        .SaveStateExt_wren(),
        .SaveStateExt_rst(), 
        .SaveStateExt_Dout(64'd0),
        .SaveStateExt_load(),

        .Savestate_CRAMAddr(),     
        .Savestate_CRAMRWrEn(),    
        .Savestate_CRAMWriteData(),
        .Savestate_CRAMReadData(8'd0),

        .SAVE_out_Din  (SAVE_out_Din ),    // data read from savestate
        .SAVE_out_Dout (SAVE_out_Dout),  // data written to savestate
        .SAVE_out_Adr  (SAVE_out_Adr ),    // all addresses are DWORD addresses!
        .SAVE_out_rnw  (SAVE_out_rnw ),     // read = 1, write = 0
        .SAVE_out_ena  (SAVE_out_ena ),     // one cycle high for each action
        .SAVE_out_done (SAVE_out_done),    // should be one cycle high when write is done or read value is valid

        .rewind_on(1'd0),
        .rewind_active(1'd0)
    );
    
    audio_filter u_audio_filter
    (
       .reset    (~reset_n),
       .clk      (hclk),

       .core_l   (snd_l),
       .core_r   (snd_r),

       .filter_l (left),
       .filter_r (right)
    );
    
// synthesis translate_off
   wire DDRAM_CLK            ;
   wire DDRAM_BUSY           ;
   wire [7:0]  DDRAM_BURSTCNT;
   wire [28:0] DDRAM_ADDR    ;
   wire [63:0] DDRAM_DOUT    ;
   wire DDRAM_DOUT_READY     ;
   wire DDRAM_RD             ;
   wire [63:0] DDRAM_DIN     ;   
   wire [7:0]  DDRAM_BE      ;   
   wire DDRAM_WE             ;

   wire [27:1] ch1_addr;         
   wire [63:0] ch1_dout;         
   wire [63:0] ch1_din ;         
   wire [7:0]  ch1_be  ;         
   wire ch1_req        ;  
   wire ch1_rnw        ;  
   wire ch1_ready      ;  
    
   ddram iddram
   (
      .DDRAM_CLK        (hclk),      
      .DDRAM_BUSY       (DDRAM_BUSY),      
      .DDRAM_BURSTCNT   (DDRAM_BURSTCNT),  
      .DDRAM_ADDR       (DDRAM_ADDR),      
      .DDRAM_DOUT       (DDRAM_DOUT),      
      .DDRAM_DOUT_READY (DDRAM_DOUT_READY),
      .DDRAM_RD         (DDRAM_RD),        
      .DDRAM_DIN        (DDRAM_DIN),       
      .DDRAM_BE         (DDRAM_BE),        
      .DDRAM_WE         (DDRAM_WE),                
                
      .ch1_addr         (ch1_addr),        
      .ch1_dout         (ch1_dout),        
      .ch1_din          (ch1_din),  
      .ch1_be           (ch1_be),      
      .ch1_req          (ch1_req),         
      .ch1_rnw          (ch1_rnw),         
      .ch1_ready        (ch1_ready)
   );
   
   assign ch1_addr      = { SAVE_out_Adr[25:0], 1'b0 };
   assign ch1_din       = SAVE_out_Din;
   assign ch1_req       = SAVE_out_ena;
   assign ch1_rnw       = SAVE_out_rnw;
   assign ch1_be        = 8'hFF; // only required for increaseSSHeaderCount
   assign SAVE_out_Dout = ch1_dout;
   assign SAVE_out_done = ch1_ready;
   
   ddrram_model iddrram_model
   (
      .DDRAM_CLK        (hclk),      
      .DDRAM_BUSY       (DDRAM_BUSY),      
      .DDRAM_BURSTCNT   (DDRAM_BURSTCNT),  
      .DDRAM_ADDR       (DDRAM_ADDR),      
      .DDRAM_DOUT       (DDRAM_DOUT),      
      .DDRAM_DOUT_READY (DDRAM_DOUT_READY),
      .DDRAM_RD         (DDRAM_RD),        
      .DDRAM_DIN        (DDRAM_DIN),       
      .DDRAM_BE         (DDRAM_BE),        
      .DDRAM_WE         (DDRAM_WE)       
   );
// synthesis translate_on
    
endmodule
