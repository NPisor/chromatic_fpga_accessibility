// system_monitor.v

module system_monitor(
    input               clk,
    input               reset,
    input               BTN_A,
    input               BTN_B,
    input               BTN_DPAD_DOWN,
    input               BTN_DPAD_LEFT,
    input               BTN_DPAD_RIGHT,
    input               BTN_DPAD_UP,
    input               BTN_MENU, // pressed = 0
    input               BTN_SEL,
    input               BTN_START,
    // Controls external mux into ADC
    output  reg         menuDisabled,
    output  reg         ADC_SEL,
    output  reg         hAdcReq_ext,
    input               LCD_INIT_DONE,
    output  reg         LCD_PWM,
    output  reg         LCD_BACKLIGHT_INIT,
    input               hAdcReady_r1,
    input   [13:0]      hAdcValue_r1,
    input   [8:0]       hButtons,
    output  reg [8:0]   MCU_buttons,
    input   [6:0]       hVolume,
    input   [7:0]       pmic_sys_status,
    input               hHeadphones,
    input               gSecondEna,
    input               gHalfSecondEna,
    output  reg         low_battery,
    output  reg         LED_Green,
    output  reg         LED_Red,
    output  reg         LED_Yellow,
    output  reg         LED_White,
    output  reg [15:0]  system_control,
    output  reg [31:0]  debug_system,
    output  reg [63:0]  paletteBGIn,
    output  reg [63:0]  paletteOBJ0In,
    output  reg [63:0]  paletteOBJ1In,
    output  reg [2:0]   gbc_color_temp,
    input               gbc_mode,
    input   [63:0]      gpd,
    input   [7:0]       uart_rx_data,
    input               uart_rx_val,
    input               uart_tx_busy,
    output  [7:0]       uart_tx_data,
    output              uart_tx_val,

    input  [31:0]       game_hash,
    input               game_hash_valid,


    output  reg         gg_reset,
    output  reg         gg_en,
    output  reg [128:0] gg_code,

    // Memory peek/poke handshake with GB core
    output  reg         peek_req,
    output  reg         peek_we,
    output  reg [15:0]  peek_addr,
    output  reg [7:0]   peek_wdata,
    input               peek_busy,
    input               peek_data_valid,
    input       [7:0]   peek_data,
    input       [1:0]   peek_status
    // Snapshot state indicator removed
);

    wire    [6:0]   rx_address;
    wire    [79:0]  rx_data;
    wire    [7:0]   rx_payload_len;
    wire            rx_data_val;

    reg [15:0] btnMenu_sr;
    reg btnMenu_r1;
    reg btnMenu_r2;
    
    reg [15:0] btnDown_sr;
    reg btnDown_r1;
    reg btnDown_r2;
    
    reg [15:0] btnUp_sr;
    reg btnUp_r1;
    reg btnUp_r2;
    
    reg [15:0] btnLeft_sr;
    reg btnLeft_r1;
    reg btnLeft_r2;
    
    reg [15:0] btnRight_sr;
    reg btnRight_r1;
    reg btnRight_r2;
    
    reg btnStart_r1;
    reg btnStart_r2;    
    reg btnSelect_r1;
    reg btnSelect_r2;    
    reg btnA_r1;
    reg btnA_r2;
    reg btnB_r1;
    reg btnB_r2;

    reg pressed;
    reg [3:0] brightness = 4'd3;
    reg [7:0] cheat_enable_mask;
    reg [128:0] gg_code_latch;
    reg [1:0] blockBrightnessReceive;

    reg [7:0] peek_slot_latch;
    reg       peek_inflight;
    reg       peek_resp_ready;
    reg [7:0] peek_resp_status;
    reg [15:0] peek_resp_addr;
    reg [7:0] peek_resp_data;
    reg       peek_is_write;
    
    reg request_buttons  = 1'b0;
    reg request_version  = 1'b0;
    reg updateBrightness = 1'b0;
    reg request_gpd      = 1'b0;
    
    reg lowpowerBacklight = 1'b0;
    reg [3:0] lowerpowerOldBL;
    reg request_SystemStatusExtended = 1'b0;
    reg game_hash_sent;

    reg [13:0] volt;
    wire       bat_is_LI;
    reg        gbc_mode_r;
    
    always@(posedge clk or posedge reset)
    begin
        if(reset) begin
            system_control     <= 16'd1;
            MCU_buttons        <= 9'd0;  
            gg_reset           <= 1'd1;
            gg_en              <= 1'd0;
            gg_code            <= 129'd0;
            LCD_BACKLIGHT_INIT <= 1'd0;
            gg_code_latch      <= 129'd0;
            cheat_enable_mask  <= 8'd0;
            game_hash_sent     <= 1'b0;
            peek_inflight      <= 1'b0;
            peek_resp_ready    <= 1'b0;
            peek_req           <= 1'b0;
            peek_we            <= 1'b0;
            peek_addr          <= 16'd0;
            peek_wdata         <= 8'd0;
            peek_resp_status   <= 8'd0;
            peek_resp_addr     <= 16'd0;
            peek_resp_data     <= 8'd0;
            peek_is_write      <= 1'b0;
            request_gpd        <= 1'b0;
            paletteBGIn        <= 64'd0;
            paletteOBJ0In      <= 64'd0;
            paletteOBJ1In      <= 64'd0;
            gbc_color_temp     <= 3'd2; // neutral default
            gbc_mode_r         <= 1'b0;
        end else begin
            gg_reset <= 1'd0;
            gg_code_latch[128] <= 1'b0; // clear strobe by default so each payload pulses high
            request_buttons              <= 1'b0;
            request_version              <= 1'b0;
            updateBrightness             <= 1'b0;
            request_SystemStatusExtended <= 1'b0;
            peek_req                     <= 1'b0;
            if (~game_hash_valid)
                game_hash_sent <= 1'b0;

            if (gbc_mode_r != gbc_mode) begin
                gbc_mode_r <= gbc_mode;
                request_SystemStatusExtended <= 1'b1;
            end
            
                if (gHalfSecondEna) begin
                    LCD_BACKLIGHT_INIT  <= 1'd1;
                end
            
            if(rx_data_val) begin
                // Decode cheat payload when present
                if(rx_address == 7'd14) begin
                    // Payload ordering (len 6): [slot][type+en][addr_hi][addr_lo][value][compare]
                    // Stored MSB-first in rx_data shift register.
                    reg [7:0] cheat_slot;
                    reg [7:0] cheat_type_byte;
                    reg [7:0] cheat_addr_hi;
                    reg [7:0] cheat_addr_lo;
                    reg [7:0] cheat_value;
                    reg [7:0] cheat_compare;

                    cheat_slot      = rx_data[47:40];
                    cheat_type_byte = rx_data[39:32];
                    // MCU sends addr_hi then addr_lo; keep that order for GB core
                    cheat_addr_hi   = rx_data[31:24];
                    cheat_addr_lo   = rx_data[23:16];
                    cheat_value     = rx_data[15:8];
                    cheat_compare   = rx_data[7:0];

                    gg_code_latch[128]    <= 1'b1; // strobe high for this payload
                    gg_code_latch[127:98] <= 30'd0;
                    gg_code_latch[97]     <= cheat_type_byte[7]; // enable bit follows MCU payload
                    gg_code_latch[96]     <= ((cheat_type_byte[6:0] == 7'h91) && (cheat_compare != cheat_value)); // enable compare only when a distinct compare byte is supplied
                    gg_code_latch[95:80]  <= 16'd0; // upper address bits unused
                    gg_code_latch[79:64]  <= {cheat_addr_hi, cheat_addr_lo};
                    gg_code_latch[63:40]  <= 24'd0;
                    gg_code_latch[39:32]  <= cheat_compare;
                    gg_code_latch[31:8]   <= 24'd0;
                    gg_code_latch[7:0]    <= cheat_value;

                    cheat_enable_mask[cheat_slot[2:0]] <= cheat_type_byte[7];
                end

                if ((rx_address == 7'd17) && (rx_payload_len == 8'd5)) begin
                    // Peek/poke request (len 5): [slot][flags][addr_hi][addr_lo][value]
                    // flags bit0: write/poke when set; 0=peek
                    if (~peek_inflight && ~peek_busy)
                    begin
                        peek_slot_latch <= rx_data[39:32];
                        peek_is_write   <= rx_data[24];
                        peek_addr       <= {rx_data[23:16], rx_data[15:8]};
                        peek_wdata      <= rx_data[7:0];
                        peek_req        <= 1'b1;
                        peek_we         <= rx_data[24];
                        peek_inflight   <= 1'b1;
                    end
                end

                if ((rx_address == 7'd17) && (rx_payload_len == 8'd2)) begin
                    // GBC color temperature (len 2). MCU sends level in first payload byte (big-endian);
                    // fallback to second byte if first is zero and second is nonzero to tolerate any ordering mismatches.
                    gbc_color_temp <= (rx_data[15:8] != 3'd0 || rx_data[7:0] == 3'd0) ? rx_data[15:8] : rx_data[7:0];
                end

                if(rx_address == 7'd13) begin
                    // Game palette data request (len 0)
                    request_gpd <= 1'b1;
                end
                if(rx_address == 7'd12) begin
                    // OBJ palette payload: bit63 selects OBJ1 when set, otherwise OBJ0
                    if (rx_data[63]) begin
                        paletteOBJ1In <= rx_data[63:0];
                    end else begin
                        paletteOBJ0In <= rx_data[63:0];
                    end
                end
                if(rx_address == 7'd11) begin
                    // BG palette payload
                    paletteBGIn <= rx_data[63:0];
                end
                if(rx_address == 7'd9) begin
                    MCU_buttons <= rx_data[8:0];
                end
                if(rx_address == 7'd6) begin
                    request_version <= 1'b1;
                end
                if(rx_address == 7'd5) begin
                    if (blockBrightnessReceive == 2'd0) begin
                        brightness      <=  rx_data[13:0];
                    end else begin
                        blockBrightnessReceive <= blockBrightnessReceive - 1;
                    end
                end
                if(rx_address == 7'd4) begin
                    system_control  <=  rx_data[15:0];
                end
                if(rx_address == 7'd2) begin
                    request_buttons <= 1'b1;
                end
            end
                
            if (menuDisabled) begin     
                if((btnLeft_sr[15:0] == 16'h8000)&&~btnMenu_r2) begin
                    if(brightness >= 1)
                    begin
                        brightness <= brightness - 9'd1;
                        pressed <= 1'd0;
                        blockBrightnessReceive <= 2'd3;
                        updateBrightness <= 1'b1;
                    end
                end
                if((btnRight_sr[15:0] == 16'h8000)&&~btnMenu_r2) begin
                    if(brightness != 15)
                    begin
                        pressed <= 1'd0;
                        brightness <= brightness + 9'd1;
                        blockBrightnessReceive <= 2'd3;
                        updateBrightness <= 1'b1;
                    end
                end
            end
            
            if (lowpowerBacklight) begin
               brightness       <= 4'd0;
               updateBrightness <= 1'b0;
            end
            
            if (volt >= 700) begin // ~1.8V
               if (~lowpowerBacklight && ~bat_is_LI && volt < 979) begin // below 2.55 V
                  request_SystemStatusExtended <= 1'b1;
                  lowpowerBacklight            <= 1'b1;
                  lowerpowerOldBL              <= brightness;
               end       
               
               if (lowpowerBacklight && ~bat_is_LI && volt > 1293) begin // above 3.4 V
                  request_SystemStatusExtended <= 1'b1;
                  lowpowerBacklight            <= 1'b0;
                  brightness                   <= lowerpowerOldBL;
               end
            end

            gg_en   <= |cheat_enable_mask;
            gg_code <= gg_code_latch;

            if (peek_data_valid)
            begin
                peek_resp_ready  <= 1'b1;
                peek_resp_addr   <= peek_addr;
                peek_resp_data   <= peek_data;
                peek_resp_status <= {6'd0, peek_status[1], peek_status[0]};
                peek_inflight    <= 1'b0;
            end

            if (write_done && (tx_channel == 4'd9)) begin
                request_gpd <= 1'b0;
            end
            if (write_done && (tx_channel == 4'd10)) begin
                game_hash_sent <= 1'b1;
            end
            if (write_done && (tx_channel == 4'd13)) begin
                peek_resp_ready <= 1'b0;
            end
        end
    end

    reg menuDown = 1'b0;
    always@(posedge clk)
    begin
        if(reset) begin
            menuDisabled <= 1'b1;
        end else begin
            btnMenu_r1 <= BTN_MENU;
            btnMenu_r2 <= btnMenu_r1;
            btnMenu_sr <= {btnMenu_sr[14:0], btnMenu_r2};
            if(btnMenu_sr[15:0] == 16'h8000) begin
               menuDown <= 1'b1;
            end
            if(btnMenu_sr[15:0] == 16'h7FFF && menuDown) begin
               menuDisabled <= ~menuDisabled;
               menuDown     <= 1'b0;
            end
            
            if (btnA_r2 | btnB_r2 | btnDown_r2 | btnUp_r2 | btnLeft_r2 | btnRight_r2 | btnSelect_r2 | btnStart_r2) menuDown <= 1'b0;
            
            btnDown_r1 <= BTN_DPAD_DOWN;
            btnDown_r2 <= btnDown_r1;
            btnDown_sr <= {btnDown_sr[14:0], btnDown_r2};
                    
            btnUp_r1 <= BTN_DPAD_UP;
            btnUp_r2 <= btnUp_r1;
            btnUp_sr <= {btnUp_sr[14:0], btnUp_r2};
            
            btnLeft_r1 <= BTN_DPAD_LEFT;
            btnLeft_r2 <= btnLeft_r1;
            btnLeft_sr <= {btnLeft_sr[14:0], btnLeft_r2};
                    
            btnRight_r1 <= BTN_DPAD_RIGHT;
            btnRight_r2 <= btnRight_r1;
            btnRight_sr <= {btnRight_sr[14:0], btnRight_r2};
            
            btnSelect_r1 <= BTN_SEL;
            btnSelect_r2 <= btnSelect_r1;
            
            btnStart_r1 <= BTN_START;
            btnStart_r2 <= btnStart_r1;
            
            btnA_r1 <= BTN_A;
            btnA_r2 <= btnA_r1;
            
            btnB_r1 <= BTN_B;
            btnB_r2 <= btnB_r1;
                    
        end
    end
    
    reg [7:0] lcdcount;
    always@(posedge clk)
        if(lcdcount < 448)
            lcdcount <= lcdcount + 1'd1;
        else
            lcdcount <= 'd0;

    assign LCD_PWM = LCD_INIT_DONE&LCD_BACKLIGHT_INIT ? (lcdcount <= {brightness[3:0], 4'd0}) : 1'd0;


    // 8.388608Mhz clock -> ~119.2ns
    // 0.005s / 119.2ns = 41946
    
    localparam ADC_INTERVAL_CYCLES = 'd41946;
    reg [15:0] adc_timer;
    reg [9:0] startup_cnt;            // wait for ~4 seconds to have stable measurements
    reg signed [10:0] startup_select; // measure if AA or lithium is used, negative -> LI, positive -> AA
    reg startup_done = 1'b0;
    
    always@(posedge clk) begin
        if(adc_timer < ADC_INTERVAL_CYCLES) begin
            adc_timer <= adc_timer + 1'd1;
            hAdcReq_ext <= 'd0;
            // Toggle the mux slightly ahead of starting the measurement
            if (startup_done) begin
                ADC_SEL <= startup_select[10];
            end else if(adc_timer == ADC_INTERVAL_CYCLES - 1000) begin
                ADC_SEL <= ~ADC_SEL;
            end
        end else begin
            adc_timer <= 'd0;
            hAdcReq_ext <= 'd1;
        end
    end

   assign     bat_is_LI = startup_select[10];
   reg [21:0] volt_sum;
   reg [8:0]  volt_cnt;
   reg        transmitVolt;

   wire [13:0] VOLTAGE_FULL   = bat_is_LI ? 14'd1423 : 14'd1367; //  3.75V LI : 3.6V AA
   wire [13:0] VOLTAGE_CRIT   = bat_is_LI ? 14'd1145 : 14'd997;  //  3.0V  LI : 2.6V AA
   wire [13:0] VOLTAGE_RED    = bat_is_LI ? 14'd1182 : 14'd1071; //  3.1V  LI : 2.8V AA

   reg blink;
    
   always@(posedge clk or posedge reset) begin

      if(reset) begin
         low_battery    <= 1'd0;
         LED_Red        <= 1'd0;
         LED_Green      <= 1'd0;
         LED_Yellow     <= 1'd0;
         blink          <= 1'd0;
         volt           <= 14'd0;
         volt_sum       <= 22'd0;
         volt_cnt       <=  9'd0;
         startup_cnt    <= 11'd0;
         startup_select <= 11'd0;
         startup_done   <= 1'b0;
         transmitVolt   <= 1'b0;
      end else begin
      
         transmitVolt <= 1'b0;
      
         debug_system <= {1'd0 , startup_select,  6'd0, volt };
      
         if(hAdcReady_r1) begin
            if (~startup_cnt[9]) begin // wait for ~4 seconds to have stable measurements
               startup_cnt <= startup_cnt + 1'd1;
            end
         
            if (startup_done) begin // average values for determined type only
               volt_sum <= volt_sum + hAdcValue_r1;
               volt_cnt <= volt_cnt + 1;
            end else if (startup_cnt[9] && hAdcValue_r1 >= 700) begin // measure if AA or lithium is used, negative -> LI, positive -> AA
               if (ADC_SEL) begin
                  startup_select <= startup_select - 1'd1;
               end else begin
                  startup_select <= startup_select + 1'd1;
               end
            end
            
            if (startup_select > 11'sd127 || startup_select < -11'sd127) begin // determine type based on which delivered higher values for some seconds
               startup_done <= 1'b1;
            end
         end
         
         if (volt_cnt[8]) begin
            volt_sum    <= 22'd0;
            volt_cnt    <=  9'd0;
            volt        <= volt_sum[21:8];
            transmitVolt <= 1'b1;
         end
         
         if (gSecondEna) blink <= ~blink;
         
         low_battery <= 1'd0;
         LED_Red     <= 1'd0;
         LED_Green   <= 1'd0;
         LED_Yellow  <= 1'd0;
         LED_White   <= 1'd0;
         
         if (volt >= 700) begin // ~1.8V
         
            if (pmic_sys_status[2]) begin // charging
            
               if(bat_is_LI && volt < VOLTAGE_FULL) begin
                  LED_White   <= 1'd1;
               end
            
            end else begin
         
               if(volt < VOLTAGE_RED) begin
                  low_battery <= 1'd1;
                  if (blink) LED_Red <= 1'd1;
               end
               
            end 
            
         end
      end 
   end 
    
    wire [6:0] tx_address;
    wire       write;

    wire [13:0] buttons = {
        4'd0,
        menuDisabled,
        ~BTN_MENU,
        BTN_DPAD_DOWN,
        BTN_DPAD_LEFT,
        BTN_DPAD_RIGHT,
        BTN_DPAD_UP,
        BTN_A,
        BTN_B,
        BTN_SEL,
        BTN_START
    };
    
    reg [13:0] version = {
        1'd0,  // 1 bit reserved   
        1'd0,  // 1 bit debug,
        6'd5,  // 6 bits minor version
        6'd18  // 6 bits major version
    };
    
    localparam  NUM_CH = 14;
    wire [$clog2(NUM_CH)-1:0] tx_channel;

    wire game_hash_pending = game_hash_valid & ~game_hash_sent;
    
    wire [NUM_CH-1:0] channelsNewDataValid = 
    {
        peek_resp_ready,                             // Peek data response (0x0D)
        1'b0,                                        // unused (0x0C)
        1'b0,                                        // unused (0x0B)
        game_hash_pending,                           // Game hash (0x0A)
        request_gpd,                                 // Game palette data (0x09)
        ~menuDisabled | request_SystemStatusExtended,// System Status Extended (0x08)
        ~menuDisabled,                               // reserved (0x07)
        ~menuDisabled | request_version,             // version info (0x06)
        ~menuDisabled,                               // pmic sys status (0x05)
        ~menuDisabled,                               // System Control (0x04)
        ~menuDisabled | updateBrightness,            // Audio + Brightness (0x03)
        ~menuDisabled | request_buttons,             // Buttons (0x02)
        (~menuDisabled & transmitVolt & bat_is_LI),  // Lithium (0x01)
        (~menuDisabled & transmitVolt & ~bat_is_LI)  // AA (0x00)
    };
    
    wire [13:0] audio_brightness = {2'd0, brightness, hHeadphones, hVolume};
    wire [13:0] mic_sys_status = {6'd0 , pmic_sys_status};
    
    wire [7:0] tx_byteCount = (tx_channel == 0)  ? 8'd2  : // AA
                              (tx_channel == 1)  ? 8'd2  : // Lithium 
                              (tx_channel == 2)  ? 8'd2  : // Buttons 
                              (tx_channel == 3)  ? 8'd2  : // Audio + Brightness 
                              (tx_channel == 4)  ? 8'd2  : // System Control 
                              (tx_channel == 5)  ? 8'd2  : // pmic sys status 
                              (tx_channel == 6)  ? 8'd2  : // version info
                              (tx_channel == 7)  ? 8'd4  : // reserved
                              (tx_channel == 8)  ? 8'd4  : // System Status Extended
                              (tx_channel == 9)  ? 8'd8  : // Game palette data (GPD)
                              (tx_channel == 10) ? 8'd4  : // Game hash
                              (tx_channel == 11) ? 8'd1  : // unused
                              (tx_channel == 12) ? 8'd1  : // unused
                              (tx_channel == 13) ? 8'd5  : // Peek response
                              8'd1;
    
    wire [7:0] tx_bytepos;
    
    wire [7:0] tx_senddata = (tx_channel == 0 && tx_bytepos == 0) ? {2'd0, volt[13:8]} : // AA
                             (tx_channel == 0 && tx_bytepos == 1) ? volt[7:0] :
                             
                             (tx_channel == 1 && tx_bytepos == 0) ? {2'd0, volt[13:8]} : // Lithium 
                             (tx_channel == 1 && tx_bytepos == 1) ? volt[7:0] :
                             
                             (tx_channel == 2 && tx_bytepos == 0) ? {2'd0, buttons[13:8]} : // Buttons 
                             (tx_channel == 2 && tx_bytepos == 1) ? buttons[7:0] : 
                             
                             (tx_channel == 3 && tx_bytepos == 0) ? {2'd0, audio_brightness[13:8]} : // Audio + Brightness 
                             (tx_channel == 3 && tx_bytepos == 1) ? audio_brightness[7:0] :
                             
                             (tx_channel == 4 && tx_bytepos == 0) ? {2'd0, system_control[13:8]} : // System Control 
                             (tx_channel == 4 && tx_bytepos == 1) ? system_control[7:0] :
                             
                             (tx_channel == 5 && tx_bytepos == 0) ? {2'd0, mic_sys_status[13:8]} : // pmic sys status 
                             (tx_channel == 5 && tx_bytepos == 1) ? mic_sys_status[7:0]  : 
                             
                             (tx_channel == 6 && tx_bytepos == 0) ? {2'd0, version[13:8]} : // version info
                             (tx_channel == 6 && tx_bytepos == 1) ? version[7:0] : 
                             
                             (tx_channel == 7 && tx_bytepos == 0) ? 8'd0 : // reserved
                             (tx_channel == 7 && tx_bytepos == 1) ? 8'd0 : 
                             (tx_channel == 7 && tx_bytepos == 2) ? 8'd0 : 
                             (tx_channel == 7 && tx_bytepos == 3) ? 8'd0 : 
                             
                             (tx_channel == 8 && tx_bytepos == 0) ? {6'd0, gbc_mode, lowpowerBacklight} : // System Status Extended
                             (tx_channel == 8 && tx_bytepos == 1) ? 8'd0 : 
                             (tx_channel == 8 && tx_bytepos == 2) ? 8'd0 : 
                             (tx_channel == 8 && tx_bytepos == 3) ? 8'd0 : 

                             (tx_channel == 9 && tx_bytepos == 0) ? gpd[7:0] : // Game palette data
                             (tx_channel == 9 && tx_bytepos == 1) ? gpd[15:8] :
                             (tx_channel == 9 && tx_bytepos == 2) ? gpd[23:16] :
                             (tx_channel == 9 && tx_bytepos == 3) ? gpd[31:24] :
                             (tx_channel == 9 && tx_bytepos == 4) ? gpd[39:32] :
                             (tx_channel == 9 && tx_bytepos == 5) ? gpd[47:40] :
                             (tx_channel == 9 && tx_bytepos == 6) ? gpd[55:48] :
                             (tx_channel == 9 && tx_bytepos == 7) ? gpd[63:56] :

                             (tx_channel == 10 && tx_bytepos == 0) ? game_hash[31:24] : // Game hash
                             (tx_channel == 10 && tx_bytepos == 1) ? game_hash[23:16] :
                             (tx_channel == 10 && tx_bytepos == 2) ? game_hash[15:8]  :
                             (tx_channel == 10 && tx_bytepos == 3) ? game_hash[7:0]   :

                             (tx_channel == 13 && tx_bytepos == 0) ? peek_slot_latch : // Peek response: slot
                             (tx_channel == 13 && tx_bytepos == 1) ? peek_resp_status : // status
                             (tx_channel == 13 && tx_bytepos == 2) ? peek_resp_addr[15:8] :
                             (tx_channel == 13 && tx_bytepos == 3) ? peek_resp_addr[7:0]  :
                             (tx_channel == 13 && tx_bytepos == 4) ? peek_resp_data :
                             
                             8'd0;
   
    
    wire uartDisabled;
    
    system_monitor_arbiter 
    #(
        .NUM_CH(NUM_CH)
    ) u_system_monitor_arbiter
    (
        .clk(clk),
        .reset(reset),
        .uartDisabled(uartDisabled),
        .menuDisabled(menuDisabled),
        .channelsNewDataValid(channelsNewDataValid),
        .uart_tx_busy(uart_tx_busy),
        .tx_address(tx_address),
        .tx_channel(tx_channel),
        .write_done(write_done),
        .write(write)
    );
    
    uart_packet_wrapper_tx u_uart_packet_wrapper_tx
    (
        .clk(clk),
        .reset(reset),
        .uart_tx_busy(uart_tx_busy),
        .uart_tx_data(uart_tx_data),
        .uart_tx_val(uart_tx_val),
        .uartDisabled(uartDisabled),
        .menuDisabled(menuDisabled),
        .write(write),
        .write_done(write_done),      
        .tx_address(tx_address),
        .tx_byteCount(tx_byteCount),
        .tx_bytepos(tx_bytepos),
        .tx_senddata(tx_senddata)
    );
    
    uart_packet_wrapper_rx u_uart_packet_wrapper_rx
    (
        .clk(clk),
        .reset(reset),
        .uart_rx_val(uart_rx_val),
        .uart_rx_data(uart_rx_data),
        .uartDisabled(uartDisabled),
        .rx_address(rx_address),
        .rx_data(rx_data),
        .rx_payload_len(rx_payload_len),
        .rx_data_val(rx_data_val)
    );

endmodule
