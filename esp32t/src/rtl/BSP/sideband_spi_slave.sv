// sideband_spi_slave.sv
// Minimal SPI (single-bit) slave on QSPI pins to detect framed requests.
// Frame: [opcode][len][payload...][crc16-x25]; only opcode 0x01 with len=0 handled.

module sideband_spi_slave (
    input  wire clk_sck,
    input  wire cs_n,
    input  wire mosi,
    output reg  snap_req_pulse = 1'b0
);
    // CRC16/X25 (poly 0x1021 reflected -> 0x8408), init 0xFFFF, final XOR 0xFFFF
    function automatic [15:0] crc16_x25_byte(
        input [15:0] crc_in,
        input [7:0]  data
    );
        integer i;
        reg [15:0] crc;
        reg [7:0] d;
        begin
            crc = crc_in;
            d   = data;
            for (i = 0; i < 8; i = i + 1) begin
                if ((crc[0] ^ d[0]) == 1'b1) begin
                    crc = (crc >> 1) ^ 16'h8408;
                end else begin
                    crc = (crc >> 1);
                end
                d = d >> 1;
            end
            crc16_x25_byte = crc;
        end
    endfunction

    reg [7:0]  shift = 8'd0;
    reg [2:0]  bit_cnt = 3'd0;
    reg [7:0]  opcode = 8'd0;
    reg [7:0]  len = 8'd0;
    reg [7:0]  payload_cnt = 8'd0;
    reg [15:0] crc = 16'hFFFF;
    reg [15:0] recv_crc = 16'd0;
    reg [1:0]  state = 2'd0;

    localparam S_OPCODE = 2'd0;
    localparam S_LEN    = 2'd1;
    localparam S_PAYLD  = 2'd2;
    localparam S_CRC0   = 2'd3;

    always @(posedge clk_sck or posedge cs_n) begin
        if (cs_n) begin
            shift        <= 8'd0;
            bit_cnt      <= 3'd0;
            opcode       <= 8'd0;
            len          <= 8'd0;
            payload_cnt  <= 8'd0;
            crc          <= 16'hFFFF;
            recv_crc     <= 16'd0;
            state        <= S_OPCODE;
            snap_req_pulse <= 1'b0;
        end else begin
            snap_req_pulse <= 1'b0;
            shift   <= {mosi, shift[7:1]};
            bit_cnt <= bit_cnt + 3'd1;
            if (bit_cnt == 3'd7) begin
                case (state)
                    S_OPCODE: begin
                        opcode <= {mosi, shift[7:1]};
                        crc    <= crc16_x25_byte(16'hFFFF, {mosi, shift[7:1]});
                        state  <= S_LEN;
                    end
                    S_LEN: begin
                        len   <= {mosi, shift[7:1]};
                        crc   <= crc16_x25_byte(crc, {mosi, shift[7:1]});
                        payload_cnt <= 8'd0;
                        state <= ( {mosi, shift[7:1]} == 8'd0 ) ? S_CRC0 : S_PAYLD;
                    end
                    S_PAYLD: begin
                        crc <= crc16_x25_byte(crc, {mosi, shift[7:1]});
                        payload_cnt <= payload_cnt + 8'd1;
                        if (payload_cnt + 8'd1 == len)
                            state <= S_CRC0;
                    end
                    S_CRC0: begin
                        recv_crc[7:0] <= {mosi, shift[7:1]};
                        state <= S_CRC0 + 1'b1; // move to CRC1
                    end
                    default: begin // CRC1
                        recv_crc[15:8] <= {mosi, shift[7:1]};
                        // Check CRC and opcode; assert pulse if matches snapshot op (0x01)
                        if (({mosi, shift[7:1], recv_crc[7:0]} == {crc[15:8], crc[7:0]}) && opcode == 8'h01)
                            snap_req_pulse <= 1'b1;
                        state <= S_OPCODE; // ready for next frame
                    end
                endcase
            end
        end
    end
endmodule
