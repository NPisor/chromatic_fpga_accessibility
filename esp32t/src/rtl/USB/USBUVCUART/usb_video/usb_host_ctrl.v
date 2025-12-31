// usb_video/usb_host_ctrl.v
`timescale 1ns/1ps

module usb_host_ctrl #(
    parameter [15:0] VENDOR_ID  = 16'h0BDA,
    parameter [15:0] PRODUCT_ID = 16'h5830
)(
    input           clk,
    input           rst_n,

    // --- UTMI+ signals for Host MAC ---
    input   [7:0]   utmi_rx_data,
    input           utmi_rx_valid,
    input           utmi_rx_active,
    input           utmi_tx_ready,
    input   [1:0]   utmi_linestate,
    input           utmi_termselect,
    input           utmi_suspend,
    input           utmi_host_disconnect,
    output  [7:0]   utmi_tx_data,
    output          utmi_tx_valid,
    output          utmi_tx_first,
    output          utmi_tx_last,
    output  [1:0]   utmi_xcvrselect,
    output          utmi_op_mode,
    output          utmi_termselect_o,

    // --- Control/status ---
    output reg      device_connected,
    output reg      matched_vid_pid,
    output reg      start_enumeration,
    output reg      start_stream
);

    // State machine states (very high-level)
    typedef enum logic [2:0] {
        S_IDLE,
        S_RESET,
        S_GET_DEV_DESC,
        S_PARSE_DEV_DESC,
        S_MATCHED,
        S_ERROR
    } state_t;

    state_t state, next_state;

    reg [7:0]  desc_buf [0:63];
    integer    desc_idx;

    // Simple state transition
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    // Next-state logic
    always_comb begin
        // defaults
        next_state         = state;
        device_connected   = 1'b0;
        start_enumeration  = 1'b0;
        matched_vid_pid    = 1'b0;
        start_stream       = 1'b0;

        case (state)
        S_IDLE: begin
            // wait for bus reset / device attach
            if (utmi_linestate == 2'b10) begin  // full-speed connected
                next_state       = S_RESET;
                device_connected = 1'b1;
            end
        end

        S_RESET: begin
            // issue USB reset on the bus, then move on
            start_enumeration = 1;
            next_state        = S_GET_DEV_DESC;
        end

        S_GET_DEV_DESC: begin
            // drive a GET_DESCRIPTOR(Device) control transfer...
            // (you'll need to hook up your control-transfer engine here)
            // once you’ve filled desc_buf, advance:
            next_state = S_PARSE_DEV_DESC;
        end

        S_PARSE_DEV_DESC: begin
            // parse desc_buf[8:9] as VID and [10:11] as PID
            if ({desc_buf[9], desc_buf[8]} == VENDOR_ID &&
                {desc_buf[11],desc_buf[10]}== PRODUCT_ID) begin
                matched_vid_pid = 1;
                next_state      = S_MATCHED;
            end else begin
                next_state = S_ERROR;
            end
        end

        S_MATCHED: begin
            // tell the UVC engine to kick off
            start_stream     = 1;
            device_connected = 1;
            matched_vid_pid  = 1;
            // stay here indefinitely
        end

        S_ERROR: begin
            // no match—just sit here
        end
        endcase
    end

    // (TODO) you’ll need to hook up utmi_tx_data/utmi_tx_valid/etc.
    // to drive the control-transfer packets in S_GET_DEV_DESC.

endmodule
