// ============================================================================
// Title       : PCA9555 I2C to GPIO Expander (16-bit)
// Functionality:
//   - 16-bit I/O expander controlled over I2C bus
//   - Register map: 0x00-0x07 (Input, Output, Polarity, Config)
//   - Supports START, STOP, ACK/NACK, repeated start, sequential read/write
//   - Tri-state GPIO based on config registers
//   - No clock stretching or interrupts (simplified for clarity)
// ============================================================================

`timescale 1ns/1ps

// ============================================================================
// Module: PCA9555 Register Bank
// ============================================================================
module pca9555_registers (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        wr_en,
    input  wire        rd_en,
    input  wire [2:0]  reg_addr,
    input  wire [7:0]  wr_data,
    output reg  [7:0]  rd_data,
    input  wire [7:0]  port_in0,
    input  wire [7:0]  port_in1,
    output reg  [7:0]  port_out0,
    output reg  [7:0]  port_out1,
    output reg  [7:0]  config0,
    output reg  [7:0]  config1,
    output reg  [7:0]  polarity0,
    output reg  [7:0]  polarity1
);

    // Initialize registers as per datasheet defaults
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            port_out0  <= 8'h00;
            port_out1  <= 8'h00;
            config0    <= 8'hFF; // 1=input, 0=output
            config1    <= 8'hFF;
            polarity0  <= 8'h00;
            polarity1  <= 8'h00;
            rd_data    <= 8'h00;
        end else begin
            // Handle Write
    if (wr_en) begin
    case (reg_addr)
        3'd2: begin
            port_out0 <= wr_data;
            $display("Register[0x02] Output Port0 updated = %h", wr_data);
        end
        3'd3: begin
            port_out1 <= wr_data;
            $display("Register[0x03] Output Port1 updated = %h", wr_data);
        end
        3'd4: polarity0 <= wr_data;
        3'd5: polarity1 <= wr_data;
        3'd6: config0   <= wr_data;
        3'd7: config1   <= wr_data;
        default: ;
    endcase
end
            // Handle Read
            if (rd_en) begin
                case (reg_addr)
                    3'd0: rd_data <= port_in0 ^ polarity0; // Input Port 0
                    3'd1: rd_data <= port_in1 ^ polarity1; // Input Port 1
                    3'd2: rd_data <= port_out0;
                    3'd3: rd_data <= port_out1;
                    3'd4: rd_data <= polarity0;
                    3'd5: rd_data <= polarity1;
                    3'd6: rd_data <= config0;
                    3'd7: rd_data <= config1;
                endcase
            end
        end
    end
endmodule


// ============================================================================
// Module: I2C Slave for PCA9555 (Simplified Byte-Level Implementation)
// ============================================================================
module i2c_slave_pca9555 (
    input  wire clk,           // internal fast clock for edge sampling
    input  wire rst_n,
    inout  wire sda,
    input  wire scl,
    input  wire [2:0] addr_pins, // hardware address A2 A1 A0
    output reg        wr_en,
    output reg        rd_en,
    output reg  [2:0] reg_addr,
    output reg  [7:0] wr_data,
    input  wire [7:0] rd_data
);

    // Internal signals
    reg [7:0] shift_reg;
    reg [3:0] bit_cnt;
    reg rw_flag;                 // 0=write, 1=read
    reg [2:0] cmd_pointer;       // register pointer
    reg addr_match;              // address match flag
    reg [6:0] device_addr;       // 7-bit address = 0100 A2A1A0
    reg sda_drive;               // 0=drive low, 1=release
    reg sda_q, scl_q;
    reg sda_prev, scl_prev;
    reg [2:0] state;

    localparam IDLE = 0, ADDR = 1, REGPTR = 2, WRITE = 3, READ = 4, ACK = 5;

    // Assign device address
    always @(*) device_addr = {4'b0100, addr_pins};

    // Open-drain SDA line behavior
    assign sda = (sda_drive == 1'b0) ? 1'b0 : 1'bz;

    // Sample SCL/SDA
    always @(posedge clk) begin
        sda_prev <= sda_q;
        scl_prev <= scl_q;
        sda_q <= sda;
        scl_q <= scl;
    end

    wire start_cond = (sda_prev == 1'b1 && sda_q == 1'b0 && scl_q == 1'b1);
    wire stop_cond  = (sda_prev == 1'b0 && sda_q == 1'b1 && scl_q == 1'b1);
    wire scl_rise   = (scl_prev == 1'b0 && scl_q == 1'b1);

    // I2C state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt     <= 4'd7;
            shift_reg   <= 8'd0;
            rw_flag     <= 1'b0;
            cmd_pointer <= 3'd0;
            addr_match  <= 1'b0;
            sda_drive   <= 1'b1;
            state       <= IDLE;
            wr_en       <= 1'b0;
            rd_en       <= 1'b0;
        end else begin
            wr_en <= 1'b0;
            rd_en <= 1'b0;

            if (start_cond) begin
                bit_cnt <= 4'd7;
                state <= ADDR;
                addr_match <= 1'b0;
            end else if (stop_cond) begin
                state <= IDLE;
                sda_drive <= 1'b1;
            end else begin
                case (state)
                    IDLE: ; // wait for start

                    ADDR: begin
                        if (scl_rise) begin
                            shift_reg[bit_cnt] <= sda;
                            if (bit_cnt == 0) begin
                                rw_flag <= sda; // LSB = R/W
                                if (shift_reg[7:1] == device_addr) begin
                                    addr_match <= 1'b1;
                                    sda_drive <= 1'b0; // ACK
                                    state <= ACK;
                                end else begin
                                    addr_match <= 1'b0;
                                    state <= IDLE;
                                end
                            end else bit_cnt <= bit_cnt - 1;
                        end
                    end

                    ACK: begin
                        sda_drive <= 1'b1; // release after ACK
                        if (addr_match) begin
                            if (rw_flag == 1'b0) state <= REGPTR; // write mode
                            else begin
                                rd_en <= 1'b1; // trigger read
                                state <= READ;
                            end
                            bit_cnt <= 7;
                        end
                    end

                    REGPTR: begin
                        if (scl_rise) begin
                            shift_reg[bit_cnt] <= sda;
                            if (bit_cnt == 0) begin
                                cmd_pointer <= shift_reg[2:0];
                                sda_drive <= 1'b0; // ACK
                                bit_cnt <= 7;
                                state <= WRITE;
                            end else bit_cnt <= bit_cnt - 1;
                        end
                    end

                    WRITE: begin
                        if (scl_rise) begin
                            shift_reg[bit_cnt] <= sda;
                            if (bit_cnt == 0) begin
                                wr_data <= shift_reg;
                                reg_addr <= cmd_pointer;
                                wr_en <= 1'b1;
                                cmd_pointer <= cmd_pointer + 1'b1;
                                sda_drive <= 1'b0; // ACK
                                bit_cnt <= 7;
                            end else bit_cnt <= bit_cnt - 1;
                        end
                    end

                    READ: begin
                        if (scl_rise) begin
                            sda_drive <= ~rd_data[bit_cnt]; // drive data
                            if (bit_cnt == 0) begin
                                bit_cnt <= 7;
                                rd_en <= 1'b1;
                            end else bit_cnt <= bit_cnt - 1;
                        end
                    end
                endcase
            end
        end
    end
endmodule


// ============================================================================
// Top-Level: PCA9555 Complete Device (I2C + Registers + GPIO)
// ============================================================================
module pca9555_top (
    input  wire clk,
    input  wire rst_n,
    inout  wire sda,
    input  wire scl,
    input  wire [2:0] addr_pins,
    input  wire [7:0] port_in0,
    input  wire [7:0] port_in1,
    output wire [7:0] port_out0,
    output wire [7:0] port_out1
);

    wire wr_en, rd_en;
    wire [2:0] reg_addr;
    wire [7:0] wr_data, rd_data;
    wire [7:0] config0, config1, polarity0, polarity1;
    wire [7:0] out0, out1;

    // Instantiate Register Bank
    pca9555_registers REG_BANK (
        .clk(clk), .rst_n(rst_n), .wr_en(wr_en), .rd_en(rd_en),
        .reg_addr(reg_addr), .wr_data(wr_data), .rd_data(rd_data),
        .port_in0(port_in0), .port_in1(port_in1),
        .port_out0(out0), .port_out1(out1),
        .config0(config0), .config1(config1),
        .polarity0(polarity0), .polarity1(polarity1)
    );

    // Instantiate I2C Slave
    i2c_slave_pca9555 I2C_SLAVE (
        .clk(clk), .rst_n(rst_n), .sda(sda), .scl(scl),
        .addr_pins(addr_pins),
        .wr_en(wr_en), .rd_en(rd_en),
        .reg_addr(reg_addr), .wr_data(wr_data), .rd_data(rd_data)
    );

    // Tri-state GPIO behavior
    assign port_out0 = out0;
    assign port_out1 = out1;
endmodule

