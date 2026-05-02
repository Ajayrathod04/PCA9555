`timescale 1ns/1ps

module tb_pca9555;
    reg clk = 0, rst_n = 0;
    reg scl = 1;
    wire sda;
    reg sda_drv = 1;
    assign sda = (sda_drv == 0) ? 1'b0 : 1'bz;

    reg [7:0] port_in0 = 8'hFF;
    reg [7:0] port_in1 = 8'hFF;
    wire [7:0] port_out0, port_out1;

    pca9555_top DUT (
        .clk(clk), .rst_n(rst_n), .sda(sda), .scl(scl),
        .addr_pins(3'b000),
        .port_in0(port_in0), .port_in1(port_in1),
        .port_out0(port_out0), .port_out1(port_out1)
    );

    always #5 clk = ~clk; // 100 MHz internal clock

    // ------------------ I2C Master Emulation ------------------
    task i2c_start;
    begin sda_drv = 1; scl = 1; #100; sda_drv = 0; #100; scl = 0; #100; end
    endtask

    task i2c_stop;
    begin sda_drv = 0; scl = 0; #100; scl = 1; #100; sda_drv = 1; #100; end
    endtask

    task i2c_write_byte(input [7:0] data);
        integer i;
    begin
        for (i=7; i>=0; i=i-1) begin
            sda_drv = data[i]; #50;
            scl = 1; #100; scl = 0; #50;
        end
        sda_drv = 1; #50; scl = 1; #100; scl = 0; #50;
    end
    endtask
    // -----------------------------------------------------------

    initial begin
        rst_n = 0; #200; rst_n = 1;
        $display("=== PCA9555 Functional Simulation ===");

        // Write 0xAA to Output Port 0
        i2c_start;
        i2c_write_byte(8'h40); // device addr + W
        i2c_write_byte(8'h02); // pointer
        i2c_write_byte(8'hAA); // data
        i2c_stop;

        #500;

        // Read back Output Port 0
        i2c_start;
        i2c_write_byte(8'h40);
        i2c_write_byte(8'h02);
        i2c_start;
        i2c_write_byte(8'h41); // read
        i2c_stop;

        #500;
        $display("port_out0 = %h", port_out0);
        $stop;
    end
endmodule
