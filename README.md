```
module uart_apb_if (
    input  wire        pclk,
    input  wire        presetn,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [7:0]  paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,

    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg         rx_ready,
    output reg  [15:0] baud_div,
    output reg         tx_irq,
    output reg         rx_irq
);

    localparam ADDR_TXDATA = 8'h00;
    localparam ADDR_RXDATA = 8'h04;
    localparam ADDR_STATUS = 8'h08;
    localparam ADDR_BAUD   = 8'h0C;

    assign pready = 1'b1;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            tx_valid <= 0;
            rx_ready <= 0;
            baud_div <= 16'd104; // 9600 baud default
            tx_irq   <= 0;
            rx_irq   <= 0;
        end else begin
            tx_valid <= 0;
            rx_ready <= 0;

            if (psel && penable) begin
                if (pwrite) begin
                    case (paddr)
                        ADDR_TXDATA: begin
                            tx_data <= pwdata[7:0];
                            tx_valid <= 1;
                        end
                        ADDR_BAUD: begin
                            baud_div <= pwdata[15:0];
                        end
                    endcase
                end else begin
                    case (paddr)
                        ADDR_RXDATA: begin
                            prdata <= {24'd0, rx_data};
                            rx_ready <= 1;
                        end
                        ADDR_STATUS: begin
                            prdata <= {30'd0, tx_ready, rx_valid};
                        end
                        ADDR_BAUD: begin
                            prdata <= {16'd0, baud_div};
                        end
                        default: prdata <= 32'd0;
                    endcase
                end
            end

            tx_irq <= ~tx_ready;
            rx_irq <= rx_valid;
        end
    end
endmodule

```
