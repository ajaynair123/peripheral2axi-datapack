module axi_data_packer_tb;

reg clk;
reg rst_n;
reg b_empty;     
reg data_empty;  
reg [31:0] burst_addr;	
reg [11:0] burst_info;	
reg [35:0] wdata_pckt;	
reg rdy_from_slv;

wire axi_wvalid;	
wire [3:0] axi_wstrb;		
wire [31:0] axi_wdata;		
wire axi_wlast;		
wire data_fifo_pop;

axi_data_packer dut (
 .clk		   (clk),
 .rst_n        (rst_n),
 .b_empty      (b_empty),     
 .data_empty   (data_empty),  
 .burst_addr	(burst_addr),	
 .burst_info	(burst_info),	
 .wdata_pckt	(wdata_pckt),	
 .rdy_from_slv (rdy_from_slv),            
 .axi_wvalid	(axi_wvalid),	
 .axi_wstrb		(axi_wstrb),		
 .axi_wdata		(axi_wdata),		
 .axi_wlast		(axi_wlast),		
 .data_fifo_pop (data_fifo_pop)
);

always #5 clk = ~clk;

initial begin
clk = 1'b1;
rst_n = 1'b0; 
b_empty = 1'b1;
data_empty = 1'b1;
rdy_from_slv = 1'b1;
#20
rst_n = 1'b1;
#10 
b_empty = 1'b0;
data_empty = 1'b0;
burst_addr = 32'hDEAD0000;
burst_info = 12'h022;	//len and size
wdata_pckt = 36'h000000000;	//data and byte en
//Data: 34,12,56,78,91,91
@(negedge data_fifo_pop)
wdata_pckt = 36'h000012343;	
@(negedge data_fifo_pop)
wdata_pckt = 36'h00ABCD567;
@(negedge data_fifo_pop)
wdata_pckt = 36'h000000781;
@(negedge data_fifo_pop)
wdata_pckt = 36'h000000911;
@(negedge axi_wlast)
data_empty = 1'b1;
#1000
$finish;
end

endmodule
