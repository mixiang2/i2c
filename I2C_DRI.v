


module I2C_DRI(
    input   wire            clk         ,  //系统时钟
    input   wire            rst_n       ,  //系统复位

    input           [25:0]  clk_freq    ,  //模块输入的时钟频率
    input           [17:0]  i2c_freq    ,  //IIC_SCL的时钟频率 

    input                   i2c_rh_wl   ,  //I2C读写控制信号
    input                   bit_ctrl    ,  //字地址位控制(16b/8b)
    input   wire            i2c_exec    ,  //一次读写开始信号
    input   wire    [6:0 ]  sensor_addr ,  //设备地址
    input   wire    [15:0]  i2c_addr    ,  //寄存器地址
    input   wire    [7:0 ]  i2c_data_w  ,  //写入寄存器的数据

    output  reg     [7:0 ]  i2c_data_r  ,  //从寄存器读出的数据
    output  reg             i2c_done    ,  //I2C一次操作完成
    output  reg             err_flag    ,  //发生错误信号
    output  reg             scl         ,  //i2c时钟
    inout   wire            sda            //i2c数据总线   
    );


//==================================================
//parameter define
//==================================================
parameter   IDLE    = 11'b000_0000_0001;//空闲状态
parameter   START   = 11'b000_0000_0010;//写起始
parameter   SLADDR  = 11'b000_0000_0100;//确认设备地址
parameter   ADDR16  = 11'b000_0000_1000;//发送16位字地址 
parameter   ADDR8   = 11'b000_0001_0000;//发送8位字地址 
parameter   DATA_WR = 11'b000_0010_0000;//写数据
parameter   RD_START= 11'b000_0100_0000;//读开始
parameter   ADDR_RD = 11'b000_1000_0000;//设备地址读操作
parameter   DATA_RD = 11'b001_0000_0000;//读数据
parameter   STOP    = 11'b010_0000_0000;//停止
parameter   ERROR   = 11'b100_0000_0000;//错误


//==================================================
//internal signals
//==================================================
reg     [2:0]   cnt_freq    ;//计数drive_flag 产生I2C时钟
wire            add_cnt_freq;
wire            end_cnt_freq;

reg     [5:0]   cnt_flag    ;//计数当前状态有多少个drive_flag
wire            add_cnt_flag;
wire            end_cnt_flag;
reg     [6:0]   x           ;//可变计数器的最大值

reg     [9:0]   cnt         ;//用来产生驱动信号drive_flag
wire            add_cnt     ;
wire            end_cnt     ;

reg             drive_flag  ;//用于驱动本模块工作的信号
reg     [10:0]  state       ;//state register
reg             work_flag   ;//work flag
reg             sda_dir     ;//三态数据线写使能
reg     [7:0]   data_shift  ;//移位寄存器
reg             i2c_ack     ;//响应信号

reg             sda_out     ;
wire            sda_in      ;
wire    [8:0]   clk_divide  ;
wire    [8:0]   MAX  ;
wire    [8:0]   FLAG0       ;
wire    [8:0]   FLAG1       ;
wire    [8:0]   FLAG2       ;
wire    [8:0]   FLAG3       ;

assign  clk_divide = (clk_freq/i2c_freq) >> 2'd2 ;  //模块驱动时钟的分频系数
assign  MAX   = clk_freq/i2c_freq;//驱动时钟的计数最大值
assign  FLAG0 = clk_divide - 1'b1 ;
assign  FLAG1 = 2*clk_divide - 1'b1 ;
assign  FLAG2 = 3*clk_divide - 1'b1 ;
assign  FLAG3 = 4*clk_divide - 1'b1 ;

//三态端口声明
assign  sda = sda_dir?sda_out:1'bz;//当主机向数据向数据总线上写数据时，sda_dir=1，给出用户数据，当接收数据时，置为高阻
assign  sda_in = sda;
//--------------------state machine define--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        state <= IDLE;
    end
    else begin
        case(state)
            IDLE:begin
                if(i2c_exec==1'b1)
                    state <= START;//接收到开始信号，进入起始状态
                else
                    state <= IDLE;
            end 

            START:begin
                if(cnt_flag=='d6 && drive_flag)
                    state <= SLADDR;//起始状态结束，进入确认设备地址状态
                else
                    state <= START;
            end

            SLADDR:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)//发送完设备地址，并且正确接收到响应
                    if(bit_ctrl)                    //判断是16位还是8位字地址
                        state <= ADDR16;
                    else
                        state <= ADDR8 ;
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//发送完设备地址，并且没有接收到响应
                    state <= ERROR;//确认地址状态结束，进入确认寄存器地址状态
                else
                    state <= SLADDR;
            end

            ADDR16:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)
                        state <= ADDR8;
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//发送完设备地址，并且没有接收到响应
                    state <= ERROR;//确认地址状态结束，进入确认寄存器地址状态
                else 
                    state <= ADDR16;
            end

            ADDR8:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)begin//已经给出写入的寄存器，并且正确接收到响应
                    if(i2c_rh_wl==1'b0)
                        state <= DATA_WR;//确认寄存器地址状态结束，进入写数据状态
                    else if(i2c_rh_wl==1'b1)
                        state <= RD_START;//确认寄存器地址状态结束，进入读开始状态
                end
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//发送完设备地址，并且没有接收到响应
                    state <= ERROR;//确认地址状态结束，进入确认寄存器地址状态
                else 
                    state <= ADDR8;
            end

            DATA_WR:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)//已经给出写入数据并正确接收到响应
                    state <= STOP;//数据写入完成进入停止状态
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//发送完设备地址，并且没有接收到响应
                    state <= ERROR;//确认地址状态结束，进入确认寄存器地址状态
                else
                    state <= DATA_WR;
            end

            RD_START:begin
                if(cnt_flag=='d3 && drive_flag)
                    state <= ADDR_RD;//进入设备地址读操作
                else
                    state <= RD_START;
            end

            ADDR_RD:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)
                    state <= DATA_RD;//进入读数据状态
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//发送完设备地址，并且没有接收到响应
                    state <= ERROR;//确认地址状态结束，进入确认寄存器地址状态
                else
                    state <= ADDR_RD;
            end

            DATA_RD:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)
                    state <= STOP;
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//发送完设备地址，并且没有接收到响应
                    state <= ERROR;//确认地址状态结束，进入确认寄存器地址状态
                else
                    state <= DATA_RD;
            end

            STOP:begin
                if(cnt_flag=='d3 && drive_flag)
                    state <= IDLE;
                else
                    state <= STOP;
            end

            ERROR:begin
                state <= IDLE;
            end

            default:begin
                state <= IDLE;
            end
        endcase
    end
end

//-------------------work_flag---------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        work_flag <= 1'b0;
    end
    else if(state==START)begin//接收到开始信号
        work_flag <= 1'b1;
    end
    else if(state==IDLE)begin
        work_flag <= 1'b0;
    end
    else if(i2c_done==1'b1)begin//一次读写完成
        work_flag <= 1'b0;
    end
end

//--------------------cnt--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        cnt <= 0;
    end
    else if(add_cnt)begin
        if(end_cnt)
            cnt <= 0;
        else
            cnt <= cnt + 1'b1;
    end
    else begin
        cnt <= 'd0;
    end
end

assign add_cnt = work_flag;//处于工作状态时一直计数       
assign end_cnt = add_cnt && cnt== MAX;//计数到最大值清零

//--------------------drive_flag--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        drive_flag <= 1'b0;
    end
    else if(cnt==FLAG0 || cnt==FLAG1 || cnt==FLAG2 || cnt==FLAG3)begin//产生一个驱动信号
        drive_flag <= 1'b1;
    end
    else begin
        drive_flag <= 1'b0;
    end
end

//--------------------cnt_freq--------------------
//对驱动信号进行计数，以此来产生I2C时钟
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        cnt_freq <= 0;
    end
    else if(work_flag == 1'b0)begin
        cnt_freq <= 'd0;
    end
    else if(add_cnt_freq)begin
        if(end_cnt_freq)
            cnt_freq <= 0;
        else
            cnt_freq <= cnt_freq + 1'b1;
    end
    else begin
        cnt_freq <= cnt_freq;
    end
end

assign add_cnt_freq = drive_flag;       
assign end_cnt_freq = add_cnt_freq && cnt_freq== 4-1; 

//--------------------scl--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        scl <= 1'b1;
    end
    else if (work_flag == 1'b1 && state == START) begin
        if(cnt_flag=='d1 && drive_flag)begin
            scl <= 1'b1;
        end
        else if(cnt_flag=='d5 && drive_flag )begin
            scl <= 1'b0;
        end
    end
    else if(work_flag == 1'b1 && state != START)begin
        if(cnt_freq=='d1 && drive_flag &&state==STOP)begin
            scl <= 1'b1;
        end
        else if(cnt_freq=='d1 && drive_flag && state!= STOP)begin
            scl <= 1'b0;
        end
        else if(cnt_freq=='d3 && drive_flag)begin
            scl <= 1'b1;
        end
    end
    else begin
        scl <= 1'b1;
    end
end

//--------------------cnt_flag--------------------
//计数当前状态下有多少个drive_flag
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
         cnt_flag <= 0;
    end
    else if(work_flag==1'b0)begin
        cnt_flag <= 'd0;
    end
    else if(add_cnt_flag)begin
        if(end_cnt_flag)
            cnt_flag <= 0;
        else
            cnt_flag <= cnt_flag + 1'b1;
    end
    else begin
        cnt_flag <= cnt_flag;
    end
end

assign add_cnt_flag = drive_flag;       
assign end_cnt_flag = add_cnt_flag && cnt_flag== x ; 

//--------------------x--------------------
//x为不同状态下，计数器的计数最大值
always  @(*)begin
    case(state)
        IDLE: x=0;
        START: x= 7 - 1;
        SLADDR,ADDR16,ADDR8,DATA_WR,ADDR_RD,DATA_RD: x=36 - 1;
        RD_START: x= 4 - 1;
        STOP: x = 4 - 1;
        default: x = 0;          
    endcase
end

//------------------sda_dir----------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        sda_dir <= 1'b0;
    end
    else if(state==START || state==RD_START || state==STOP)begin
        sda_dir <= 1'b1;//在写开始，读开始，和结束状态，由用户操纵数据总线，产生开始或者停止信号，所以写数据使能有效
    end
    else if(state==SLADDR || state==ADDR16 ||state==ADDR8 ||state==DATA_WR || state==ADDR_RD)begin
        if(cnt_flag < 'd32)begin//给出用户数据时，写数据使能有效
            sda_dir <= 1'b1;
        end
        else begin
            sda_dir <= 1'b0;//等待从设备响应时，写数据使能无效
        end
    end
    else if(state==DATA_RD)begin
        if(cnt_flag < 'd32)begin
            sda_dir <= 1'b0;//接收数据状态，此时由从机发送数据给主机，写数据使能无效
        end
        else begin
            sda_dir <= 1'b1;//接收数据完成，主机需要对从机做出应答，写数据使能有效
        end
    end
    else begin
        sda_dir <= 1'b0;
    end
end

//--------------------data_shift--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        data_shift <= 'd0;
    end
    else begin
        case(state)
            IDLE:begin
                data_shift <= 'd0;//空闲状态，让移位寄存器保持为0
            end

            START:begin
                data_shift <= {sensor_addr[6:0],1'b0};//写开始状态，给出设备写指令
            end

            SLADDR:begin
                if(end_cnt_flag && i2c_ack==1'b1 && bit_ctrl == 1'b1)
                    data_shift <= i2c_addr[15:8];
                else if(end_cnt_flag && i2c_ack==1'b1 && bit_ctrl == 1'b0)
                    data_shift <= i2c_addr[7:0];
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            ADDR16:begin
                if(end_cnt_flag && i2c_ack==1'b1)
                    data_shift <= i2c_addr[7:0];//确认设备状态结束时，给出寄存器地址
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            ADDR8:begin
                if(end_cnt_flag && i2c_ack==1'b1 && i2c_rh_wl==1'b0)//即将进入写数据状态
                    data_shift <= i2c_data_w;//确认寄存器地址状态结束后给出要写入的数据
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            DATA_WR:begin
                if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};//将数据写入到寄存器中
                else
                   data_shift <= data_shift; 
            end

            RD_START:begin
                data_shift <=  {sensor_addr[6:0],1'b1};//读开始时，将读命令填充入移位寄存器
            end


            ADDR_RD:begin
                if(end_cnt_flag && i2c_ack==1'b1)
                    data_shift <= 'd0;
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            DATA_RD:begin
                if(cnt_flag<'d32 && cnt_flag[1:0]==2'd1 && drive_flag)
                    data_shift <= {data_shift[6:0],sda_in};//将从寄存器中读出的数据填充入移位寄存器
                else
                    data_shift <= data_shift;
            end

            default:begin
                data_shift <= data_shift;
            end
        endcase
    end
end



//--------------------sda_out--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        sda_out <= 1'b1;
    end
    else begin
        case(state)
            START:begin
                if(cnt_flag=='d4 && drive_flag)
                    sda_out <= 1'b0;//产生起始位
                else
                    sda_out <= sda_out;
            end
 
            SLADDR,ADDR16,ADDR8,DATA_WR,ADDR_RD:begin
                sda_out <= data_shift[7];//将数据发送至数据总线上
            end

            RD_START:begin
                if(cnt_flag=='d0)
                    sda_out <= 1'b1;//产生读起始位
                else if(cnt_flag=='d1 && drive_flag)
                    sda_out <= 1'b0;
            end

            DATA_RD:begin
                if(cnt_flag>='d32)
                    sda_out <= 1'b1;//产生NACK
                else
                    sda_out <= sda_out;
            end

            STOP:begin
                    if(cnt_flag=='d0 && sda_dir)
                        sda_out <= 1'b0;
                    else if(cnt_flag=='d1 && drive_flag)
                        sda_out <= 1'b1;
            end
            
            default:sda_out <= 1'b1;
        endcase
    end
end

//--------------------i2c_done--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        i2c_done <= 1'b0;
    end
    else if(state==STOP && end_cnt_flag)begin//即将结束本次读写操作的时候，产生完成信号
            i2c_done <= 1'b1;
    end
    else begin
        i2c_done <= 1'b0;
    end
end

//--------------------i2c_ack--------------------
//是否接收到ACK或者产生NACK
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        i2c_ack <= 1'b0;
    end
    else begin
        case(state)
            SLADDR:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//写完设备地址，并且接收到响应
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            ADDR16:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//写完寄存器地址，接收到响应
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            ADDR8:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//写完寄存器地址，接收到响应
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            DATA_WR:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//写完数据，并且接收到响应
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            ADDR_RD:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//读指令发送完毕，接收到响应
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            DATA_RD:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b1)
                    i2c_ack <= 1'b1;//数据全部读完，主机给出NACK
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            default: i2c_ack <= 1'b0;
        endcase
    end
end

//--------------------i2c_data_r--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        i2c_data_r <= 1'b0;
    end
    else if(i2c_done)begin//即将结束本次读写操作的时候，产生完成信号
        i2c_data_r <= data_shift;
    end
    else begin
        i2c_data_r <= i2c_data_r;
    end
end

always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        err_flag <= 1'b0;
    end
    else if(state==ERROR)begin//即将结束本次读写操作的时候，产生完成信号
        err_flag <= 1'b1;
    end
    else begin
        err_flag <= 1'b0;
    end
end

endmodule
