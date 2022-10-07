


module I2C_DRI(
    input   wire            clk         ,  //ϵͳʱ��
    input   wire            rst_n       ,  //ϵͳ��λ

    input           [25:0]  clk_freq    ,  //ģ�������ʱ��Ƶ��
    input           [17:0]  i2c_freq    ,  //IIC_SCL��ʱ��Ƶ�� 

    input                   i2c_rh_wl   ,  //I2C��д�����ź�
    input                   bit_ctrl    ,  //�ֵ�ַλ����(16b/8b)
    input   wire            i2c_exec    ,  //һ�ζ�д��ʼ�ź�
    input   wire    [6:0 ]  sensor_addr ,  //�豸��ַ
    input   wire    [15:0]  i2c_addr    ,  //�Ĵ�����ַ
    input   wire    [7:0 ]  i2c_data_w  ,  //д��Ĵ���������

    output  reg     [7:0 ]  i2c_data_r  ,  //�ӼĴ�������������
    output  reg             i2c_done    ,  //I2Cһ�β������
    output  reg             err_flag    ,  //���������ź�
    output  reg             scl         ,  //i2cʱ��
    inout   wire            sda            //i2c��������   
    );

//==================================================
//parameter define
//==================================================
parameter   IDLE    = 11'b000_0000_0001;//����״̬
parameter   START   = 11'b000_0000_0010;//д��ʼ
parameter   SLADDR  = 11'b000_0000_0100;//ȷ���豸��ַ
parameter   ADDR16  = 11'b000_0000_1000;//����16λ�ֵ�ַ 
parameter   ADDR8   = 11'b000_0001_0000;//����8λ�ֵ�ַ 
parameter   DATA_WR = 11'b000_0010_0000;//д����
parameter   RD_START= 11'b000_0100_0000;//����ʼ
parameter   ADDR_RD = 11'b000_1000_0000;//�豸��ַ������
parameter   DATA_RD = 11'b001_0000_0000;//������
parameter   STOP    = 11'b010_0000_0000;//ֹͣ
parameter   ERROR   = 11'b100_0000_0000;//����


//==================================================
//internal signals
//==================================================
reg     [2:0]   cnt_freq    ;//����drive_flag ����I2Cʱ��
wire            add_cnt_freq;
wire            end_cnt_freq;

reg     [5:0]   cnt_flag    ;//������ǰ״̬�ж��ٸ�drive_flag
wire            add_cnt_flag;
wire            end_cnt_flag;
reg     [5:0]   x           ;//�ɱ�����������ֵ

reg     [9:0]   cnt         ;//�������������ź�drive_flag
wire            add_cnt     ;
wire            end_cnt     ;

reg             drive_flag  ;//����������ģ�鹤�����ź�
reg     [10:0]  state       ;//state register
reg             work_flag   ;//work flag
reg             sda_dir     ;//��̬������дʹ��
reg     [7:0]   data_shift  ;//��λ�Ĵ���
reg             i2c_ack     ;//��Ӧ�ź�

reg             sda_out     ;
wire            sda_in      ;
wire    [8:0]   clk_divide  ;
wire    [8:0]   MAX  ;
wire    [8:0]   FLAG0       ;
wire    [8:0]   FLAG1       ;
wire    [8:0]   FLAG2       ;
wire    [8:0]   FLAG3       ;

assign  clk_divide = (clk_freq/i2c_freq) >> 2'd2 ;  //ģ������ʱ�ӵķ�Ƶϵ��
assign  MAX   = clk_freq/i2c_freq;//����ʱ�ӵļ������ֵ
assign  FLAG0 = clk_divide - 1'b1 ;
assign  FLAG1 = 2*clk_divide - 1'b1 ;
assign  FLAG2 = 3*clk_divide - 1'b1 ;
assign  FLAG3 = 4*clk_divide - 1'b1 ;

//��̬�˿�����
assign  sda = sda_dir?sda_out:1'bz;//������������������������д����ʱ��sda_dir=1�������û����ݣ�����������ʱ����Ϊ����
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
                    state <= START;//���յ���ʼ�źţ�������ʼ״̬
                else
                    state <= IDLE;
            end 

            START:begin
                if(cnt_flag=='d6 && drive_flag)
                    state <= SLADDR;//��ʼ״̬����������ȷ���豸��ַ״̬
                else
                    state <= START;
            end

            SLADDR:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)//�������豸��ַ��������ȷ���յ���Ӧ
                    if(bit_ctrl)                    //�ж���16λ����8λ�ֵ�ַ
                        state <= ADDR16;
                    else
                        state <= ADDR8 ;
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//�������豸��ַ������û�н��յ���Ӧ
                    state <= ERROR;//ȷ�ϵ�ַ״̬����������ȷ�ϼĴ�����ַ״̬
                else
                    state <= SLADDR;
            end

            ADDR16:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)
                        state <= ADDR8;
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//�������豸��ַ������û�н��յ���Ӧ
                    state <= ERROR;//ȷ�ϵ�ַ״̬����������ȷ�ϼĴ�����ַ״̬
                else 
                    state <= ADDR16;
            end

            ADDR8:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)begin//�Ѿ�����д��ļĴ�����������ȷ���յ���Ӧ
                    if(i2c_rh_wl==1'b0)
                        state <= DATA_WR;//ȷ�ϼĴ�����ַ״̬����������д����״̬
                    else if(i2c_rh_wl==1'b1)
                        state <= RD_START;//ȷ�ϼĴ�����ַ״̬�������������ʼ״̬
                end
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//�������豸��ַ������û�н��յ���Ӧ
                    state <= ERROR;//ȷ�ϵ�ַ״̬����������ȷ�ϼĴ�����ַ״̬
                else 
                    state <= ADDR8;
            end

            DATA_WR:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)//�Ѿ�����д�����ݲ���ȷ���յ���Ӧ
                    state <= STOP;//����д����ɽ���ֹͣ״̬
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//�������豸��ַ������û�н��յ���Ӧ
                    state <= ERROR;//ȷ�ϵ�ַ״̬����������ȷ�ϼĴ�����ַ״̬
                else
                    state <= DATA_WR;
            end

            RD_START:begin
                if(cnt_flag=='d3 && drive_flag)
                    state <= ADDR_RD;//�����豸��ַ������
                else
                    state <= RD_START;
            end

            ADDR_RD:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)
                    state <= DATA_RD;//���������״̬
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//�������豸��ַ������û�н��յ���Ӧ
                    state <= ERROR;//ȷ�ϵ�ַ״̬����������ȷ�ϼĴ�����ַ״̬
                else
                    state <= ADDR_RD;
            end

            DATA_RD:begin
                if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b1)
                    state <= STOP;
                else if(cnt_flag=='d35 && drive_flag && i2c_ack==1'b0)//�������豸��ַ������û�н��յ���Ӧ
                    state <= ERROR;//ȷ�ϵ�ַ״̬����������ȷ�ϼĴ�����ַ״̬
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
    else if(state==START)begin//���յ���ʼ�ź�
        work_flag <= 1'b1;
    end
    else if(state==IDLE)begin
        work_flag <= 1'b0;
    end
    else if(i2c_done==1'b1)begin//һ�ζ�д���
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

assign add_cnt = work_flag;//���ڹ���״̬ʱһֱ����       
assign end_cnt = add_cnt && cnt== MAX;//���������ֵ����

//--------------------drive_flag--------------------
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        drive_flag <= 1'b0;
    end
    else if(cnt==FLAG0 || cnt==FLAG1 || cnt==FLAG2 || cnt==FLAG3)begin//����һ�������ź�
        drive_flag <= 1'b1;
    end
    else begin
        drive_flag <= 1'b0;
    end
end

//--------------------cnt_freq--------------------
//�������źŽ��м������Դ�������I2Cʱ��
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
//������ǰ״̬���ж��ٸ�drive_flag
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
//xΪ��ͬ״̬�£��������ļ������ֵ
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
        sda_dir <= 1'b1;//��д��ʼ������ʼ���ͽ���״̬�����û������������ߣ�������ʼ����ֹͣ�źţ�����д����ʹ����Ч
    end
    else if(state==SLADDR || state==ADDR16 ||state==ADDR8 ||state==DATA_WR || state==ADDR_RD)begin
        if(cnt_flag < 'd32)begin//�����û�����ʱ��д����ʹ����Ч
            sda_dir <= 1'b1;
        end
        else begin
            sda_dir <= 1'b0;//�ȴ����豸��Ӧʱ��д����ʹ����Ч
        end
    end
    else if(state==DATA_RD)begin
        if(cnt_flag < 'd32)begin
            sda_dir <= 1'b0;//��������״̬����ʱ�ɴӻ��������ݸ�������д����ʹ����Ч
        end
        else begin
            sda_dir <= 1'b1;//����������ɣ�������Ҫ�Դӻ�����Ӧ��д����ʹ����Ч
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
                data_shift <= 'd0;//����״̬������λ�Ĵ�������Ϊ0
            end

            START:begin
                data_shift <= {sensor_addr[6:0],1'b0};//д��ʼ״̬�������豸дָ��
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
                    data_shift <= i2c_addr[7:0];//ȷ���豸״̬����ʱ�������Ĵ�����ַ
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            ADDR8:begin
                if(end_cnt_flag && i2c_ack==1'b1 && i2c_rh_wl==1'b0)//��������д����״̬
                    data_shift <= i2c_data_w;//ȷ�ϼĴ�����ַ״̬���������Ҫд�������
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            DATA_WR:begin
                if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};//������д�뵽�Ĵ�����
                else
                   data_shift <= data_shift; 
            end

            RD_START:begin
                data_shift <=  {sensor_addr[6:0],1'b1};//����ʼʱ�����������������λ�Ĵ���
            end


            ADDR_RD:begin
                if(end_cnt_flag && i2c_ack==1'b1)
                    data_shift <= 'd0;
                else if(cnt_flag<'d32 && cnt_flag[1:0]==2'd3 && drive_flag)
                    data_shift <= {data_shift[6:0],1'b0};
            end

            DATA_RD:begin
                if(cnt_flag<'d32 && cnt_flag[1:0]==2'd1 && drive_flag)
                    data_shift <= {data_shift[6:0],sda_in};//���ӼĴ����ж����������������λ�Ĵ���
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
                    sda_out <= 1'b0;//������ʼλ
                else
                    sda_out <= sda_out;
            end
 
            SLADDR,ADDR16,ADDR8,DATA_WR,ADDR_RD:begin
                sda_out <= data_shift[7];//�����ݷ���������������
            end

            RD_START:begin
                if(cnt_flag=='d0)
                    sda_out <= 1'b1;//��������ʼλ
                else if(cnt_flag=='d1 && drive_flag)
                    sda_out <= 1'b0;
            end

            DATA_RD:begin
                if(cnt_flag>='d32)
                    sda_out <= 1'b1;//����NACK
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
    else if(state==STOP && end_cnt_flag)begin//�����������ζ�д������ʱ�򣬲�������ź�
            i2c_done <= 1'b1;
    end
    else begin
        i2c_done <= 1'b0;
    end
end

//--------------------i2c_ack--------------------
//�Ƿ���յ�ACK���߲���NACK
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        i2c_ack <= 1'b0;
    end
    else begin
        case(state)
            SLADDR:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//д���豸��ַ�����ҽ��յ���Ӧ
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            ADDR16:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//д��Ĵ�����ַ�����յ���Ӧ
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            ADDR8:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//д��Ĵ�����ַ�����յ���Ӧ
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            DATA_WR:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//д�����ݣ����ҽ��յ���Ӧ
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            ADDR_RD:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b0)
                    i2c_ack <= 1'b1;//��ָ�����ϣ����յ���Ӧ
                else if(end_cnt_flag)
                    i2c_ack <= 1'b0;
            end

            DATA_RD:begin
                if(cnt_flag>='d32 && cnt_flag[1:0]=='d1 && drive_flag && sda==1'b1)
                    i2c_ack <= 1'b1;//����ȫ�����꣬��������NACK
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
    else if(i2c_done)begin//�����������ζ�д������ʱ�򣬲�������ź�
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
    else if(state==ERROR)begin//�����������ζ�д������ʱ�򣬲�������ź�
        err_flag <= 1'b1;
    end
    else begin
        err_flag <= 1'b0;
    end
end

endmodule
