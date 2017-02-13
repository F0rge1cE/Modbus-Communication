#include "DSP28x_Project.h"// Device Headerfile and Examples Include File
#include "cmd_and_state.h"
#include "DSP281x_ModbusModule.h"

#define RX_EN       GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1 //485芯片需要的发送/接收使能信号
#define TX_EN       GpioDataRegs.GPBSET.bit.GPIO61 = 1   //参考板子原理图，根据具体板子需要修改

interrupt void scicTxFifoIsr(void);//fifo发送中断函数
interrupt void scicRxFifoIsr(void);//fifo接收中断函数

void scic_fifo_init(void);//scib fifo模式 初始化函数
void ModbusModule_init(void); //ModbusModule结构初始化函数
void Delay(unsigned int j);

volatile struct MODBUS ModbusModule;  //Modbus结构变量定义见DSP281x_ModbusModule.h

////////////////帧构建与解释函数定义///////////////////////////////////////////////////
Uint16 RTU_Master_FrameAnalyse(Uint16  *dest_p);   //主站帧解析函数
void  ConstructFrame_RTU_Read_HldReg( Uint16 board_adr,Uint16 start_address,Uint16 lenth);  //code 03
void  ConstructFrame_RTU_Set_N_HldReg( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth); //code 16
void  ConstructFrame_RTU_Set_N_Coil( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth); //code 15
void  ConstructFrame_RTU_Read_N_Coil( Uint16 board_adr,Uint16 start_address,Uint16 lenth) ;//code 01
////////////////////////////////////////////////////////////////////////////////////

Uint16 GetCRC16(Uint16 volatile *ptr, Uint16 len); //CRC计算函数

//Uint16 CRC_Check(Uint16 *CRC_buf,Uint16 BuffLen);

Uint16 Word_Hi(Uint16 input); //提取高字节函数
Uint16 Word_Lo(Uint16 input); //提取低字节函数


//unsigned int tx_buffer [100];//发送数据缓冲数组
//unsigned int rx_buffer [8]; //接收数据缓存

Uint16 tx_buffer [32];//发送数据缓冲数组
Uint16 rx_buffer [32]; //接收数据缓存

Uint16 Slave_response[4][32];//帧分析的结果——读到的数据或写操作执行结果

Uint16 Set_coil_data[8]; //欲设置位的数据
Uint16 Set_regs_data[32]; //欲设置word的数据

Uint16 Frame_type_send = 0; //发送帧的类型
//////////0:16Bit set   1:4Bit read  2:8Byte write   3:4Byte read_1  4:4Byte read_2

unsigned int f1=0,f2=0;  //f1统计发送次数  f2统计接收次数

void main(void)
{
   Uint16 i,j;

   InitSysCtrl();//系统时钟等初始化

   InitSciGpio();//sci端口初始化
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

    EDIS;

   DINT; //关掉中断

   InitPieCtrl(); //pie模块初始化

   IER = 0x0000;//关cpu中断
   IFR = 0x0000;//清cpu中断标志

   InitPieVectTable();//初始化中断向量表,  经试验   此句注释掉也可以 ，因为下边对中断向量表进行了重新赋值

   EALLOW; // This is needed to write to EALLOW protected registers
   PieVectTable.SCIRXINTC = &scicRxFifoIsr; //中断向量表重新赋值（自定义中断函数的入口地址）
   PieVectTable.SCITXINTC = &scicTxFifoIsr; //中断向量表重新赋值
   EDIS;   // This is needed to disable write to EALLOW protected registers

   scic_fifo_init();  // 初始化 SCI-B寄存器配置
                     EALLOW;
                     GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;
                     GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
                     GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
                     EDIS;
                     TX_EN;


   for(i = 0; i<32; i++) //发送缓存初始化
   {
      tx_buffer[i] = 0;
   }
   for(i = 0; i<32; i++) //接收缓存初始化
     {
        rx_buffer[i] = 0;
     }
   for(j = 0; j<4; j++) //帧分析结果缓存初始化
	  {
	  	for (i = 0; i < 32; i++){
	        Slave_response[j][i] = 0;
	  	}
	  }

   ModbusModule_init(); //ModbusModule结构初始化

/****************************************************/
/*****************构建测试帧 调试区域*****************/
/****************************************************/

 /*  tx_buffer[0] = 0x01;  //测试用帧:读0x0000
   tx_buffer[1] = 0x03;
   tx_buffer[2] = 0x00;
   tx_buffer[3] = 0x00;
   tx_buffer[4] = 0x00;
   tx_buffer[5] = 0x01;
   tx_buffer[6] = 0x84;
   tx_buffer[7] = 0x0A;  */

  /* tx_buffer[0] = 0x01;  //测试用帧:写 0x0001
     tx_buffer[1] = 0x06;
     tx_buffer[2] = 0x00;
     tx_buffer[3] = 0x01;
     tx_buffer[4] = 0x1A;
   	 tx_buffer[5] = 0x0A;
   	 tx_buffer[6] = 0x53;
   	 tx_buffer[7] = 0x6D; */

  //ConstructFrame_RTU_Read_HldReg(0x01 ,0x000A,0x0001);
  // Frame_type_send = 3;
  // ModbusModule.Rxlen = 7 ;


   for(i = 0; i<32; i++) //测试帧
           {
   	    Set_regs_data[i] = 0x1011;
           }
      ConstructFrame_RTU_Set_N_HldReg( 0x01,Set_regs_data,0x0000,0x0001);
      ModbusModule.Rxlen = 8;
      ModbusModule.FunCode = 0x10;
      Frame_type_send = 2;


      //for(i = 0; i<32; i++) //测试帧
      //    {
     	// Set_coil_data[i] = 0x1111;
      //                   }

      //ConstructFrame_RTU_Set_N_Coil(0x01,Set_coil_data,0x2000,0x0010);

/****************************************************/
/****************测试帧构建完成***********************/
/****************************************************/


   
      for(i = 0; i<32; i++)
      {
         tx_buffer[i] = ModbusModule.Buf[i];
      }





   // Enable interrupts required for this. example 中断使能
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER8.bit.INTx5=1;     // PIE Group 8, INT5  SCIRXINTC SCIC
   PieCtrlRegs.PIEIER8.bit.INTx6=1;     // PIE Group 8, INT6   SCITXINTC SCIC
   IER = 0x80;   // Enable CPU INT
   EINT;//开总中断 INTM

   // Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)

	   ;

}

interrupt void scicTxFifoIsr(void) //fifo发送中断服务子程序
{
    Uint16 i;
	f1++;
	
/*	switch(Frame_type_send){

	    case(0):{
	    	ConstructFrame_RTU_Set_N_Coil( 0x01,Set_coil_data,0x2000,0x0010);
	    	ModbusModule.Rxlen = 8;
	    	ModbusModule.FunCode = 0x0F;
	    	
	    	Frame_type_send++;
	    	break;
	    }
	    
	    case(1):{
	    	ConstructFrame_RTU_Read_N_Coil( 0x01,0x2001,0x0004);
	    	ModbusModule.Rxlen = 6;
	    	ModbusModule.FunCode = 0x01;
	    	
	    	Frame_type_send++;
	    	break;
	    }
	    
	    case(2):{
	    	ConstructFrame_RTU_Set_N_HldReg( 0x01,Set_regs_data,0x0000,0x0008);
	    	ModbusModule.Rxlen = 8;
	    	ModbusModule.FunCode = 0x10;
	    	
	    	Frame_type_send++;
	    	break;
	    }
	    
	    case(3):{
	    	ConstructFrame_RTU_Read_HldReg( 0x01,0x0008,0x0004);
	    	ModbusModule.Rxlen = 13;
	    	ModbusModule.FunCode = 0x03;
	    	
	    	Frame_type_send++;
	    	break;
	    }
	    
	    case(4):{
	    	ConstructFrame_RTU_Read_HldReg( 0x01,0x000C,0x0004); //扩充到4word，方便接收
	    	ModbusModule.Rxlen = 13;
	    	ModbusModule.FunCode = 0x03;
	    	
	    	Frame_type_send = 0;
	    	break;
	    }
	    default:
	    {
	    	Frame_type_send = 0;
	        break;
	    }
	    
	}   */
	
	
	for(i = 0; i<ModbusModule.Txlen; i++)
      {
         tx_buffer[i] = ModbusModule.Buf[i];
      }	

    for(i=0; i< ModbusModule.Txlen; i++)
    {
       ScicRegs.SCITXBUF=tx_buffer[i];     // Send data 将缓存中的数据发出
       Delay(50000);
       while(ScicRegs.SCICTL2.bit.TXRDY != 1) {} //发送超过FIFO深度时的数据要等待

    }

	RX_EN;
	//开启MAX485芯片的RX功能，因HMI响应时间未知，需要尽快开启？

   //ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;  //
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1; // 清接收中断标志

    PieCtrlRegs.PIEACK.all|=0x80;      // Issue PIE ACK
}

interrupt void scicRxFifoIsr(void)//fifo接收中断服务子程序
{
    Uint16 i;
	f2++;
	
    for(i=0; i< ModbusModule.Rxlen; i++)
    {
    	rx_buffer[i]=ScicRegs.SCIRXBUF.bit.RXDT; // 将fifo中的数据读到缓存
      Delay(50000);
    }
    TX_EN;//开启MAX485芯片的TX功能

//	GpioDataRegs.GPBSET.bit.GPIO61 = 1;

    for(i = 0; i<32; i++)
          {
             ModbusModule.Buf[i] = rx_buffer[i];
          }

    if (Frame_type_send == 0){
        RTU_Master_FrameAnalyse(Slave_response[4]);
    }
    else{
    	RTU_Master_FrameAnalyse(Slave_response[Frame_type_send-1]);
    }


    ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;  // 很重要 若不清fifo发送中断标志则，不进入发送中断
    //ScicRegs.SCIFFRX.bit.RXFFINTCLR=1; // 清接收中断标志

    PieCtrlRegs.PIEACK.all|=0x80;      // Issue PIE ack
}

void scic_fifo_init()
{
   //ScicRegs.SCICCR.all =0x0017; // enable loopback (for test only)

  ScicRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
   ScicRegs.SCICTL1.all =0x0003;   // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
   ScicRegs.SCIHBAUD    =0x0001;
   ScicRegs.SCILBAUD    =0x00e7;  //波特率9600

   ScicRegs.SCIFFTX.bit.SCIFFENA = 1;//使能sci fifo功能
   ScicRegs.SCIFFTX.bit.TXFFIENA = 1;//fifo 发送中断使能
   ScicRegs.SCIFFTX.bit.TXFFIL = 0; //发送中断级别，此时为上电默认值0

   ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;//清接收fifo溢出标志
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;//不清除接收fifo中断标志位
   ScicRegs.SCIFFRX.bit.RXFFIENA = 1;//使能fifo 接收中断

   ScicRegs.SCIFFRX.bit.RXFFIL = 1; //fifo接收中断级别为1

   ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;
   ScicRegs.SCIFFCT.all=0x00; //为默认值 效果为  禁用自动波特率调整  fifo传送延时为0

   ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset  重启sci
   ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;//重新使能发送fifo操作
   ScicRegs.SCIFFRX.bit.RXFIFORESET=1;//重新使能接收fifo操作
  // ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset  重启sci


}


/* Delta 官方提供参考CRC-16 计算程序
// CRC_buf 傳入的通訊字元, BuffLen 通訊字元的長度
Uint16 CRC_Check(Uint16 *CRC_buf,Uint16 BuffLen) {
Uint16 CRC_ReturnValue = 0xFFFF;
Uint16 i=0,j;
while ( BuffLen -- ) {
CRC_ReturnValue ^= CRC_buf[i++]; j = 8;
do{
if ( CRC_ReturnValue & 0x01 ) {
CRC_ReturnValue = ( CRC_ReturnValue >> 1 ) ^ 0xA001; }
else {
CRC_ReturnValue = CRC_ReturnValue >> 1; }
-- j; }
while ( j ); }
return CRC_ReturnValue; // 回傳 CRC-16 計算結果
}
*/


Uint16 GetCRC16(Uint16 volatile *ptr, Uint16 len)
//产生的CRC是 高位在MSB 低位在LSB，根据协议要调整
{
	Uint16 i;
	Uint16 crc=0xFFFF;
	if(len==0)
	{
	     len=1;
	}
    while(len--){
	  crc^=(*ptr);
	  for(i=0; i<8; i++){
	      if(crc&1){
	          crc>>=1;
	          crc^=0xA001;
	      }
	      else{
	          crc>>=1;
	      }
	  }
	        ptr++;
	}



	return(crc);
}

//---------------------------------------------
//CMD: READ_HLD_REG
//读取寄存器，function code = 0x03
//---------------------------------------------
void  ConstructFrame_RTU_Read_HldReg( Uint16 board_adr,Uint16 start_address,Uint16 lenth)
{
	Uint16 i = 0 , j = 0;
	board_adr = 0x01;

	ModbusModule.Buf[i++] =  board_adr;    //从机地址，本项目中均为0x01
	ModbusModule.Buf[i++] = READ_HLD_REG;  //function code = 0x03
	ModbusModule.Buf[i++] = Word_Hi(start_address); //地址-高字节
	ModbusModule.Buf[i++] = Word_Lo(start_address); //地址-低字节
	ModbusModule.Buf[i++] = Word_Hi(lenth); //读取字节数-高字节
	ModbusModule.Buf[i++] = Word_Lo(lenth); //读取字节数-低字节

	j = GetCRC16(ModbusModule.Buf,i);
	ModbusModule.Buf[i++] = Word_Lo(j); //CRC校验-高字节
	ModbusModule.Buf[i++] = Word_Hi(j); //CRC校验-低字节

	ModbusModule.Txlen = i;
	ModbusModule.Point = 0;

	ModbusModule.FunCode =0x03;  //////要不要写呢。。。？？？

/*	ModbusModule.FunCode =
	ModbusModule.Txlen = i;
	ModbusModule.Rxlen =
	ModbusModule.Point = 0;
	ModbusModule.ID =
	ModbusModule.Status =
*/
}


//---------------------------------------------
//CMD: SET_N_HLD_REG
//写N个寄存器，function code = 0x10
//com_buf是数据暂时存放的数组
//---------------------------------------------
void  ConstructFrame_RTU_Set_N_HldReg( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth)
{
	Uint16 i = 0 , j = 0;
	board_adr = 0x01;

	ModbusModule.Buf[i++] =  board_adr;    //从机地址，本项目中均为0x01
	ModbusModule.Buf[i++] = SET_N_HLD_REG;  //function code = 0x10
	ModbusModule.Buf[i++] = Word_Hi(start_address); //地址-高字节
	ModbusModule.Buf[i++] = Word_Lo(start_address); //地址-低字节
	ModbusModule.Buf[i++] = Word_Hi(lenth); //写入字节数-高字节
	ModbusModule.Buf[i++] = Word_Lo(lenth); //写入字节数-低字节


	ModbusModule.Buf[i++] = lenth<<1; //计算Byte_count

	for(j=0;j<lenth;j++){
	    ModbusModule.Buf[i++] = Word_Hi( * (com_buf+j) );
	    ModbusModule.Buf[i++] = Word_Lo( * (com_buf+j) );
	 }

	j = GetCRC16(ModbusModule.Buf,i);
	ModbusModule.Buf[i++] = Word_Lo(j); //CRC校验-高字节
	ModbusModule.Buf[i++] = Word_Hi(j); //CRC校验-低字节

	ModbusModule.Txlen = i;
	ModbusModule.Point = 0;

	/*	ModbusModule.FunCode =
		ModbusModule.Txlen = i;
		ModbusModule.Rxlen =
		ModbusModule.Point = 0;
		ModbusModule.ID =
		ModbusModule.Status =
	*/

}


//---------------------------------------------
//CMD: SET_N_COIL
//写N个位（线圈coil），function code = 0x0F
//com_buf是数据暂时存放的数组
//---------------------------------------------
void  ConstructFrame_RTU_Set_N_Coil ( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth)
{
	Uint16 i = 0 , j = 0;
	Uint16 byte_num=0,word_num=0;
	board_adr = 0x01;

	ModbusModule.Buf[i++] =  board_adr;    //从机地址，本项目中均为0x01
	ModbusModule.Buf[i++] = SET_N_COIL;  //function code = 0x0F
	ModbusModule.Buf[i++] = Word_Hi(start_address); //地址-高字节
	ModbusModule.Buf[i++] = Word_Lo(start_address); //地址-低字节
	ModbusModule.Buf[i++] = Word_Hi(lenth); //写入位的个数-高字节
	ModbusModule.Buf[i++] = Word_Lo(lenth); //写入位的个数-低字节

	byte_num =(lenth+7)>>3; //计算Byte_count
	word_num =byte_num>>1;  //1个Uint16的Word等于两个Byte

	ModbusModule.Buf[i++] = byte_num;

	for(j=0;j<word_num;j++){     //数据字节构成

	  ModbusModule.Buf[i++] = Word_Hi( * (com_buf+j) );
	  ModbusModule.Buf[i++] = Word_Lo( * (com_buf+j) );
	 }
	if(byte_num & BIT0){
		 ModbusModule.Buf[i++] = Word_Lo( * (com_buf+j) );
	 }
	//意外情况处理：当byte_num=1时，word_num=0,进入if语句块，从而写入数据块

	j = GetCRC16(ModbusModule.Buf,i);
	ModbusModule.Buf[i++] = Word_Lo(j); //CRC校验-高字节
	ModbusModule.Buf[i++] = Word_Hi(j); //CRC校验-低字节

	ModbusModule.Txlen = i;
	ModbusModule.Point = 0;

	/*	ModbusModule.FunCode =
		ModbusModule.Txlen = i;
		ModbusModule.Rxlen =
		ModbusModule.Point = 0;
		ModbusModule.ID =
		ModbusModule.Status =  */

}

//---------------------------------------------
//CMD: READ_N_COIL
//读取寄存器，function code = 0x01
//---------------------------------------------
void  ConstructFrame_RTU_Read_N_Coil ( Uint16 board_adr,Uint16 start_address,Uint16 lenth)
{
	    Uint16 i = 0 , j = 0;
		board_adr = 0x01;

		ModbusModule.Buf[i++] =  board_adr;    //从机地址，本项目中均为0x01
		ModbusModule.Buf[i++] = READ_N_COIL;  //function code = 0x01
		ModbusModule.Buf[i++] = Word_Hi(start_address); //地址-高字节
		ModbusModule.Buf[i++] = Word_Lo(start_address); //地址-低字节
		ModbusModule.Buf[i++] = Word_Hi(lenth); //读取位的个数-高字节
		ModbusModule.Buf[i++] = Word_Lo(lenth); //读取位的个数-低字节

		j = GetCRC16(ModbusModule.Buf,i);
		ModbusModule.Buf[i++] = Word_Lo(j); //CRC校验-高字节
		ModbusModule.Buf[i++] = Word_Hi(j); //CRC校验-低字节

		ModbusModule.Txlen = i;
		ModbusModule.Point = 0;

	/*	ModbusModule.FunCode =
		ModbusModule.Txlen = i;
		ModbusModule.Rxlen =
		ModbusModule.Point = 0;
		ModbusModule.ID =
		ModbusModule.Status =
	*/

}



//---------------------------------------------
//Master收到Slave返回的应答帧，接收到的帧也放在ModbusModule中
//应答帧解析函数
//Return:
//1——CRC校验不通过
//2——站号不匹配——————未使用
//3——功能码不对应
//4——function code 0x10回应地址不匹配
//5——function code 0x10回应数据字数不匹配
//---------------------------------------------
Uint16 RTU_Master_FrameAnalyse(Uint16  *dest_p) //*dest_p为存放分析结果数据的数组    接收到的结果应该在ModbusModule.Buf[]中
{
    Uint16 crc_rx_temp; //接收到的帧中的CRC值，由两个字节拼成
    Uint16 crc_self_calculate; //主机计算的CRC值，用于校验

  //  Uint16 RegAddr,RegNum;
    Uint16 rx_byte_num;
    Uint16 i;

    crc_rx_temp  = ModbusModule.Buf[ModbusModule.Rxlen-1]<<8; //接收到最后1字节是CRC HI Byte
    crc_rx_temp += ModbusModule.Buf[ModbusModule.Rxlen-2]; //接收到倒数第2字节是CRC LO Byte

    crc_self_calculate=GetCRC16(ModbusModule.Buf,ModbusModule.Rxlen-2);

    if (crc_self_calculate != crc_rx_temp)
    	{return 1;} //1——CRC校验不通过

    //if ( ModbusModule.ID != ModbusModule.Buf[0] )     //站号在本应用中为固定值，不用判断
    //  {return 2;} //2——站号不匹配——slave用

    if ( ModbusModule.FunCode != ModbusModule.Buf[1] )
        {return 3;} //3——功能码不对应

   // ModbusModule.FunCode=ModbusModule.Buf[1];

   //////////////与接收有关  switch (ModbusModule.FunCode){
    switch (ModbusModule.Buf[1]){

    case(READ_HLD_REG):  //function code = 0x03
		 for (i=0; i<ModbusModule.Buf[2];i+=2){
		     *(dest_p + i/2) = (ModbusModule.Buf[i+3]<<8) + ModbusModule.Buf[i+4];
		         }

        break;

    case(SET_N_HLD_REG):  //function code = 0x10

	/*	RegAddr= (ModbusModule.Buf[2]<<8) + ModbusModule.Buf[3]; //接收到帧中的地址
		RegNum = (ModbusModule.Buf[4]<<8) + ModbusModule.Buf[5]; //接收到帧中的字节数
		   if ( RegAddr != ModbusModule.RegAddr ){  //需要初始化ModbusModule
		    return 4;
		   }
		   if ( RegNum  != ModbusModule.RegNum ){
		    return 5;
		   }
    */
		break;

    case(SET_N_COIL):  //function code = 0x0F

		break;

    case(READ_N_COIL):  //function code = 0x01
		 rx_byte_num=ModbusModule.Buf[2];

		  for (i=0;i<rx_byte_num; i+=2){
		    *(dest_p + i/2) = (ModbusModule.Buf[i+4]<<8) + ModbusModule.Buf[i+3];
		        }     //参考协议，编号大的位放在较后的字节
		              //得到的结果，是由每16位，按顺序组成的一个Uint16

		  if(rx_byte_num&BIT0){
		    *(dest_p + i/2) &=0xFF00; //读取的小于等于8位时，Uint16的高8位默认置1
		    *(dest_p + i/2) |=ModbusModule.Buf[i+3]&0xFF; //low 8 bit renew
		  }

		break;

    }

    return 0;



}



////////////////////////////////////////////////////
/////////////////////辅助函数////////////////////////
////////////////////////////////////////////////////

void Delay(unsigned int j) //非严格的延迟函数，进一步可以调用DSP2833x_usDelay.asm
{
	unsigned int k;   //可能会被编译器优化掉，待查！！！
    for (k=0;k<j;k++);
}

Uint16 Word_Hi(Uint16 input) //提取16位中高8位（高字节）函数
{
	input = (input & 0xFF00) >>8 ;
	return input;
}

Uint16 Word_Lo(Uint16 input) //提取16位中低8位（低字节）函数
{
	input = (input & 0x00FF) ;
	return input;
}

void ModbusModule_init(void){
	Uint16 i;
	for( i = 0;i < 256 ;i++ )
	    {
	        ModbusModule.Buf[i] = 0;
	    }
	    ModbusModule.Txlen  = 0;
	    ModbusModule.Rxlen  = 0;
	    ModbusModule.Point  = 0;
	    ModbusModule.ID  = 1;
	    ModbusModule.FunCode= SET_N_COIL;
	  	ModbusModule.Status = IDLE_WAIT;
}
////////////////////////////////////////////////////
//////////////END OF FILE///////////////////////////
////////////////////////////////////////////////////

