#include "DSP28x_Project.h"// Device Headerfile and Examples Include File
#include "cmd_and_state.h"
#include "DSP281x_ModbusModule.h"

#define RX_EN       GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1 //485оƬ��Ҫ�ķ���/����ʹ���ź�
#define TX_EN       GpioDataRegs.GPBSET.bit.GPIO61 = 1   //�ο�����ԭ��ͼ�����ݾ��������Ҫ�޸�

interrupt void scicTxFifoIsr(void);//fifo�����жϺ���
interrupt void scicRxFifoIsr(void);//fifo�����жϺ���

void scic_fifo_init(void);//scib fifoģʽ ��ʼ������
void ModbusModule_init(void); //ModbusModule�ṹ��ʼ������
void Delay(unsigned int j);

volatile struct MODBUS ModbusModule;  //Modbus�ṹ���������DSP281x_ModbusModule.h

////////////////֡��������ͺ�������///////////////////////////////////////////////////
Uint16 RTU_Master_FrameAnalyse(Uint16  *dest_p);   //��վ֡��������
void  ConstructFrame_RTU_Read_HldReg( Uint16 board_adr,Uint16 start_address,Uint16 lenth);  //code 03
void  ConstructFrame_RTU_Set_N_HldReg( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth); //code 16
void  ConstructFrame_RTU_Set_N_Coil( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth); //code 15
void  ConstructFrame_RTU_Read_N_Coil( Uint16 board_adr,Uint16 start_address,Uint16 lenth) ;//code 01
////////////////////////////////////////////////////////////////////////////////////

Uint16 GetCRC16(Uint16 volatile *ptr, Uint16 len); //CRC���㺯��

//Uint16 CRC_Check(Uint16 *CRC_buf,Uint16 BuffLen);

Uint16 Word_Hi(Uint16 input); //��ȡ���ֽں���
Uint16 Word_Lo(Uint16 input); //��ȡ���ֽں���


//unsigned int tx_buffer [100];//�������ݻ�������
//unsigned int rx_buffer [8]; //�������ݻ���

Uint16 tx_buffer [32];//�������ݻ�������
Uint16 rx_buffer [32]; //�������ݻ���

Uint16 Slave_response[4][32];//֡�����Ľ���������������ݻ�д����ִ�н��

Uint16 Set_coil_data[8]; //������λ������
Uint16 Set_regs_data[32]; //������word������

Uint16 Frame_type_send = 0; //����֡������
//////////0:16Bit set   1:4Bit read  2:8Byte write   3:4Byte read_1  4:4Byte read_2

unsigned int f1=0,f2=0;  //f1ͳ�Ʒ��ʹ���  f2ͳ�ƽ��մ���

void main(void)
{
   Uint16 i,j;

   InitSysCtrl();//ϵͳʱ�ӵȳ�ʼ��

   InitSciGpio();//sci�˿ڳ�ʼ��
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

   DINT; //�ص��ж�

   InitPieCtrl(); //pieģ���ʼ��

   IER = 0x0000;//��cpu�ж�
   IFR = 0x0000;//��cpu�жϱ�־

   InitPieVectTable();//��ʼ���ж�������,  ������   �˾�ע�͵�Ҳ���� ����Ϊ�±߶��ж���������������¸�ֵ

   EALLOW; // This is needed to write to EALLOW protected registers
   PieVectTable.SCIRXINTC = &scicRxFifoIsr; //�ж����������¸�ֵ���Զ����жϺ�������ڵ�ַ��
   PieVectTable.SCITXINTC = &scicTxFifoIsr; //�ж����������¸�ֵ
   EDIS;   // This is needed to disable write to EALLOW protected registers

   scic_fifo_init();  // ��ʼ�� SCI-B�Ĵ�������
                     EALLOW;
                     GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0;
                     GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
                     GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
                     EDIS;
                     TX_EN;


   for(i = 0; i<32; i++) //���ͻ����ʼ��
   {
      tx_buffer[i] = 0;
   }
   for(i = 0; i<32; i++) //���ջ����ʼ��
     {
        rx_buffer[i] = 0;
     }
   for(j = 0; j<4; j++) //֡������������ʼ��
	  {
	  	for (i = 0; i < 32; i++){
	        Slave_response[j][i] = 0;
	  	}
	  }

   ModbusModule_init(); //ModbusModule�ṹ��ʼ��

/****************************************************/
/*****************��������֡ ��������*****************/
/****************************************************/

 /*  tx_buffer[0] = 0x01;  //������֡:��0x0000
   tx_buffer[1] = 0x03;
   tx_buffer[2] = 0x00;
   tx_buffer[3] = 0x00;
   tx_buffer[4] = 0x00;
   tx_buffer[5] = 0x01;
   tx_buffer[6] = 0x84;
   tx_buffer[7] = 0x0A;  */

  /* tx_buffer[0] = 0x01;  //������֡:д 0x0001
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


   for(i = 0; i<32; i++) //����֡
           {
   	    Set_regs_data[i] = 0x1011;
           }
      ConstructFrame_RTU_Set_N_HldReg( 0x01,Set_regs_data,0x0000,0x0001);
      ModbusModule.Rxlen = 8;
      ModbusModule.FunCode = 0x10;
      Frame_type_send = 2;


      //for(i = 0; i<32; i++) //����֡
      //    {
     	// Set_coil_data[i] = 0x1111;
      //                   }

      //ConstructFrame_RTU_Set_N_Coil(0x01,Set_coil_data,0x2000,0x0010);

/****************************************************/
/****************����֡�������***********************/
/****************************************************/


   
      for(i = 0; i<32; i++)
      {
         tx_buffer[i] = ModbusModule.Buf[i];
      }





   // Enable interrupts required for this. example �ж�ʹ��
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER8.bit.INTx5=1;     // PIE Group 8, INT5  SCIRXINTC SCIC
   PieCtrlRegs.PIEIER8.bit.INTx6=1;     // PIE Group 8, INT6   SCITXINTC SCIC
   IER = 0x80;   // Enable CPU INT
   EINT;//�����ж� INTM

   // Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)

	   ;

}

interrupt void scicTxFifoIsr(void) //fifo�����жϷ����ӳ���
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
	    	ConstructFrame_RTU_Read_HldReg( 0x01,0x000C,0x0004); //���䵽4word���������
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
       ScicRegs.SCITXBUF=tx_buffer[i];     // Send data �������е����ݷ���
       Delay(50000);
       while(ScicRegs.SCICTL2.bit.TXRDY != 1) {} //���ͳ���FIFO���ʱ������Ҫ�ȴ�

    }

	RX_EN;
	//����MAX485оƬ��RX���ܣ���HMI��Ӧʱ��δ֪����Ҫ���쿪����

   //ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;  //
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1; // ������жϱ�־

    PieCtrlRegs.PIEACK.all|=0x80;      // Issue PIE ACK
}

interrupt void scicRxFifoIsr(void)//fifo�����жϷ����ӳ���
{
    Uint16 i;
	f2++;
	
    for(i=0; i< ModbusModule.Rxlen; i++)
    {
    	rx_buffer[i]=ScicRegs.SCIRXBUF.bit.RXDT; // ��fifo�е����ݶ�������
      Delay(50000);
    }
    TX_EN;//����MAX485оƬ��TX����

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


    ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;  // ����Ҫ ������fifo�����жϱ�־�򣬲����뷢���ж�
    //ScicRegs.SCIFFRX.bit.RXFFINTCLR=1; // ������жϱ�־

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
   ScicRegs.SCILBAUD    =0x00e7;  //������9600

   ScicRegs.SCIFFTX.bit.SCIFFENA = 1;//ʹ��sci fifo����
   ScicRegs.SCIFFTX.bit.TXFFIENA = 1;//fifo �����ж�ʹ��
   ScicRegs.SCIFFTX.bit.TXFFIL = 0; //�����жϼ��𣬴�ʱΪ�ϵ�Ĭ��ֵ0

   ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;//�����fifo�����־
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;//���������fifo�жϱ�־λ
   ScicRegs.SCIFFRX.bit.RXFFIENA = 1;//ʹ��fifo �����ж�

   ScicRegs.SCIFFRX.bit.RXFFIL = 1; //fifo�����жϼ���Ϊ1

   ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;
   ScicRegs.SCIFFCT.all=0x00; //ΪĬ��ֵ Ч��Ϊ  �����Զ������ʵ���  fifo������ʱΪ0

   ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset  ����sci
   ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;//����ʹ�ܷ���fifo����
   ScicRegs.SCIFFRX.bit.RXFIFORESET=1;//����ʹ�ܽ���fifo����
  // ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset  ����sci


}


/* Delta �ٷ��ṩ�ο�CRC-16 �������
// CRC_buf �����ͨӍ��Ԫ, BuffLen ͨӍ��Ԫ���L��
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
return CRC_ReturnValue; // �؂� CRC-16 Ӌ��Y��
}
*/


Uint16 GetCRC16(Uint16 volatile *ptr, Uint16 len)
//������CRC�� ��λ��MSB ��λ��LSB������Э��Ҫ����
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
//��ȡ�Ĵ�����function code = 0x03
//---------------------------------------------
void  ConstructFrame_RTU_Read_HldReg( Uint16 board_adr,Uint16 start_address,Uint16 lenth)
{
	Uint16 i = 0 , j = 0;
	board_adr = 0x01;

	ModbusModule.Buf[i++] =  board_adr;    //�ӻ���ַ������Ŀ�о�Ϊ0x01
	ModbusModule.Buf[i++] = READ_HLD_REG;  //function code = 0x03
	ModbusModule.Buf[i++] = Word_Hi(start_address); //��ַ-���ֽ�
	ModbusModule.Buf[i++] = Word_Lo(start_address); //��ַ-���ֽ�
	ModbusModule.Buf[i++] = Word_Hi(lenth); //��ȡ�ֽ���-���ֽ�
	ModbusModule.Buf[i++] = Word_Lo(lenth); //��ȡ�ֽ���-���ֽ�

	j = GetCRC16(ModbusModule.Buf,i);
	ModbusModule.Buf[i++] = Word_Lo(j); //CRCУ��-���ֽ�
	ModbusModule.Buf[i++] = Word_Hi(j); //CRCУ��-���ֽ�

	ModbusModule.Txlen = i;
	ModbusModule.Point = 0;

	ModbusModule.FunCode =0x03;  //////Ҫ��Ҫд�ء�����������

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
//дN���Ĵ�����function code = 0x10
//com_buf��������ʱ��ŵ�����
//---------------------------------------------
void  ConstructFrame_RTU_Set_N_HldReg( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth)
{
	Uint16 i = 0 , j = 0;
	board_adr = 0x01;

	ModbusModule.Buf[i++] =  board_adr;    //�ӻ���ַ������Ŀ�о�Ϊ0x01
	ModbusModule.Buf[i++] = SET_N_HLD_REG;  //function code = 0x10
	ModbusModule.Buf[i++] = Word_Hi(start_address); //��ַ-���ֽ�
	ModbusModule.Buf[i++] = Word_Lo(start_address); //��ַ-���ֽ�
	ModbusModule.Buf[i++] = Word_Hi(lenth); //д���ֽ���-���ֽ�
	ModbusModule.Buf[i++] = Word_Lo(lenth); //д���ֽ���-���ֽ�


	ModbusModule.Buf[i++] = lenth<<1; //����Byte_count

	for(j=0;j<lenth;j++){
	    ModbusModule.Buf[i++] = Word_Hi( * (com_buf+j) );
	    ModbusModule.Buf[i++] = Word_Lo( * (com_buf+j) );
	 }

	j = GetCRC16(ModbusModule.Buf,i);
	ModbusModule.Buf[i++] = Word_Lo(j); //CRCУ��-���ֽ�
	ModbusModule.Buf[i++] = Word_Hi(j); //CRCУ��-���ֽ�

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
//дN��λ����Ȧcoil����function code = 0x0F
//com_buf��������ʱ��ŵ�����
//---------------------------------------------
void  ConstructFrame_RTU_Set_N_Coil ( Uint16 board_adr,Uint16 *com_buf,Uint16 start_address,Uint16 lenth)
{
	Uint16 i = 0 , j = 0;
	Uint16 byte_num=0,word_num=0;
	board_adr = 0x01;

	ModbusModule.Buf[i++] =  board_adr;    //�ӻ���ַ������Ŀ�о�Ϊ0x01
	ModbusModule.Buf[i++] = SET_N_COIL;  //function code = 0x0F
	ModbusModule.Buf[i++] = Word_Hi(start_address); //��ַ-���ֽ�
	ModbusModule.Buf[i++] = Word_Lo(start_address); //��ַ-���ֽ�
	ModbusModule.Buf[i++] = Word_Hi(lenth); //д��λ�ĸ���-���ֽ�
	ModbusModule.Buf[i++] = Word_Lo(lenth); //д��λ�ĸ���-���ֽ�

	byte_num =(lenth+7)>>3; //����Byte_count
	word_num =byte_num>>1;  //1��Uint16��Word��������Byte

	ModbusModule.Buf[i++] = byte_num;

	for(j=0;j<word_num;j++){     //�����ֽڹ���

	  ModbusModule.Buf[i++] = Word_Hi( * (com_buf+j) );
	  ModbusModule.Buf[i++] = Word_Lo( * (com_buf+j) );
	 }
	if(byte_num & BIT0){
		 ModbusModule.Buf[i++] = Word_Lo( * (com_buf+j) );
	 }
	//�������������byte_num=1ʱ��word_num=0,����if���飬�Ӷ�д�����ݿ�

	j = GetCRC16(ModbusModule.Buf,i);
	ModbusModule.Buf[i++] = Word_Lo(j); //CRCУ��-���ֽ�
	ModbusModule.Buf[i++] = Word_Hi(j); //CRCУ��-���ֽ�

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
//��ȡ�Ĵ�����function code = 0x01
//---------------------------------------------
void  ConstructFrame_RTU_Read_N_Coil ( Uint16 board_adr,Uint16 start_address,Uint16 lenth)
{
	    Uint16 i = 0 , j = 0;
		board_adr = 0x01;

		ModbusModule.Buf[i++] =  board_adr;    //�ӻ���ַ������Ŀ�о�Ϊ0x01
		ModbusModule.Buf[i++] = READ_N_COIL;  //function code = 0x01
		ModbusModule.Buf[i++] = Word_Hi(start_address); //��ַ-���ֽ�
		ModbusModule.Buf[i++] = Word_Lo(start_address); //��ַ-���ֽ�
		ModbusModule.Buf[i++] = Word_Hi(lenth); //��ȡλ�ĸ���-���ֽ�
		ModbusModule.Buf[i++] = Word_Lo(lenth); //��ȡλ�ĸ���-���ֽ�

		j = GetCRC16(ModbusModule.Buf,i);
		ModbusModule.Buf[i++] = Word_Lo(j); //CRCУ��-���ֽ�
		ModbusModule.Buf[i++] = Word_Hi(j); //CRCУ��-���ֽ�

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
//Master�յ�Slave���ص�Ӧ��֡�����յ���֡Ҳ����ModbusModule��
//Ӧ��֡��������
//Return:
//1����CRCУ�鲻ͨ��
//2����վ�Ų�ƥ�䡪����������δʹ��
//3���������벻��Ӧ
//4����function code 0x10��Ӧ��ַ��ƥ��
//5����function code 0x10��Ӧ����������ƥ��
//---------------------------------------------
Uint16 RTU_Master_FrameAnalyse(Uint16  *dest_p) //*dest_pΪ��ŷ���������ݵ�����    ���յ��Ľ��Ӧ����ModbusModule.Buf[]��
{
    Uint16 crc_rx_temp; //���յ���֡�е�CRCֵ���������ֽ�ƴ��
    Uint16 crc_self_calculate; //���������CRCֵ������У��

  //  Uint16 RegAddr,RegNum;
    Uint16 rx_byte_num;
    Uint16 i;

    crc_rx_temp  = ModbusModule.Buf[ModbusModule.Rxlen-1]<<8; //���յ����1�ֽ���CRC HI Byte
    crc_rx_temp += ModbusModule.Buf[ModbusModule.Rxlen-2]; //���յ�������2�ֽ���CRC LO Byte

    crc_self_calculate=GetCRC16(ModbusModule.Buf,ModbusModule.Rxlen-2);

    if (crc_self_calculate != crc_rx_temp)
    	{return 1;} //1����CRCУ�鲻ͨ��

    //if ( ModbusModule.ID != ModbusModule.Buf[0] )     //վ���ڱ�Ӧ����Ϊ�̶�ֵ�������ж�
    //  {return 2;} //2����վ�Ų�ƥ�䡪��slave��

    if ( ModbusModule.FunCode != ModbusModule.Buf[1] )
        {return 3;} //3���������벻��Ӧ

   // ModbusModule.FunCode=ModbusModule.Buf[1];

   //////////////������й�  switch (ModbusModule.FunCode){
    switch (ModbusModule.Buf[1]){

    case(READ_HLD_REG):  //function code = 0x03
		 for (i=0; i<ModbusModule.Buf[2];i+=2){
		     *(dest_p + i/2) = (ModbusModule.Buf[i+3]<<8) + ModbusModule.Buf[i+4];
		         }

        break;

    case(SET_N_HLD_REG):  //function code = 0x10

	/*	RegAddr= (ModbusModule.Buf[2]<<8) + ModbusModule.Buf[3]; //���յ�֡�еĵ�ַ
		RegNum = (ModbusModule.Buf[4]<<8) + ModbusModule.Buf[5]; //���յ�֡�е��ֽ���
		   if ( RegAddr != ModbusModule.RegAddr ){  //��Ҫ��ʼ��ModbusModule
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
		        }     //�ο�Э�飬��Ŵ��λ���ڽϺ���ֽ�
		              //�õ��Ľ��������ÿ16λ����˳����ɵ�һ��Uint16

		  if(rx_byte_num&BIT0){
		    *(dest_p + i/2) &=0xFF00; //��ȡ��С�ڵ���8λʱ��Uint16�ĸ�8λĬ����1
		    *(dest_p + i/2) |=ModbusModule.Buf[i+3]&0xFF; //low 8 bit renew
		  }

		break;

    }

    return 0;



}



////////////////////////////////////////////////////
/////////////////////��������////////////////////////
////////////////////////////////////////////////////

void Delay(unsigned int j) //���ϸ���ӳٺ�������һ�����Ե���DSP2833x_usDelay.asm
{
	unsigned int k;   //���ܻᱻ�������Ż��������飡����
    for (k=0;k<j;k++);
}

Uint16 Word_Hi(Uint16 input) //��ȡ16λ�и�8λ�����ֽڣ�����
{
	input = (input & 0xFF00) >>8 ;
	return input;
}

Uint16 Word_Lo(Uint16 input) //��ȡ16λ�е�8λ�����ֽڣ�����
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

