struct MODBUS
{
	Uint16 Buf[256];
	Uint16 Txlen;
	Uint16 Rxlen;
	Uint16 Point;
	Uint16 ID;
	Uint16 FunCode;
	Uint16 Status;

	Uint16 Regaddr;
	Uint16 Regdata;
};
//extern volatile struct MODBUS ModbusModule;



//#define SEND_END 0x00;
//#define RECE_START 0x01;
//#define RECE_END 0x02;
//#define SEND_START 0x03;
//#define READ_HLD_REG 03;
//#define SET_N_HLD_REG 16;
