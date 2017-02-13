//------------MODBUS CMD-----------------------

//只需要用到15 16 01 03命令

#define READ_N_COIL      01
#define READ_N_DI        02
#define READ_HLD_REG     03
#define SET_1_COIL       05
#define SET_1_HLD_REG    06
#define SET_N_COIL       15   //0x0F
#define SET_N_HLD_REG    16   //0x10

//------------Modbus_MODULE.Status---------------
#define IDLE_WAIT       0x00        // 空闲态，等待起始位
#define RECE_START       0x01        // 收到起始位，等待结束位
#define RECE_END         0x02        // 收到结束位，等待发送
#define SEND_START       0x03        // 发送起始位
#define SEND_END        0x04        // 发送完毕
#define SEND_EXCHANGE  0x05  // 发送内容切换（读/写，站1/站2）
//可以使用状态机判断工作情况，用FIFO的话，可以不用状态机

