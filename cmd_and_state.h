//------------MODBUS CMD-----------------------

//ֻ��Ҫ�õ�15 16 01 03����

#define READ_N_COIL      01
#define READ_N_DI        02
#define READ_HLD_REG     03
#define SET_1_COIL       05
#define SET_1_HLD_REG    06
#define SET_N_COIL       15   //0x0F
#define SET_N_HLD_REG    16   //0x10

//------------Modbus_MODULE.Status---------------
#define IDLE_WAIT       0x00        // ����̬���ȴ���ʼλ
#define RECE_START       0x01        // �յ���ʼλ���ȴ�����λ
#define RECE_END         0x02        // �յ�����λ���ȴ�����
#define SEND_START       0x03        // ������ʼλ
#define SEND_END        0x04        // �������
#define SEND_EXCHANGE  0x05  // ���������л�����/д��վ1/վ2��
//����ʹ��״̬���жϹ����������FIFO�Ļ������Բ���״̬��

