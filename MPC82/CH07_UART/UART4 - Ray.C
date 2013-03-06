/**********UART5.C ******UART�PLCD�ǿ�**********
*�ʧ@�G�����ӤH�q������ơA�e��LCD���
*�w��GSW3-3(TxD1)ON
************************************************/
#include "..\MPC82.H"   //�Ȧs���βպA�w�q
#include "MUSIC.H"
#include "math.h"
//#define RAYPWM
#define BUFFER
#define MUSIC
//#define TIMER0
#define TIMER2
#define PARSER
//#define LCD
#define TT  57600  //Timer���ɮɶ�=(1/1.8432MHz)*57600=31250uS
#ifdef RAYPWM
unsigned char gg = 8;
#endif
#ifdef TIMER0
unsigned char hh = 0;
#endif
#ifdef TIMER2
unsigned char ii = 0;
#endif
#ifdef PARSER
#define OFF 1
#define ON 2
#define WAIT 3
#define CHANNEL 0x00
unsigned char channel;
unsigned char note;
unsigned char velocity;
unsigned int action; //1 =note off ; 2=note on ; 3= wait
#endif
#ifdef LCD
unsigned char jj=0;
unsigned char line;
#endif
#ifdef BUFFER
#define BUFFER_SIZE 960
volatile unsigned int produceCount, consumeCount;
unsigned char buffer[BUFFER_SIZE];
/*
 = { 0x90,0x48,0x50,0x91,0x3C,0x50,0x92,0x54,0x50,0x90,0x48,0x0,0x90,0x49,0x50,0x91,0x3C,0x0,0x91,0x3D,0x50,0x92,0x54,0x0,0x92,0x55,0x50,0x90,0x4A,0x50,0x91,0x3E,0x50,0x92,0x56,0x50,0x80,
0x49,0x0,0x81,0x3D,0x0,0x82,0x55,0x0,0x90,0x4B,0x50,0x91,0x3F,0x50,0x92,0x57,0x50,0x80,0x4A,0x0,0x81,0x3E,0x0,0x82,0x56,0x0,0x90,0x4C,0x50,0x91,0x40,0x50,0x92,0x58,0x50,0x80,0x4B,0x0,
0x81,0x3F,0x0,0x82,0x57,0x0,0x90,0x4D,0x50,0x91,0x41,0x50,0x92,0x59,0x50,0x80,0x4C,0x0,0x81,0x40,0x0,0x82,0x58,0x0,0x90,0x4E,0x50,0x91,0x42,0x50,0x92,0x5A,0x50,0x80,0x4D,0x0,0x81,0x41,0x0,
0x82,0x59,0x0,0x90,0x4F,0x50,0x91,0x43,0x50,0x92,0x5B,0x50,0x80,0x4E,0x0,0x81,0x42,0x0,0x82,0x5A,0x0,0x90,0x50,0x50,0x91,0x44,0x50,0x92,0x5C,0x50,0x80,0x4F,0x0,0x81,0x43,0x0,0x82,0x5B,
0x0,0x90,0x51,0x50,0x91,0x45,0x50,0x92,0x5D,0x50,0x80,0x50,0x0,0x81,0x44,0x0,0x82,0x5C,0x0,0x90,0x52,0x50,0x91,0x46,0x50,0x92,0x5E,0x50,0x80,0x51,0x0,0x81,0x45,0x0,0x82,0x5D,0x0,
0x90,0x53,0x0,0x91,0x47,0x50,0x92,0x5F,0x50,0x80,0x52,0x0,0x81,0x46,0x0,0x82,0x5E,0x0,0x90,0x54,0x50,0x91,0x48,0x50,0x92,0x60,0x50,0x80,0x53,0x0,0x81,0x47,0x0,0x82,0x5F,0x0,0x90,0x55,
0x50,0x91,0x49,0x50,0x92,0x61,0x50,0x80,0x54,0x0,0x81,0x48,0x0,0x82,0x60,0x0,0x90,0x56,0x50,0x91,0x4A,0x50,0x92,0x62,0x50,0x80,0x55,0x0,0x81,0x49,0x0,0x82,0x61,0x0,0x90,0x57,0x50,0x91,
0x4B,0x50,0x92,0x63,0x50,0x80,0x56,0x0,0x81,0x4A,0x0,0x82,0x62,0x0,0x80,0x57,0x0,0x81,0x4B,0x0,0x82,0x63,0x0 };
*/
#endif
#ifdef MUSIC
unsigned int  code Table[]  //�w�q���W�}�C���,0������
    = {    0   ,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
           0   ,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
           0   ,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
           DO1 ,DO_1,RE1 ,RE_1,MI1 ,FA1 ,FA_1,SO1 ,SO_1,LA1 ,LA_1,SI1 ,
           DO2 ,DO_2,RE2 ,RE_2,MI2 ,FA2 ,FA_2,SO2 ,SO_2,LA2 ,LA_2,SI2 ,
           DO3 ,DO_3,RE3 ,RE_3,MI3 ,FA3 ,FA_3,SO3 ,SO_3,LA3 ,LA_3,SI3 ,
           DO4 ,DO_4,RE4 ,RE_4,MI4 ,FA4 ,FA_4,SO4 ,SO_4,LA4 ,LA_4,SI4 ,
           DO5 ,DO_5,RE5 ,RE_5,MI5 ,FA5 ,FA_5,SO5 ,SO_5,LA5 ,LA_5,SI5 ,
           DO6 ,DO_6,RE6 ,RE_6,MI6 ,FA6 ,FA_6,SO6 ,SO_6,LA6 ,LA_6,SI6 ,
           DO7 ,DO_7,RE7 ,RE_7,MI7 ,FA7 ,FA_7,SO7 ,SO_7,LA7 ,LA_7,SI7 ,
           0   ,   0,   0,   0,   0,   0,   0,  0
      };
#endif

void consumeToken(unsigned char incomingByte);

main()
{
#ifdef BUFFER
    produceCount = 0;
    consumeCount = 0;
#endif
#ifdef PARSER
    note = velocity = 0;
    action = WAIT;
#endif
#ifdef LCD
    line = 0;
    LCD_init();   	//LCD�ҩl�{��
    LCD_Cmd(0x80);        //��ХѲĤ@��}�l���
    LCD_Cmd(0x0f);//*0000 1111,��ܹ�ON,��ܴ��,��а{�{
#endif
    UART_init(31250);      //�]�w��C���Ҥ��j�v
#ifdef TIMER2
    //PCON2=5; //Fosc=Fosc/32�A�ɶ�=31250uS*32=1��(�n������L��)
    T2CON = 0x00; /* 0000 1000�A��T2EX�}��J�t�tĲ�o�|���s���J
                    bit3:EXEN2=1,�ϥΥ~��T2EX���}
			        bit1:C/T=0,�����p��
			        bit0:CP/RL2=0,���s���J*/
    RCAP2=T2R=65536-TT; //�]�wTimer2��T2�۰ʸ��J�Ȧs��
#endif
    EA=1;
#ifdef MUSIC
    CCAPM0 = ECOM+MAT+TOG+ECCF; //MAT=1�APAC�p�ƻPCCAP0�ǰt�ɡA�OCCF0=1
    //ECOM=1�A�P�������\��
    //TOG=1�A(CH:CL)=CCAP0�ɡA�OCEX0�}�Ϭ�
    //ECCF=1�A�P�঳�ǰt(CCF0=1)�ɡA���ͤ��_
    AUXIE = EPCA; //�P��PCA���_
    CCF0 = 0;		//�M���Ҳ�0������X��
#endif
#ifdef RAYPWM
    CCAPM1=ECOM+PWM; //�P��CEX1�������PWM��X
    CMOD=0x00; //CPS1-0=00,Fpwm=Fosc/12/256=22.1184MHz/12/256=7.2KHz
    CR = 1;
    CCAP1H = gg; //�]�wCEX1�ߪi�ɶ�
#endif
    ES=1;            //�P���C���_
#ifdef TIMER2
    ET2=1;      //�P��Timer2���_
    //TR2=1;
#endif
#ifdef TIMER0
    //TMOD=0x01;   //�]�wTimer0��mode1�����p��
    TL0=65536 - TT;
    TH0=65536 - TT >> 8; //�]�w�p�ɭ�
    ET0=1;  //�P��Timer0��
    //TR0 = 1;
#endif
    while(1)
    {
#ifdef BUFFER
        while (abs(produceCount - consumeCount) == 0)
            ; // buffer is empty

        consumeToken( buffer[consumeCount++]);
        if( consumeCount >= BUFFER_SIZE )
            consumeCount = 0;
#else
        ;   	//�ۧڪ���A���ܥi���䥦�u�@
#endif
    }
}

void consumeToken(unsigned char incomingByte)
{
#ifdef PARSER
    if ( (incomingByte >> 4) == 9 ) // Note on
    {
        channel = (incomingByte & 0x0F);
        //LED0=~channel;
#ifdef LCD
        if(channel <= 0x09)
            LCD_Data(channel + '0');  //�r���e��LCD���
        else
            LCD_Data(channel  -10 + 'A');
        LCD_Data(',');
#endif
        action = ON;
    }
    else if ( (incomingByte >> 4) == 8 )// Note off
    {
        channel = (incomingByte & 0x0F);
        action = OFF;
    }
    else if(incomingByte < 0x80)//if (action != WAIT )
    {
        if (0 == note) // note on, wait for note value
        {
            note=incomingByte;
            if( CHANNEL == channel && action == ON )
            {
#ifdef MUSIC
                CCAP0L=Table[note];	   //�]�w����Ȧs���C�줸��
                CCAP0H=Table[note]>>8; //�]�w����Ȧs�����줸��
#endif
                LED0=~note;  //�N�����쪺�r����LED��X
#ifdef LCD
                char raynote = (note & 0xF0);
                raynote >>= 4;
                if(raynote <= 0x09)
                    LCD_Data(raynote + '0');  //�r���e��LCD���
                else
                    LCD_Data(raynote - 10 + 'A');
                raynote = (note & 0x0F);
                if(raynote <= 0x09)
                    LCD_Data(raynote + '0');  //�r���e��LCD���
                else
                    LCD_Data(raynote - 10 + 'A');
                LCD_Data(';');
#endif
            }
        }
        else //if ( note != 0 && action != WAIT)  // velocity
        {
            velocity=incomingByte;
            if(action == ON)
            {
                if( CHANNEL == channel )
                {
                    if( velocity != 0 )
                    {
#ifdef MUSIC
                        CR = 1;             //�Ұ�PCA�p�ơA�}�l�o��
#endif
#ifdef TIMER2
                        //TR2 = 1;         //�Ұ�Timer2�}�l�p��
#endif
                        LED1=~velocity;  //�N�����쪺�r����LED��X
#ifdef LCD
                        char raynote = (velocity & 0xF0);
                        raynote >>= 4;
                        if(raynote <= 0x09)
                            LCD_Data(raynote + '0');  //�r���e��LCD���
                        else
                            LCD_Data(raynote - 10 + 'A');
                        raynote = (velocity & 0x0F);
                        if(raynote <= 0x09)
                            LCD_Data(raynote + '0');  //�r���e��LCD���
                        else
                            LCD_Data(raynote - 10 + 'A');
                        LCD_Data(' ');
#endif
                    }
                    else
                    {
#ifdef MUSIC
                        //if( CHANNEL == channel )
                        //CR = 0;
#endif
                    }
                }
                else
                {
                    //produceCount = produceCount;
                }
                //Midi_Send(0x90,note,velocity);
            }
            else //if(action == OFF)
            {
#ifdef MUSIC
                //if( CHANNEL == channel )
                //CR = 0;
#endif
                //Midi_Send(0x80,note,velocity);
            }
            note=0;
            velocity=0;
            action=WAIT;
        }
    }
    else
    {
        //produceCount = produceCount;
    }
#else

#endif
}
#ifdef TIMER2
//*****************************************************
void T2_int (void) interrupt 5   //Timer2���_���
{
    //if (TF2 ==1)  //�Y�O�p�ɷ���OLED���[�A���쭫�s���J
    //{
    TF2=0;    //�M��TF2=0
    //LED1=~ii++; //LED���[��X
    //}
    //else  //�Y�OT2EX�}��J�t�tĲ�o�OLED=0�A�j�����s���J
    //{
    //EXF2=0;   //�M��EXF2=0
    //i=0;      //i=0
    // LED=~i;
    //}
}
#endif
/*****************************************************/
void SCON_int(void)  interrupt 4  //��C���_���
{
    RI=0;             //���������A�ORI=0
    if(SBUF < 0xF0)
    {
#ifdef BUFFER
        while ( abs(produceCount - consumeCount) == BUFFER_SIZE )
            ; // buffer is full

        buffer[produceCount++] = SBUF;
        if(produceCount >= BUFFER_SIZE)
            produceCount = 0;
#else
        consumeToken(SBUF);
#endif
#ifdef LCD
        if(0 == line && jj > 15) {
            line = 1;    //�bLCD�u���J4�Ӧr
            LCD_Cmd(0xC0);
        }
        else if(jj>31) {
            jj=0;    //�bLCD�u���J4�Ӧr
            line = 0;
            LCD_Cmd(0x80);
        }
#endif
    }
}
/***********************************************************
*��ƦW��: UART_init
*�\��y�z: UART�ҩl�{��
*��J�ѼơGbps
************************************************************/
void UART_init(unsigned int bps)  //UART�ҩl�{��
{
    P0M0=0;
    P0M1=0xFF; //�]�wP0����������X(M0-1=01)
    SCON = 0x50;     //�]�wUART��C�ǿ鬰MODE1�έP�౵��
    TMOD = T0_M0 + T1_M1;//0x20;     //�]�wTIMER1��MODE2

    AUXR2 = T1X12;
    PCON = SMOD;
    TH1=212;	 //211~213

    //TH1 = 256-(57600/bps);  //�]�p�ɾ��M�w��C�ǿ��j�v
    TR1 = 1;          //�}�l�p��
}

#ifdef MUSIC
/***********************************************************
*��ƦW��: PCA���_���
*�\��y�z: �۰ʥOCEX0�Ϭ�
************************************************************/
void PCA_Interrupt() interrupt 10
{
    CCF0 = 0;	    //�M���Ҳ�0������X��
    CL = CH =0;   //PCA�p�ƾ���0�}�l�W��
}
#endif

#ifdef TIMER0
/***************************************/
void T0_int(void) interrupt 1  //Timer0���_���
{
    TL0 = 65536 - TT ;
    TH0 = 65536 - TT >> 8; //���s�]�w�p�ɭ�
    //SPEAK=!SPEAK;     //��z�Ϭۿ�X
    //LED=~hh++; //LED���[��X
}
#endif

#ifdef LCD
/***********************************************************
*��ƦW��: LCD_Data
*�\��y�z: �ǰe��ƨ��r��LCD
*��J�ѼơGdat
************************************************************/
void LCD_Data(char dat)  //�ǰe��ƨ�LCD
{
    Data=dat; //��ưe��BUS
    P0_7=0;    //�L�o��ܦr�����
    RS=1;
    RW=0;
    EN=1;//��Ƽg�J��LCD��
    Delay_ms(1);   //LCD���ݼg�J����
    EN=0;          //�T��LCD

    jj++;
}
/***************************************************************
*��ƦW��: LCD_Cmd
*�\��y�z: �ǰe�R�O���r��LCD
*��J�ѼơGCmd
************************************************************/
void LCD_Cmd(unsigned char Cmd) //�ǰe�R�O��LCD
{
    Data=Cmd;  //�R�O�e��BUS
    RS=0;
    RW=0;
    EN=1; //�R�O�g�J��LCD��
    Delay_ms(1);    //LCD���ݼg�J����
    EN=0;           //�T��LCD
}
/***************************************************************
*��ƦW��: LCD_init
*�\��y�z: �ҩl�Ƥ�r��LCD
*****************************************************************/
void LCD_init(void)    //LCD���ҩl�{��
{
    LCD_Cmd(0x38);/*0011 1000,8bit�ǿ�,���2��,5*7�r��
                    bit4:DL=1,8bit�ǿ�,
                    bit3:N=1,���2��
                    bit2:F=0,5*7�r��*/
    LCD_Cmd(0x0c);/*0000 1100,��ܹ�ON,����ܴ��,��Ф��{�{
                    bit2:D=1,��ܹ�ON
                    bit1:C=0,����ܴ��
	                bit0:B=0,��Ф��{�{*/
    LCD_Cmd(0x06);/*0000 0110,//��ܧ���Хk��,��в���T��
                    bit1:I/D=1,��ܧ���Хk��,
                    bit0:S=0,��в���T��*/
    LCD_Cmd(0x01); //�M����ܹ�
    LCD_Cmd(0x02); //��Ц^���
}
#endif