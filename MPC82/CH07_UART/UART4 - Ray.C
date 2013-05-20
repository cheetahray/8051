/**********UART5.C ******UART與LCD傳輸**********
*動作：接收個人電腦的資料，送到LCD顯示
*硬體：SW3-3(TxD1)ON
************************************************/
#include "..\MPC82.H"   //暫存器及組態定義
#include "math.h"
//#define BUFFER
//#define MUSIC		 //P12	   	CR
//#define DEBUG
//#define CHANNEL16		  //P10 P12 P13
//#define LEDRay
#ifdef DEBUG
#include <stdio.h>   //加入標準輸出入函數
unsigned char oldCHANNEL=0xFF;
#endif
#ifndef MUSIC
#define HARDRAYPWM		  //P14 P15 P16		CR
#ifdef	HARDRAYPWM
#define PCATIMER
#define TTT  256
unsigned char P00VAR, P01VAR, P02VAR, P03VAR, P04VAR, P05VAR, P06VAR, P07VAR, P20VAR, P21VAR, P22VAR, P23VAR;
#define DIF00 0x10	//D5~FF
#define DIF01 0x30	//D5~FF
#define DIF02 0x40	//D5~FF
#define DIF03 0x53	//85~FF
#define DIF04 0x60	//D7~FF
#define DIF05 0x70	//D7~FF
#define DIF06 0x90	//D7~FF
#define DIF07 0xA0	//D7~FF
#define DIF20 0xC1	//D7~FF
#define DIF21 0xD0	//D7~FF
#define DIF22 0xE0
#define DIF23 0xF0
#define DIF14 0xFF	//D9~FF
#define DIF15 0xB2	//D5~FF
#define DIF16 0xFF	//D6~FF
#endif
#define TIMER0
#define SIMULATION
#else
#include "MUSIC.H"
#endif
#define TIMER2
#define PARSER
//#define LCD
#define TT  32768  //Timer延時時間=(1/1.8432MHz)*57600=31250uS
#ifdef TIMER2
unsigned char i14,i15,i16,i00,i01,i02,i03,i04, i05, i06, i07, i20, i21, i22, i23;
#endif
void softPWM();
#ifdef PARSER
#define IGNORE -1
#define OFF 1
#define ON 2
#define WAIT 3
unsigned char rayCHANNEL = 0, oneCHANNEL = 0, twoCHANNEL = 0;//#define rayCHANNEL 0x00
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
#define BUFFER_SIZE 900
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
unsigned int  code Table[]  //定義音頻陣列資料,0為休止符
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
#ifdef SIMULATION
int notecount;
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
    action = IGNORE;
#endif
#ifdef LCD
    line = 0;
    LCD_init();   	//LCD啟始程式
    LCD_Cmd(0x80);        //游標由第一行開始顯示
    LCD_Cmd(0x0f);//*0000 1111,顯示幕ON,顯示游標,游標閃爍
#endif
    TMOD = 0;
    UART_init(31250);      //設定串列環境及鮑率
#ifdef TIMER2
    //PCON2=5; //Fosc=Fosc/32，時間=31250uS*32=1秒(軟體模擬無效)
    T2CON = 0x00; /* 0000 1000，由T2EX腳輸入負緣觸發會重新載入
                    bit3:EXEN2=1,使用外部T2EX接腳
			        bit1:C/T=0,內部計時
			        bit0:CP/RL2=0,重新載入*/
    T2MOD = 0x00;
    RCAP2=T2R=65536-TT; //設定Timer2及T2自動載入暫存器
#endif
    EA=1;
#ifdef MUSIC
    CCAPM0 = ECOM+MAT+TOG+ECCF; //MAT=1，PAC計數與CCAP0匹配時，令CCF0=1
    //ECOM=1，致能比較器功能
    //TOG=1，(CH:CL)=CCAP0時，令CEX0腳反相
    //ECCF=1，致能有匹配(CCF0=1)時，產生中斷
    AUXIE = EPCA; //致能PCA中斷
    CCF0 = 0;		//清除模組0的比較旗標
#endif
    //AUXIE |= ES2;
#ifdef HARDRAYPWM
    CCAPM0=CCAPM1=CCAPM2=CCAPM3=CCAPM4/*=CCAPM5*/=ECOM+PWM; //致能CEX1比較器及PWM輸出
    CMOD=0x00; //CPS1-0=00,Fpwm=Fosc/12/256=22.1184MHz/12/256=7.2KHz
    //PCAPWM0=PCAPWM1=PCAPWM2=PCAPWM3=PCAPWM4=PCAPWM5=ECAPH;
    CCAP0H=CCAP1H=CCAP2H=CCAP3H=CCAP4H/*=CCAP5H*/=~0x00; //設定(P12/CEX0)，平均電壓為0V
    CR = 1;
#endif
#ifdef PCATIMER
    //CMOD = 0; //PCA計數時脈來源CPS1-0:00=Fosc/12
    CCAPM5=ECOM+MAT+ECCF;
    //MAT=1，PAC計數與CCAP0匹配時，令CCF0=1
    //ECOM=1，致能比較器功能
    //ECCF=1，致能有匹配(CCFn=1)時，產生中斷
    CCAP5L=TTT;
    CCAP5H=TTT>>8; //設定模組5比較暫存器
    AUXIE = EPCA;      //致能PCA中斷
    CCF5=0;  //清除模組0-5的比較旗標
    //CR = 1;
    P00VAR=DIF00;
    P01VAR=DIF01;
    P02VAR=DIF02;
    P03VAR=DIF03;
    P04VAR=DIF04;
    P05VAR=DIF05;
    P06VAR=DIF06;
    P07VAR=DIF07;
    P20VAR=DIF20;
    P21VAR=DIF21;
    P22VAR=DIF22;
    P23VAR=DIF23;
#endif
#ifdef TIMER2
    i14=i15=i16=i00=i01=i02=i03=i04=i05=i06=i07=i20=i21=i22=i23=0;
#endif
    ES=1;            //致能串列中斷
#ifdef TIMER2
    ET2=1;      //致能Timer2中斷
    TR2=1;
#endif
#ifdef TIMER0
    TMOD |= T0_M0;	//設定Timer0為mode1內部計時
    TL0=0;	//TL0=65536 - TT;
    TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
    ET0=1;	//致能Timer0中
    TR0=1;	//啟動Timer0開始計時
#endif
#ifdef SIMULATION
    notecount = 0;
    EX0=1;
    EX1=1;
    EX2=1;
    EX3=1;//致能外部INT0~3中斷
    IT0=1;
    IT1=1;
    IT2=1;
    IT3=1;//設定INT0~3腳負緣觸發中斷
#endif
    while(1)
    {
#ifdef BUFFER
        while (abs(produceCount - consumeCount) == 0)
        {
            softPWM();
        }
        consumeToken( buffer[consumeCount++]);
        if( consumeCount >= BUFFER_SIZE )
            consumeCount = 0;
#else
        softPWM();	//自我空轉，表示可做其它工作
#endif
    }
}

void consumeToken(unsigned char incomingByte)
{
    unsigned char controlray = (incomingByte >> 4);
#ifdef PARSER
    if ( controlray > 9 )
    {
        action = IGNORE;
    }
    else if ( controlray == 9 ) // Note on
    {
        channel = (incomingByte & 0x0F);
#ifdef SIMULATION
        notecount = 0;
#endif
#ifdef LCD
        if(channel <= 0x09)
            LCD_Data(channel + '0');  //字元送到LCD顯示
        else
            LCD_Data(channel  -10 + 'A');
        LCD_Data(',');
#endif
        action = ON;
    }
    else if ( controlray == 8 )// Note off
    {
        channel = (incomingByte & 0x0F);
        action = OFF;
    }
    else if(incomingByte < 0x80 && action != IGNORE)
    {
        if (0 == note) // note on, wait for note value
        {
            note=incomingByte;
            //if( oneCHANNEL == channel && action > OFF )
            {
#ifdef MUSIC
                CCAP0L=Table[note];	   //設定比較暫存器低位元組
                CCAP0H=Table[note]>>8; //設定比較暫存器高位元組
#endif
#ifdef LEDRay
#ifndef CHANNEL16
                LED0=~note;  //將接收到的字元由LED輸出
#endif
#endif
#ifdef LCD
                char raynote = (note & 0xF0);
                raynote >>= 4;
                if(raynote <= 0x09)
                    LCD_Data(raynote + '0');  //字元送到LCD顯示
                else
                    LCD_Data(raynote - 10 + 'A');
                raynote = (note & 0x0F);
                if(raynote <= 0x09)
                    LCD_Data(raynote + '0');  //字元送到LCD顯示
                else
                    LCD_Data(raynote - 10 + 'A');
                LCD_Data(';');
#endif
            }
        }
        else
        {
            velocity=incomingByte;
            if(action > OFF)
            {
                if( oneCHANNEL == channel )
                {
                    if( velocity != 0 )
                    {
#ifdef MUSIC
                        CR = 1;
#endif
                        switch(note)
                        {
                        case 60:
                            i00 = 4;
                            P00VAR = DIF00;
                            break;
                        case 61:
                            i01 = 4;
                            P01VAR = DIF01;
                            break;
                        case 62:
                            i02 = 3;
                            P02VAR = DIF02;
                            break;
                        case 63:
                            i03 = 3;
                            P03VAR = DIF03;
                            break;
                        case 64:
                            i04 = 3;
                            P04VAR = DIF04;
                            break;
                        case 65:
                            i05 = 3;
                            P05VAR = DIF05;
                            break;
                        case 66:
                            i06 = 3;
                            P06VAR = DIF06;
                            break;
                        case 67:
                            i07 = 3;
                            P07VAR = DIF07;
                            break;
                        case 68:
                            i20 = 2;
                            P20VAR = DIF20;
                            break;
                        case 69:
                            i21 = 2;
                            P21VAR = DIF21;
                            break;
                        }
#ifdef HARDRAYPWM
                        //CCAP0H=0x10;  //設定(P12/CEX0)脈波時間，平均電壓為4.6V
                        //CCAP1H=0x20;  //設定(P13/CEX1)脈波時間，平均電壓為4.4V
                        //CCAP2H=0x40;  //設定(P14/CEX2)脈波時間，平均電壓為3.8V
                        //CCAP3H=0x80;  //設定(P15/CEX3)脈波時間，平均電壓為2.6V
                        //CCAP4H=0xA0;  //設定(P16/CEX4)脈波時間，平均電壓為1.8V
                        //CCAP5H=0xFF;  //設定(P17/CEX5)脈波時間，平均電壓為0.01V
                        //記得統一加上 inverse ~
#endif
#ifdef LCD
                        char raynote = (velocity & 0xF0);
                        raynote >>= 4;
                        if(raynote <= 0x09)
                            LCD_Data(raynote + '0');  //字元送到LCD顯示
                        else
                            LCD_Data(raynote - 10 + 'A');
                        raynote = (velocity & 0x0F);
                        if(raynote <= 0x09)
                            LCD_Data(raynote + '0');  //字元送到LCD顯示
                        else
                            LCD_Data(raynote - 10 + 'A');
                        LCD_Data(' ');
#endif
                    }
                    else
                    {
                        //i11 = 0xFF;
                    }
                }
#ifdef SIMULATION
                else if( 1 == channel )
                {
                    if( velocity != 0 )
                    {
                        switch(note)
                        {
                        case 60:
                            i14 = 4;
                            CCAP2H = ~DIF14;
                            break;
                        case 61:
                            i15 = 4;
                            CCAP3H = ~DIF15;
                            break;
                        }
                    }
                    else
                    {
                        //i11 = 0xFF;
                    }
                }
                else if( 5 == channel )
                {
                    if( velocity != 0 )
                    {
                        switch(note)
                        {
                        case 60:
                            i16 = 3;
                            CCAP4H = ~DIF16;
                            break;
                        }
                    }
                    else
                    {
                        //i11 = 0xFF;
                    }
                }
                else if( 2 == channel )
                {
                    if( velocity != 0 )
                    {
                        switch(note)
                        {
                        case 60:
                            i22 = 4;
                            P22VAR = DIF22;
                            break;
                        }
                    }
                    else
                    {
                        //i11 = 0xFF;
                    }
                }
                else if( 3 == channel )
                {
                    if( velocity != 0 )
                    {
                        switch(note)
                        {
                        case 60:
                            P32 = 1;
                            break;
                        case 61:
                            P32 = 0;
							break;
                        }
                        i23 = 200;
                        P23VAR = DIF23;
                    }
                    else
                    {
                        //i11 = 0xFF;
                    }
                }
#endif
                else
                {
                    //produceCount = produceCount;
                }
                //Midi_Send(0x90,note,velocity);
            }
            else
            {
                //i11 = 0xFF;
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
void softPWM()
{
#ifdef DEBUG
    if(oldCHANNEL != oneCHANNEL)
    {
        oldCHANNEL = oneCHANNEL;
        TI=1;
        printf("%d\n",(int)oneCHANNEL);
        Delay_ms(1);
        TI=0;
    }
#endif
#ifdef PCATIMER
    if(CL > P00VAR)
        P00 = 0;
    if(CL > P01VAR)
        P01 = 0;
    if(CL > P02VAR)
        P02 = 0;
    if(CL > P03VAR)
        P03 = 0;
    if(CL > P04VAR)
        P04 = 0;
    if(CL > P05VAR)
        P05 = 0;
    if(CL > P06VAR)
        P06 = 0;
    if(CL > P07VAR)
        P07 = 0;
    if(CL > P20VAR)
        P20 = 0;
    if(CL > P21VAR)
        P21 = 0;
    if(CL > P22VAR)
        P22 = 0;
    if(CL > P23VAR)
        P23 = 0;
#endif
}
#ifdef TIMER2
//*****************************************************
void T2_int (void) interrupt 5   //Timer2中斷函數
{
    TF2=0;    //清除TF2=0
    switch(i14)
    {
    case 0:
        break;
    case 1:
        CCAP2H = ~0x00;
        i14--;
        break;
    default:
        i14--;
        break;
    }
    switch(i15)
    {
    case 0:
        break;
    case 1:
        CCAP3H = ~0x00;
        i15--;
        break;
    default:
        i15--;
        break;
    }
    switch(i16)
    {
    case 0:
        break;
    case 1:
        CCAP4H = ~0x00;
        i16--;
        break;
    default:
        i16--;
        break;
    }
    switch(i05)
    {
    case 0:
        break;
    case 1:
        P05VAR = 0x00;
        i05--;
        break;
    default:
        i05--;
        break;
    }
    switch(i06)
    {
    case 0:
        break;
    case 1:
        P06VAR = 0x00;
        i06--;
        break;
    default:
        i06--;
        break;
    }
    switch(i07)
    {
    case 0:
        break;
    case 1:
        P07VAR = 0x00;
        i07--;
        break;
    default:
        i07--;
        break;
    }
    switch(i20)
    {
    case 0:
        break;
    case 1:
        P20VAR = 0x00;
        i20--;
        break;
    default:
        i20--;
        break;
    }
    switch(i21)
    {
    case 0:
        break;
    case 1:
        P21VAR = 0x00;
        i21--;
        break;
    default:
        i21--;
        break;
    }
    switch(i22)
    {
    case 0:
        break;
    case 1:
        P22VAR = 0x00;
        i22--;
        break;
    default:
        i22--;
        break;
    }
    switch(i23)
    {
    case 0:
        break;
    case 1:
        P23VAR = 0x00;
        i23--;
        break;
    default:
        if(0 == P11 && i23 < 160 )
            i23 = 1;
        else
            i23--;
        break;
    }
    switch(i00)
    {
    case 0:
        break;
    case 1:
        P00VAR = 0x00;
        i00--;
        break;
    default:
        i00--;
        break;
    }
    switch(i01)
    {
    case 0:
        break;
    case 1:
        P01VAR = 0x00;
        i01--;
        break;
    default:
        i01--;
        break;
    }
    switch(i02)
    {
    case 0:
        break;
    case 1:
        P02VAR = 0x00;
        i02--;
        break;
    default:
        i02--;
        break;
    }
    switch(i03)
    {
    case 0:
        break;
    case 1:
        P03VAR = 0x00;
        i03--;
        break;
    default:
        i03--;
        break;
    }
    switch(i04)
    {
    case 0:
        break;
    case 1:
        P04VAR = 0x00;
        i04--;
        break;
    default:
        i04--;
        break;
    }
}
#endif
/*****************************************************/
void SCON_int(void)  interrupt 4  //串列中斷函數
{
    if(1 == RI)
    {
        RI = 0;    //接收完畢，令RI=0
        if(SBUF < 0xF0)
        {
#ifdef BUFFER
            while ( abs(produceCount - consumeCount) == BUFFER_SIZE )
                ; // buffer is full

            buffer[produceCount++] = SBUF;
            //SBUF = buffer[produceCount++];
            if(produceCount >= BUFFER_SIZE)
                produceCount = 0;
#else
            consumeToken(SBUF);
#endif
#ifdef LCD
            if(0 == line && jj > 15) {
                line = 1;    //在LCD只能輸入4個字
                LCD_Cmd(0xC0);
            }
            else if(jj>31) {
                jj=0;    //在LCD只能輸入4個字
                line = 0;
                LCD_Cmd(0x80);
            }
#endif
        }
    }
    //else
    //TI=0;
}
/*
//**********************************************************
void S2CON_int (void)  interrupt 12  //串列中斷函數
{
    if(S2CON & S2RI)  //若為接收所產生的中斷
    {
        S2CON &= ~S2RI;   //清除接收旗標令S2RI=0
#ifdef LEDRay
        //LED = ~S2BUF;     //將接收到的字元由LED輸出
#endif
		//S2BUF = ~LED;     //將temp發射出去
    }
    else
		S2CON &= ~S2TI; //若為發射所產生的中斷，清除發射旗標令S2TI=0
}
*/
/***********************************************************
*函數名稱: UART_init
*功能描述: UART啟始程式
*輸入參數：bps
************************************************************/
void UART_init(unsigned int bps)  //UART啟始程式
{
    P0M1=0xFF;
    //P1M1=0x70; //設定P0為推挽式輸出(M0-1=01)
    P2M1=0x0F;
    REN = 1;
    SM1=1;//SCON = 0x50;     //設定UART串列傳輸為MODE1及致能接收
#ifdef CHANNEL16
    S2CON = S2REN;// + S2SM1;
#endif
    TMOD |= T1_M1;  //設定TIMER1為MODE2
    AUXR2 = T1X12 + URM0X6;// + S2TX12 + S2SMOD + S2TR;	// T1X12 for uart1	URM0X6 for uart2
    PCON = SMOD;
#ifdef DEBUG
    TH1=112;
#else
    TH1=212;	//S2BRT = 212 //211~213 //TH1 = 256-(57600/bps);  //設計時器決定串列傳輸鮑率
#endif
    TR1 = 1;          //開始計時
}

/***********************************************************
*函數名稱: PCA中斷函數
*功能描述: 自動令CEX0反相
************************************************************/
void PCA_Interrupt() interrupt 10
{
#ifdef MUSIC
    CCF0 = 0;   //清除模組0的比較旗標
#endif
    CL=CH=0;   //PCA計數器由0開始上數
#ifdef PCATIMER
    if(CCF5)
    {
        CCF5=0; //清除模組0-5的比較旗標
    }//第T*6秒動作，PCA計數器由0上數
    P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
    P2 = 0x0F;//P20 = P21 = P22 = P23 = 1;
#endif
}

#ifdef TIMER0
/***************************************/
void T0_int(void) interrupt 1  //Timer0中斷函數
{
#ifdef CHANNEL16
    int pcai=7,pcaj=0;
    rayCHANNEL = 0;
    P10=0;
    //Delay_ms(1);   //載入74165並列資料
    P10=1 ;   //開始串列傳輸
    while((S2CON & S2RI)==0); //若RI=0表示未接收完畢，再繼續檢查
    S2CON &= ~S2RI;         //若RI=1表示已接收1個字元完畢，清除RI=0
    while(pcai>=0)
    {
        if((S2BUF >> pcai) & 1)
        {
            rayCHANNEL += (1<<pcaj);
        }
        pcai--;
        pcaj++;
    }
#ifdef LEDRay
    LED=~rayCHANNEL;     //將接收到的字元由LED輸出
#endif
    twoCHANNEL = (rayCHANNEL & 0x0F);
    oneCHANNEL = (rayCHANNEL >> 4);
    TL0=0;	//TL0=65536 - TT;
    TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
#endif
}
#endif

#ifdef LCD
/***********************************************************
*函數名稱: LCD_Data
*功能描述: 傳送資料到文字型LCD
*輸入參數：dat
************************************************************/
void LCD_Data(char dat)  //傳送資料到LCD
{
    Data=dat; //資料送到BUS
    P07=0;    //過濾顯示字型資料
    RS=1;
    RW=0;
    EN=1;//資料寫入到LCD內
    Delay_ms(1);   //LCD等待寫入完成
    EN=0;          //禁能LCD

    jj++;
}
/***************************************************************
*函數名稱: LCD_Cmd
*功能描述: 傳送命令到文字型LCD
*輸入參數：Cmd
************************************************************/
void LCD_Cmd(unsigned char Cmd) //傳送命令到LCD
{
    Data=Cmd;  //命令送到BUS
    RS=0;
    RW=0;
    EN=1; //命令寫入到LCD內
    Delay_ms(1);    //LCD等待寫入完成
    EN=0;           //禁能LCD
}
/***************************************************************
*函數名稱: LCD_init
*功能描述: 啟始化文字型LCD
*****************************************************************/
void LCD_init(void)    //LCD的啟始程式
{
    LCD_Cmd(0x38);/*0011 1000,8bit傳輸,顯示2行,5*7字型
                    bit4:DL=1,8bit傳輸,
                    bit3:N=1,顯示2行
                    bit2:F=0,5*7字型*/
    LCD_Cmd(0x0c);/*0000 1100,顯示幕ON,不顯示游標,游標不閃爍
                    bit2:D=1,顯示幕ON
                    bit1:C=0,不顯示游標
	                bit0:B=0,游標不閃爍*/
    LCD_Cmd(0x06);/*0000 0110,//顯示完游標右移,游標移位禁能
                    bit1:I/D=1,顯示完游標右移,
                    bit0:S=0,游標移位禁能*/
    LCD_Cmd(0x01); //清除顯示幕
    LCD_Cmd(0x02); //游標回原位
}
#endif
#ifdef SIMULATION
/*********************************/
void EX0_int(void) interrupt 0   //INT0中斷函數0
{
}
/*********************************************/
void EX1_int(void) interrupt 2   //INT1中斷函數2
{
    i14 = 3;
    CCAP2H = ~DIF14;
}
/*********************************************/
void EX2_int(void) interrupt 6   //INT2中斷函數6
{
    i22 = 4;
    P22VAR = DIF22;
}
/*********************************************/
void EX3_int(void) interrupt 7   //INT3中斷函數7
{
    i23 = 115;
    P23VAR = DIF23;
}
#endif