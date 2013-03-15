/**********UART5.C ******UART與LCD傳輸**********
*動作：接收個人電腦的資料，送到LCD顯示
*硬體：SW3-3(TxD1)ON
************************************************/
#include "..\MPC82.H"   //暫存器及組態定義
#include "math.h"
#define BUFFER
//#define MUSIC		 //P12	   	CR
#define HARDRAYPWM		  //P14 P15 P16 P17
#ifndef MUSIC
#define CHANNEL16		  //P10 P12 P13	 	CR
#else
#include "MUSIC.H"
#endif
#define TIMER2
#define PARSER
//#define LCD
#ifdef TIMER2
#define TT  57600  //Timer延時時間=(1/1.8432MHz)*57600=31250uS
unsigned char i11;
#endif
#define TIMER0
unsigned char PWM10_VAR, PWM11_VAR;
void softPWM();
#ifdef PARSER
#define OFF 1
#define ON 2
#define WAIT 3
unsigned char rayCHANNEL = 0;//#define rayCHANNEL 0x00
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
    //AUXIE += ES2;
#ifdef HARDRAYPWM
    /*CCAPM0=CCAPM1=*/CCAPM2=CCAPM3=CCAPM4=CCAPM5=ECOM+PWM; //致能CEX1比較器及PWM輸出
    CMOD=0x00; //CPS1-0=00,Fpwm=Fosc/12/256=22.1184MHz/12/256=7.2KHz
    //PCAPWM0=PCAPWM1=PCAPWM2=PCAPWM3=PCAPWM4=PCAPWM5=ECAPH;
    /*CCAP0H=CCAP1H=*/CCAP2H=CCAP3H=CCAP4H=CCAP5H=~0x00;//0x00; //設定(P12/CEX0)，平均電壓為0V
    CR = 1;
#endif
    PWM10_VAR=PWM11_VAR=0x00;
    i11=0xFF;
    ES=1;            //致能串列中斷
#ifdef TIMER2
    ET2=1;      //致能Timer2中斷
    TR2=1;
#endif
#ifdef TIMER0
    TMOD += T0_M1;	//設定Timer0為mode1內部計時
    TL0=0;	//TL0=65536 - TT;
    TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
    ET0=1;	//致能Timer0中
    TR0=1;	//啟動Timer0開始計時
#endif
    while(1)
    {
#ifdef BUFFER
        while (abs(produceCount - consumeCount) == 0)
        {
            softPWM();
#ifdef CHANNEL16
            if(S2CON & S2RI) //若RI=0表示未接收完畢，再繼續檢查
            {
                S2CON &= ~S2RI;//RI=0;         //若RI=1表示已接收1個字元完畢，清除RI=0
                LED=~S2BUF;     //將接收到的字元由LED輸出
                P1_0=0;
                P1_0=1 ;   //開始串列傳輸
            }
#endif
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
#ifdef PARSER
    if ( (incomingByte >> 4) == 9 ) // Note on
    {
        channel = (incomingByte & 0x0F);
        //LED0=~channel;
#ifdef LCD
        if(channel <= 0x09)
            LCD_Data(channel + '0');  //字元送到LCD顯示
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
            if( rayCHANNEL == channel && action == ON )
            {
#ifdef MUSIC
                CCAP0L=Table[note];	   //設定比較暫存器低位元組
                CCAP0H=Table[note]>>8; //設定比較暫存器高位元組
#endif
                LED0=~note;  //將接收到的字元由LED輸出
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
        else //if ( note != 0 && action != WAIT)  // velocity
        {
            velocity=incomingByte;
            if(action == ON)
            {
                if( rayCHANNEL == channel )
                {
                    if( velocity != 0 )
                    {
#ifdef MUSIC
                        CR = 1;             //啟動PCA計數，開始發音
#endif
                        i11 = 0;
#ifdef HARDRAYPWM
                        //CCAP0H=0x10;  //設定(P12/CEX0)脈波時間，平均電壓為4.6V
                        //CCAP1H=0x20;  //設定(P13/CEX1)脈波時間，平均電壓為4.4V
                        //CCAP2H=0x40;  //設定(P14/CEX2)脈波時間，平均電壓為3.8V
                        //CCAP3H=0x80;  //設定(P15/CEX3)脈波時間，平均電壓為2.6V
                        //CCAP4H=0xA0;  //設定(P16/CEX4)脈波時間，平均電壓為1.8V
                        //CCAP5H=0xFF;  //設定(P17/CEX5)脈波時間，平均電壓為0.01V
                        //記得統一加上 inverse ~
#endif
                        LED1=~velocity;  //將接收到的字元由LED輸出
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
#ifdef MUSIC
                        //if( rayCHANNEL == channel )
                        //CR = 0;
#endif
                        //i11 = 0xFF;
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
                //if( rayCHANNEL == channel )
                //CR = 0;
#endif
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
    //if(TL0 > PWM10_VAR)
    //P1_0=0;//若計時值 >PWM0值，PWM10=0
    if(TL0 > PWM11_VAR)
        P1_1=0;//若計時值 >PWM1值，PWM11=0
    else
        P1_1=1;
}
#ifdef TIMER2
//*****************************************************
void T2_int (void) interrupt 5   //Timer2中斷函數
{
    //if (TF2 ==1)  //若是計時溢位令LED遞加，溢位重新載入
    //{
    TF2=0;    //清除TF2=0
    if(i11 == 0)
    {
        PWM11_VAR = 0xE5;
#ifdef MUSIC
        i11++;
#endif
    }
    else
    {
        PWM11_VAR = 0x50;
    }
#ifndef MUSIC
    i11++;
    if(i11 >= 2)
        i11 = 0;
#endif
    //LED1=~ii++; //LED遞加輸出
    //}
    //else  //若是T2EX腳輸入負緣觸發令LED=0，強迫重新載入
    //{
    //EXF2=0;   //清除EXF2=0
    //i=0;      //i=0
    // LED=~i;
    //}
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

            buffer[produceCount] = SBUF;
            SBUF = buffer[produceCount++];
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
    else
        TI=0;
}
/*
//**********************************************************
void S2CON_int (void)  interrupt 12  //串列中斷函數
{
    if(S2CON & S2RI)  //若為接收所產生的中斷
    {
        S2CON &= ~S2RI;   //清除接收旗標令S2RI=0
        //LED = ~S2BUF;     //將接收到的字元由LED輸出
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
    P0M0=0;
    P0M1=0xFF; //設定P0為推挽式輸出(M0-1=01)
    REN = 1;
    SM1=1;//SCON = 0x50;     //設定UART串列傳輸為MODE1及致能接收
#ifdef CHANNEL16
    S2CON = S2REN;// + S2SM1;
#endif
    TMOD += T1_M1;  //設定TIMER1為MODE2
    AUXR2 = T1X12 + URM0X6;// + S2TX12 + S2SMOD + S2TR;	// T1X12 for uart1	URM0X6 for uart2
    PCON = SMOD;
    TH1=212;	//S2BRT = 212 //211~213 //TH1 = 256-(57600/bps);  //設計時器決定串列傳輸鮑率
    TR1 = 1;          //開始計時
}

#ifdef MUSIC
/***********************************************************
*函數名稱: PCA中斷函數
*功能描述: 自動令CEX0反相
************************************************************/
void PCA_Interrupt() interrupt 10
{
    CCF0 = 0;	    //清除模組0的比較旗標
    CL = CH =0;   //PCA計數器由0開始上數
}
#endif

#ifdef TIMER0
/***************************************/
void T0_int(void) interrupt 1  //Timer0中斷函數
{
    TL0 = 0;//TL0 = 65536 - TT ;
    TH0 = 0;//TH0 = 65536 - TT >> 8; //重新設定計時值
    //SPEAK=!SPEAK;     //喇叭反相輸出
    //LED=~hh++; //LED遞加輸出
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
    P0_7=0;    //過濾顯示字型資料
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