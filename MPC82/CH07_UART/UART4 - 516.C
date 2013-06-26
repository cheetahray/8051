/**********UART5.C ******UART與LCD傳輸**********
*動作：接收個人電腦的資料，送到LCD顯示
*硬體：SW3-3(TxD1)ON
************************************************/
#include "..\REG_MG84FG516.H"   //暫存器及組態定義
#include "math.h"
//#define DEBUG
#define CHANNEL16		  //P10 P12 P13
//#define LEDRay
#ifdef DEBUG
#include <stdio.h>   //加入標準輸出入函數
unsigned char oldCHANNEL=0xFF;
#endif
//#define HARDRAYPWM		  //P14 P15 P16		CR
#ifdef	HARDRAYPWM
//#define PCATIMER
#ifndef PCATIMER
#define TIMER0
#endif
#define TTT  256
unsigned char P00VAR,P01VAR,P02VAR,P03VAR,P04VAR,P05VAR,P06VAR,P07VAR,P11VAR,P14VAR,P15VAR,P16VAR,P17VAR,P20VAR,P21VAR,P22VAR,P23VAR,P24VAR,P25VAR,P26VAR,P27VAR,P32VAR,P34VAR,P35VAR,P36VAR,P37VAR,P40VAR,P41VAR,P42VAR,P43VAR,P46VAR;
#ifndef LEDRay
unsigned char P50VAR,P51VAR,P52VAR,P53VAR,P54VAR,P55VAR,P56VAR,P57VAR;
#endif
#else
#define BUFFER
#endif
#define SIMULATION
#define TIMER2
#define PARSER
#define TT  34286  //Timer延時時間=(1/1.8432MHz)*57600=31250uS
#ifdef TIMER2
unsigned char i00,i01,i02,i03,i04,i05,i06,i07,i11,i14,i15,i16,i17,i20,i21,i22,i23,i24,i25,i26,i27,i32,i34,i35,i36,i37,i40,i41,i42,i43,i46,i10000;
#ifndef LEDRay
i50,i51,i52,i53,i54,i55,i56,i57;
#endif
#endif
void softPWM();
#ifdef PARSER
#define IGNORE -1
#define OFF 1
#define ON 2
#define WAIT 3
unsigned char rayCHANNEL = 0, oneCHANNEL = 2,twoCHANNEL = 0;//#define rayCHANNEL 0x00
#define e04 4	 //撥弦	duration
#define e05	55	 //壓弦1秒
#define e06 250	 //風鈴計算時間
#define e07 10	 //和弦duration
#define e08 4    //撥弦壓弦時間差
#define e09 5	 //木鐵琴全滿Velocity
unsigned char e13;//木鐵琴槌時間
unsigned char e23;//鼓棒時間
#define yesoff
//#define ukulelechord
#ifndef ukulelechord
unsigned char ukubool;
#endif
unsigned char channel;
unsigned char note;
unsigned char velocity;
unsigned int action; //1 =note off ; 2=note on ; 3= wait
#endif
#ifdef BUFFER
#define BUFFER_SIZE 900
volatile unsigned int produceCount, consumeCount;
unsigned char buffer[BUFFER_SIZE];
#endif
#ifdef SIMULATION
int notecount;
unsigned int pressure = 0;
#endif
void consumeToken(unsigned char incomingByte);
void go_crazy();
void rayoff();
main()
{
    IFD = XTAL;
    IFADRH = 0;
    IFADRL = CKCON2;
    IFMT = PageP;
    ISPCR = ISPEN;
    SCMD = 0x46;
    SCMD = 0xb9;
    Delay_ms(30);
#ifdef BUFFER
    produceCount = 0;
    consumeCount = 0;
#endif
#ifdef PARSER
    note = velocity = 0;
    action = IGNORE;
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
    //AUXIE |= ES2;
#ifdef HARDRAYPWM
    CCAPM0=CCAPM1=CCAPM2=CCAPM3=CCAPM4=ECOM+PWM; //致能CEX1比較器及PWM輸出
    CMOD=0x00; //CPS1-0=00,Fpwm=Fosc/12/256=22.1184MHz/12/256=7.2KHz
    //PCAPWM0=PCAPWM1=PCAPWM2=PCAPWM3=PCAPWM4=PCAPWM5=ECAPH;
    CCAP0H=CCAP1H=CCAP2H=CCAP3H=CCAP4H=~0x00;//0x00; //設定(P12/CEX0)，平均電壓為0V
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
#else
    CCAPM5=ECOM+PWM;
    CCAP5H=~0x00;
#endif
    CR = 1;
    P00VAR=P01VAR=P02VAR=P03VAR=P04VAR=P05VAR=P06VAR=P07VAR=P11VAR=P14VAR=P15VAR=P16VAR=P17VAR=P20VAR=P21VAR=P22VAR=P23VAR=P24VAR=P25VAR=P26VAR=P27VAR=P32VAR=P34VAR=P35VAR=P36VAR=P40VAR=P41VAR=P42VAR=P43VAR=P46VAR=0;
#ifndef LEDRay
    P50VAR=P51VAR=P52VAR=P53VAR=P54VAR=P55VAR=P56VAR=P57VAR=0;
#endif
#else
    P00=P01=P02=P03=P04=P05=P06=P07=P11=P14=P15=P16=P17=P20=P21=P22=P23=P24=P25=P26=P27=P32=P34=P35=P36=P37=P40=P41=P42=P43=P46=P50=P51=P52=P53=P54=P55=P56=P57=0;
#endif
#ifdef TIMER2
    i00=i01=i02=i03=i04=i05=i06=i07=i11=i14=i15=i16=i17=i20=i21=i22=i23=i24=i25=i26=i27=i32=i34=i35=i36=i37=i40=i41=i42=i43=i46=i10000=0;
#ifndef LEDRay
    i50=i51=i52=i53=i54=i55=i56=i57=0;
#endif
#endif
    ES=1;            //致能串列中斷
#ifdef TIMER2
    ET2=1;      //致能Timer2中斷
    TR2=1;
#endif
#ifdef TIMER0
    TMOD |= T0_M1;	//設定Timer0為mode1內部計時
    TL0=0;	//TL0=65536 - TT;
    TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
    ET0=1;	//致能Timer0中
    TR0=1;	//啟動Timer0開始計時
#endif
#ifdef SIMULATION
    notecount = 0;
#endif
    go_crazy();
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
        action = ON;
    }
    else if ( controlray == 8 )// Note off
    {
        channel = (incomingByte & 0x0F);
        action = OFF;
    }
    else if(incomingByte < 0x80 && action != IGNORE)
    {
        if (!note) // note on, wait for note value
        {
            note=incomingByte-twoCHANNEL;
            if( action >= OFF && oneCHANNEL == channel )
            {
#ifdef LEDRay
#ifndef CHANNEL16
                LED=~note;  //將接收到的字元由LED輸出
#endif
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
                    if( velocity )
                    {
                        switch( oneCHANNEL )
                        {
                        case 0:
                        case 1:
                            e13 = velocity/42 + 4;
                            switch(note)
                            {
                            case 36:
                                i00 = e13;
                                break;
                            case 37:
                                i01 = e13;
                                break;
                            case 38:
                                i02 = e13;
                                break;
                            case 39:
                                i03 = e13;
                                break;
                            case 40:
                                i04 = e13;
                                break;
                            case 41:
                                i05 = e13;
                                break;
                            case 42:
                                i06 = e13;
                                break;
                            case 43:
                                i07 = e13;
                                break;
                            case 44:
                                i11 = e13;
                                break;
                            case 45:
                                i14 = e13;
                                break;
                            case 46:
                                i15 = e13;
                                break;
                            case 47:
                                i16 = e13;
                                break;
                            case 48:
                                i17 = e13;
                                break;
                            case 49:
                                i20 = e13;
                                break;
                            case 50:
                                i21 = e13;
                                break;
                            case 51:
                                i22 = e13;
                                break;
                            case 52:
                                i23 = e13;
                                break;
                            case 53:
                                i24 = e13;
                                break;
                            case 54:
                                i25 = e13;
                                break;
                            case 55:
                                i26 = e13;
                                break;
                            case 56:
                                i32 = e13;
                                break;
#ifndef LEDRay
                            case 57:
                                i57 = e13;
                                break;
#endif
                            case 58:
                                i34 = e13;
                                break;
                            case 59:
                                i35 = e13;
                                break;
                            case 60:
                                i36 = e13;
                                break;
                            case 61:
                                i37 = e13;
                                break;
                            case 62:
                                i40 = e13;
                                break;
                            case 63:
                                i41 = e13;
                                break;
                            case 64:
                                i27 = e13;
                                break;
                            case 65:
                                i43 = e13;
                                break;
                            case 66:
                                i46 = e13;
                                break;
#ifndef LEDRay
                            case 67:
                                i50 = e13;
                                break;
                            case 68:
                                i51 = e13;
                                break;
                            case 69:
                                i52 = e13;
                                break;
                            case 70:
                                i53 = e13;
                                break;
                            case 71:
                                i54 = e13;
                                break;
                            case 72:
                                i55 = e13;
                                break;
                            case 73:
                                i56 = e13;
                                break;
#endif
                            }
                            break;
                        case 2:
                        case 3:
                            e13 = velocity/42 + 4;
                            switch(note)
                            {
                            case 36:
                                i00 = e13;
                                break;
                            case 37:
                                i01 = e13;
                                break;
                            case 38:
                                i02 = e13;
                                break;
                            case 39:
                                i03 = e13;
                                break;
                            case 40:
                                i04 = e13;
                                break;
                            case 41:
                                i05 = e13;
                                break;
                            case 42:
                                i06 = e13;
                                break;
                            case 43:
                                i07 = e13;
                                break;
                            case 44:
                                i11 = e13;
                                break;
                            case 45:
                                i14 = e13;
                                break;
                            case 46:
                                i15 = e13;
                                break;
                            case 47:
                                i16 = e13;
                                break;
                            case 48:
                                i17 = e13;
                                break;
                            case 49:
                                i20 = e13;
                                break;
                            case 50:
                                i21 = e13;
                                break;
                            case 51:
                                i22 = e13;
                                break;
                            case 52:
                                i23 = e13;
                                break;
                            case 53:
                                i24 = e13;
                                break;
                            case 54:
                                i25 = e13;
                                break;
                            case 55:
                                i26 = e13;
                                break;
#ifndef LEDRay
                            case 56:
                                i56 = e13;
                                break;
                            case 57:
                                i55 = e13;
                                break;
#endif
                            case 58:
                                i34 = e13;
                                break;
                            case 59:
                                i35 = e13;
                                break;
                            case 60:
                                i36 = e13;
                                break;
                            case 61:
                                i37 = e13;
                                break;
                            case 62:
                                i40 = e13;
                                break;
                            case 63:
                                i41 = e13;
                                break;
                            case 64:
                                i42 = e13;
                                break;
                            case 65:
                                i43 = e13;
                                break;
                            case 66:
                                i46 = e13;
                                break;
#ifndef LEDRay
                            case 67:
                                i50 = e13;
                                break;
                            case 68:
                                i51 = e13;
                                break;
                            case 69:
                                i52 = e13;
                                break;
                            case 70:
                                i53 = e13;
                                break;
                            case 71:
                                i54 = e13;
                                break;
#endif
                            }
                            break;
                        case 4:
                        case 5:
                        case 6:
                            i46 = e05;
                            switch(note)
                            {
                            case 36:
                                if(i01)
                                {
                                    i01 = 1;
                                }
                                if(i02)
                                {
                                    i02 = 1;
                                }
                                if(i03)
                                {
                                    i03 = 1;
                                }
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i00 = e04;
                                break;
                            case 37:
                                if(i02)
                                {
                                    i02 = 1;
                                }
                                if(i03)
                                {
                                    i03 = 1;
                                }
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i01 = e05;
                                i56 = e08;
                                break;
                            case 38:
                                if(i03)
                                {
                                    i03 = 1;
                                }
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i02 = e05;
                                i56 = e08;
                                break;
                            case 39:
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i03 = e05;
                                i56 = e08;
                                break;
                            case 40:
                                i04 = e05;
                                i56 = e08;
                                break;
                            case 41:
                                if(i06)
                                {
                                    i06 = 1;
                                }
                                if(i07)
                                {
                                    i07 = 1;
                                }
                                if(i11)
                                {
                                    i11 = 1;
                                }
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i05 = e04;
                                break;
                            case 42:
                                if(i07)
                                {
                                    i07 = 1;
                                }
                                if(i11)
                                {
                                    i11 = 1;
                                }
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i06 = e05;
                                i51 = e08;
                                break;
                            case 43:
                                if(i11)
                                {
                                    i11 = 1;
                                }
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i07 = e05;
                                i51 = e08;
                                break;
                            case 44:
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i11 = e05;
                                i51 = e08;
                                break;
                            case 45:
                                i14 = e05;
                                i51 = e08;
                                break;
                            case 46:
                                if(i16)
                                {
                                    i16 = 1;
                                }
                                if(i17)
                                {
                                    i17 = 1;
                                }
                                if(i20)
                                {
                                    i20 = 1;
                                }
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i15 = e04;
                                break;
                            case 47:
                                if(i17)
                                {
                                    i17 = 1;
                                }
                                if(i20)
                                {
                                    i20 = 1;
                                }
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i16 = e05;
                                i52 = e08;
                                break;
                            case 48:
                                if(i20)
                                {
                                    i20 = 1;
                                }
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i17 = e05;
                                i52 = e08;
                                break;
                            case 49:
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i20 = e05;
                                i52 = e08;
                                break;
                            case 50:
                                i21 = e05;
                                i52 = e08;
                                break;
                            case 51:
                                if(i23)
                                {
                                    i23 = 1;
                                }
                                if(i24)
                                {
                                    i24 = 1;
                                }
                                if(i25)
                                {
                                    i25 = 1;
                                }
                                i22 = e04;
                                break;
                            case 52:
                                if(i24)
                                {
                                    i24 = 1;
                                }
                                if(i25)
                                {
                                    i25 = 1;
                                }
                                i23 = e05;
                                i53 = e08;
                                break;
                            case 53:
                                if(i25)
                                {
                                    i25 = 1;
                                }
                                i24 = e05;
                                i53 = e08;
                                break;
                            case 54:
                                i25 = e05;
                                i53 = e08;
                                break;
                            case 55:
                                if(i32)
                                {
                                    i32 = 1;
                                }
#ifndef LEDRay
                                if(i50)
                                {
                                    i50 = 1;
                                }
#endif
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                i26 = e04;
                                break;
                            case 56:
#ifndef LEDRay
                                if(i50)
                                {
                                    i50 = 1;
                                }
#endif
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                i32 = e05;
                                i54 = e08;
                                break;
                            case 57:
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
#ifndef LEDRay
                                i50 = e05;
#endif
                                i54 = e08;
                                break;
                            case 58:
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                i34 = e05;
                                i54 = e08;
                                break;
                            case 59:
                                i35 = e05;
                                i54 = e08;
                                break;
                            case 60:
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                if(i42)
                                {
                                    i42 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                i36 = e04;
                                break;
                            case 61:
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                if(i42)
                                {
                                    i42 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                i37 = e05;
                                i55 = e08;
                                break;
                            case 62:
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                if(i42)
                                {
                                    i42 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                i40 = e05;
                                i55 = e08;
                                break;
                            case 63:
                                if(i42)
                                {
                                    i42 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                i41 = e05;
                                i55 = e08;
                                break;
                            case 64:
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                i42 = e05;
                                i55 = e08;
                                break;
                            case 65:
                                i43 = e05;
                                i55 = e08;
                                break;
                            }
                            break;
                        case 7:
                            i42 = e05;
                            switch(note)
                            {
                            case 36:
                                if(i01)
                                {
                                    i01 = 1;
                                }
                                if(i02)
                                {
                                    i02 = 1;
                                }
                                if(i03)
                                {
                                    i03 = 1;
                                }
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i00 = e04;
                                break;
                            case 37:
                                if(i02)
                                {
                                    i02 = 1;
                                }
                                if(i03)
                                {
                                    i03 = 1;
                                }
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i01 = e05;
                                i56 = e08;
                                break;
                            case 38:
                                if(i03)
                                {
                                    i03 = 1;
                                }
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i02 = e05;
                                i56 = e08;
                                break;
                            case 39:
                                if(i04)
                                {
                                    i04 = 1;
                                }
                                i03 = e05;
                                i56 = e08;
                                break;
                            case 40:
                                i04 = e05;
                                i56 = e08;
                                break;
                            case 41:
                                if(i06)
                                {
                                    i06 = 1;
                                }
                                if(i07)
                                {
                                    i07 = 1;
                                }
                                if(i11)
                                {
                                    i11 = 1;
                                }
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i05 = e04;
                                break;
                            case 42:
                                if(i07)
                                {
                                    i07 = 1;
                                }
                                if(i11)
                                {
                                    i11 = 1;
                                }
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i06 = e05;
                                i51 = e08;
                                break;
                            case 43:
                                if(i11)
                                {
                                    i11 = 1;
                                }
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i07 = e05;
                                i51 = e08;
                                break;
                            case 44:
                                if(i14)
                                {
                                    i14 = 1;
                                }
                                i11 = e05;
                                i51 = e08;
                                break;
                            case 45:
                                i14 = e05;
                                i51 = e08;
                                break;
                            case 46:
                                if(i16)
                                {
                                    i16 = 1;
                                }
                                if(i17)
                                {
                                    i17 = 1;
                                }
                                if(i20)
                                {
                                    i20 = 1;
                                }
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i15 = e04;
                                break;
                            case 47:
                                if(i17)
                                {
                                    i17 = 1;
                                }
                                if(i20)
                                {
                                    i20 = 1;
                                }
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i16 = e05;
                                i52 = e08;
                                break;
                            case 48:
                                if(i20)
                                {
                                    i20 = 1;
                                }
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i17 = e05;
                                i52 = e08;
                                break;
                            case 49:
                                if(i21)
                                {
                                    i21 = 1;
                                }
                                i20 = e05;
                                i52 = e08;
                                break;
                            case 50:
                                i21 = e05;
                                i52 = e08;
                                break;
                            case 51:
                                if(i23)
                                {
                                    i23 = 1;
                                }
                                if(i24)
                                {
                                    i24 = 1;
                                }
                                if(i25)
                                {
                                    i25 = 1;
                                }
                                if(i26)
                                {
                                    i26 = 1;
                                }
                                if(i32)
                                {
                                    i32 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i22 = e04;
                                break;
                            case 52:
                                if(i24)
                                {
                                    i24 = 1;
                                }
                                if(i25)
                                {
                                    i25 = 1;
                                }
                                if(i26)
                                {
                                    i26 = 1;
                                }
                                if(i32)
                                {
                                    i32 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i23 = e05;
                                i53 = e08;
                                break;
                            case 53:
                                if(i25)
                                {
                                    i25 = 1;
                                }
                                if(i26)
                                {
                                    i26 = 1;
                                }
                                if(i32)
                                {
                                    i32 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i24 = e05;
                                i53 = e08;
                                break;
                            case 54:
                                if(i26)
                                {
                                    i26 = 1;
                                }
                                if(i32)
                                {
                                    i32 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i25 = e05;
                                i53 = e08;
                                break;
                            case 55:
                                if(i32)
                                {
                                    i32 = 1;
                                }
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i26 = e05;
                                i53 = e08;
                                break;
                            case 56:
                                if(i43)
                                {
                                    i43 = 1;
                                }
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i32 = e05;
                                i53 = e08;
                                break;
                            case 57:
                                if(i34)
                                {
                                    i34 = 1;
                                }
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i43 = e05;
                                i53 = e08;
                                break;
                            case 58:
                                if(i35)
                                {
                                    i35 = 1;
                                }
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i34 = e05;
                                i53 = e08;
                                break;
                            case 59:
                                if(i36)
                                {
                                    i36 = 1;
                                }
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i35 = e05;
                                i53 = e08;
                                break;
                            case 60:
                                if(i37)
                                {
                                    i37 = 1;
                                }
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i36 = e05;
                                i53 = e08;
                                break;
                            case 61:
                                if(i40)
                                {
                                    i40 = 1;
                                }
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i37 = e05;
                                i53 = e08;
                                break;
                            case 62:
                                if(i41)
                                {
                                    i41 = 1;
                                }
                                i40 = e05;
                                i53 = e08;
                                break;
                            case 63:
                                i41 = e05;
                                i53 = e08;
                                break;
                            }
                            break;
                        case 8:
                            switch(note)
                            {
                            case 36:
#ifndef ukulelechord
                                i00 = e04;
#else
                                P17VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 37:
#ifndef ukulelechord
                                i01 = e05;
                                i51 = e08;
#else
                                P25VAR = P01VAR = P23VAR = P20VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 38:
#ifndef ukulelechord
                                i02 = e05;
                                i51 = e08;
#else
                                P26VAR = P02VAR = P06VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 39:
#ifndef ukulelechord
                                i03 = e05;
                                i51 = e08;
#else
                                P03VAR = P07VAR = P15VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 40:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i22 = e04;
                                }
                                else
                                {
                                    i04 = e05;
                                    i51 = e08;
                                }
#else
                                P25VAR = P04VAR = P16VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 41:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i23 = e05;
                                    i52 = e08;
                                }
                                else
                                {
                                    i05 = e05;
                                    i51 = e08;
                                }
#else
                                P26VAR = P23VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 42:
#ifndef ukulelechord
                                i06 = e05;
                                i52 = e08;
#else
                                P32VAR = P01VAR = P06VAR = P20VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 43:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i24 = e04;
                                }
                                else
                                {
                                    i07 = e05;
                                    i52 = e08;
                                }
#else
                                P02VAR = P07VAR = P16VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 44:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i25 = e05;
                                    i56 = e08;
                                }
                                else
                                {
                                    i11 = e05;
                                    i52 = e08;
                                }
#else
                                P25VAR = P03VAR = P11VAR = P17VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 45:
#ifndef ukulelechord
                                if(ukubool%3!=0)
                                {
                                    if(ukubool%3==1)
                                    {
                                        i35 = e04;
                                    }
                                    else
                                    {
                                        i26 = e05;
                                        i56 = e08;
                                    }
                                }
                                else
                                {
                                    i14 = e05;
                                    i52 = e08;
                                }
                                ukubool++;
#else
                                P01VAR = P26VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 46:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i15 = e05;
                                    i53 = e08;
                                }
                                else
                                {
                                    i32 = e05;
                                    i56 = e08;
                                }
#else
                                P32VAR = P02VAR = P23VAR = P15VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 47:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i16 = e05;
                                    i53 = e08;
                                }
                                else
                                {
                                    i37 = e05;
                                    i56 = e08;
                                }
#else
                                P33VAR = P03VAR = P06VAR = P16VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 48:
#ifndef ukulelechord
                                if(ukubool++%2==0)
                                {
                                    i17 = e05;
                                    i53 = e08;
                                }
                                else
                                {
                                    i34 = e05;
                                    i56 = e08;
                                }
#else
                                P17VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 49:
#ifndef ukulelechord
                                i20 = e05;
                                i53 = e08;
#else
                                P25VAR = P01VAR = P23VAR = P20VAR = 255;
                                i37 = e07;
#endif
                                break;
                            case 50:
#ifndef ukulelechord
                                i21 = e05;
                                i53 = e08;
#else
                                P26VAR = P02VAR = P06VAR = 255;
                                i37 = e07;
#endif
                                break;
                            }
                            break;
                        case 9:
                            e23 = velocity/42 + 3;
                            switch(note)
                            {
                            case 36:
                                i00 = e23+1;
                                break;
                            case 37:
                                i01 = e23+4;
                                break;
                            case 38:
                                i02 = e23;
                                break;
                            case 39:
                                i03 = e23-1;
                                break;
                            case 40:
                                i04 = e23-1;
                                break;
                            case 41:
                                i05 = e23-1;
                                break;
                            case 42:
                                i06 = e23-1;
                                break;
                            case 43:
                                i07 = e23+1;
                                break;
                            case 44:
                                i11 = e23+1;
                                break;
                            case 45:
                                i14 = e23+1;
                                break;
                            case 46:
                                i15 = e23+1;
#ifndef LEDRay
                                i56 = e23;
#endif
                                break;
                            case 47:
                                i16 = e23+1;
#ifndef LEDRay
                                i56 = e23;
#endif
                                break;
                            case 48:
                                i17 = e23+1;
#ifndef LEDRay
                                i57 = e23;
#endif
                                break;
                            case 49:
                                i20 = e23+1;
#ifndef LEDRay
                                i57 = e23;
#endif
                                break;
                            case 50:
                                i21 = e23;
                                break;
                            case 51:
                                i22 = e23;
                                break;
                            case 52:
                                i23 = e23;
                                break;
                            case 53:
                                i24 = e23-1;
                                break;
                            case 54:
                                i25 = e23-1;
                                break;
                            case 55:
                                i26 = e23;
                                break;
                            case 56:
                                i27 = e23;
                                break;
                            case 57:
                                i32 = e23;
                                break;
                            case 58:
                                i34 = e23;
                                break;
                            case 59:
                                i35 = e23;
                                break;
                            case 60:
                                i36 = e23;
                                break;
                            case 61:
                                i37 = e23;
                                break;
                            case 62:
                                i40 = e23;
                                break;
                            case 63:
                                i41 = e23;
                                break;
                            case 64:
                                i42 = e23;
                                break;
                            case 65:
                                i43 = e23;
                                break;
                            case 66:
                                i46 = e23;
                                break;
#ifndef LEDRay
                            case 67:
                                i50 = e23;
                                break;
                            case 68:
                                i51 = e23;
                                break;
                            case 69:
                                i52 = e23;
                                break;
                            case 70:
                                i53 = e23;
                                break;
                            case 71:
                                i54 = e23;
                                break;
                            case 72:
                                i55 = e06;
                                SFRPI = 1;
                                P62 = 1;
                                SFRPI = 0;
                                break;
                            case 73:
                                i55 = e06;
                                SFRPI = 1;
                                P62 = 0;
                                SFRPI = 0;
                                break;
#endif
                            }
                            break;
                        }
                    }
                    else
                    {
                        rayoff();//produceCount = produceCount;
                    }
                    //Midi_Send(0x90,note,velocity);
                }
            }
            else if(action == OFF)
            {
                rayoff();
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
#ifdef HARDRAYPWM
    if(TL0 > P00VAR)
        P00 = 0;
    if(TL0 > P01VAR)
        P01 = 0;
    if(TL0 > P02VAR)
        P02 = 0;
    if(TL0 > P03VAR)
        P03 = 0;
    if(TL0 > P04VAR)
        P04 = 0;
    if(TL0 > P05VAR)
        P05 = 0;
    if(TL0 > P06VAR)
        P06 = 0;
    if(TL0 > P07VAR)
        P07 = 0;
    if(TL0 > P11VAR)
        P11 = 0;
    if(TL0 > P14VAR)
        P14 = 0;
    if(TL0 > P15VAR)
        P15 = 0;
    if(TL0 > P16VAR)
        P16 = 0;
    if(TL0 > P17VAR)
        P17 = 0;
    if(TL0 > P20VAR)
        P20 = 0;
    if(TL0 > P21VAR)
        P21 = 0;
    if(TL0 > P32VAR)
        P32 = 0;
    if(TL0 > P34VAR)
        P34 = 0;
    if(TL0 > P35VAR)
        P35 = 0;
    if(TL0 > P36VAR)
        P36 = 0;
    if(TL0 > P37VAR)
        P37 = 0;
    if(TL0 > P40VAR)
        P40 = 0;
    if(TL0 > P41VAR)
        P41 = 0;
    if(TL0 > P42VAR)
        P42 = 0;
    if(TL0 > P43VAR)
        P43 = 0;
    if(TL0 > P46VAR)
        P46 = 0;
#ifndef LEDRay
    if(TL0 > P50VAR)
        P50 = 0;
    if(TL0 > P51VAR)
        P51 = 0;
    if(TL0 > P52VAR)
        P52 = 0;
    if(TL0 > P53VAR)
        P53 = 0;
    if(TL0 > P54VAR)
        P54 = 0;
    if(TL0 > P55VAR)
        P55 = 0;
    if(TL0 > P56VAR)
        P56 = 0;
    if(TL0 > P57VAR)
        P57 = 0;
#endif
#endif
}
#ifdef TIMER2
//*****************************************************
void T2_int (void) interrupt 5   //Timer2中斷函數
{
    TF2=0;    //清除TF2=0
    switch( oneCHANNEL )
    {
    case 0:
    case 1:
        switch(i00)
        {
        case 0:
            break;
        case 1:
            P00 = 0;
            i00--;
            break;
        default:
            if(e13 == i00)
                P00 = 1;
            i00--;
            break;
        }
        switch(i01)
        {
        case 0:
            break;
        case 1:
            P01 = 0;
            i01--;
            break;
        default:
            if(e13 == i01)
                P01 = 1;
            i01--;
            break;
        }
        switch(i02)
        {
        case 0:
            break;
        case 1:
            P02 = 0;
            i02--;
            break;
        default:
            if(e13 == i02)
                P02 = 1;
            i02--;
            break;
        }
        switch(i03)
        {
        case 0:
            break;
        case 1:
            P03 = 0;
            i03--;
            break;
        default:
            if(e13 == i03)
                P03 = 1;
            i03--;
            break;
        }
        switch(i04)
        {
        case 0:
            break;
        case 1:
            P04 = 0;
            i04--;
            break;
        default:
            if(e13 == i04)
                P04 = 1;
            i04--;
            break;
        }
        switch(i05)
        {
        case 0:
            break;
        case 1:
            P05 = 0;
            i05--;
            break;
        default:
            if(e13 == i05)
                P05 = 1;
            i05--;
            break;
        }
        switch(i06)
        {
        case 0:
            break;
        case 1:
            P06 = 0;
            i06--;
            break;
        default:
            if(e13 == i06)
                P06 = 1;
            i06--;
            break;
        }
        switch(i07)
        {
        case 0:
            break;
        case 1:
            P07 = 0;
            i07--;
            break;
        default:
            if(e13 == i07)
                P07 = 1;
            i07--;
            break;
        }
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11 = 0;
            i11--;
            break;
        default:
            if(e13 == i11)
                P11 = 1;
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14 = 0;
            i14--;
            break;
        default:
            if(e13 == i14)
                P14 = 1;
            i14--;
            break;
        }
        switch(i15)
        {
        case 0:
            break;
        case 1:
            P15 = 0;
            i15--;
            break;
        default:
            if(e13 == i15)
                P15 = 1;
            i15--;
            break;
        }
        switch(i16)
        {
        case 0:
            break;
        case 1:
            P16 = 0;
            i16--;
            break;
        default:
            if(e13 == i16)
                P16 = 1;
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17 = 0;
            i17--;
            break;
        default:
            if(e13 == i17)
                P17 = 1;
            i17--;
            break;
        }
        switch(i20)
        {
        case 0:
            break;
        case 1:
            P20 = 0;
            i20--;
            break;
        default:
            if(e13 == i20)
                P20 = 1;
            i20--;
            break;
        }
        switch(i21)
        {
        case 0:
            break;
        case 1:
            P21 = 0;
            i21--;
            break;
        default:
            if(e13 == i21)
                P21 = 1;
            i21--;
            break;
        }
        switch(i22)
        {
        case 0:
            break;
        case 1:
            //CCAP0H = 255;
            P22 = 0;
            i22--;
            break;
        default:
            if(e13 == i22)
                P22 = 1;
            i22--;
            break;
        }
        switch(i23)
        {
        case 0:
            break;
        case 1:
            //CCAP1H = 255;
            P23 = 0;
            i23--;
            break;
        default:
            if(e13 == i23)
                P23 = 1;
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            //CCAP2H = 255;
            P24 = 0;
            i24--;
            break;
        default:
            if(e13 == i24)
                P24 = 1;
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            //CCAP3H = 255;
            P25 = 0;
            i25--;
            break;
        default:
            if(e13 == i25)
                P25 = 1;
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            //CCAP4H = 255;
            P26 = 0;
            i26--;
            break;
        default:
            if(e13 == i26)
                P26 = 1;
            i26--;
            break;
        }
        switch(i27)
        {
        case 0:
            break;
        case 1:
            P27 = 0;
            i27--;
            break;
        default:
            if(e13 == i27)
                P27 = 1;
            i27--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32 = 0;
            i32--;
            break;
        default:
            if(e13 == i32)
                P32 = 1;
            i32--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34 = 0;
            i34--;
            break;
        default:
            if(e13 == i34)
                P34 = 1;
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35 = 0;
            i35--;
            break;
        default:
            if(e13 == i35)
                P35 = 1;
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36 = 0;
            i36--;
            break;
        default:
            if(e13 == i36)
                P36 = 1;
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37 = 0;
            i37--;
            break;
        default:
            if(e13 == i37)
                P37 = 1;
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40 = 0;
            i40--;
            break;
        default:
            if(e13 == i40)
                P40 = 1;
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41 = 0;
            i41--;
            break;
        default:
            if(e13 == i41)
                P41 = 1;
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42 = 0;
            i42--;
            break;
        default:
            if(e13 == i42)
                P42 = 1;
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43 = 0;
            i43--;
            break;
        default:
            if(e13 == i43)
                P43 = 1;
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46 = 0;
            i46--;
            break;
        default:
            if(e13 == i46)
                P46 = 1;
            i46--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50 = 0;
            i50--;
            break;
        default:
            if(e13 == i50)
                P50 = 1;
            i50--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            P51 = 0;
            i51--;
            break;
        default:
            if(e13 == i51)
                P51 = 1;
            i51--;
            break;
        }
        switch(i52)
        {
        case 0:
            break;
        case 1:
            P52 = 0;
            i52--;
            break;
        default:
            if(e13 == i52)
                P52 = 1;
            i52--;
            break;
        }
        switch(i53)
        {
        case 0:
            break;
        case 1:
            P53 = 0;
            i53--;
            break;
        default:
            if(e13 == i53)
                P53 = 1;
            i53--;
            break;
        }
        switch(i54)
        {
        case 0:
            break;
        case 1:
            P54 = 0;
            i54--;
            break;
        default:
            if(e13 == i54)
                P54 = 1;
            i54--;
            break;
        }
        switch(i55)
        {
        case 0:
            break;
        case 1:
            P55 = 0;
            i55--;
            break;
        default:
            if(e13 == i55)
                P55 = 1;
            i55--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            P56 = 0;
            i56--;
            break;
        default:
            if(e13 == i56)
                P56 = 1;
            i56--;
            break;
        }
        switch(i57)
        {
        case 0:
            break;
        case 1:
            P57 = 0;
            i57--;
            break;
        default:
            if(e13 == i57)
                P57 = 1;
            i57--;
            break;
        }
#endif
        break;
    case 2:
    case 3:
        switch(i00)
        {
        case 0:
            break;
        case 1:
            P00 = 0;
            i00--;
            break;
        default:
            if(e13 == i00)
                P00 = 1;
            i00--;
            break;
        }
        switch(i01)
        {
        case 0:
            break;
        case 1:
            P01 = 0;
            i01--;
            break;
        default:
            if(e13 == i01)
                P01 = 1;
            i01--;
            break;
        }
        switch(i02)
        {
        case 0:
            break;
        case 1:
            P02 = 0;
            i02--;
            break;
        default:
            if(e13 == i02)
                P02 = 1;
            i02--;
            break;
        }
        switch(i03)
        {
        case 0:
            break;
        case 1:
            P03 = 0;
            i03--;
            break;
        default:
            if(e13 == i03)
                P03 = 1;
            i03--;
            break;
        }
        switch(i04)
        {
        case 0:
            break;
        case 1:
            P04 = 0;
            i04--;
            break;
        default:
            if(e13 == i04)
                P04 = 1;
            i04--;
            break;
        }
        switch(i05)
        {
        case 0:
            break;
        case 1:
            P05 = 0;
            i05--;
            break;
        default:
            if(e13 == i05)
                P05 = 1;
            i05--;
            break;
        }
        switch(i06)
        {
        case 0:
            break;
        case 1:
            P06 = 0;
            i06--;
            break;
        default:
            if(e13 == i06)
                P06 = 1;
            i06--;
            break;
        }
        switch(i07)
        {
        case 0:
            break;
        case 1:
            P07 = 0;
            i07--;
            break;
        default:
            if(e13 == i07)
                P07 = 1;
            i07--;
            break;
        }
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11 = 0;
            i11--;
            break;
        default:
            if(e13 == i11)
                P11 = 1;
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14 = 0;
            i14--;
            break;
        default:
            if(e13 == i14)
                P14 = 1;
            i14--;
            break;
        }
        switch(i15)
        {
        case 0:
            break;
        case 1:
            P15 = 0;
            i15--;
            break;
        default:
            if(e13 == i15)
                P15 = 1;
            i15--;
            break;
        }
        switch(i16)
        {
        case 0:
            break;
        case 1:
            P16 = 0;
            i16--;
            break;
        default:
            if(e13 == i16)
                P16 = 1;
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17 = 0;
            i17--;
            break;
        default:
            if(e13 == i17)
                P17 = 1;
            i17--;
            break;
        }
        switch(i20)
        {
        case 0:
            break;
        case 1:
            P20 = 0;
            i20--;
            break;
        default:
            if(e13 == i20)
                P20 = 1;
            i20--;
            break;
        }
        switch(i21)
        {
        case 0:
            break;
        case 1:
            P21 = 0;
            i21--;
            break;
        default:
            if(e13 == i21)
                P21 = 1;
            i21--;
            break;
        }
        switch(i22)
        {
        case 0:
            break;
        case 1:
            //CCAP0H = 255;
            P22 = 0;
            i22--;
            break;
        default:
            if(e13 == i22)
                P22 = 1;
            i22--;
            break;
        }
        switch(i23)
        {
        case 0:
            break;
        case 1:
            //CCAP1H = 255;
            P23 = 0;
            i23--;
            break;
        default:
            if(e13 == i23)
                P23 = 1;
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            //CCAP2H = 255;
            P24 = 0;
            i24--;
            break;
        default:
            if(e13 == i24)
                P24 = 1;
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            //CCAP3H = 255;
            P25 = 0;
            i25--;
            break;
        default:
            if(e13 == i25)
                P25 = 1;
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            //CCAP4H = 255;
            P26 = 0;
            i26--;
            break;
        default:
            if(e13 == i26)
                P26 = 1;
            i26--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34 = 0;
            i34--;
            break;
        default:
            if(e13 == i34)
                P34 = 1;
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35 = 0;
            i35--;
            break;
        default:
            if(e13 == i35)
                P35 = 1;
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36 = 0;
            i36--;
            break;
        default:
            if(e13 == i36)
                P36 = 1;
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37 = 0;
            i37--;
            break;
        default:
            if(e13 == i37)
                P37 = 1;
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40 = 0;
            i40--;
            break;
        default:
            if(e13 == i40)
                P40 = 1;
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41 = 0;
            i41--;
            break;
        default:
            if(e13 == i41)
                P41 = 1;
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42 = 0;
            i42--;
            break;
        default:
            if(e13 == i42)
                P42 = 1;
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43 = 0;
            i43--;
            break;
        default:
            if(e13 == i43)
                P43 = 1;
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46 = 0;
            i46--;
            break;
        default:
            if(e13 == i46)
                P46 = 1;
            i46--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50 = 0;
            i50--;
            break;
        default:
            if(e13 == i50)
                P50 = 1;
            i50--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            P51 = 0;
            i51--;
            break;
        default:
            if(e13 == i51)
                P51 = 1;
            i51--;
            break;
        }
        switch(i52)
        {
        case 0:
            break;
        case 1:
            P52 = 0;
            i52--;
            break;
        default:
            if(e13 == i52)
                P52 = 1;
            i52--;
            break;
        }
        switch(i53)
        {
        case 0:
            break;
        case 1:
            P53 = 0;
            i53--;
            break;
        default:
            if(e13 == i53)
                P53 = 1;
            i53--;
            break;
        }
        switch(i54)
        {
        case 0:
            break;
        case 1:
            P54 = 0;
            i54--;
            break;
        default:
            if(e13 == i54)
                P54 = 1;
            i54--;
            break;
        }
        switch(i55)
        {
        case 0:
            break;
        case 1:
            P55 = 0;
            i55--;
            break;
        default:
            if(e13 == i55)
                P55 = 1;
            i55--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            P56 = 0;
            i56--;
            break;
        default:
            if(e13 == i56)
                P56 = 1;
            i56--;
            break;
        }
#endif
        break;
    case 4:
    case 5:
    case 6:
        switch(i00)
        {
        case 0:
            break;
        case 1:
            P00 = 0;
            i00--;
            break;
        default:
            if(e04 == i00)
                P00 = 1;
            i00--;
            break;
        }
        switch(i01)
        {
        case 0:
            break;
        case 1:
            P01 = 0;
            i01--;
            break;
        default:
            if(e05 == i01)
                P01 = 1;
            i01--;
            break;
        }
        switch(i02)
        {
        case 0:
            break;
        case 1:
            P02 = 0;
            i02--;
            break;
        default:
            if(e05 == i02)
                P02 = 1;
            i02--;
            break;
        }
        switch(i03)
        {
        case 0:
            break;
        case 1:
            P03 = 0;
            i03--;
            break;
        default:
            if(e05 == i03)
                P03 = 1;
            i03--;
            break;
        }
        switch(i04)
        {
        case 0:
            break;
        case 1:
            P04 = 0;
            i04--;
            break;
        default:
            if(e05 == i04)
                P04 = 1;
            i04--;
            break;
        }
        switch(i05)
        {
        case 0:
            break;
        case 1:
            P05 = 0;
            i05--;
            break;
        default:
            if(e04 == i05)
                P05 = 1;
            i05--;
            break;
        }
        switch(i06)
        {
        case 0:
            break;
        case 1:
            P06 = 0;
            i06--;
            break;
        default:
            if(e05 == i06)
                P06 = 1;
            i06--;
            break;
        }
        switch(i07)
        {
        case 0:
            break;
        case 1:
            P07 = 0;
            i07--;
            break;
        default:
            if(e05 == i07)
                P07 = 1;
            i07--;
            break;
        }
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11 = 0;
            i11--;
            break;
        default:
            if(e05 == i11)
                P11 = 1;
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14 = 0;
            i14--;
            break;
        default:
            if(e05 == i14)
                P14 = 1;
            i14--;
            break;
        }
        switch(i15)
        {
        case 0:
            break;
        case 1:
            P15 = 0;
            i15--;
            break;
        default:
            if(e04 == i15)
                P15 = 1;
            i15--;
            break;
        }
        switch(i16)
        {
        case 0:
            break;
        case 1:
            P16 = 0;
            i16--;
            break;
        default:
            if(e05 == i16)
                P16 = 1;
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17 = 0;
            i17--;
            break;
        default:
            if(e05 == i17)
                P17 = 1;
            i17--;
            break;
        }
        switch(i20)
        {
        case 0:
            break;
        case 1:
            P20 = 0;
            i20--;
            break;
        default:
            if(e05 == i20)
                P20 = 1;
            i20--;
            break;
        }
        switch(i21)
        {
        case 0:
            break;
        case 1:
            P21 = 0;
            i21--;
            break;
        default:
            if(e05 == i21)
                P21 = 1;
            i21--;
            break;
        }
        switch(i22)
        {
        case 0:
            break;
        case 1:
            //CCAP0H = 255;
            P22 = 0;
            i22--;
            break;
        default:
            if(e04 == i22)
                P22 = 1;
            i22--;
            break;
        }
        switch(i23)
        {
        case 0:
            break;
        case 1:
            //CCAP1H = 255;
            P23 = 0;
            i23--;
            break;
        default:
            if(e05 == i23)
                P23 = 1;
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            //CCAP2H = 255;
            P24 = 0;
            i24--;
            break;
        default:
            if(e05 == i24)
                P24 = 1;
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            //CCAP3H = 255;
            P25 = 0;
            i25--;
            break;
        default:
            if(e05 == i25)
                P25 = 1;
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            //CCAP4H = 255;
            P26 = 0;
            i26--;
            break;
        default:
            if(e04 == i26)
                P26 = 1;
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32 = 0;
            i32--;
            break;
        default:
            if(e05 == i32)
                P32 = 1;
            i32--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50 = 0;
            i50--;
            break;
        default:
            if(e05 == i50)
                P50 = 1;
            i50--;
            break;
        }
#endif
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34 = 0;
            i34--;
            break;
        default:
            if(e05 == i34)
                P34 = 1;
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35 = 0;
            i35--;
            break;
        default:
            if(e05 == i35)
                P35 = 1;
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36 = 0;
            i36--;
            break;
        default:
            if(e04 == i36)
                P36 = 1;
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37 = 0;
            i37--;
            break;
        default:
            if(e05 == i37)
                P37 = 1;
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40 = 0;
            i40--;
            break;
        default:
            if(e05 == i40)
                P40 = 1;
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41 = 0;
            i41--;
            break;
        default:
            if(e05 == i41)
                P41 = 1;
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42 = 0;
            i42--;
            break;
        default:
            if(e05 == i42)
                P42 = 1;
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43 = 0;
            i43--;
            break;
        default:
            if(e05 == i43)
                P43 = 1;
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46 = 0;
            i46--;
            break;
        default:
            if(e05 == i46)
                P46 = 1;
            i46--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            i00 = e04;
            i56--;
            break;
        default:
            i56--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            i05 = e04;
            i51--;
            break;
        default:
            i51--;
            break;
        }
        switch(i52)
        {
        case 0:
            break;
        case 1:
            i15 = e04;
            i52--;
            break;
        default:
            i52--;
            break;
        }
        switch(i53)
        {
        case 0:
            break;
        case 1:
            i22 = e04;
            i53--;
            break;
        default:
            i53--;
            break;
        }
        switch(i54)
        {
        case 0:
            break;
        case 1:
            i26 = e04;
            i54--;
            break;
        default:
            i54--;
            break;
        }
        switch(i55)
        {
        case 0:
            break;
        case 1:
            i36 = e04;
            i55--;
            break;
        default:
            i55--;
            break;
        }
        break;
    case 7:
        switch(i00)
        {
        case 0:
            break;
        case 1:
            P00 = 0;
            i00--;
            break;
        default:
            if(e04 == i00)
                P00 = 1;
            i00--;
            break;
        }
        switch(i01)
        {
        case 0:
            break;
        case 1:
            P01 = 0;
            i01--;
            break;
        default:
            if(e05 == i01)
                P01 = 1;
            i01--;
            break;
        }
        switch(i02)
        {
        case 0:
            break;
        case 1:
            P02 = 0;
            i02--;
            break;
        default:
            if(e05 == i02)
                P02 = 1;
            i02--;
            break;
        }
        switch(i03)
        {
        case 0:
            break;
        case 1:
            P03 = 0;
            i03--;
            break;
        default:
            if(e05 == i03)
                P03 = 1;
            i03--;
            break;
        }
        switch(i04)
        {
        case 0:
            break;
        case 1:
            P04 = 0;
            i04--;
            break;
        default:
            if(e05 == i04)
                P04 = 1;
            i04--;
            break;
        }
        switch(i05)
        {
        case 0:
            break;
        case 1:
            P05 = 0;
            i05--;
            break;
        default:
            if(e04 == i05)
                P05 = 1;
            i05--;
            break;
        }
        switch(i06)
        {
        case 0:
            break;
        case 1:
            P06 = 0;
            i06--;
            break;
        default:
            if(e05 == i06)
                P06 = 1;
            i06--;
            break;
        }
        switch(i07)
        {
        case 0:
            break;
        case 1:
            P07 = 0;
            i07--;
            break;
        default:
            if(e05 == i07)
                P07 = 1;
            i07--;
            break;
        }
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11 = 0;
            i11--;
            break;
        default:
            if(e05 == i11)
                P11 = 1;
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14 = 0;
            i14--;
            break;
        default:
            if(e05 == i14)
                P14 = 1;
            i14--;
            break;
        }
        switch(i15)
        {
        case 0:
            break;
        case 1:
            P15 = 0;
            i15--;
            break;
        default:
            if(e04 == i15)
                P15 = 1;
            i15--;
            break;
        }
        switch(i16)
        {
        case 0:
            break;
        case 1:
            P16 = 0;
            i16--;
            break;
        default:
            if(e05 == i16)
                P16 = 1;
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17 = 0;
            i17--;
            break;
        default:
            if(e05 == i17)
                P17 = 1;
            i17--;
            break;
        }
        switch(i20)
        {
        case 0:
            break;
        case 1:
            P20 = 0;
            i20--;
            break;
        default:
            if(e05 == i20)
                P20 = 1;
            i20--;
            break;
        }
        switch(i21)
        {
        case 0:
            break;
        case 1:
            P21 = 0;
            i21--;
            break;
        default:
            if(e05 == i21)
                P21 = 1;
            i21--;
            break;
        }
        switch(i22)
        {
        case 0:
            break;
        case 1:
            //CCAP0H = 255;
            P22 = 0;
            i22--;
            break;
        default:
            if(e04 == i22)
                P22 = 1;
            i22--;
            break;
        }
        switch(i23)
        {
        case 0:
            break;
        case 1:
            //CCAP1H = 255;
            P23 = 0;
            i23--;
            break;
        default:
            if(e05 == i23)
                P23 = 1;
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            //CCAP2H = 255;
            P24 = 0;
            i24--;
            break;
        default:
            if(e05 == i24)
                P24 = 1;
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            //CCAP3H = 255;
            P25 = 0;
            i25--;
            break;
        default:
            if(e05 == i25)
                P25 = 1;
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            //CCAP4H = 255;
            P26 = 0;
            i26--;
            break;
        default:
            if(e05 == i26)
                P26 = 1;
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32 = 0;
            i32--;
            break;
        default:
            if(e05 == i32)
                P32 = 1;
            i32--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43 = 0;
            i43--;
            break;
        default:
            if(e05 == i43)
                P43 = 1;
            i43--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34 = 0;
            i34--;
            break;
        default:
            if(e05 == i34)
                P34 = 1;
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35 = 0;
            i35--;
            break;
        default:
            if(e05 == i35)
                P35 = 1;
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36 = 0;
            i36--;
            break;
        default:
            if(e05 == i36)
                P36 = 1;
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37 = 0;
            i37--;
            break;
        default:
            if(e05 == i37)
                P37 = 1;
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40 = 0;
            i40--;
            break;
        default:
            if(e05 == i40)
                P40 = 1;
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41 = 0;
            i41--;
            break;
        default:
            if(e05 == i41)
                P41 = 1;
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42 = 0;
            i42--;
            break;
        default:
            if(e05 == i42)
                P42 = 1;
            i42--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            i00 = e04;
            i56--;
            break;
        default:
            i56--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            i05 = e04;
            i51--;
            break;
        default:
            i51--;
            break;
        }
        switch(i52)
        {
        case 0:
            break;
        case 1:
            i15 = e04;
            i52--;
            break;
        default:
            i52--;
            break;
        }
        switch(i53)
        {
        case 0:
            break;
        case 1:
            i22 = e04;
            i53--;
            break;
        default:
            i53--;
            break;
        }
        break;
    case 8:
        //case 9:
        //case 10:
        switch(i00)
        {
        case 0:
            break;
        case 1:
            P00 = 0;
            i00--;
            break;
        default:
            if(e04 == i00)
                P00 = 1;
            i00--;
            break;
        }
        switch(i01)
        {
        case 0:
            break;
        case 1:
            P01 = 0;
            i01--;
            break;
        default:
            if(e05 == i01)
                P01 = 1;
            i01--;
            break;
        }
        switch(i02)
        {
        case 0:
            break;
        case 1:
            P02 = 0;
            i02--;
            break;
        default:
            if(e05 == i02)
                P02 = 1;
            i02--;
            break;
        }
        switch(i03)
        {
        case 0:
            break;
        case 1:
            P03 = 0;
            i03--;
            break;
        default:
            if(e05 == i03)
                P03 = 1;
            i03--;
            break;
        }
        switch(i04)
        {
        case 0:
            break;
        case 1:
            P04 = 0;
            i04--;
            break;
        default:
            if(e05 == i04)
                P04 = 1;
            i04--;
            break;
        }
        switch(i05)
        {
        case 0:
            break;
        case 1:
            P05 = 0;
            i05--;
            break;
        default:
            if(e05 == i05)
                P05 = 1;
            i05--;
            break;
        }
        switch(i06)
        {
        case 0:
            break;
        case 1:
            P06 = 0;
            i06--;
            break;
        default:
            if(e05 == i06)
                P06 = 1;
            i06--;
            break;
        }
        switch(i07)
        {
        case 0:
            break;
        case 1:
            P07 = 0;
            i07--;
            break;
        default:
            if(e05 == i07)
                P07 = 1;
            i07--;
            break;
        }
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11 = 0;
            i11--;
            break;
        default:
            if(e05 == i11)
                P11 = 1;
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14 = 0;
            i14--;
            break;
        default:
            if(e05 == i14)
                P14 = 1;
            i14--;
            break;
        }
        switch(i15)
        {
        case 0:
            break;
        case 1:
            P15 = 0;
            i15--;
            break;
        default:
            if(e05 == i15)
                P15 = 1;
            i15--;
            break;
        }
        switch(i16)
        {
        case 0:
            break;
        case 1:
            P16 = 0;
            i16--;
            break;
        default:
            if(e05 == i16)
                P16 = 1;
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17 = 0;
            i17--;
            break;
        default:
            if(e05 == i17)
                P17 = 1;
            i17--;
            break;
        }
        switch(i20)
        {
        case 0:
            break;
        case 1:
            P20 = 0;
            i20--;
            break;
        default:
            if(e05 == i20)
                P20 = 1;
            i20--;
            break;
        }
        switch(i21)
        {
        case 0:
            break;
        case 1:
            P21 = 0;
            i21--;
            break;
        default:
            if(e05 == i21)
                P21 = 1;
            i21--;
            break;
        }
        switch(i22)
        {
        case 0:
            break;
        case 1:
            //CCAP0H = 255;
            P22 = 0;
            i22--;
            break;
        default:
            if(e04 == i22)
                P22 = 1;
            i22--;
            break;
        }
        switch(i23)
        {
        case 0:
            break;
        case 1:
            //CCAP1H = 255;
            P23 = 0;
            i23--;
            break;
        default:
            if(e05 == i23)
                P23 = 1;
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            //CCAP2H = 255;
            P24 = 0;
            i24--;
            break;
        default:
            if(e04 == i24)
                P24 = 1;
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            //CCAP3H = 255;
            P25 = 0;
            i25--;
            break;
        default:
            if(e05 == i25)
                P25 = 1;
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            //CCAP4H = 255;
            P26 = 0;
            i26--;
            break;
        default:
            if(e05 == i26)
                P26 = 1;
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32 = 0;
            i32--;
            break;
        default:
            if(e05 == i32)
                P32 = 1;
            i32--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37 = 0;
            i37--;
            break;
        default:
            if(e05 == i37)
                P37 = 1;
            i37--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34 = 0;
            i34--;
            break;
        default:
            if(e05 == i34)
                P34 = 1;
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35 = 0;
            i35--;
            break;
        default:
            if(e04 == i35)
                P35 = 1;
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36 = 0;
            i36--;
            break;
        default:
            if(e05 == i36)
                P36 = 1;
            i36--;
            break;
        }
#ifdef ukulelechord
        switch(i37)
        {
        case 0:
            break;
        case (e07-6):
            P35 = 1;
            i37--;
            break;
        case (e07-4):
            P22 = 1;
            i37--;
            break;
        case (e07-2):
            P00 = 1;
            i37--;
            break;
        case e07:
            P24 = 1;
            i37--;
            break;
        default:
            i37--;
            break;
        }
#else
        switch(i56)
        {
        case 0:
            break;
        case 1:
            i24 = e04;
            i56--;
            break;
        default:
            i56--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            i00 = e04;
            i51--;
            break;
        default:
            i51--;
            break;
        }
        switch(i52)
        {
        case 0:
            break;
        case 1:
            i22 = e04;
            i52--;
            break;
        default:
            i52--;
            break;
        }
        switch(i53)
        {
        case 0:
            break;
        case 1:
            i35 = e04;
            i53--;
            break;
        default:
            i53--;
            break;
        }
#endif
        break;
    case 9:
        switch(i00)
        {
        case 0:
            break;
        case 1:
            P00 = 0;
            i00--;
            break;
        default:
            if(e23+1 == i00)
                P00 = 1;
            i00--;
            break;
        }
        switch(i01)
        {
        case 0:
            break;
        case 1:
            P01 = 0;
            i01--;
            break;
        default:
            if(e23+4 == i01)
                P01 = 1;
            i01--;
            break;
        }
        switch(i02)
        {
        case 0:
            break;
        case 1:
            P02 = 0;
            i02--;
            break;
        default:
            if(e23 == i02)
                P02 = 1;
            i02--;
            break;
        }
        switch(i03)
        {
        case 0:
            break;
        case 1:
            P03 = 0;
            i03--;
            break;
        default:
            if(e23-1 == i03)
                P03 = 1;
            i03--;
            break;
        }
        switch(i04)
        {
        case 0:
            break;
        case 1:
            P04 = 0;
            i04--;
            break;
        default:
            if(e23-1 == i04)
                P04 = 1;
            i04--;
            break;
        }
        switch(i05)
        {
        case 0:
            break;
        case 1:
            P05 = 0;
            i05--;
            break;
        default:
            if(e23-1 == i05)
                P05 = 1;
            i05--;
            break;
        }
        switch(i06)
        {
        case 0:
            break;
        case 1:
            P06 = 0;
            i06--;
            break;
        default:
            if(e23-1 == i06)
                P06 = 1;
            i06--;
            break;
        }
        switch(i07)
        {
        case 0:
            break;
        case 1:
            P07 = 0;
            i07--;
            break;
        default:
            if(e23+1 == i07)
                P07 = 1;
            i07--;
            break;
        }
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11 = 0;
            i11--;
            break;
        default:
            if(e23+1 == i11)
                P11 = 1;
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14 = 0;
            i14--;
            break;
        default:
            if(e23+1 == i14)
                P14 = 1;
            i14--;
            break;
        }
        switch(i15)
        {
        case 0:
            break;
        case 1:
            P15 = 0;
            i15--;
            break;
        default:
            if(e23+1 == i15)
                P15 = 1;
            i15--;
            break;
        }
        switch(i16)
        {
        case 0:
            break;
        case 1:
            P16 = 0;
            i16--;
            break;
        default:
            if(e23+1 == i16)
                P16 = 1;
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17 = 0;
            i17--;
            break;
        default:
            if(e23+1 == i17)
                P17 = 1;
            i17--;
            break;
        }
        switch(i20)
        {
        case 0:
            break;
        case 1:
            P20 = 0;
            i20--;
            break;
        default:
            if(e23+1 == i20)
                P20 = 1;
            i20--;
            break;
        }
        switch(i21)
        {
        case 0:
            break;
        case 1:
            P21 = 0;
            i21--;
            break;
        default:
            if(e23 == i21)
                P21 = 1;
            i21--;
            break;
        }
        switch(i22)
        {
        case 0:
            break;
        case 1:
            //CCAP0H = 255;
            P22 = 0;
            i22--;
            break;
        default:
            if(e23 == i22)
                P22 = 1;
            i22--;
            break;
        }
        switch(i23)
        {
        case 0:
            break;
        case 1:
            //CCAP1H = 255;
            P23 = 0;
            i23--;
            break;
        default:
            if(e23 == i23)
                P23 = 1;
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            //CCAP2H = 255;
            P24 = 0;
            i24--;
            break;
        default:
            if(e23-1 == i24)
                P24 = 1;
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            //CCAP3H = 255;
            P25 = 0;
            i25--;
            break;
        default:
            if(e23-1 == i25)
                P25 = 1;
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            //CCAP4H = 255;
            P26 = 0;
            i26--;
            break;
        default:
            if(e23 == i26)
                P26 = 1;
            i26--;
            break;
        }
        switch(i27)
        {
        case 0:
            break;
        case 1:
            //CCAP5H = 255;
            P27 = 0;
            i27--;
            break;
        default:
            if(e23 == i27)
                P27 = 1;
            i27--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32 = 0;
            i32--;
            break;
        default:
            if(e23 == i32)
                P32 = 1;
            i32--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34 = 0;
            i34--;
            break;
        default:
            if(e23 == i34)
                P34 = 1;
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35 = 0;
            i35--;
            break;
        default:
            if(e23 == i35)
                P35 = 1;
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36 = 0;
            i36--;
            break;
        default:
            if(e23 == i36)
                P36 = 1;
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37 = 0;
            i37--;
            break;
        default:
            if(e23 == i37)
                P37 = 1;
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40 = 0;
            i40--;
            break;
        default:
            if(e23 == i40)
                P40 = 1;
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41 = 0;
            i41--;
            break;
        default:
            if(e23 == i41)
                P41 = 1;
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42 = 0;
            i42--;
            break;
        default:
            if(e23 == i42)
                P42 = 1;
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43 = 0;
            i43--;
            break;
        default:
            if(e23 == i43)
                P43 = 1;
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46 = 0;
            i46--;
            break;
        default:
            if(e23 == i46)
                P46 = 1;
            i46--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50 = 0;
            i50--;
            break;
        default:
            if(e23 == i50)
                P50 = 1;
            i50--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            P51 = 0;
            i51--;
            break;
        default:
            if(e23 == i51)
                P51 = 1;
            i51--;
            break;
        }
        switch(i52)
        {
        case 0:
            break;
        case 1:
            P52 = 0;
            i52--;
            break;
        default:
            if(e23 == i52)
                P52 = 1;
            i52--;
            break;
        }
        switch(i53)
        {
        case 0:
            break;
        case 1:
            P53 = 0;
            i53--;
            break;
        default:
            if(e23 == i53)
                P53 = 1;
            i53--;
            break;
        }
        switch(i54)
        {
        case 0:
            break;
        case 1:
            P54 = 0;
            i54--;
            break;
        default:
            if(e23 == i54)
                P54 = 1;
            i54--;
            break;
        }
        switch(i55)
        {
        case 0:
            break;
        default:
            if(i55 < (e06-50) )
            {
                SFRPI = 1;
                if(!P63)
                    P55 = 0;
                SFRPI = 0;
            }
            else if(e06 == i06)
                P55 = 1;
            i55--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            P56 = 0;
            i56--;
            break;
        default:
            if(e23 == i56)
                P56 = 1;
            i56--;
            break;
        }
        switch(i57)
        {
        case 0:
            break;
        case 1:
            P57 = 0;
            i57--;
            break;
        default:
            if(e23 == i57)
                P57 = 1;
            i57--;
            break;
        }
#endif
        break;
    }
}
#endif
/*****************************************************/
void SCON_int(void)  interrupt 4  //串列中斷函數
{
    if(RI)
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
        }
    }
    //else
    //TI=0;
}

/***********************************************************
*函數名稱: UART_init
*功能描述: UART啟始程式
*輸入參數：bps
************************************************************/
void UART_init(unsigned int bps)  //UART啟始程式
{
    P0M0=0xFF;
    P1M0=0xFB; //設定P0為推挽式輸出(M0-1=01)
    P2M0=0xFF;
    P3M1=0xF4;
    P4M0=0x4F;
    P5M0=0xFF;
    REN = 1;
    SM1=1;//SCON = 0x50;     //設定UART串列傳輸為MODE1及致能接收
    TMOD |= T1_M1;  //設定TIMER1為MODE2
    AUXR2 = T1X12;// + S2TX12 + S2SMOD + S2TR;	// T1X12 for uart1	URM0X6 for uart2
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
    CL=CH=0;   //PCA計數器由0開始上數
#ifdef PCATIMER
    if(CCF5)
    {
        CCF5=0; //清除模組0-5的比較旗標
    }//第T*6秒動作，PCA計數器由0上數
    go_mad();
#endif
}
#if defined(PCATIMER) || defined(TIMER0)
void go_mad()
{
    switch( oneCHANNEL )
    {
    case 0:
    case 1:
        if(P00VAR && !i00)
        {
            i00 = 5;
        }
        if(P01VAR && !i01)
        {
            i01 = 5;
        }
        if(P02VAR && !i02)
        {
            i02 = 5;
        }
        if(P03VAR && !i03)
        {
            i03 = 5;
        }
        if(P04VAR && !i04)
        {
            i04 = 5;
        }
        if(P05VAR && !i05)
        {
            i05 = 5;
        }
        if(P06VAR && !i06)
        {
            i06 = 5;
        }
        if(P07VAR && !i07)
        {
            i07 = 5;
        }
        if(P11VAR && !i11)
        {
            i11 = 5;
        }
        if(P14VAR && !i14)
        {
            i14 = 5;
        }
        if(P15VAR && !i15)
        {
            i15 = 5;
        }
        if(P16VAR && !i16)
        {
            i16 = 5;
        }
        if(P17VAR && !i17)
        {
            i17 = 5;
        }
        if(P20VAR && !i20)
        {
            i20 = 5;
        }
        if(P21VAR && !i21)
        {
            i21 = 5;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = 5;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = 5;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = 5;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = 5;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = 5;
        }
        if(P27VAR && !i27)
        {
            CCAP5H = ~P27VAR;
            i27 = 5;
        }
        if(P32VAR && !i32)
        {
            i32 = 5;
        }
        if(P34VAR && !i34)
        {
            i34 = 5;
        }
        if(P35VAR && !i35)
        {
            i35 = 5;
        }
        if(P36VAR && !i36)
        {
            i36 = 5;
        }
        if(P37VAR && !i37)
        {
            i37 = 5;
        }
        if(P40VAR && !i40)
        {
            i40 = 5;
        }
        if(P41VAR && !i41)
        {
            i41 = 5;
        }
        /*
        if(P42VAR && !i42)
        {
            i42 = 5;
        }
         */
        if(P43VAR && !i43)
        {
            i43 = 5;
        }
        if(P46VAR && !i46)
        {
            i46 = 5;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = 5;
        }
        if(P51VAR && !i51)
        {
            i51 = 5;
        }
        if(P52VAR && !i52)
        {
            i52 = 5;
        }
        if(P53VAR && !i53)
        {
            i53 = 5;
        }
        if(P54VAR && !i54)
        {
            i54 = 5;
        }
        if(P55VAR && !i55)
        {
            i55 = 5;
        }
        if(P56VAR && !i56)
        {
            i56 = 5;
        }
        if(P57VAR && !i57)
        {
            i57 = 5;
        }
        P5 |= 0xFF;
#endif
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0xF4;
        P4 |= 0x4B;
        break;
    case 2:
    case 3:
        if(P00VAR && !i00)
        {
            i00 = 5;
        }
        if(P01VAR && !i01)
        {
            i01 = 5;
        }
        if(P02VAR && !i02)
        {
            i02 = 5;
        }
        if(P03VAR && !i03)
        {
            i03 = 5;
        }
        if(P04VAR && !i04)
        {
            i04 = 5;
        }
        if(P05VAR && !i05)
        {
            i05 = 5;
        }
        if(P06VAR && !i06)
        {
            i06 = 5;
        }
        if(P07VAR && !i07)
        {
            i07 = 5;
        }
        if(P11VAR && !i11)
        {
            i11 = 5;
        }
        if(P14VAR && !i14)
        {
            i14 = 5;
        }
        if(P15VAR && !i15)
        {
            i15 = 5;
        }
        if(P16VAR && !i16)
        {
            i16 = 5;
        }
        if(P17VAR && !i17)
        {
            i17 = 5;
        }
        if(P20VAR && !i20)
        {
            i20 = 5;
        }
        if(P21VAR && !i21)
        {
            i21 = 5;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = 5;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = 5;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = 5;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = 5;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = 5;
        }
#ifndef LEDRay
        if(P56VAR && !i56)
        {
            i56 = 5;
        }
        if(P55VAR && !i55)
        {
            i55 = 5;
        }
#endif
        if(P34VAR && !i34)
        {
            i34 = 5;
        }
        if(P35VAR && !i35)
        {
            i35 = 5;
        }
        if(P36VAR && !i36)
        {
            i36 = 5;
        }
        if(P37VAR && !i37)
        {
            i37 = 5;
        }
        if(P40VAR && !i40)
        {
            i40 = 5;
        }
        if(P41VAR && !i41)
        {
            i41 = 5;
        }
        if(P42VAR && !i42)
        {
            i42 = 5;
        }
        if(P43VAR && !i43)
        {
            i43 = 5;
        }
        if(P46VAR && !i46)
        {
            i46 = 5;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = 5;
        }
        if(P51VAR && !i51)
        {
            i51 = 5;
        }
        if(P52VAR && !i52)
        {
            i52 = 5;
        }
        if(P53VAR && !i53)
        {
            i53 = 5;
        }
        if(P54VAR && !i54)
        {
            i54 = 5;
        }
        P5 |= 0x7F;
#endif
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0xF4;
        P4 |= 0x4F;
        break;
    case 4:
    case 5:
    case 6:
        if(P00VAR && !i00)
        {
            i00 = e04;
        }
        if(P01VAR && !i01)
        {
            i01 = e05;
        }
        if(P02VAR && !i02)
        {
            i02 = e05;
        }
        if(P03VAR && !i03)
        {
            i03 = e05;
        }
        if(P04VAR && !i04)
        {
            i04 = e05;
        }
        if(P05VAR && !i05)
        {
            i05 = e04;
        }
        if(P06VAR && !i06)
        {
            i06 = e05;
        }
        if(P07VAR && !i07)
        {
            i07 = e05;
        }
        if(P11VAR && !i11)
        {
            i11 = e05;
        }
        if(P14VAR && !i14)
        {
            i14 = e05;
        }
        if(P15VAR && !i15)
        {
            i15 = e04;
        }
        if(P16VAR && !i16)
        {
            i16 = e05;
        }
        if(P17VAR && !i17)
        {
            i17 = e05;
        }
        if(P20VAR && !i20)
        {
            i20 = e05;
        }
        if(P21VAR && !i21)
        {
            i21 = e05;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = e04;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = e05;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = e05;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = e05;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = e04;
        }
        if(P32VAR && !i32)
        {
            i32 = e05;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = e05;
        }
        P50 = 1;
#endif
        if(P34VAR && !i34)
        {
            i34 = e05;
        }
        if(P35VAR && !i35)
        {
            i35 = e05;
        }
        if(P36VAR && !i36)
        {
            i36 = e04;
        }
        if(P37VAR && !i37)
        {
            i37 = e05;
        }
        if(P40VAR && !i40)
        {
            i40 = e05;
        }
        if(P41VAR && !i41)
        {
            i41 = e05;
        }
        if(P42VAR && !i42)
        {
            i42 = e05;
        }
        if(P43VAR && !i43)
        {
            i43 = e05;
        }
        if(P46VAR && !i46)
        {
            i46 = e05;
        }
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0xF4;
        P4 |= 0x4F;
        break;
    case 7:
        if(P00VAR && !i00)
        {
            i00 = e04;
        }
        if(P01VAR && !i01)
        {
            i01 = e05;
        }
        if(P02VAR && !i02)
        {
            i02 = e05;
        }
        if(P03VAR && !i03)
        {
            i03 = e05;
        }
        if(P04VAR && !i04)
        {
            i04 = e05;
        }
        if(P05VAR && !i05)
        {
            i05 = e04;
        }
        if(P06VAR && !i06)
        {
            i06 = e05;
        }
        if(P07VAR && !i07)
        {
            i07 = e05;
        }
        if(P11VAR && !i11)
        {
            i11 = e05;
        }
        if(P14VAR && !i14)
        {
            i14 = e05;
        }
        if(P15VAR && !i15)
        {
            i15 = e04;
        }
        if(P16VAR && !i16)
        {
            i16 = e05;
        }
        if(P17VAR && !i17)
        {
            i17 = e05;
        }
        if(P20VAR && !i20)
        {
            i20 = e05;
        }
        if(P21VAR && !i21)
        {
            i21 = e05;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = e04;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = e05;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = e05;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = e05;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = e05;
        }
        if(P32VAR && !i32)
        {
            i32 = e05;
        }
        if(P43VAR && !i43)
        {
            i43 = e05;
        }
        if(P34VAR && !i34)
        {
            i34 = e05;
        }
        if(P35VAR && !i35)
        {
            i35 = e05;
        }
        if(P36VAR && !i36)
        {
            i36 = e05;
        }
        if(P37VAR && !i37)
        {
            i37 = e05;
        }
        if(P40VAR && !i40)
        {
            i40 = e05;
        }
        if(P41VAR && !i41)
        {
            i41 = e05;
        }
        if(P42VAR && !i42)
        {
            i42 = e05;
        }
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0xF4;
        P4 |= 0x07;
        break;
    case 8:
        //case 9:
        //case 10:
        if(P00VAR && !i00)
        {
            i00 = e04;
        }
        if(P01VAR && !i01)
        {
            i01 = e05;
        }
        if(P02VAR && !i02)
        {
            i02 = e05;
        }
        if(P03VAR && !i03)
        {
            i03 = e05;
        }
        if(P04VAR && !i04)
        {
            i04 = e05;
        }
        if(P05VAR && !i05)
        {
            i05 = e05;
        }
        if(P06VAR && !i06)
        {
            i06 = e05;
        }
        if(P07VAR && !i07)
        {
            i07 = e05;
        }
        if(P11VAR && !i11)
        {
            i11 = e05;
        }
        if(P14VAR && !i14)
        {
            i14 = e05;
        }
        if(P15VAR && !i15)
        {
            i15 = e05;
        }
        if(P16VAR && !i16)
        {
            i16 = e05;
        }
        if(P17VAR && !i17)
        {
            i17 = e05;
        }
        if(P20VAR && !i20)
        {
            i20 = e05;
        }
        if(P21VAR && !i21)
        {
            i21 = e05;
        }
        if(P22VAR && !i22)
        {
            i22 = e04;
            CCAP0H = ~P22VAR;
        }
        if(P23VAR && !i23)
        {
            i23 = e05;
            CCAP1H = ~P23VAR;
        }
        if(P24VAR && !i24)
        {
            i24 = e04;
            CCAP2H = ~P24VAR;
        }
        if(P25VAR && !i25)
        {
            i25 = e05;
            CCAP3H = ~P25VAR;
        }
        if(P26VAR && !i26)
        {
            i26 = e05;
            CCAP4H = ~P26VAR;
        }
        if(P32VAR && !i32)
        {
            i32 = e05;
        }
        if(P37VAR && !i37)
        {
            i37 = e05;
        }
        if(P34VAR && !i34)
        {
            i34 = e05;
        }
        if(P35VAR && !i35)
        {
            i35 = e04;
        }
        if(P36VAR && !i36)
        {
            i36 = e05;
        }
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0xF4;
        break;
    case 9:
        if(P00VAR && !i00)
        {
            i00 = 4;
        }
        if(P01VAR && !i01)
        {
            i01 = 4;
        }
        if(P02VAR && !i02)
        {
            i02 = 4;
        }
        if(P03VAR && !i03)
        {
            i03 = 4;
        }
        if(P04VAR && !i04)
        {
            i04 = 4;
        }
        if(P05VAR && !i05)
        {
            i05 = 4;
        }
        if(P06VAR && !i06)
        {
            i06 = 4;
        }
        if(P07VAR && !i07)
        {
            i07 = 4;
        }
        if(P11VAR && !i11)
        {
            i11 = 4;
        }
        if(P14VAR && !i14)
        {
            i14 = 4;
        }
        if(P15VAR && !i15)
        {
            i15 = 4;
        }
        if(P16VAR && !i16)
        {
            i16 = 4;
        }
        if(P17VAR && !i17)
        {
            i17 = 4;
        }
        if(P20VAR && !i20)
        {
            i20 = 4;
        }
        if(P21VAR && !i21)
        {
            i21 = 4;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = 4;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = 4;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = 4;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = 4;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = 4;
        }
        if(P27VAR && !i27)
        {
            CCAP5H = ~P27VAR;
            i27 = 4;
        }
        if(P32VAR && !i32)
        {
            i32 = 4;
        }
        if(P34VAR && !i34)
        {
            i34 = 4;
        }
        if(P35VAR && !i35)
        {
            i35 = 4;
        }
        if(P36VAR && !i36)
        {
            i36 = 4;
        }
        if(P37VAR && !i37)
        {
            i37 = 4;
        }
        if(P40VAR && !i40)
        {
            i40 = 4;
        }
        if(P41VAR && !i41)
        {
            i41 = 4;
        }
        if(P42VAR && !i42)
        {
            i42 = 4;
        }
        if(P43VAR && !i43)
        {
            i43 = 4;
        }
        if(P46VAR && !i46)
        {
            i46 = 4;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = 4;
        }
        if(P51VAR && !i51)
        {
            i51 = 4;
        }
        if(P52VAR && !i52)
        {
            i52 = 4;
        }
        if(P53VAR && !i53)
        {
            i53 = 4;
        }
        if(P54VAR && !i54)
        {
            i54 = 4;
        }
        if(P55VAR && !i55)
        {
            i55 = e06;
        }
        if(P56VAR && !i56)
        {
            i56 = 4;
        }
        if(P57VAR && !i57)
        {
            i57 = 4;
        }
        P5 |= 0xFF;
#endif
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0xF4;
        P4 |= 0x4F;
        break;
    }
}
#endif
void go_crazy()
{
#ifdef CHANNEL16
    int ll;
    P10=0;
    rayCHANNEL = 0;
    P10=1 ;   //開始串列傳輸
    rayCHANNEL |= P12;
    for(ll=0; ll<7; ll++)
    {
        P13 = 0;
        rayCHANNEL <<= 1;
        P13 = 1;
        rayCHANNEL |= P12;
    }
#ifdef LEDRay
    LED=~rayCHANNEL;     //將接收到的字元由LED輸出
#endif
    //rayCHANNEL = ~rayCHANNEL;
    twoCHANNEL = (rayCHANNEL & 0x0F) << 1;
    oneCHANNEL = (rayCHANNEL >> 4);
#elif defined(SIMULATION)
    /*
    if(i10000 == 255)
    {
        //SFRPI = 1;
        P11 = 1;
        //SFRPI = 0;
    }
    else if(i10000 == 128)
    {
        //SFRPI = 1;
        P11 = 0;
        //SFRPI = 0;
    }
    i10000++;
    */
#endif
}

#ifdef TIMER0
/***************************************/
void T0_int(void) interrupt 1  //Timer0中斷函數
{
    /*
    if(i10000++ == 255)
    {
        go_crazy();
    }
    */
    go_mad();
    TL0=0;	//TL0=65536 - TT;
    TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
}
#endif

void rayoff()
{
#ifdef yesoff
    switch( oneCHANNEL )
    {
    case 4:
    case 5:
    case 6:
        i46 = 1;
        switch(note)
        {
            //case 36:
            //i00 = 1;
            //break;
        case 37:
            i01 = 1;
            break;
        case 38:
            i02 = 1;
            break;
        case 39:
            i03 = 1;
            break;
        case 40:
            i04 = 1;
            break;
            //case 41:
            //i05 = 1;
            //break;
        case 42:
            i06 = 1;
            break;
        case 43:
            i07 = 1;
            break;
        case 44:
            i11 = 1;
            break;
        case 45:
            i14 = 1;
            break;
            //case 46:
            //i15 = 1;
            //break;
        case 47:
            i16 = 1;
            break;
        case 48:
            i17 = 1;
            break;
        case 49:
            i20 = 1;
            break;
        case 50:
            i21 = 1;
            break;
            //case 51:
            //i22 = 1;
            //break;
        case 52:
            i23 = 1;
            break;
        case 53:
            i24 = 1;
            break;
        case 54:
            i25 = 1;
            break;
            //case 55:
            //i26 = 1;
            //break;
        case 56:
            i32 = 1;
            break;
        case 57:
            i50 = 1;
            break;
        case 58:
            i34 = 1;
            break;
        case 59:
            i35 = 1;
            break;
            //case 60:
            //i36 = 1;
            //break;
        case 61:
            i37 = 1;
            break;
        case 62:
            i40 = 1;
            break;
        case 63:
            i41 = 1;
            break;
        case 64:
            i42 = 1;
            break;
        case 65:
            i43 = 1;
            break;
        }
        break;
    case 7:
        i42 = 1;
        switch(note)
        {
        case 37:
            i01 = 1;
            break;
        case 38:
            i02 = 1;
            break;
        case 39:
            i03 = 1;
            break;
        case 40:
            i04 = 1;
            break;
        case 42:
            i06 = 1;
            break;
        case 43:
            i07 = 1;
            break;
        case 44:
            i11 = 1;
            break;
        case 45:
            i14 = 1;
            break;
        case 47:
            i16 = 1;
            break;
        case 48:
            i17 = 1;
            break;
        case 49:
            i20 = 1;
            break;
        case 50:
            i21 = 1;
            break;
        case 52:
            i23 = 1;
            break;
        case 53:
            i24 = 1;
            break;
        case 54:
            i25 = 1;
            break;
        case 55:
            i26 = 1;
            break;
        case 56:
            i32 = 1;
            break;
        case 57:
            i43 = 1;
            break;
        case 58:
            i34 = 1;
            break;
        case 59:
            i35 = 1;
            break;
        case 60:
            i36 = 1;
            break;
        case 61:
            i37 = 1;
            break;
        case 62:
            i40 = 1;
            break;
        case 63:
            i41 = 1;
            break;
        }
        break;
    case 8:
        switch(note)
        {
        case 36:
#ifndef ukulelechord
#else
            P17VAR = 255;
            i37 = e07;
#endif
            break;
        case 37:
#ifndef ukulelechord
            i01 = 1;
#else
            P25VAR = P01VAR = P23VAR = P20VAR = 255;
            i37 = e07;
#endif
            break;
        case 38:
#ifndef ukulelechord
            i02 = 1;
#else
            P26VAR = P02VAR = P06VAR = 255;
            i37 = e07;
#endif
            break;
        case 39:
#ifndef ukulelechord
            i03 = 1;
#else
            P03VAR = P07VAR = P15VAR = 255;
            i37 = e07;
#endif
            break;
        case 40:
#ifndef ukulelechord
            i04 = 1;
#else
            P25VAR = P04VAR = P16VAR = 255;
            i37 = e07;
#endif
            break;
        case 41:
#ifndef ukulelechord
            i23 = 1;
            i05 = 1;
#else
            P26VAR = P23VAR = 255;
            i37 = e07;
#endif
            break;
        case 42:
#ifndef ukulelechord
            i06 = 1;
#else
            P32VAR = P01VAR = P06VAR = P20VAR = 255;
            i37 = e07;
#endif
            break;
        case 43:
#ifndef ukulelechord
            i07 = 1;
#else
            P02VAR = P07VAR = P16VAR = 255;
            i37 = e07;
#endif
            break;
        case 44:
#ifndef ukulelechord
            i25 = 1;
            i11 = 1;
#else
            P25VAR = P03VAR = P11VAR = P17VAR = 255;
            i37 = e07;
#endif
            break;
        case 45:
#ifndef ukulelechord
            i26 = 1;
            i14 = 1;
#else
            P01VAR = P26VAR = 255;
            i37 = e07;
#endif
            break;
        case 46:
#ifndef ukulelechord
            i15 = 1;
            i32 = 1;
#else
            P32VAR = P02VAR = P23VAR = P15VAR = 255;
            i37 = e07;
#endif
            break;
        case 47:
#ifndef ukulelechord
            i16 = 1;
            i37 = 1;
#else
            P33VAR = P03VAR = P06VAR = P16VAR = 255;
            i37 = e07;
#endif
            break;
        case 48:
#ifndef ukulelechord
            i17 = 1;
            i34 = 1;
#else
            P17VAR = 255;
            i37 = e07;
#endif
            break;
        case 49:
#ifndef ukulelechord
            i20 = 1;
#else
            P25VAR = P01VAR = P23VAR = P20VAR = 255;
            i37 = e07;
#endif
            break;
        case 50:
#ifndef ukulelechord
            i21 = 1;
#else
            P26VAR = P02VAR = P06VAR = 255;
            i37 = e07;
#endif
            break;
        }
        break;
    }
#endif
}