/**********UART5.C ******UART與LCD傳輸**********
*動作：接收個人電腦的資料，送到LCD顯示
*硬體：SW3-3(TxD1)ON
************************************************/
#include "..\REG_MG84FG516.H"   //暫存器及組態定義
#include "math.h"
//#define DEBUG
//#define CHANNEL16		  //P10 P12 P13
#define LEDRay
#ifdef DEBUG
#include <stdio.h>   //加入標準輸出入函數
unsigned char oldCHANNEL=0xFF;
#endif
#define HARDRAYPWM		  //P14 P15 P16		CR
#ifdef	HARDRAYPWM
#define PCATIMER
#define TTT  256
unsigned char P00VAR,P01VAR,P02VAR,P03VAR,P04VAR,P05VAR,P06VAR,P07VAR,P11VAR,P14VAR,P15VAR,P16VAR,P17VAR,P20VAR,P21VAR,P22VAR,P23VAR,P24VAR,P25VAR,P26VAR,P32VAR,P33VAR,P34VAR,P35VAR,P36VAR,P37VAR,P40VAR,P41VAR,P42VAR,P43VAR,P46VAR;
#ifndef LEDRay
unsigned char P50VAR,P51VAR,P52VAR,P53VAR,P54VAR,P55VAR,P56VAR,P57VAR;
#endif
#endif
#define TIMER0
#define SIMULATION
#define TIMER2
#define PARSER
#define TT  34286  //Timer延時時間=(1/1.8432MHz)*57600=31250uS
#ifdef TIMER2
unsigned char i00,i01,i02,i03,i04,i05,i06,i07,i11,i14,i15,i16,i17,i20,i21,i22,i23,i24,i25,i26,i32,i33,i34,i35,i36,i37,i40,i41,i42,i43,i46,i10000;
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
unsigned char rayCHANNEL = 0, oneCHANNEL = 10,twoCHANNEL = 24;//#define rayCHANNEL 0x00
#define e04 3
#define e05	55
#define e06 250
#define e07 10
#define ukulelechord
unsigned char channel;
unsigned char note;
unsigned char velocity;
unsigned int action; //1 =note off ; 2=note on ; 3= wait
#endif
#ifdef SIMULATION
int notecount;
unsigned int pressure = 0;
#endif
void consumeToken(unsigned char incomingByte);

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
    CCAPM0=CCAPM1=CCAPM2=CCAPM3=CCAPM4/*=CCAPM5*/=ECOM+PWM; //致能CEX1比較器及PWM輸出
    CMOD=0x00; //CPS1-0=00,Fpwm=Fosc/12/256=22.1184MHz/12/256=7.2KHz
    //PCAPWM0=PCAPWM1=PCAPWM2=PCAPWM3=PCAPWM4=PCAPWM5=ECAPH;
    CCAP0H=CCAP1H=CCAP2H=CCAP3H=CCAP4H/*=CCAP5H*/=~0x00;//0x00; //設定(P12/CEX0)，平均電壓為0V
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
    P00VAR=P01VAR=P02VAR=P03VAR=P04VAR=P05VAR=P06VAR=P07VAR=P11VAR=P14VAR=P15VAR=P16VAR=P17VAR=P20VAR=P21VAR=P22VAR=P23VAR=P24VAR=P25VAR=P26VAR=P32VAR=P33VAR=P34VAR=P35VAR=P36VAR=P40VAR=P41VAR=P42VAR=P43VAR=P46VAR=0;
#ifndef LEDRay
    P50VAR=P51VAR=P52VAR=P53VAR=P54VAR=P55VAR=P56VAR=P57VAR=0;
#endif
#endif
#ifdef TIMER2
    i00=i01=i02=i03=i04=i05=i06=i07=i11=i14=i15=i16=i17=i20=i21=i22=i23=i24=i25=i26=i32=i33=i34=i35=i36=i37=i40=i41=i42=i43=i46=i10000=0;
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
    TMOD |= T0_M0;	//設定Timer0為mode1內部計時
    TL0=0;	//TL0=65536 - TT;
    TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
    ET0=1;	//致能Timer0中
    TR0=1;	//啟動Timer0開始計時
#endif
#ifdef SIMULATION
    notecount = 0;
#endif
    while(1)
    {
        softPWM();	//自我空轉，表示可做其它工作
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
            if( action > OFF && oneCHANNEL == channel )
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
                if( velocity && oneCHANNEL == channel )
                {
                    switch( oneCHANNEL )
                    {
                    case 0:
                    case 1:
#ifndef LEDRay
                        P57VAR = 255;
#endif
                        switch(note)
                        {
                        case 36:
                            P00VAR = 240;
                            break;
                        case 37:
                            P01VAR = 240;
                            break;
                        case 38:
                            P02VAR = 240;
                            break;
                        case 39:
                            P03VAR = 240;
                            break;
                        case 40:
                            P04VAR = 240;
                            break;
                        case 41:
                            P05VAR = 240;
                            break;
                        case 42:
                            P06VAR = 240;
                            break;
                        case 43:
                            P07VAR = 240;
                            break;
                        case 44:
                            P11VAR = 240;
                            break;
                        case 45:
                            P14VAR = 240;
                            break;
                        case 46:
                            P15VAR = 240;
                            break;
                        case 47:
                            P16VAR = 240;
                            break;
                        case 48:
                            P17VAR = 240;
                            break;
                        case 49:
                            P20VAR = 240;
                            break;
                        case 50:
                            P21VAR = 240;
                            break;
                        case 51:
                            P22VAR = 240;
                            break;
                        case 52:
                            P23VAR = 240;
                            break;
                        case 53:
                            P24VAR = 240;
                            break;
                        case 54:
                            P25VAR = 240;
                            break;
                        case 55:
                            P26VAR = 240;
                            break;
                        case 56:
                            P32VAR = 240;
                            break;
                        case 57:
                            P33VAR = 240;
                            break;
                        case 58:
                            P34VAR = 240;
                            break;
                        case 59:
                            P35VAR = 240;
                            break;
                        case 60:
                            P36VAR = 240;
                            break;
                        case 61:
                            P37VAR = 240;
                            break;
                        case 62:
                            P40VAR = 240;
                            break;
                        case 63:
                            P41VAR = 240;
                            break;
                        case 64:
                            P42VAR = 240;
                            break;
                        case 65:
                            P43VAR = 240;
                            break;
                        case 66:
                            P46VAR = 240;
                            break;
#ifndef LEDRay
                        case 67:
                            P50VAR = 240;
                            break;
                        case 68:
                            P51VAR = 240;
                            break;
                        case 69:
                            P52VAR = 240;
                            break;
                        case 70:
                            P53VAR = 240;
                            break;
                        case 71:
                            P54VAR = 240;
                            break;
                        case 72:
                            P55VAR = 240;
                            break;
                        case 73:
                            P56VAR = 240;
                            break;
#endif
                        }
                        break;
                    case 2:
                    case 3:
                    case 12:
                    case 13:
                        switch(note)
                        {
                        case 36:
                            P00VAR = 240;
                            break;
                        case 37:
                            P01VAR = 240;
                            break;
                        case 38:
                            P02VAR = 240;
                            break;
                        case 39:
                            P03VAR = 240;
                            break;
                        case 40:
                            P04VAR = 240;
                            break;
                        case 41:
                            P05VAR = 240;
                            break;
                        case 42:
                            P06VAR = 240;
                            break;
                        case 43:
                            P07VAR = 240;
                            break;
                        case 44:
                            P11VAR = 240;
                            break;
                        case 45:
                            P14VAR = 240;
                            break;
                        case 46:
                            P15VAR = 240;
                            break;
                        case 47:
                            P16VAR = 240;
                            break;
                        case 48:
                            P17VAR = 240;
                            break;
                        case 49:
                            P20VAR = 240;
                            break;
                        case 50:
                            P21VAR = 240;
                            break;
                        case 51:
                            P22VAR = 240;
                            break;
                        case 52:
                            P23VAR = 240;
                            break;
                        case 53:
                            P24VAR = 240;
                            break;
                        case 54:
                            P25VAR = 240;
                            break;
                        case 55:
                            P26VAR = 240;
                            break;
                        case 56:
                            P32VAR = 240;
                            break;
                        case 57:
                            P33VAR = 240;
                            break;
                        case 58:
                            P34VAR = 240;
                            break;
                        case 59:
                            P35VAR = 240;
                            break;
                        case 60:
                            P36VAR = 240;
                            break;
                        case 61:
                            P37VAR = 240;
                            break;
                        case 62:
                            P40VAR = 240;
                            break;
                        case 63:
                            P41VAR = 240;
                            break;
                        case 64:
                            P42VAR = 240;
                            break;
                        case 65:
                            P43VAR = 240;
                            break;
                        case 66:
                            P46VAR = 240;
                            break;
#ifndef LEDRay
                        case 67:
                            P50VAR = 240;
                            break;
                        case 68:
                            P51VAR = 240;
                            break;
                        case 69:
                            P52VAR = 240;
                            break;
                        case 70:
                            P53VAR = 240;
                            break;
                        case 71:
                            P54VAR = 240;
                            break;
#endif
                        }
                        break;
                    case 4:
                    case 5:
                    case 6:
                        P46VAR = 255;
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
                            P00VAR = 255;
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
                            P01VAR = P00VAR = 255;
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
                            P02VAR = P00VAR = 255;
                            break;
                        case 39:
                            if(i04)
                            {
                                i04 = 1;
                            }
                            P03VAR = P00VAR = 255;
                            break;
                        case 40:
                            P04VAR = P00VAR = 255;
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
                            P05VAR = 255;
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
                            P06VAR = P05VAR = 255;
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
                            P07VAR = P05VAR = 255;
                            break;
                        case 44:
                            if(i14)
                            {
                                i14 = 1;
                            }
                            P11VAR = P05VAR = 255;
                            break;
                        case 45:
                            P14VAR = P05VAR = 255;
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
                            P15VAR = 255;
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
                            P16VAR = P15VAR = 255;
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
                            P17VAR = P15VAR = 255;
                            break;
                        case 49:
                            if(i21)
                            {
                                i21 = 1;
                            }
                            P20VAR = P15VAR = 255;
                            break;
                        case 50:
                            P21VAR = P15VAR = 255;
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
                            P22VAR = 255;
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
                            P23VAR = P22VAR = 255;
                            break;
                        case 53:
                            if(i25)
                            {
                                i25 = 1;
                            }
                            P24VAR = P22VAR = 255;
                            break;
                        case 54:
                            P25VAR = P22VAR = 255;
                            break;
                        case 55:
                            if(i32)
                            {
                                i32 = 1;
                            }
                            if(i33)
                            {
                                i33 = 1;
                            }
                            if(i34)
                            {
                                i34 = 1;
                            }
                            if(i35)
                            {
                                i35 = 1;
                            }
                            P26VAR = 255;
                            break;
                        case 56:
                            if(i33)
                            {
                                i33 = 1;
                            }
                            if(i34)
                            {
                                i34 = 1;
                            }
                            if(i35)
                            {
                                i35 = 1;
                            }
                            P32VAR = P26VAR = 255;
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
                            P33VAR = P26VAR = 255;
                            break;
                        case 58:
                            if(i35)
                            {
                                i35 = 1;
                            }
                            P34VAR = P26VAR = 255;
                            break;
                        case 59:
                            P35VAR = P26VAR = 255;
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
                            P36VAR = 255;
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
                            P37VAR = P36VAR = 255;
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
                            P40VAR = P36VAR = 255;
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
                            P41VAR = P36VAR = 255;
                            break;
                        case 64:
                            if(i43)
                            {
                                i43 = 1;
                            }
                            P42VAR = P36VAR = 255;
                            break;
                        case 65:
                            P43VAR = P36VAR = 255;
                            break;
                        }
                        break;
                    case 7:
                        P42VAR = 255;
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
                            P00VAR = 255;
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
                            P01VAR = P00VAR = 255;
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
                            P02VAR = P00VAR = 255;
                            break;
                        case 39:
                            if(i04)
                            {
                                i04 = 1;
                            }
                            P03VAR = P00VAR = 255;
                            break;
                        case 40:
                            P04VAR = P00VAR = 255;
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
                            P05VAR = 255;
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
                            P06VAR = P05VAR = 255;
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
                            P07VAR = P05VAR = 255;
                            break;
                        case 44:
                            if(i14)
                            {
                                i14 = 1;
                            }
                            P11VAR = P05VAR = 255;
                            break;
                        case 45:
                            P14VAR = P05VAR = 255;
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
                            P15VAR = 255;
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
                            P16VAR = P15VAR = 255;
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
                            P17VAR = P15VAR = 255;
                            break;
                        case 49:
                            if(i21)
                            {
                                i21 = 1;
                            }
                            P20VAR = P15VAR = 255;
                            break;
                        case 50:
                            P21VAR = P15VAR = 255;
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
                            if(i33)
                            {
                                i33 = 1;
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
                            P22VAR = 255;
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
                            if(i33)
                            {
                                i33 = 1;
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
                            P23VAR = P22VAR = 255;
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
                            if(i33)
                            {
                                i33 = 1;
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
                            P24VAR = P22VAR = 255;
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
                            if(i33)
                            {
                                i33 = 1;
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
                            P25VAR = P22VAR = 255;
                            break;
                        case 55:
                            if(i32)
                            {
                                i32 = 1;
                            }
                            if(i33)
                            {
                                i33 = 1;
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
                            P26VAR = P22VAR = 255;
                            break;
                        case 56:
                            if(i33)
                            {
                                i33 = 1;
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
                            P32VAR = P22VAR = 255;
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
                            P33VAR = P22VAR = 255;
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
                            P34VAR = P22VAR = 255;
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
                            P35VAR = P22VAR = 255;
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
                            P36VAR = P22VAR = 255;
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
                            P37VAR = P22VAR = 255;
                            break;
                        case 62:
                            if(i41)
                            {
                                i41 = 1;
                            }
                            P40VAR = P22VAR = 255;
                            break;
                        case 63:
                            P41VAR = P22VAR = 255;
                            break;
                        }
                        break;
                    case 8:
                    case 9:
                    case 10:
                        P36VAR = 255;
                        switch(note)
                        {
                        case 36:
#ifndef ukulelechord
                            P00VAR = 255;
#else
                            P17VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 37:
#ifndef ukulelechord
                            P01VAR = P00VAR = 255;
#else
                            P25VAR = P01VAR = P23VAR = P20VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 38:
#ifndef ukulelechord
                            P02VAR = P00VAR = 255;
#else
                            P26VAR = P02VAR = P06VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 39:
#ifndef ukulelechord
                            P03VAR = P00VAR = 255;
#else
                            P03VAR = P07VAR = P15VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 40:
#ifndef ukulelechord
                            if(!P00VAR)
                            {
                                P22VAR = 255;
                            }
                            else
                            {
                                P04VAR = P00VAR = 255;
                            }
#else
                            P25VAR = P04VAR = P16VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 41:
#ifndef ukulelechord
                            if(!P00VAR)
                            {
                                P23VAR = P22VAR = 255;
                            }
                            else
                            {
                                P05VAR = P00VAR = 255;
                            }
#else
                            P26VAR = P23VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 42:
#ifndef ukulelechord
                            P06VAR = P22VAR = 255;
#else
                            P32VAR = P01VAR = P06VAR = P20VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 43:
#ifndef ukulelechord
                            if(!P22VAR)
                            {
                                P24VAR = 255;
                            }
                            else
                            {
                                P07VAR = P22VAR = 255;
                            }
#else
                            P02VAR = P07VAR = P16VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 44:
#ifndef ukulelechord
                            if(!P22VAR)
                            {
                                P25VAR = P24VAR = 255;
                            }
                            else
                            {
                                P11VAR = P22VAR = 255;
                            }
#else
                            P25VAR = P03VAR = P11VAR = P17VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 45:
#ifndef ukulelechord
                            if(!P22VAR)
                            {
                                if(!P24VAR)
                                {
                                    P35VAR = 255;
                                }
                                else
                                {
                                    P26VAR = P24VAR = 255;
                                }
                            }
                            else
                            {
                                P14VAR = P22VAR = 255;
                            }
#else
                            P01VAR = P26VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 46:
#ifndef ukulelechord
                            if(!P24VAR)
                            {
                                P15VAR = P35VAR = 255;
                            }
                            else
                            {
                                P32VAR = P24VAR = 255;
                            }
#else
                            P32VAR = P02VAR = P23VAR = P15VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 47:
#ifndef ukulelechord
                            if(!P24VAR)
                            {
                                P16VAR = P35VAR = 255;
                            }
                            else
                            {
                                P33VAR = P24VAR = 255;
                            }
#else
                            P33VAR = P03VAR = P06VAR = P16VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 48:
#ifndef ukulelechord
                            if(!P24VAR)
                            {
                                P17VAR = P35VAR = 255;
                            }
                            else
                            {
                                P34VAR = P24VAR = 255;
                            }
#else
                            P17VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 49:
#ifndef ukulelechord
                            P20VAR = P35VAR = 255;
#else
                            P25VAR = P01VAR = P23VAR = P20VAR = 255;
                            i37 = e07;
#endif
                            break;
                        case 50:
#ifndef ukulelechord
                            P21VAR = P20VAR = 255;
#else
                            P26VAR = P02VAR = P06VAR = 255;
                            i37 = e07;
#endif
                            break;
                        }
                        break;
                    case 11:
                        switch(note)
                        {
                        case 36:
                            P00VAR = 255;
                            break;
                        case 37:
                            P01VAR = 255;
                            break;
                        case 38:
                            P02VAR = 208;
                            break;
                        case 39:
                            P03VAR = 208;
                            break;
                        case 40:
                            P04VAR = 208;
                            break;
                        case 41:
                            P05VAR = 208;
                            break;
                        case 42:
                            P06VAR = 208;
                            break;
                        case 43:
                            P07VAR = 208;
                            break;
                        case 44:
                            P11VAR = 208;
                            break;
                        case 45:
                            P14VAR = 208;
                            break;
                        case 46:
                            P15VAR = 208;
#ifndef LEDRay
                            P56VAR = 255;
#endif
                            break;
                        case 47:
                            P16VAR = 208;
#ifndef LEDRay
                            P56VAR = 255;
#endif
                            break;
                        case 48:
                            P17VAR = 208;
#ifndef LEDRay
                            P57VAR = 255;
#endif
                            break;
                        case 49:
                            P20VAR = 208;
#ifndef LEDRay
                            P57VAR = 255;
#endif
                            break;
                        case 50:
                            P21VAR = 240;
                            break;
                        case 51:
                            P22VAR = 240;
                            break;
                        case 52:
                            P23VAR = 240;
                            break;
                        case 53:
                            P24VAR = 240;
                            break;
                        case 54:
                            P25VAR = 240;
                            break;
                        case 55:
                            P26VAR = 240;
                            break;
                        case 56:
                            P32VAR = 240;
                            break;
                        case 57:
                            P33VAR = 240;
                            break;
                        case 58:
                            P34VAR = 240;
                            break;
                        case 59:
                            P35VAR = 240;
                            break;
                        case 60:
                            P36VAR = 240;
                            break;
                        case 61:
                            P37VAR = 255;
                            break;
                        case 62:
                            P40VAR = 255;
                            break;
                        case 63:
                            P41VAR = 255;
                            break;
                        case 64:
                            P42VAR = 255;
                            break;
                        case 65:
                            P43VAR = 240;
                            break;
                        case 66:
                            P46VAR = 240;
                            break;
#ifndef LEDRay
                        case 67:
                            P50VAR = 240;
                            break;
                        case 68:
                            P51VAR = 240;
                            break;
                        case 69:
                            P52VAR = 240;
                            break;
                        case 70:
                            P53VAR = 255;
                            break;
                        case 71:
                            P54VAR = 255;
                            break;
                        case 72:
                            P55VAR = 255;
                            SFRPI = 1;
                            P62 = 1;
                            SFRPI = 0;
                            break;
                        case 73:
                            P55VAR = 255;
                            SFRPI = 1;
                            P62 = 0;
                            SFRPI = 0;
                            break;
#endif
                        }
                        break;
#ifdef HARDRAYPWM
                        //CCAP0H=0x10;  //設定(P12/CEX0)脈波時間，平均電壓為4.6V
                        //CCAP1H=0x20;  //設定(P13/CEX1)脈波時間，平均電壓為4.4V
                        //CCAP2H=0x40;  //設定(P14/CEX2)脈波時間，平均電壓為3.8V
                        //CCAP3H=0x80;  //設定(P15/CEX3)脈波時間，平均電壓為2.6V
                        //CCAP4H=0xA0;  //設定(P16/CEX4)脈波時間，平均電壓為1.8V
                        //CCAP5H=0xFF;  //設定(P17/CEX5)脈波時間，平均電壓為0.01V
                        //記得統一加上 inverse ~
#endif
                    }
                }
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
    if(CL > P11VAR)
        P11 = 0;
    if(CL > P14VAR)
        P14 = 0;
    if(CL > P15VAR)
        P15 = 0;
    if(CL > P16VAR)
        P16 = 0;
    if(CL > P17VAR)
        P17 = 0;
    if(CL > P20VAR)
        P20 = 0;
    if(CL > P21VAR)
        P21 = 0;
    if(CL > P32VAR)
        P32 = 0;
    if(CL > P33VAR)
        P33 = 0;
    if(CL > P34VAR)
        P34 = 0;
    if(CL > P35VAR)
        P35 = 0;
    if(CL > P36VAR)
        P36 = 0;
    if(CL > P37VAR)
        P37 = 0;
    if(CL > P40VAR)
        P40 = 0;
    if(CL > P41VAR)
        P41 = 0;
    if(CL > P42VAR)
        P42 = 0;
    if(CL > P43VAR)
        P43 = 0;
    if(CL > P46VAR)
        P46 = 0;
#ifndef LEDRay
    if(CL > P50VAR)
        P50 = 0;
    if(CL > P51VAR)
        P51 = 0;
    if(CL > P52VAR)
        P52 = 0;
    if(CL > P53VAR)
        P53 = 0;
    if(CL > P54VAR)
        P54 = 0;
    if(CL > P55VAR)
        P55 = 0;
    if(CL > P56VAR)
        P56 = 0;
    if(CL > P57VAR)
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
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11VAR = 0;
            i11--;
            break;
        default:
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14VAR = 0;
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
            P15VAR = 0;
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
            P16VAR = 0;
            i16--;
            break;
        default:
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17VAR = 0;
            i17--;
            break;
        default:
            i17--;
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
            CCAP0H = 255;
            P22VAR = 0;
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
            CCAP1H = 255;
            P23VAR = 0;
            i23--;
            break;
        default:
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            CCAP2H = 255;
            P24VAR = 0;
            i24--;
            break;
        default:
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            CCAP3H = 255;
            P25VAR = 0;
            i25--;
            break;
        default:
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            CCAP4H = 255;
            P26VAR = 0;
            i26--;
            break;
        default:
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i33)
        {
        case 0:
            break;
        case 1:
            P33VAR = 0;
            i33--;
            break;
        default:
            i33--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34VAR = 0;
            i34--;
            break;
        default:
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35VAR = 0;
            i35--;
            break;
        default:
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37VAR = 0;
            i37--;
            break;
        default:
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40VAR = 0;
            i40--;
            break;
        default:
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41VAR = 0;
            i41--;
            break;
        default:
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42VAR = 0;
            i42--;
            break;
        default:
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43VAR = 0;
            i43--;
            break;
        default:
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46VAR = 0;
            i46--;
            break;
        default:
            i46--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50VAR = 0;
            i50--;
            break;
        default:
            i50--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            P51VAR = 0;
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
            P52VAR = 0;
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
            P53VAR = 0;
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
            P54VAR = 0;
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
            P55VAR = 0;
            i55--;
            break;
        default:
            i55--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            P56VAR = 0;
            i56--;
            break;
        default:
            i56--;
            break;
        }
        switch(i57)
        {
        case 0:
            break;
        case 1:
            P57VAR = 0;
            i57--;
            break;
        default:
            i57--;
            break;
        }
#endif
        break;
    case 2:
    case 3:
    case 12:
    case 13:
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
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11VAR = 0;
            i11--;
            break;
        default:
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14VAR = 0;
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
            P15VAR = 0;
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
            P16VAR = 0;
            i16--;
            break;
        default:
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17VAR = 0;
            i17--;
            break;
        default:
            i17--;
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
            CCAP0H = 255;
            P22VAR = 0;
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
            CCAP1H = 255;
            P23VAR = 0;
            i23--;
            break;
        default:
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            CCAP2H = 255;
            P24VAR = 0;
            i24--;
            break;
        default:
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            CCAP3H = 255;
            P25VAR = 0;
            i25--;
            break;
        default:
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            CCAP4H = 255;
            P26VAR = 0;
            i26--;
            break;
        default:
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i33)
        {
        case 0:
            break;
        case 1:
            P33VAR = 0;
            i33--;
            break;
        default:
            i33--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34VAR = 0;
            i34--;
            break;
        default:
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35VAR = 0;
            i35--;
            break;
        default:
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37VAR = 0;
            i37--;
            break;
        default:
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40VAR = 0;
            i40--;
            break;
        default:
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41VAR = 0;
            i41--;
            break;
        default:
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42VAR = 0;
            i42--;
            break;
        default:
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43VAR = 0;
            i43--;
            break;
        default:
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46VAR = 0;
            i46--;
            break;
        default:
            i46--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50VAR = 0;
            i50--;
            break;
        default:
            i50--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            P51VAR = 0;
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
            P52VAR = 0;
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
            P53VAR = 0;
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
            P54VAR = 0;
            i54--;
            break;
        default:
            i54--;
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
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11VAR = 0;
            i11--;
            break;
        default:
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14VAR = 0;
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
            P15VAR = 0;
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
            P16VAR = 0;
            i16--;
            break;
        default:
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17VAR = 0;
            i17--;
            break;
        default:
            i17--;
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
            CCAP0H = 255;
            P22VAR = 0;
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
            CCAP1H = 255;
            P23VAR = 0;
            i23--;
            break;
        default:
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            CCAP2H = 255;
            P24VAR = 0;
            i24--;
            break;
        default:
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            CCAP3H = 255;
            P25VAR = 0;
            i25--;
            break;
        default:
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            CCAP4H = 255;
            P26VAR = 0;
            i26--;
            break;
        default:
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i33)
        {
        case 0:
            break;
        case 1:
            P33VAR = 0;
            i33--;
            break;
        default:
            i33--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34VAR = 0;
            i34--;
            break;
        default:
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35VAR = 0;
            i35--;
            break;
        default:
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37VAR = 0;
            i37--;
            break;
        default:
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40VAR = 0;
            i40--;
            break;
        default:
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41VAR = 0;
            i41--;
            break;
        default:
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42VAR = 0;
            i42--;
            break;
        default:
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43VAR = 0;
            i43--;
            break;
        default:
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46VAR = 0;
            i46--;
            break;
        default:
            i46--;
            break;
        }
        break;
    case 7:
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
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11VAR = 0;
            i11--;
            break;
        default:
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14VAR = 0;
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
            P15VAR = 0;
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
            P16VAR = 0;
            i16--;
            break;
        default:
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17VAR = 0;
            i17--;
            break;
        default:
            i17--;
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
            CCAP0H = 255;
            P22VAR = 0;
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
            CCAP1H = 255;
            P23VAR = 0;
            i23--;
            break;
        default:
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            CCAP2H = 255;
            P24VAR = 0;
            i24--;
            break;
        default:
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            CCAP3H = 255;
            P25VAR = 0;
            i25--;
            break;
        default:
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            CCAP4H = 255;
            P26VAR = 0;
            i26--;
            break;
        default:
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i33)
        {
        case 0:
            break;
        case 1:
            P33VAR = 0;
            i33--;
            break;
        default:
            i33--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34VAR = 0;
            i34--;
            break;
        default:
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35VAR = 0;
            i35--;
            break;
        default:
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37VAR = 0;
            i37--;
            break;
        default:
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40VAR = 0;
            i40--;
            break;
        default:
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41VAR = 0;
            i41--;
            break;
        default:
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42VAR = 0;
            i42--;
            break;
        default:
            i42--;
            break;
        }
        break;
    case 8:
    case 9:
    case 10:
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
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11VAR = 0;
            i11--;
            break;
        default:
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14VAR = 0;
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
            P15VAR = 0;
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
            P16VAR = 0;
            i16--;
            break;
        default:
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17VAR = 0;
            i17--;
            break;
        default:
            i17--;
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
            CCAP0H = 255;
            P22VAR = 0;
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
            CCAP1H = 255;
            P23VAR = 0;
            i23--;
            break;
        default:
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            CCAP2H = 255;
            P24VAR = 0;
            i24--;
            break;
        default:
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            CCAP3H = 255;
            P25VAR = 0;
            i25--;
            break;
        default:
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            CCAP4H = 255;
            P26VAR = 0;
            i26--;
            break;
        default:
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i33)
        {
        case 0:
            break;
        case 1:
            P33VAR = 0;
            i33--;
            break;
        default:
            i33--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34VAR = 0;
            i34--;
            break;
        default:
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35VAR = 0;
            i35--;
            break;
        default:
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
#ifdef ukulelechord
        switch(i37)
        {
        case 0:
            break;
        case (e07-6):
            P35VAR = 255;
            i37--;
            break;
        case (e07-4):
            P22VAR = 255;
            i37--;
            break;
        case (e07-2):
            P00VAR = 255;
            i37--;
            break;
        case e07:
            P24VAR = 255;
            i37--;
            break;
        default:
            i37--;
            break;
        }
#endif
        break;
    case 11:
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
        switch(i11)
        {
        case 0:
            break;
        case 1:
            P11VAR = 0;
            i11--;
            break;
        default:
            i11--;
            break;
        }
        switch(i14)
        {
        case 0:
            break;
        case 1:
            P14VAR = 0;
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
            P15VAR = 0;
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
            P16VAR = 0;
            i16--;
            break;
        default:
            i16--;
            break;
        }
        switch(i17)
        {
        case 0:
            break;
        case 1:
            P17VAR = 0;
            i17--;
            break;
        default:
            i17--;
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
            CCAP0H = 255;
            P22VAR = 0;
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
            CCAP1H = 255;
            P23VAR = 0;
            i23--;
            break;
        default:
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            CCAP2H = 255;
            P24VAR = 0;
            i24--;
            break;
        default:
            i24--;
            break;
        }
        switch(i25)
        {
        case 0:
            break;
        case 1:
            CCAP3H = 255;
            P25VAR = 0;
            i25--;
            break;
        default:
            i25--;
            break;
        }
        switch(i26)
        {
        case 0:
            break;
        case 1:
            CCAP4H = 255;
            P26VAR = 0;
            i26--;
            break;
        default:
            i26--;
            break;
        }
        switch(i32)
        {
        case 0:
            break;
        case 1:
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i33)
        {
        case 0:
            break;
        case 1:
            P33VAR = 0;
            i33--;
            break;
        default:
            i33--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            P34VAR = 0;
            i34--;
            break;
        default:
            i34--;
            break;
        }
        switch(i35)
        {
        case 0:
            break;
        case 1:
            P35VAR = 0;
            i35--;
            break;
        default:
            i35--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
        switch(i37)
        {
        case 0:
            break;
        case 1:
            P37VAR = 0;
            i37--;
            break;
        default:
            i37--;
            break;
        }
        switch(i40)
        {
        case 0:
            break;
        case 1:
            P40VAR = 0;
            i40--;
            break;
        default:
            i40--;
            break;
        }
        switch(i41)
        {
        case 0:
            break;
        case 1:
            P41VAR = 0;
            i41--;
            break;
        default:
            i41--;
            break;
        }
        switch(i42)
        {
        case 0:
            break;
        case 1:
            P42VAR = 0;
            i42--;
            break;
        default:
            i42--;
            break;
        }
        switch(i43)
        {
        case 0:
            break;
        case 1:
            P43VAR = 0;
            i43--;
            break;
        default:
            i43--;
            break;
        }
        switch(i46)
        {
        case 0:
            break;
        case 1:
            P46VAR = 0;
            i46--;
            break;
        default:
            i46--;
            break;
        }
#ifndef LEDRay
        switch(i50)
        {
        case 0:
            break;
        case 1:
            P50VAR = 0;
            i50--;
            break;
        default:
            i50--;
            break;
        }
        switch(i51)
        {
        case 0:
            break;
        case 1:
            P51VAR = 0;
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
            P52VAR = 0;
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
            P53VAR = 0;
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
            P54VAR = 0;
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
            P55VAR = 0;
            i55--;
            break;
        default:
            i55--;
            break;
        }
        switch(i56)
        {
        case 0:
            break;
        case 1:
            P56VAR = 0;
            i56--;
            break;
        default:
            i56--;
            break;
        }
        switch(i57)
        {
        case 0:
            break;
        case 1:
            P57VAR = 0;
            i57--;
            break;
        default:
            if(i57 < (e06-50) )
            {
                SFRPI = 1;
                if(!P63)
                    i57 = 2;
                SFRPI = 0;
            }
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
            consumeToken(SBUF);
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
    P2M0=0x7F;
    P3M1=0xFC;
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
    switch( oneCHANNEL )
    {
    case 0:
    case 1:
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
            i02 = 3;
        }
        if(P03VAR && !i03)
        {
            i03 = 3;
        }
        if(P04VAR && !i04)
        {
            i04 = 3;
        }
        if(P05VAR && !i05)
        {
            i05 = 3;
        }
        if(P06VAR && !i06)
        {
            i06 = 3;
        }
        if(P07VAR && !i07)
        {
            i07 = 3;
        }
        if(P11VAR && !i11)
        {
            i11 = 3;
        }
        if(P14VAR && !i14)
        {
            i14 = 3;
        }
        if(P15VAR && !i15)
        {
            i15 = 3;
        }
        if(P16VAR && !i16)
        {
            i16 = 3;
        }
        if(P17VAR && !i17)
        {
            i17 = 3;
        }
        if(P20VAR && !i20)
        {
            i20 = 3;
        }
        if(P21VAR && !i21)
        {
            i21 = 3;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = 3;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = 3;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = 3;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = 3;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = 3;
        }
        if(P32VAR && !i32)
        {
            i32 = 3;
        }
        if(P33VAR && !i33)
        {
            i33 = 3;
        }
        if(P34VAR && !i34)
        {
            i34 = 3;
        }
        if(P35VAR && !i35)
        {
            i35 = 3;
        }
        if(P36VAR && !i36)
        {
            i36 = 3;
        }
        if(P37VAR && !i37)
        {
            i37 = 3;
        }
        if(P40VAR && !i40)
        {
            i40 = 3;
        }
        if(P41VAR && !i41)
        {
            i41 = 3;
        }
        if(P42VAR && !i42)
        {
            i42 = 3;
        }
        if(P43VAR && !i43)
        {
            i43 = 3;
        }
        if(P46VAR && !i46)
        {
            i46 = 3;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = 3;
        }
        if(P51VAR && !i51)
        {
            i51 = 3;
        }
        if(P52VAR && !i52)
        {
            i52 = 3;
        }
        if(P53VAR && !i53)
        {
            i53 = 3;
        }
        if(P54VAR && !i54)
        {
            i54 = 3;
        }
        if(P55VAR && !i55)
        {
            i55 = 3;
        }
        if(P56VAR && !i56)
        {
            i56 = 3;
        }
        if(P57VAR && !i57)
        {
            i57 = e05;
        }
        P5 |= 0xFF;
#endif
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0x3C;
        P4 |= 0x4F;
        break;
    case 2:
    case 3:
    case 12:
    case 13:
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
            i02 = 3;
        }
        if(P03VAR && !i03)
        {
            i03 = 3;
        }
        if(P04VAR && !i04)
        {
            i04 = 3;
        }
        if(P05VAR && !i05)
        {
            i05 = 3;
        }
        if(P06VAR && !i06)
        {
            i06 = 3;
        }
        if(P07VAR && !i07)
        {
            i07 = 3;
        }
        if(P11VAR && !i11)
        {
            i11 = 3;
        }
        if(P14VAR && !i14)
        {
            i14 = 3;
        }
        if(P15VAR && !i15)
        {
            i15 = 3;
        }
        if(P16VAR && !i16)
        {
            i16 = 3;
        }
        if(P17VAR && !i17)
        {
            i17 = 3;
        }
        if(P20VAR && !i20)
        {
            i20 = 3;
        }
        if(P21VAR && !i21)
        {
            i21 = 3;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = 3;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = 3;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = 3;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = 3;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = 3;
        }
        if(P32VAR && !i32)
        {
            i32 = 3;
        }
        if(P33VAR && !i33)
        {
            i33 = 3;
        }
        if(P34VAR && !i34)
        {
            i34 = 3;
        }
        if(P35VAR && !i35)
        {
            i35 = 3;
        }
        if(P36VAR && !i36)
        {
            i36 = 3;
        }
        if(P37VAR && !i37)
        {
            i37 = 3;
        }
        if(P40VAR && !i40)
        {
            i40 = 3;
        }
        if(P41VAR && !i41)
        {
            i41 = 3;
        }
        if(P42VAR && !i42)
        {
            i42 = 3;
        }
        if(P43VAR && !i43)
        {
            i43 = 3;
        }
        if(P46VAR && !i46)
        {
            i46 = 3;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = 3;
        }
        if(P51VAR && !i51)
        {
            i51 = 3;
        }
        if(P52VAR && !i52)
        {
            i52 = 3;
        }
        if(P53VAR && !i53)
        {
            i53 = 3;
        }
        if(P54VAR && !i54)
        {
            i54 = 3;
        }
        P5 |= 0x1F;
#endif
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0x3C;
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
        if(P33VAR && !i33)
        {
            i33 = e05;
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
        P3 |= 0x3C;
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
        if(P33VAR && !i33)
        {
            i33 = e05;
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
        P3 |= 0x3C;
        P4 |= 0x07;
        break;
    case 8:
    case 9:
    case 10:
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
        if(P33VAR && !i33)
        {
            i33 = e05;
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
        P3 |= 0x7C;
        break;
    case 11:
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
            i02 = 3;
        }
        if(P03VAR && !i03)
        {
            i03 = 3;
        }
        if(P04VAR && !i04)
        {
            i04 = 3;
        }
        if(P05VAR && !i05)
        {
            i05 = 3;
        }
        if(P06VAR && !i06)
        {
            i06 = 3;
        }
        if(P07VAR && !i07)
        {
            i07 = 3;
        }
        if(P11VAR && !i11)
        {
            i11 = 3;
        }
        if(P14VAR && !i14)
        {
            i14 = 3;
        }
        if(P15VAR && !i15)
        {
            i15 = 3;
        }
        if(P16VAR && !i16)
        {
            i16 = 3;
        }
        if(P17VAR && !i17)
        {
            i17 = 3;
        }
        if(P20VAR && !i20)
        {
            i20 = 3;
        }
        if(P21VAR && !i21)
        {
            i21 = 3;
        }
        if(P22VAR && !i22)
        {
            CCAP0H = ~P22VAR;
            i22 = 3;
        }
        if(P23VAR && !i23)
        {
            CCAP1H = ~P23VAR;
            i23 = 3;
        }
        if(P24VAR && !i24)
        {
            CCAP2H = ~P24VAR;
            i24 = 3;
        }
        if(P25VAR && !i25)
        {
            CCAP3H = ~P25VAR;
            i25 = 3;
        }
        if(P26VAR && !i26)
        {
            CCAP4H = ~P26VAR;
            i26 = 3;
        }
        if(P32VAR && !i32)
        {
            i32 = 3;
        }
        if(P33VAR && !i33)
        {
            i33 = 3;
        }
        if(P34VAR && !i34)
        {
            i34 = 3;
        }
        if(P35VAR && !i35)
        {
            i35 = 3;
        }
        if(P36VAR && !i36)
        {
            i36 = 3;
        }
        if(P37VAR && !i37)
        {
            i37 = 3;
        }
        if(P40VAR && !i40)
        {
            i40 = 3;
        }
        if(P41VAR && !i41)
        {
            i41 = 3;
        }
        if(P42VAR && !i42)
        {
            i42 = 3;
        }
        if(P43VAR && !i43)
        {
            i43 = 3;
        }
        if(P46VAR && !i46)
        {
            i46 = 3;
        }
#ifndef LEDRay
        if(P50VAR && !i50)
        {
            i50 = 3;
        }
        if(P51VAR && !i51)
        {
            i51 = 3;
        }
        if(P52VAR && !i52)
        {
            i52 = 3;
        }
        if(P53VAR && !i53)
        {
            i53 = 3;
        }
        if(P54VAR && !i54)
        {
            i54 = 3;
        }
        if(P55VAR && !i55)
        {
            i55 = 3;
        }
        if(P56VAR && !i56)
        {
            i56 = 3;
        }
        if(P57VAR && !i57)
        {
            i57 = e06;
        }
        P5 |= 0xFF;
#endif
        P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
        P1 |= 0xF2;
        P20 = P21 = 1;
        P3 |= 0x3C;
        P4 |= 0x4F;
        break;
    }
#endif
}

#ifdef TIMER0
/***************************************/
void T0_int(void) interrupt 1  //Timer0中斷函數
{
#ifdef CHANNEL16
    if(i10000++ == 255)
    {
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
        twoCHANNEL = (rayCHANNEL & 0x0F) << 1;
        oneCHANNEL = (rayCHANNEL >> 4);
        TL0=0;	//TL0=65536 - TT;
        TH0=0;	//Timer0由0開始計時		//TH0=65536 - TT >> 8; //設定計時值
    }
#elif defined(SIMULATION)
    /*
    switch(pressure++)
    {
    case 0:
        P21VAR = 210;
        P5 = 1;
        break;
    case 2340:
        P22VAR = 210;
        P5 = 2;
        break;
    case 4681:
        P23VAR = 210;
        P5 = 4;
        break;
    case 7201:
        P24VAR = 210;
        P5 = 8;
        break;
    case 9362:
        P25VAR = 210;
        P5 = 16;
        break;
    case 11702:
        P26VAR = 210;
        P5 = 32;
        break;
    case 14043:
        P20VAR = 210;
        P5 = 64;
        break;
    case 16383:
        P21VAR = 210;
        P5 = 1;
        break;
    case 18724:
        P22VAR = 210;
        P5 = 2;
        break;
    case 21064:
        P23VAR = 210;
        P5 = 4;
        break;
    case 23405:
        P24VAR = 210;
        P5 = 8;
        break;
    case 25745:
        P25VAR = 210;
        P5 = 16;
        break;
    case 28086:
        P26VAR = 210;
        P5 = 32;
        break;
    case 30426:
        P20VAR = 210;
        P5 = 64;
        break;
    case 32767:
        P21VAR = 210;
        P5 = 1;
        break;
    case 35107:
        P22VAR = 210;
        P5 = 2;
        break;
    case 37448:
        P23VAR = 210;
        P5 = 4;
        break;
    case 39788:
        P24VAR = 210;
        P5 = 8;
        break;
    case 42129:
        P25VAR = 210;
        P5 = 16;
        break;
    case 44469:
        P26VAR = 210;
        P5 = 32;
        break;
    case 46810:
        P20VAR = 210;
        P5 = 64;
        break;
    case 49150:
        P21VAR = 210;
        P5 = 1;
        break;
    case 51491:
        P22VAR = 210;
        P5 = 2;
        break;
    case 53831:
        P23VAR = 210;
        P5 = 4;
        break;
    case 56172:
        P24VAR = 210;
        P5 = 8;
        break;
    case 58512:
        P25VAR = 210;
        P5 = 16;
        break;
    case 60853:
        P26VAR = 210;
        P5 = 32;
        break;
    case 63193:
        P20VAR = 210;
        P5 = 64;
        break;
    }
    */
#endif
}
#endif