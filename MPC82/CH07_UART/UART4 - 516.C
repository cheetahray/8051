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
#endif
#define TIMER0
#define SIMULATION
#define TIMER2
#define PARSER
#define TT  34286  //Timer延時時間=(1/1.8432MHz)*57600=31250uS
#ifdef TIMER2
unsigned char i00,i01,i02,i03,i04,i05,i06,i07,i11,i14,i15,i16,i17,i20,i21,i22,i23,i24,i25,i26,i32,i33,i34,i35,i36,i37,i40,i41,i42,i43,i46,i10000;
#endif
void softPWM();
#ifdef PARSER
#define IGNORE -1
#define OFF 1
#define ON 2
#define WAIT 3
unsigned char rayCHANNEL = 0, oneCHANNEL = 10,twoCHANNEL = 24;//#define rayCHANNEL 0x00
#define e04 3
#define e05	60
unsigned char ohno[4];
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
    ohno[0] = ohno[1] = ohno[2] = ohno[3] = 0;
#endif
#ifdef TIMER2
    i00=i01=i02=i03=i04=i05=i06=i07=i11=i14=i15=i16=i17=i20=i21=i22=i23=i24=i25=i26=i32=i33=i34=i35=i36=i37=i40=i41=i42=i43=i46=i10000=0;
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
        if (0 == note) // note on, wait for note value
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
                if( velocity != 0 && oneCHANNEL == channel )
                {
                    switch( oneCHANNEL )
                    {
                    case 0:
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        break;
                    case 7:
                        break;
                    case 8:
                        break;
                    case 9:
                        break;
                    case 10:
                        switch(note)
                        {
                        case 36:
                            i24 = e04;
                            CCAP2H = ~0xFF;
                            ohno[1] = 1;
                            break;
                        case 37:
                            i01 = e05;
                            P01VAR = 0xFF;
                            i24 = e04;
                            CCAP2H = ~0xFF;
                            ohno[1] = 1;
                            break;
                        case 38:
                            i02 = e05;
                            P02VAR = 0xFF;
                            i24 = e04;
                            CCAP2H = ~0xFF;
                            ohno[1] = 1;
                            break;
                        case 39:
                            i03 = e05;
                            P03VAR = 0xFF;
                            i24 = e04;
                            CCAP2H = ~0xFF;
                            ohno[1] = 1;
                            break;
                        case 40:
                            if(ohno[1])
                            {
                                i22 = e04;
                                CCAP0H = ~0xFF;
                                ohno[2] = 1;
                            }
                            else
                            {
                                i04 = e05;
                                P04VAR = 0xFF;
                                i24 = e04;
                                CCAP2H = ~0xFF;
                                ohno[1] = 1;
                            }
                            break;
                        case 41:
                            if(ohno[1])
                            {
                                i07 = e05;
                                P07VAR = 0xFF;
                                i22 = e04;
                                CCAP0H = ~0xFF;
                                ohno[2] = 1;
                            }
                            else
                            {
                                i05 = e05;
                                P05VAR = 0xFF;
                                i24 = e04;
                                CCAP2H = ~0xFF;
                                ohno[1] = 1;
                            }
                            break;
                        case 42:
                            i06 = e05;
                            P06VAR = 0xFF;
                            i22 = e04;
                            CCAP0H = ~0xFF;
                            ohno[2] = 1;
                            break;
                        case 43:
                            if(ohno[2])
                            {
                                i23 = e04;
                                CCAP1H = ~0xFF;
                                ohno[0] = 1;
                            }
                            else
                            {
                                i00 = e05;
                                P00VAR = 0xFF;
                                i22 = e04;
                                CCAP0H = ~0xFF;
                                ohno[2] = 1;
                            }
                            break;
                        case 44:
                            if(ohno[2])
                            {
                                i11 = e05;
                                P11VAR = 0xFF;
                                i23 = e04;
                                CCAP1H = ~0xFF;
                                ohno[0] = 1;
                            }
                            else
                            {
                                i35 = e05;
                                P35VAR = 0xFF;
                                i22 = e04;
                                CCAP0H = ~0xFF;
                                ohno[2] = 1;
                            }
                            break;
                        case 45:
                            if(ohno[2])
                            {
                                if(ohno[0])
                                {
                                    i25 = e04;
                                    CCAP3H = ~0xFF;
                                    ohno[3] = 1;
                                }
                                else
                                {
                                    i14 = e05;
                                    P14VAR = 0xFF;
                                    i23 = e04;
                                    CCAP1H = ~0xFF;
                                    ohno[0] = 1;
                                }
                            }
                            else
                            {
                                i26 = e05;
                                CCAP4H = ~0xFF;
                                i22 = e04;
                                CCAP0H = ~0xFF;
                                ohno[2] = 1;
                            }
                            break;
                        case 46:
                            if(ohno[3])
                            {
                                i15 = e05;
                                P15VAR = 0xFF;
                                i23 = e04;
                                CCAP1H = ~0xFF;
                                ohno[0] = 1;
                            }
                            else
                            {
                                i32 = e05;
                                P32VAR = 0xFF;
                                i25 = e04;
                                CCAP3H = ~0xFF;
                                ohno[3] = 1;
                            }
                            break;
                        case 47:
                            if(ohno[3])
                            {
                                i16 = e05;
                                P16VAR = 0xFF;
                                i23 = e04;
                                CCAP1H = ~0xFF;
                                ohno[0] = 1;
                            }
                            else
                            {
                                i36 = e05;
                                P36VAR = 0xFF;
                                i25 = e04;
                                CCAP3H = ~0xFF;
                                ohno[3] = 1;
                            }
                            break;
                        case 48:
                            if(ohno[3])
                            {
                                i17 = e05;
                                P17VAR = 0xFF;
                                i23 = e04;
                                CCAP1H = ~0xFF;
                                ohno[0] = 1;
                            }
                            else
                            {
                                i34 = e05;
                                P34VAR = 0xFF;
                            }
                            break;
                        case 49:
                            i20 = e05;
                            P20VAR = 0xFF;
                            i25 = e04;
                            CCAP3H = ~0xFF;
                            ohno[3] = 1;
                            break;
                        case 50:
                            i21 = e05;
                            P21VAR = 0xFF;
                            i25 = e04;
                            CCAP3H = ~0xFF;
                            ohno[3] = 1;
                            break;
                        }
                        break;
                    case 11:
                        switch(note)
                        {
                        case 41:
                            i26 = 3;
                            CCAP4H = ~0xD3;
                            break;
                        case 48:
                            i24 = 4;
                            CCAP2H = ~0xFE;
                            break;
                        case 49:
                            i25 = 4;
                            CCAP3H = ~0xFE;
                            break;
                        case 55:
                            i00 = 4;
                            P00VAR = 0xFE;
                            break;
                        case 56:
                            i01 = 4;
                            P01VAR = 0xFE;
                            break;
                        case 57:
                            i02 = 3;
                            P02VAR = 0xFE;
                            break;
                        case 58:
                            i03 = 3;
                            P03VAR = 0xFE;
                            break;
                        case 59:
                            i04 = 3;
                            P04VAR = 0xFE;
                            break;
                        case 60:
                            i05 = 3;
                            P05VAR = 0xFE;
                            break;
                        case 61:
                            i06 = 3;
                            P06VAR = 0xFE;
                            break;
                        case 62:
                            i07 = 3;
                            P07VAR = 0xFE;
                            break;
                        case 63:
                            i20 = 2;
                            P20VAR = 0xFE;
                            break;
                        case 64:
                            i21 = 2;
                            P21VAR = 0xFE;
                            break;
                        case 70:
                            i22 = 4;
                            CCAP0H = ~0xFE;
                            break;
                        case 71:
                            break;
                        case 72:
                            P32 = 1;
                            i23 = 200;
                            CCAP1H = ~0xFE;
                            break;
                        case 73:
                            P32 = 0;
                            i23 = 200;
                            CCAP1H = ~0xFE;
                            break;
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
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:
        break;
    case 8:
        break;
    case 9:
        break;
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
            ohno[1] = 0;
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
            ohno[1] = 0;
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
            ohno[1] = 0;
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
            ohno[1] = 0;
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
            ohno[1] = 0;
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
            ohno[2] = 0;
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
            ohno[0] = 0;
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
            ohno[0] = 0;
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
            ohno[0] = 0;
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
            ohno[0] = 0;
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
            ohno[0] = 0;
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
            ohno[3] = 0;
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
            ohno[3] = 0;
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
            CCAP0H = ~0x00;
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
            ohno[2] = 0;
            CCAP1H = ~0x00;
            P23VAR = 0;
            //P32 = 0;
            i23--;
            break;
        default:
            //if(0 == P11 && i23 < 160 )
            //i23 = 1;
            //else
            i23--;
            break;
        }
        switch(i24)
        {
        case 0:
            break;
        case 1:
            ohno[2] = 0;
            CCAP2H = ~0x00;
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
            ohno[2] = 0;
            CCAP3H = ~0x00;
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
            ohno[2] = 0;
            CCAP4H = ~0x00;
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
            ohno[3] = 0;
            P32VAR = 0;
            i32--;
            break;
        default:
            i32--;
            break;
        }
        switch(i36)
        {
        case 0:
            break;
        case 1:
            ohno[3] = 0;
            P36VAR = 0;
            i36--;
            break;
        default:
            i36--;
            break;
        }
        switch(i34)
        {
        case 0:
            break;
        case 1:
            ohno[3] = 0;
            P34VAR = 0;
            i34--;
            break;
        case (e05-6):
            i25 = e04;
            CCAP3H = ~0xFF;
            ohno[3] = 1;
            i34--;
            break;
        case (e05-4):
            i22 = e04;
            CCAP0H = ~0xFF;
            i34--;
            break;
        case (e05-2):
            i24 = e04;
            CCAP2H = ~0xFF;
            i34--;
            break;
        case e05:
            i23 = e04;
            CCAP1H = ~0xFF;
            //i25 = e04;
            //CCAP3H = ~0xFF;
            //ohno[3] = 1;
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
        break;
    case 11:
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
    if(P01VAR != 0 && 0 == i01)
    {
        i01 = 3;
    }
    if(P02VAR != 0 && 0 == i02)
    {
        i02 = 3;
    }
    if(P03VAR != 0 && 0 == i03)
    {
        i03 = 3;
    }
    if(P04VAR != 0 && 0 == i04)
    {
        i04 = 3;
    }
    if(P05VAR != 0 && 0 == i05)
    {
        i05 = 3;
    }
    if(P06VAR != 0 && 0 == i06)
    {
        i06 = 3;
    }
    if(P07VAR != 0 && 0 == i07)
    {
        i07 = 3;
    }
    if(P11VAR != 0 && 0 == i11)
    {
        i11 = 3;
    }
    if(P14VAR != 0 && 0 == i14)
    {
        i14 = 3;
    }
    if(P15VAR != 0 && 0 == i15)
    {
        i15 = 3;
    }
    if(P16VAR != 0 && 0 == i16)
    {
        i16 = 3;
    }
    if(P17VAR != 0 && 0 == i17)
    {
        i17 = 3;
    }
    if(P20VAR != 0 && 0 == i20)
    {
        i20 = 3;
    }
    if(P21VAR != 0 && 0 == i21)
    {
        i21 = 3;
    }
    if(P22VAR != 0 && 0 == i22)
    {
        CCAP0H = ~P22VAR;
        i22 = 3;
    }
    if(P23VAR != 0 && 0 == i23)
    {
        CCAP1H = ~P23VAR;
        i23 = 3;
    }
    if(P24VAR != 0 && 0 == i24)
    {
        CCAP2H = ~P24VAR;
        i24 = 3;
    }
    if(P25VAR != 0 && 0 == i25)
    {
        CCAP3H = ~P25VAR;
        i25 = 3;
    }
    if(P26VAR != 0 && 0 == i26)
    {
        CCAP4H = ~P26VAR;
        i26 = 3;
    }
    if(P32VAR != 0 && 0 == i32)
    {
        i32 = 3;
    }
    if(P33VAR != 0 && 0 == i33)
    {
        i33 = 3;
    }
    if(P34VAR != 0 && 0 == i34)
    {
        i34 = 3;
    }
    if(P35VAR != 0 && 0 == i35)
    {
        i35 = 3;
    }
    if(P36VAR != 0 && 0 == i36)
    {
        i36 = 3;
    }
    if(P37VAR != 0 && 0 == i37)
    {
        i37 = 3;
    }
    if(P40VAR != 0 && 0 == i40)
    {
        i40 = 3;
    }
    if(P41VAR != 0 && 0 == i41)
    {
        i41 = 3;
    }
    if(P42VAR != 0 && 0 == i42)
    {
        i42 = 3;
    }
    if(P43VAR != 0 && 0 == i43)
    {
        i43 = 3;
    }
    if(P46VAR != 0 && 0 == i46)
    {
        i46 = 3;
    }
    P0 = 0xFF;//P00 = P01 = P02 = P03 = P04 = P05 = P06 = P07 = 1;
    P1 |= 0xF2;
    P20 = P21 = 1;
    P3 |= 0xFC;
    P4 |= 0x4F;
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