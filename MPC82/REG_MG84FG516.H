// (System)
sfr ACC       = 0xE0;
sfr B         = 0xF0;
sfr SP        = 0x81;
sfr DPL       = 0x82;
sfr DPH       = 0x83;
sfr PSW       = 0xD0;

// (Interrupt)
sfr IE        = 0xA8;
sfr EIE1      = 0xAD;
sfr AUXIE     = 0xAD;
sfr EIE2      = 0xA5;
sfr XICON     = 0xC0;
sfr IP0L      = 0xB8;
sfr IP        = 0xB8;
sfr IP0H      = 0xB7;
sfr IPH       = 0xB7;
sfr EIP1L     = 0xAE;
sfr AUXIP     = 0xAE;
sfr EIP2L     = 0xA6;
sfr EIP1H     = 0xAF;
sfr AUXIPH    = 0xAF;
sfr EIP2H     = 0xA7;

#define EPCA  0x04  //PCA中斷致能

// (I/O Port)
sfr P0        = 0x80;
sfr P1        = 0x90;
sfr P2        = 0xA0;
sfr P3        = 0xB0;
sfr P4        = 0xE8;
sfr P5        = 0xF8;
sfr P6        = 0xF8;                            // Page P
sfr P0M0      = 0x93;
sfr P1M0      = 0x91;
sfr P1AIO     = 0x92;
sfr P2M0      = 0x95;
sfr P3M0      = 0xB1;
sfr P3M1      = 0xB2;
sfr P4M0      = 0xB3;
sfr PUCON0    = 0xB4;
sfr PUCON1    = 0xB4;                            // Page P
sfr P5M0      = 0xB5;
sfr P6M0      = 0xB5;                            // Page P
sfr LED       = 0xF8;
// (Timer)
sfr TCON      = 0x88;
sfr TMOD      = 0x89;
sfr T2CON     = 0xC8;
sfr T3CON     = 0xC8;                            // Page P
sfr T2MOD     = 0xC9;
sfr T3MOD     = 0xC9;                            // Page P
sfr16  RCAP2  = 0xCA; //Timer2重新載入/捕捉暫存器
sfr RCAP2L    = 0xCA;
sfr RCAP3L    = 0xCA;                            // Page P
sfr RCAP2H    = 0xCB;
sfr RCAP3H    = 0xCB;                            // Page P
sfr TL0       = 0x8A;
sfr TL1       = 0x8B;
sfr TH0       = 0x8C;
sfr TH1       = 0x8D;
sfr16 T2R     = 0xCC;   //Timer2計時/計數暫存器
sfr TL2       = 0xCC;
sfr TL3       = 0xCC;                            // Page P
sfr TH2       = 0xCD;
sfr TH3       = 0xCD;                            // Page P

#define T0_M0   0x01 //Timer0模式位元0
#define T0_M1   0x02 //Timer0模式位元1	
#define T1_M1   0x20 //Timer1模式位元1

// (Serial Port)
sfr SCON      = 0x98;
sfr S0CON     = 0x98;
sfr S1CON     = 0x98;                            // Page P
sfr SBUF      = 0x99;
sfr S0BUF     = 0x99;
sfr S1BUF     = 0x99;                            // Page P
sfr SADDR     = 0xA9;
sfr SADEN     = 0xB9;
sfr SCFG      = 0x9A;
sfr S0CFG     = 0x9A;
sfr S1BRT     = 0x9A;                            // Page P

#define URM0X6  0x20  //1=系統頻率/2，0=UART mode0時脈為系統頻率/12

sfr S1CFG     = 0x9B;
sfr S2CFG     = 0x9B;
sfr S1BRC     = 0x9B;                            // Page P


// (ADC)
sfr ADCFG0    = 0xC3;
sfr ADCON0    = 0xC4;
sfr ADCDL     = 0xC5;
sfr ADCDH     = 0xC6;

// (Keypad Interrupt)
sfr KBCON     = 0xD6;
sfr KBPATN    = 0xD5;
sfr KBMASK    = 0xD7;

// (SPI)
sfr SPSTAT    = 0x84;
sfr SPCON     = 0x85;
sfr SPCTL     = 0x85;
sfr SPDAT     = 0x86;
sfr SPISTAT   = 0x84;
sfr SPICON    = 0x85;
sfr SPICTL    = 0x85;
sfr SPIDAT    = 0x86;

#define  ECCF   0x01  //1=致能有匹配或捕捉(CCFn=1)時產生中斷
#define  PWM    0x02  //1=致能由CEXn腳輸出PWM波形
#define  MAT    0x08  //1=設定PAC計數溢位及符合模組動作時，令CCON內的位元CCFn=1
#define  ECOM   0x40  //1=致能比較器功能，0=禁能比較器功能(預定)

// (PCA)
sfr CCON      = 0xD8;
sfr CMOD      = 0xD9;
sfr CL        = 0xE9;
sfr CH        = 0xF9;
sfr CCAPM0    = 0xDA;
sfr CCAPM1    = 0xDB;
sfr CCAPM2    = 0xDC;
sfr CCAPM3    = 0xDD;
sfr CCAPM4    = 0xDE;
sfr CCAPM5    = 0xDF;
sfr CCAP0H    = 0xFA;
sfr CCAP0L    = 0xEA;
sfr CCAP1H    = 0xFB;
sfr CCAP1L    = 0xEB;
sfr CCAP2H    = 0xFC;
sfr CCAP2L    = 0xEC;
sfr CCAP3H    = 0xFD;
sfr CCAP3L    = 0xED;
sfr CCAP4H    = 0xFE;
sfr CCAP4L    = 0xEE;
sfr CCAP5H    = 0xFF;
sfr CCAP5L    = 0xEF;
sfr PCAPWM0   = 0xF2;
sfr PCAPWM1   = 0xF3;
sfr PCAPWM2   = 0xF4;
sfr PCAPWM3   = 0xF5;
sfr PCAPWM4   = 0xF6;
sfr PCAPWM5   = 0xF7;

// (TWSI)
sfr SIADR     = 0xD1;
sfr SIDAT     = 0xD2;
sfr SISTA     = 0xD3;
sfr SICON     = 0xD4;

// (Clock)
sfr CKCON0    = 0xC7;
sfr CKCON1    = 0xBF;

// (USB)
sfr USBDAT    = 0xAA;
sfr USBADR    = 0xAB;

// (Others)
sfr PCON      = 0x87;
sfr PCON0     = 0x87;
sfr PCON1     = 0x97;
sfr SFIE      = 0x8E;
sfr STRETCH   = 0x8F;
sfr AUXR0     = 0xA1;
sfr AUXR1     = 0xA2;
sfr AUXR2     = 0xA3;
sfr SFRPI     = 0xAC;
sfr WDTCR     = 0xE1;

#define SMOD   0x80  //1=UART1鮑率加倍
#define T1X12   0x08  //1=T1時脈為系統頻率，0=T1時脈為系統頻率/12

// (ISP)
sfr IFD       = 0xE2;
sfr IFADRH    = 0xE3;
sfr IFADRL    = 0xE4;
sfr IFMT      = 0xE5;
sfr SCMD      = 0xE6;
sfr ISPCR     = 0xE7;

#define CKCON2  0x40
#define XTAL    0x71
#define PageP   0x04
#define ISPEN   0x80   //1=致能IAP/ISP功能

// (bits in TCON)
sbit TF1      = TCON^7;
sbit TR1      = TCON^6;
sbit TF0      = TCON^5;
sbit TR0      = TCON^4;
sbit IE1      = TCON^3;
sbit IT1      = TCON^2;
sbit IE0      = TCON^1;
sbit IT0      = TCON^0;

//(bits in S0CON)
sbit FE       = S0CON^7;
sbit SM00     = S0CON^7;
sbit SM10     = S0CON^6;
sbit SM20     = S0CON^5;
sbit REN0     = S0CON^4;
sbit TB80     = S0CON^3;
sbit RB80     = S0CON^2;
sbit TI0      = S0CON^1;
sbit RI0      = S0CON^0;

sbit SM0      = SCON^7;
sbit SM1      = SCON^6;
sbit SM2      = SCON^5;
sbit REN      = SCON^4;
sbit TB8      = SCON^3;
sbit RB8      = SCON^2;
sbit TI       = SCON^1;
sbit RI       = SCON^0;

//(bits in S1CON)
sbit SM01     = S1CON^7;
sbit SM11     = S1CON^6;
sbit SM21     = S1CON^5;
sbit REN1     = S1CON^4;
sbit TB81     = S1CON^3;
sbit RB81     = S1CON^2;
sbit TI1      = S1CON^1;
sbit RI1      = S1CON^0;

// (bits in IE)
sbit EA       = IE^7;
sbit GF4      = IE^6;
sbit ET2      = IE^5;
sbit ES0      = IE^4;
sbit ET1      = IE^3;
sbit EX1      = IE^2;
sbit ET0      = IE^1;
sbit EX0      = IE^0;

sbit ES       = IE^4;

// (bits in IP0L)
sbit PX3L     = IP0L^7;
sbit PX2L     = IP0L^6;
sbit PT2L     = IP0L^5;
sbit PSL      = IP0L^4;
sbit PT1L     = IP0L^3;
sbit PX1L     = IP0L^2;
sbit PT0L     = IP0L^1;
sbit PX0L     = IP0L^0;

sbit PT2      = IP^5;
sbit PS       = IP^4;
sbit PT1      = IP^3;
sbit PX1      = IP^2;
sbit PT0      = IP^1;
sbit PX0      = IP^0;

// (bits in XICON)
sbit PX3      = XICON^7;
sbit IT3H     = XICON^7;
sbit EX3      = XICON^6;
sbit IE3      = XICON^5;
sbit IT3      = XICON^4;
sbit PX2      = XICON^3;
sbit IT2H     = XICON^3;
sbit EX2      = XICON^2;
sbit IE2      = XICON^1;
sbit IT2      = XICON^0;

// (bits in T2CON)
sbit TF2      = T2CON^7;
sbit EXF2     = T2CON^6;
sbit RCLK     = T2CON^5;
sbit TCLK     = T2CON^4;
sbit EXEN2    = T2CON^3;
sbit TR2      = T2CON^2;
sbit C_T2     = T2CON^1;
sbit CP_RL2   = T2CON^0;

// (bits in T3CON)
sbit TF3      = T3CON^7;
sbit EXF3     = T3CON^6;
sbit T3M1     = T3CON^5;
sbit T3M0     = T3CON^4;
sbit EXEN3    = T3CON^3;
sbit TR3      = T3CON^2;
sbit C_T3     = T3CON^1;
sbit CP_RL3   = T3CON^0;

// (bits in PSW)
sbit CY       = PSW^7;
sbit AC       = PSW^6;
sbit F0       = PSW^5;
sbit RS1      = PSW^4;
sbit RS0      = PSW^3;
sbit OV       = PSW^2;
sbit F1       = PSW^1;
sbit P        = PSW^0;

// (bits in CCON)
sbit CF       = CCON^7;
sbit CR       = CCON^6;
sbit CCF5     = CCON^5;
sbit CCF4     = CCON^4;
sbit CCF3     = CCON^3;
sbit CCF2     = CCON^2;
sbit CCF1     = CCON^1;
sbit CCF0     = CCON^0;

// (bits in P0)
sbit P07      = P0^7;
sbit P06      = P0^6;
sbit P05      = P0^5;
sbit P04      = P0^4;
sbit P03      = P0^3;
sbit P02      = P0^2;
sbit P01      = P0^1;
sbit P00      = P0^0;

// (bits in P1)
sbit P17      = P1^7;
sbit P16      = P1^6;
sbit P15      = P1^5;
sbit P14      = P1^4;
sbit P13      = P1^3;
sbit P12      = P1^2;
sbit P11      = P1^1;
sbit P10      = P1^0;

// (bits in P2)
sbit P27      = P2^7;
sbit P26      = P2^6;
sbit P25      = P2^5;
sbit P24      = P2^4;
sbit P23      = P2^3;
sbit P22      = P2^2;
sbit P21      = P2^1;
sbit P20      = P2^0;

// (bits in P3)
sbit P37      = P3^7;
sbit P36      = P3^6;
sbit P35      = P3^5;
sbit P34      = P3^4;
sbit P33      = P3^3;
sbit P32      = P3^2;
sbit P31      = P3^1;
sbit P30      = P3^0;

// (bits in P4)
sbit P46      = P4^6;
sbit P45      = P4^5;
sbit P44      = P4^4;
sbit P43      = P4^3;
sbit P42      = P4^2;
sbit P41      = P4^1;
sbit P40      = P4^0;

// (bits in P5)
sbit P57      = P5^7;
sbit P56      = P5^6;
sbit P55      = P5^5;
sbit P54      = P5^4;
sbit P53      = P5^3;
sbit P52      = P5^2;
sbit P51      = P5^1;
sbit P50      = P5^0;

// (bits in P6)
sbit P67      = P6^7;
sbit P66      = P6^6;
sbit P65      = P6^5;
sbit P64      = P6^4;
sbit P63      = P6^3;
sbit P62      = P6^2;
sbit P61      = P6^1;
sbit P60      = P6^0;

//(bits in P0, alternate)
sbit KBI7     = P0^7;
sbit KBI6     = P0^6;
sbit KBI5     = P0^5;
sbit KBI4     = P0^4;
sbit KBI3     = P0^3;
sbit KBI2     = P0^2;
sbit KBI1     = P0^1;
sbit KBI0     = P0^0;

sbit T3       = P0^1;
sbit T3EX     = P0^0;

// (bits in P1, alternate)
sbit TXD1     = P1^3;
sbit RXD1     = P1^2;
sbit T2EX     = P1^1;
sbit T2       = P1^0;

sbit SPI_CLK  = P1^7;
sbit SPI_MISO = P1^6;
sbit SPI_MOSI = P1^5;
sbit SPI_SS   = P1^4;

sbit T2CKO    = P1^0;

sbit AIN7     = P1^7;
sbit AIN6     = P1^6;
sbit AIN5     = P1^5;
sbit AIN4     = P1^4;
sbit AIN3     = P1^3;
sbit AIN2     = P1^2;
sbit AIN1     = P1^1;
sbit AIN0     = P1^0;

//(bits in P2, alternate)
sbit CEX5     = P2^7;
sbit CEX4     = P2^6;
sbit CEX3     = P2^5;
sbit CEX2     = P2^4;
sbit CEX1     = P2^3;
sbit CEX0     = P2^2;
sbit ECI      = P2^1;
sbit SICKO    = P2^0;

// (bits in P3, alternate)
sbit RD       = P3^7;
sbit WR       = P3^6;
sbit T1       = P3^5;
sbit T0       = P3^4;
sbit INT1     = P3^3;
sbit INT0     = P3^2;
sbit TXD      = P3^1;
sbit RXD      = P3^0;
sbit T1CKO    = P3^5;
sbit T0CKO    = P3^4;

// (bits in P4, alternate)
sbit INT2     = P4^3;
sbit INT3     = P4^2;
sbit TWI_SDA  = P4^1;
sbit TWI_SCL  = P4^0;

sbit ALE      = P4^6;

// (bits in P6, alternate)
sbit CKO      = P6^0;
sbit XOUT     = P6^0;
sbit XIN      = P6^1;

/*******************************************
*函數名稱: Delay_ms
*功能描述: 延時以1mS為單位，石英晶體為22.1184MHz
*********************************************/
void Delay_ms(unsigned int dly) 
{  
  unsigned int dly1; //內循環變數
  while(dly--)         // 外循環，遞減至0
   {dly1=950;  while(dly1--);} //內循環，延時1ms
}
//********宣告函數********************************** 
extern void Delay_ms(unsigned int dly);//以mS為單位延時

extern void UART_init(unsigned int BPS);  //UART啟始程式
