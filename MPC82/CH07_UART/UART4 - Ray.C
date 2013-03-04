/**********UART5.C ******UART與LCD傳輸**********
*動作：接收個人電腦的資料，送到LCD顯示
*硬體：SW3-3(TxD1)ON
************************************************/
#include "..\MPC82.H"   //暫存器及組態定義
#include "MUSIC.H"
//#define MUSIC
#define PARSER
//#define LCD
#define BUFFER
#ifdef PARSER
#define OFF 1
#define ON 2
#define WAIT 3
#define CHANNEL 0x00
char channel;
char note;
char velocity;
int action; //1 =note off ; 2=note on ; 3= wait
#endif
#ifdef LCD
char jj=0;
char line;
#endif
#ifdef BUFFER
#define BUFFER_SIZE 960
volatile unsigned int produceCount, consumeCount;
char buffer[BUFFER_SIZE];
#endif
#ifdef MUSIC
unsigned int  code Table[]  //定義音頻陣列資料,0為休止符  
   ={    0   ,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
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

void consumeToken(char incomingByte); 

main()
{   
	#ifdef BUFFER
	produceCount = consumeCount = 0;
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
	UART_init(31250);      //設定串列環境及鮑率   
	#ifdef MUSIC
	CCAPM0 = ECOM+MAT+TOG+ECCF; //MAT=1，PAC計數與CCAP0匹配時，令CCF0=1
                              //ECOM=1，致能比較器功能
					          //TOG=1，(CH:CL)=CCAP0時，令CEX0腳反相
						      //ECCF=1，致能有匹配(CCF0=1)時，產生中斷
	AUXIE = EPCA; //致能PCA中斷
  	CCF0 = 0;		//清除模組0的比較旗標
	#endif
	EA=1;ES=1;            //致能串列中斷   
	while(1)
	#ifdef BUFFER
	{
		while (produceCount - consumeCount == 0)
           ; // buffer is empty
 
        consumeToken( buffer[consumeCount++]);
		if( consumeCount >= BUFFER_SIZE)
        	consumeCount = 0;
	}
	#else
		;   	//自我空轉，表示可做其它工作
	#endif
}

void consumeToken(char incomingByte)
{
		#ifdef PARSER
		if (incomingByte & 0x90 ) // Note on
	    { 
			channel = (incomingByte & 0x0F); 
			#ifdef MUSIC
			//CR = 0;
			#endif
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
	    else if (incomingByte & 0x80 ) // Note off
	    { 
			channel = (incomingByte & 0x0F);
			#ifdef MUSIC
		    //CR = 0;
			#endif
			action = OFF;
	    }
		else if ( 0 == velocity &&	action != WAIT )
		{
		    if (0 == note) // note on, wait for note value
		    { 
		      	note=incomingByte;
				if( CHANNEL == channel
				#ifdef MUSIC
				 && action == ON 
				#endif 
				)
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
			  	if( CHANNEL == channel )
		      	{
					#ifdef MUSIC
					CR = 1;             //啟動PCA計數，開始發音
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
				 //Midi_Send(0x90,note,velocity); 
		       }
		       else //if(action == OFF)
			   { 
			   	  #ifdef MUSIC
			  	  if( CHANNEL == channel )
		      		CR = 0;
				  #endif		
		          //Midi_Send(0x80,note,velocity); 
		       }
		       note=0;
		       velocity=0;
		       action=WAIT;
			}
		}
	    else{ }	
		#else
		
		#endif
}

/*****************************************************/
void SCON_int(void)  interrupt 4  //串列中斷函數  
{  
	RI=0;             //接收完畢，令RI=0      
	if(SBUF < 0xF0)
	{
		#ifdef BUFFER
		while (produceCount - consumeCount + 1 == BUFFER_SIZE)
            ; // buffer is full
 
        buffer[produceCount++] = SBUF;
		if(produceCount >= BUFFER_SIZE)
			produceCount = 0;
		#else
		consumeToken(SBUF);
		#endif	
		#ifdef LCD
		if(0 == line && jj > 15) {line = 1; LCD_Cmd(0xC0);}//在LCD只能輸入4個字
	  	else if(jj>31) {jj=0; line = 0; LCD_Cmd(0x80);}//在LCD只能輸入4個字
		#endif
	}
}
/***********************************************************
*函數名稱: UART_init
*功能描述: UART啟始程式
*輸入參數：bps
************************************************************/
void UART_init(unsigned int bps)  //UART啟始程式
{  
	P0M0=0; P0M1=0xFF; //設定P0為推挽式輸出(M0-1=01)
   SCON = 0x50;     //設定UART串列傳輸為MODE1及致能接收
   TMOD = 0x20;     //設定TIMER1為MODE2

	AUXR2 = T1X12;
	PCON = SMOD;
	TH1=212;	 //211~213

   //TH1 = 256-(57600/bps);  //設計時器決定串列傳輸鮑率
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
    RS=1;RW=0;EN=1;//資料寫入到LCD內
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
    RS=0;RW=0;EN=1; //命令寫入到LCD內
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