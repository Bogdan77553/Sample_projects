/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
USART_InitTypeDef USART_InitStructure2;
NVIC_InitTypeDef  NVIC_InitStructure;


char uart1_rx_buf[128];
uint8_t uart1_rx_bit=0;

char uart3_rx_buf[1000];
uint16_t uart3_rx_bit=0;

void send_Uart(USART_TypeDef* USARTx, unsigned char c)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)== RESET){}
		USART_SendData(USARTx, c);
}

void send_comand(USART_TypeDef* USARTx, char * ss)
{
 uint8_t i=0;
 while(ss[i]){
   send_Uart(USARTx,ss[i]);
  i++;
 }

}

unsigned char getch_Uart(USART_TypeDef* USARTx)
{
	while(USART_GetFlagStatus(USARTx,USART_FLAG_RXNE) == RESET){}
	return USART_ReceiveData(USARTx);
}



void delay(unsigned long p)
{
	while(p>0){p--;}
}

void init()
{
//------------------------------------------------------------
//	UART1
//------------------------------------------------------------
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
//Configure GPIO pin
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//	Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//	Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	Configure UART1
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);

__enable_irq();
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

//------------------------------------------------------------
//	UART2
//------------------------------------------------------------
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//Configure GPIO pin

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		//	Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//	Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure2.USART_BaudRate = 9600;
  USART_InitStructure2.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure2.USART_StopBits = USART_StopBits_1;
  USART_InitStructure2.USART_Parity = USART_Parity_No;
  USART_InitStructure2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure2.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure2);

  USART_Cmd(USART2, ENABLE);

  //------------------------------------------------------------
//	UART3
//------------------------------------------------------------
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//Configure GPIO pin

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		//	Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	//	Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  USART_InitStructure2.USART_BaudRate = 9600;
  USART_InitStructure2.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure2.USART_StopBits = USART_StopBits_1;
  USART_InitStructure2.USART_Parity = USART_Parity_No;
  USART_InitStructure2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure2.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure2);

  USART_Cmd(USART3, ENABLE);
}

int main(void)
{ //AT+CGDCONT=1,"IP","www.kyivstar.net"

    char test[37]={'A','T','+','C','G','D','C','O','N','T','=','1',',','"','I','P' ,'"',',','"','w','w','w','.','k','y','i','v','s','t','a','r','.','n','e','t','"','\r'  };

    init();

send_comand(USART2,test);
send_comand(USART2,"\r\n");


/*
    while(1) {
if(strcmp(uart1_rx_buf,"MODEM:STARTUP")==0){
send_str2(uart1_rx_buf);
uart1_rx_bit=0;
send_str2("\n");
send_str2("1if\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

}
*/
   while(1) {
if(strcmp(uart1_rx_buf,"+PBREADY")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"2if_+PBREADY\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

}


    send_comand(USART2,"*******************************\r\n");
    send_comand(USART2,test);

 while(1) {
send_comand(USART1,"AT+CREG?\r");
delay(10000000);
if(strcmp(uart1_rx_buf,"+CREG: 0,3OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
}

if(strcmp(uart1_rx_buf,"+CREG: 0,1OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}
}


    while(1) {
send_comand(USART1,"AT+XISP=0\r");
delay(10000000);
if(strcmp(uart1_rx_buf,"OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"AT+XISP=0\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

}

   while(1) {
send_comand(USART1,test);
delay(10000000);
if(strcmp(uart1_rx_buf,"OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"SsulkaOK\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

}
 while(1) {
send_comand(USART1,"at+xiic=1\r");
delay(10000000);
if(strcmp(uart1_rx_buf,"OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"at+xiic=1\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

}
//send_str1("at+tcpsetup=0,89.22.47.180,8056\r");

send_comand(USART1,"at+tcpsetup=0,89.22.47.180,8056\r");
while(1) {
delay(10000000);
if(strcmp(uart1_rx_buf,"OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"at+tcpsetup=0,89.22.47.180,8056\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
}
if(strcmp(uart1_rx_buf,"OK+TCPSETUP:0,OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"+TCPSETUP:0,OK\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

if(strcmp(uart1_rx_buf,"+TCPSETUP:0,OK")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"+TCPSETUP:0,OK\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}

if(strcmp(uart1_rx_buf,"+TCPSETUP:0,FAIL")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"+TCPSETUP:0,FAIL\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}
if(strcmp(uart1_rx_buf,"+TCPSETUP:Error 1")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"+TCPSETUP:Error 1\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}
}



 while(1) {
         send_comand(USART1,"AT+TCPSEND=0,253\r");
delay(10000000);
if(strcmp(uart1_rx_buf,">")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"1if\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));

 while(1) {
         //send_comand(USART1,"0123456789\r");
         send_comand(USART2,"11111111111111111111111\r\n");

while(1){
        if(uart3_rx_bit>250){break;}
        if (USART3->SR & USART_SR_RXNE){
uart3_rx_buf[uart3_rx_bit]=USART3->DR;
uart3_rx_bit++;}
//send_comand(USART2,"111\r\n");
}
send_comand(USART1, uart3_rx_buf);
send_Uart(USART1," ");
send_Uart(USART1,"*");
send_Uart(USART1,"\r");
//send_comand(USART1,"\r");

delay(10000000);
if(strcmp(uart1_rx_buf,"OK+TCPSEND:0,253")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"11111if\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}
if(strcmp(uart1_rx_buf,"OK+TCPSEND:0,253+TCPCLOSE:0,Link Closed")==0){
send_comand(USART2,uart1_rx_buf);
uart1_rx_bit=0;
send_comand(USART2,"12222if\r\n");
memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
break;
}
send_comand(USART2,uart1_rx_buf);
send_comand(USART2,"1112121212121211212if\r\n");
break;
}

break;
}

}



/*
send_comand(USART2,"11111111111111111111111\r\n");

while(1){
        if(uart3_rx_bit>250){break;}
        if (USART3->SR & USART_SR_RXNE){
uart3_rx_buf[uart3_rx_bit]=USART3->DR;
uart3_rx_bit++;}
//send_comand(USART2,"111\r\n");
}
send_comand(USART2, uart3_rx_buf);
*/
send_comand(USART2,"11111111111111111111111\r\n");
}

void USART1_IRQHandler (void){
    	char uart_data;
		if (USART1->SR & USART_SR_RXNE){
            uart_data=USART1->DR;
            if(uart_data!='\r') {
                if(uart_data!='\n'){
                uart1_rx_buf[uart1_rx_bit]=USART1->DR;
                uart1_rx_bit++;
                }
            }

		}
}


