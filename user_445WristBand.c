/******************************************************************************
MSP430G2553 Project Creator

GE 423  - Dan Block
        Spring(2012)

        Written(by) : Steve(Keres)
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430g2553.h"
#include "UART.h"
#include <cstdlib>

#define NUMBER_OF_MOTORS 16
#define SPI_BUFF_SIZE 24

#define WIFI_EN 0x01
#define MODE 0x4
#define XLAT 0x8
#define BLANK 0x10

char newprint = 0;
unsigned long timecnt = 0;//keeps track of milliseconds
unsigned long pwm_timecnt = 0;//keeps track of each cylce of PWM

unsigned int motors[NUMBER_OF_MOTORS];//array of motor intensities. varies from 0-4095

volatile unsigned int debug_count = 0;//debug uses to keep track of how many times something executes

volatile signed char spi_index = SPI_BUFF_SIZE-1;//index into spi_sout_buff, keeps track of how much we've sent
volatile unsigned char spi_sout_buff[SPI_BUFF_SIZE];//holds the SPI outs

char recchar;//variable that holds values to read in from the UART line
char recchar2;//variable that holds values to read in from the SPI line
char rx_started = 0;//new recieve message flag
//char rxbuff[255];//buffer to store message
//char msgindex = 0;//keeping count of message length
char receive_length;//message's proposed length
char connection_ID;//connection ID of the module
char junk_count;//the JUNK values of IPD count before recieving data
volatile char error_flag = 0;

void updateTLC_array();
void sendTLC_array();

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

	DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
	BCSCTL1 = CALBC1_16MHZ; 

	//Port 1 GPIO's 
	//0 1 2 3 4 5 6 7
	//O / / IGIG/ / /
	//  RXTX    S S S
	// S - denotes SPI
	// RXTX - denotes UART
	// IG - denotes Input but grounded

	// Initialize Port 1 GPIO's
	P1SEL &= ~0x01;  // See page 42 and 43 of the G2553's datasheet, It shows that when both P1SEL and P1SEL2 bits are zero   
	P1SEL2 &= ~0x01; // the corresponding pin is set as a I/O pin.  Datasheet: http://coecsl.ece.illinois.edu/ge423/datasheets/MSP430Ref_Guides/msp430g2553datasheet.pdf  
	P1REN = 0x0;  // No resistors enabled for Port 1
	P1DIR |= 0x1; // Set P1.0 to output to drive LED on LaunchPad board.  Make sure shunt jumper is in place at LaunchPad's Red LED
	P1OUT &= ~0x01;  // Initially set P1.0 to 0

	//seting unused pins to inputs
	P1SEL &= ~0x01;  
	P1SEL2 &= ~0x01; 
	P1DIR &= ~0x18; 


	//Port 2 GPIO's 
	//0 1 2 3 4 5 6 7
	//O I O O O / IGIG
	//  R       Timer 1.2
	// R - denotes pull up resistor
	// IG - denotes Input but grounded

	//Initialize Port 2 GPIO's
	//set all of the GPIO's to GPIO's
	P2SEL &= ~0xDF;
	P2SEL2 &= ~0xDF;
	//set PWM Mode
	P2SEL |= 0x20;
	P2SEL2 &= ~0x20;
	//setting the Bit Bang instead
//	P2SEL &= ~0x20;
//	P2SEL &= ~0x20;
	P2REN = 0x02;
	P2DIR = 0x3D;
	P2OUT |= WIFI_EN;
	P2OUT &= ~XLAT;
	// P2OUT &= ~BLANK;
	P2OUT |= BLANK;
	P2OUT &= ~MODE;
	
	// Timer A Config
	TA0CCTL0 = CCIE;       		// Enable Periodic interrupt
	TA0CCR0 = 16000;                // period = 1ms
	TA0CTL = TASSEL_2 + MC_1; // source SMCLK, up mode

	// Timer A GSCLK Setup
	// Hobby Servo Set Up, Should Change Later
//	TA1CCR0 = 40000;                             // PWM Period
//	TA1CCTL1 = OUTMOD_7;                         // TA1CCR1 reset/set
//	TA1CCTL2 = OUTMOD_7;                         // TA1CCR2 reset/set
//	TA1CCR1 = 3400;                   // TA1CCR1 PWM duty cycle about 50% initially of max pulse width
//	TA1CCR2 = 3400;                   // TA1CCR2 PWM duty cycle about 50% initially of max pulse width
//	TA1CTL = TASSEL_2 + ID_3 + MC_1;                  // SMCLK, up mode, ID - sclock divider for 8
//	TA1CCTL0 = CCIE;       		// Enable Periodic interrupt
	TA1CCR0 = 40;                             // PWM Period
	TA1CCTL1 = OUTMOD_7;                         // TA1CCR1 reset/set
	TA1CCTL2 = OUTMOD_7+CCIE;                         // TA1CCR2 reset/set
	TA1CCR1 = 1;                   // TA1CCR1 PWM duty cycle about 50% initially of max pulse width
	TA1CCR2 = 20;                   // TA1CCR2 PWM duty cycle about 50% initially of max pulse width
	TA1CTL = TASSEL_2 + /*ID_3*/ + MC_1;                  // SMCLK, up mode, ID - sclock divider for 8
	//seems to be around 190ns period


	//SPI Config - Might move to new file in future
	P1SEL |= BIT5 + /*BIT6 +*/ BIT7;//setting pins to SPI mode
  	P1SEL2 |= BIT5 + /*BIT6 +*/ BIT7;
	UCB0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC; // 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2; // SMCLK
//     UCB0BR0 |= 0x01; // 1:1
    UCB0BR0 = 80;//may want to adjust
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST; // clear SW
    IFG2 &= ~UCB0RXIE;
    IFG2 &= ~UCB0TXIE;
  	IE2 |= UCB0RXIE;                          // Enable USCI0 RX interrupt
  	IE2 |= UCB0TXIE;                          // Enable USCI0 RX interrupt

  	//set pin 6 to input
  	P1SEL &= ~BIT6;
  	P1SEL2 &= ~BIT6;

	Init_UART(115200,1);	// Initialize UART for 9600 baud serial communication

	_BIS_SR(GIE); 		// Enable global ianterrupt

	int i;
	for (i = 0; i < NUMBER_OF_MOTORS; i++)
	{
		motors[i] = 4095;
	}
//	for (i = 0; i < NUMBER_OF_MOTORS; i++)
//	{
//		if(i % 2)
//		motors[i] = 4095;
//	}

	/**********************MAIN LOOP*******************************/
	while(1) {

		if(newmsg) {
			newmsg = 0;
		}

		if (newprint)  {
			if (rxbuff[0] == '1')
			{
				P1OUT ^= 0x1;		// Blink LED
			}
//			UART_printf("AT+CIPMUX=1\r\nAT+CIPSERVER=1\r\n");
			//jank work around because printf wasn't working with the longer strings
			if((timecnt>=2500)&&(timecnt<3000))
				 UART_printf("AT+CIPMUX=1\r\n");//AT+CIPSERVER=1\r\n");
			if((timecnt>=3000)&&(timecnt<3500))
				 UART_printf("AT+CIPSERVER=1\r\n");
			if((timecnt > 4000) && ((timecnt%500) == 0))
			{
				updateTLC_array();
			}
//			debug_count++;
//			P2OUT ^= 0x20;
//
//			if ((timecnt%(4096*2)) == 0) {
//				P2OUT |= BLANK;
//				P2OUT &= ~BLANK;
//			}

//			UART_printf("Hello %d\n\r",(int)(timecnt/500));
//			int i;
//			for (i = 0; i < NUMBER_OF_MOTORS; i++)
//			{
//				motors[i] = timecnt;
//			}
//			newprint = 0;
		}

	}
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	timecnt++; // Keep track of time for main while loop. 

	if ((timecnt%500) == 0) {
	newprint = 1;  // flag main while loop that .5 seconds have gone by.  
	}

	if((timecnt%500) == 0)
	{
		debug_count++;
		sendTLC_array();
	}
}

// Timer A0 interrupt service routine
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_1 (void)
{
	pwm_timecnt++; // Keep track of PWM counts

	if ((pwm_timecnt%(4096*2)) == 0) {
//		P2OUT |= (XLAT+BLANK);
//		P2OUT &= ~(XLAT+BLANK);
		P2OUT |= BLANK;
		P2OUT &= ~BLANK;
	}

	TA1IV &= ~TA1IV_TACCR2;
}


/*
// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {

}
*/


// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {

	if(IFG2&UCA0TXIFG) {		// USCI_A0 requested TX interrupt
		if(printf_flag) {
			if (currentindex == txcount) {
				senddone = 1;
				printf_flag = 0;
				IFG2 &= ~UCA0TXIFG;
			} else {
				UCA0TXBUF = printbuff[currentindex];
				currentindex++;
			}
		} else if(UART_flag) {
			if(!donesending) {
				UCA0TXBUF = txbuff[txindex];
				if(txbuff[txindex] == 255) {
					donesending = 1;
					txindex = 0;
				}
				else txindex++;
			}
		} else {  // interrupt after sendchar call so just set senddone flag since only one char is sent
			senddone = 1;
		}

		IFG2 &= ~UCA0TXIFG;
	}

	if(IFG2&UCB0TXIFG) {	// USCI_B0 requested TX interrupt (UCB0TXBUF is empty)
		if (spi_index < 0)
		{
			debug_count++;
//			P2OUT |= (XLAT+BLANK);
//			P2OUT &= ~(XLAT+BLANK);
//			P2OUT |= BLANK;
//			P2OUT &= ~BLANK;
			P2OUT |= XLAT;
			P2OUT &= ~XLAT;
			spi_index = SPI_BUFF_SIZE - 1;
		}
		else
		{		
			UCB0TXBUF = spi_sout_buff[spi_index];
			spi_index--;
		}
		IFG2 &= ~UCB0TXIFG;   // clear IFG
	}
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

	if(IFG2&UCB0RXIFG) {  // USCI_B0 requested RX interrupt (UCB0RXBUF is full)
		recchar2 = UCB0RXBUF;
		IFG2 &= ~UCB0RXIFG;   // clear IFG
	}

	if(IFG2&UCA0RXIFG) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)
		//interrupts when recieved UART
		//pareses incoming strings +IPD,X,X:MESSAGE_HERE
		recchar = UCA0RXBUF;// acquire character

		if(!started) {	// Haven't started a message yet
			if(recchar == '+') {//incoming message starts with a +
				started = 1;
				newmsg = 0;
				junk_count = 1;
			}
		}
		else {	// In process of receiving a message		
			//IMPORTANT: this is not tolerant to 2+ digits
			error_flag = 0;//changes to 1 if incoming data isn't as expected
			switch(junk_count)
			{
				case 1://I
					recchar = UCA0RXBUF;
					if (recchar != 'I')
						error_flag = 1;
					junk_count++;
				break;
				case 2://P
					recchar = UCA0RXBUF;
					if (recchar != 'P')
						error_flag = 1;
					junk_count++;
				break;
				case 3://D
					recchar = UCA0RXBUF;
					if (recchar != 'D')
						error_flag = 1;
					junk_count++;
				break;
				case 4://,
					recchar = UCA0RXBUF;
					if (recchar != ',')
						error_flag = 1;
					junk_count++;
				break;
				case 5://[connection_ID]
					recchar = UCA0RXBUF;
					connection_ID = atoi(&recchar);
					junk_count++;
				break;
				case 6://,
					recchar = UCA0RXBUF;
					if (recchar != ',')
						error_flag = 1;
					junk_count++;
				break;
				case 7://message length
					recchar = UCA0RXBUF;
					receive_length = atoi(&recchar);
					junk_count++;
				break;
				case 8://:
					recchar = UCA0RXBUF;
					if (recchar != ':')
						error_flag = 1;
					junk_count++;
				default://actually get message here
					recchar = UCA0RXBUF;
					if((msgindex < receive_length) && (msgindex < 255)&&(recchar != '\n')&&(recchar != '\r')) {
						// if (recchar == '1')
						// {
						// 	P1OUT ^= 0x1;
						// }
						rxbuff[msgindex] = recchar;
						msgindex++;
					}
					else {	// designated length hit
					newmsg = 1;
					rxbuff[msgindex] = '\0';	// "Null"-terminate the array
					started = 0;
					msgindex = 0;
					}
			}

			if (error_flag == 1)
			{
				started = 0;
				newmsg = 0;
				msgindex = 0;
			}

		 }

		IFG2 &= ~UCA0RXIFG;
	}

}

void updateTLC_array() {

	unsigned char ledCounter = NUMBER_OF_MOTORS >> 1;
	unsigned char buff_idx = SPI_BUFF_SIZE-1;

	while (ledCounter-- > 0) {

		unsigned char i = ledCounter << 1;

		// UCB0TXBUF = motors[i + 1] >> 4;
		// while (!(IFG2 & UCB0TXIFG))
		// 	; // TX buffer ready?
		// unsigned char unib = motors[i + 1] << 4;
		// unsigned char lnib = (motors[i] >> 8) & 0x0F;
		// UCB0TXBUF = unib | lnib;
		// while (!(IFG2 & UCB0TXIFG))
		// 	; // TX buffer ready?

		// UCB0TXBUF = motors[i];
		// while (!(IFG2 & UCB0TXIFG))
		// 	; // TX buffer ready?
		//got rid of the while spins because ISR is more efficient I think
		spi_sout_buff[buff_idx] = motors[i + 1] >> 4;
		buff_idx--;
		unsigned char unib = motors[i + 1] << 4;
		unsigned char lnib = (motors[i] >> 8) & 0x0F;
		spi_sout_buff[buff_idx] = unib | lnib;
		buff_idx--;
		spi_sout_buff[buff_idx] = motors[i];
		buff_idx--;
	}
}

void sendTLC_array() {
	UCB0TXBUF = spi_sout_buff[spi_index];
}
