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

#define NUMBER_OF_MOTORS 10

#define WIFI_EN 0x01
#define MODE 0x4
#define XLAT 0x8
#define BLANK 0x10

char newprint = 0;
unsigned long timecnt = 0;

unsigned int motors[NUMBER_OF_MOTORS] = {0, };

char recchar;//variable that holds junk values to read in from the serial line
char rx_started = 0;//new recieve message flag
//char rxbuff[255];//buffer to store message
//char msgindex = 0;//keeping count of message length
char receive_length;//message's proposed length
char connection_ID;//connection ID of the module
char junk_count;//the JUNK values of IPD count before recieving data


void updateTLC();

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
	P2REN = 0x02;
	P2DIR = 0x3D;
	P2OUT |= WIFI_EN;
	P2OUT &= ~XLAT;
	P2OUT &= ~BLANK;
	P2OUT &= ~MODE;
	
	// Timer A Config
	TACCTL0 = CCIE;       		// Enable Periodic interrupt
	TACCR0 = 16000;                // period = 1ms   
	TACTL = TASSEL_2 + MC_1; // source SMCLK, up mode

	// Timer A GSCLK Setup


	//SPI Config - Might move to new file in future
	P1SEL |= BIT5 + /*BIT6 +*/ BIT7;//setting pins to SPI mode
  	P1SEL2 |= BIT5 + /*BIT6 +*/ BIT7;
	UCB0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC; // 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2; // SMCLK
    UCB0BR0 |= 0x01; // 1:1
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST; // clear SW

	Init_UART(115200,1);	// Initialize UART for 9600 baud serial communication

	_BIS_SR(GIE); 		// Enable global interrupt

//	UART_printf("AT+CIPMUX=1\r\nAT+CIPSERVER=1\r\n");

	while(1) {

		if(newmsg) {
			newmsg = 0;
		}

		if (newprint)  {
			if (rxbuff[0] == 1)
			{
				P1OUT ^= 0x1;		// Blink LED
			}
//			UART_printf("AT+CIPMUX=1\r\nAT+CIPSERVER=1\r\n");
			//jank work around because printf wasn't working with the longer strings
			if((timecnt>=2500)&&(timecnt<3000))
				UART_printf("AT+CIPMUX=1\r\n");//AT+CIPSERVER=1\r\n");
			if((timecnt>=3000)&&(timecnt<3500))
				UART_printf("AT+CIPSERVER=1\r\n");

//			UART_printf("Hello %d\n\r",(int)(timecnt/500));

			newprint = 0;
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
	updateTLC();
	}

	P2OUT |= XLAT;
	P2OUT &= ~XLAT;

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

		IFG2 &= ~UCB0TXIFG;   // clear IFG
	}
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

	if(IFG2&UCB0RXIFG) {  // USCI_B0 requested RX interrupt (UCB0RXBUF is full)

		IFG2 &= ~UCB0RXIFG;   // clear IFG
	}

	if(IFG2&UCA0RXIFG) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)

//    Uncomment this block of code if you would like to use this COM protocol that uses 253 as STARTCHAR and 255 as STOPCHAR
/*		if(!started) {	// Haven't started a message yet
			if(UCA0RXBUF == 253) {
				started = 1;
				newmsg = 0;
			}
		}
		else {	// In process of receiving a message		
			if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
				rxbuff[msgindex] = UCA0RXBUF;

				msgindex++;
			} else {	// Stop char received or too much data received
				if(UCA0RXBUF == 255) {	// Message completed
					newmsg = 1;
					rxbuff[msgindex] = 255;	// "Null"-terminate the array
				}
				started = 0;
				msgindex = 0;
			}
		}
*/
		// recchar = UCA0RXBUF;// acquire character


		// sendchar(recchar);// echo the character
		// UART_printf(" - was recieved\n");

		if(!started) {	// Haven't started a message yet
			if(UCA0RXBUF == '+') {
				started = 1;
				newmsg = 0;
				junk_count = 0;
			}
		}
		else {	// In process of receiving a message		
			if (junk_count < 4)//junk IPD,
			{
				recchar = UCA0RXBUF;
				junk_count++;
			}
			else if (junk_count == 4)//store the connection ID
			{
				connection_ID = UCA0RXBUF;
				junk_count++;
			}
			else if (junk_count == 5)//junk ,
			{
				recchar = UCA0RXBUF;
				junk_count++;
			}
			else if (junk_count == 6)//store the length that we should be recieving, need to make sure messages are shorter than 10 characters
			{
				receive_length = UCA0RXBUF;
				junk_count++;
			}
			else if (junk_count == 7)//junk :
			{
				recchar = UCA0RXBUF;
				junk_count++;
			}
			else if((msgindex < receive_length) && (msgindex < 255)) {
				rxbuff[msgindex] = UCA0RXBUF;

				msgindex++;
			} else {	// designated length hit
				// if(UCA0RXBUF == 255) {	// Message completed
					newmsg = 1;
					rxbuff[msgindex] = 255;	// "Null"-terminate the array
				// }
				started = 0;
				msgindex = 0;
			}
		}

		IFG2 &= ~UCA0RXIFG;
	}

}

void updateTLC() {

	unsigned char ledCounter = NUMBER_OF_MOTORS >> 1;

	while (ledCounter-- > 0) {

		unsigned char i = ledCounter << 1;

		UCB0TXBUF = motors[i + 1] >> 4;
		// while (!(IFG2 & UCB0TXIFG))
			// ; // TX buffer ready?
		unsigned char unib = motors[i + 1] << 4;
		unsigned char lnib = (motors[i] >> 8) & 0x0F;
		UCB0TXBUF = unib | lnib;
		// while (!(IFG2 & UCB0TXIFG))
			// ; // TX buffer ready?

		UCB0TXBUF = motors[i];
		// while (!(IFG2 & UCB0TXIFG))
			// ; // TX buffer ready?
		//got rid of the while spins because ISR is more efficient I think
	}
}
