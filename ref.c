#include <msp430g2553.h>

#define SCLK_PIN	BIT5
#define MOSI_PIN	BIT7

#define GSCLK_PIN	BIT4
#define BLANK_PIN	BIT2
#define XLAT_PIN	BIT1
#define DCPRG_PIN	BIT0
#define VPRG_PIN	BIT0

typedef unsigned char u_char;
typedef unsigned int u_int;

#define NUMBER_OF_LEDS 32

u_int leds[NUMBER_OF_LEDS] = { 0, };
u_char timerCounter = 0;

void updateTLC();

void main(void) {

	WDTCTL = WDTPW + WDTHOLD; // disable WDT
	BCSCTL1 = CALBC1_16MHZ; // 16MHz clock
	DCOCTL = CALDCO_16MHZ;
	BCSCTL2 |= DIVS_3; // divide clock by 8

	P1OUT &= ~(VPRG_PIN);
	P1DIR |= VPRG_PIN;

	P1DIR |= GSCLK_PIN; // port 1.4 configured as SMCLK out
	P1SEL |= GSCLK_PIN;

	P2OUT &= ~(BLANK_PIN + XLAT_PIN + DCPRG_PIN);
	P2DIR |= BLANK_PIN + XLAT_PIN + DCPRG_PIN;

	// setup timer
	CCR0 = 0xFFF;
	TACTL = TASSEL_2 + MC_1 + ID_0; // SMCLK, up mode, 1:1
	CCTL0 = CCIE; // CCR0 interrupt enabled

	// setup UCB0
	P1SEL |= SCLK_PIN + MOSI_PIN;
	P1SEL2 |= SCLK_PIN + MOSI_PIN;

	UCB0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC; // 3-pin, 8-bit SPI master
	UCB0CTL1 |= UCSSEL_2; // SMCLK
	UCB0BR0 |= 0x01; // 1:1
	UCB0BR1 = 0;
	UCB0CTL1 &= ~UCSWRST; // clear SW

	updateTLC();
	P2OUT |= XLAT_PIN;
	P2OUT &= ~XLAT_PIN;

	_bis_SR_register(GIE);

	leds[0] = 0x0F;
	u_char counter = 0;
	u_char direction = 0;
	u_char first = 0;
	u_char firstDirection = 0;
	u_int brightness = 0xFFF;

	while (1) {
		//
		if (direction) {
			counter--;
			leds[counter] = brightness;
			leds[counter + 1] = 0;
			if (counter == first) {
				direction = 0;
				// first
				if (firstDirection) {
					first--;
					if (first == 0)
						firstDirection = 0;
				} else {
					leds[first - 1] = brightness;
					first++;
					if (first == (NUMBER_OF_LEDS - 2))
						firstDirection = 1;
				}
			}
		} else {
			counter++;
			leds[counter] = brightness;
			leds[counter - 1] = 0;
			if (counter == (NUMBER_OF_LEDS - 1)) {
				direction = 1;
			}
		}

		//update all leds
		u_char c = 0;
		while(c < NUMBER_OF_LEDS) {
			if(leds[c] > 0) leds[c] = brightness;
			c++;
		}
		//update brightness
		brightness -= 0x02;

		// sleep
		_bis_SR_register(LPM0_bits);
	}
}

void updateTLC() {

	u_char ledCounter = NUMBER_OF_LEDS >> 1;

	while (ledCounter-- > 0) {

		u_char i = ledCounter << 1;

		UCB0TXBUF = leds[i + 1] >> 4;
		while (!(IFG2 & UCB0TXIFG))
			; // TX buffer ready?
		u_char unib = leds[i + 1] << 4;
		u_char lnib = (leds[i] >> 8) & 0x0F;
		UCB0TXBUF = unib | lnib;
		while (!(IFG2 & UCB0TXIFG))
			; // TX buffer ready?

		UCB0TXBUF = leds[i];
		while (!(IFG2 & UCB0TXIFG))
			; // TX buffer ready?
	}
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0(void) {
	P2OUT |= BLANK_PIN;
	P2OUT |= XLAT_PIN;
	P2OUT &= ~XLAT_PIN;
	P2OUT &= ~BLANK_PIN;

	timerCounter++;
	if (timerCounter == 0x08) { // 0x08 - 2ms * 8 = 16.384ms, ~61Hz
		updateTLC();
		timerCounter = 0;
		_bic_SR_register_on_exit(LPM0_bits);
	}
}