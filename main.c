/*
 * RC360Detector.c
 *
 * Created: 10/20/2015 9:43:03 AM
 * Author : Matt
 */ 

/*
    5-10-07
    Copyright Spark Fun Electronics© 2007
    Nathan Seidle
    nathan at sparkfun.com
    
    ATmega168
	
	Example Blink
	Toggles all IO pins at 1Hz
*/

#include <avr/io.h>
#include <stdint.h>

/* Which analog pin we want to read from.  The pins are labeled "ADC0"
 * "ADC1" etc on the pinout in the data sheet.  In this case ADC_PIN
 * being 0 means we want to use ADC0.  On the ATmega328P this is also
 * the same as pin PC0 */
#define ADC_PIN			0
#define ADC_THRESHOLD	110
#define	LED_PIN			PB0
/* This function just keeps the reading code out of the loop itself.
 * It takes the analog pin number as a parameter and returns the
 * analog reading on that pin as a result.
 *
 * Look for its definition below main. */
uint16_t adc_read(uint8_t adcx);

//Define functions
//======================
void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay
//======================

int main (void)
{
    ioinit(); //Setup IO pins and defaults
/*
    while(1)
    {
		PORTC = 0xFF;
		PORTB = 0xFF;
		PORTD = 0xFF;
		delay_ms(500);

		PORTC = 0x00;
		PORTB = 0x00;
		PORTD = 0x00;
		delay_ms(500);
    }
  */
	for (;;) {

		if (adc_read(ADC_PIN) > ADC_THRESHOLD){
			PORTB |= _BV(LED_PIN);
		} else {
			PORTB &= ~_BV(LED_PIN);
		}
	} 
    return(0);
}

void ioinit (void)
{
    //1 = output, 0 = input
    //DDRB = 0b11111111; //All outputs
    //DDRC = 0b11111111; //All outputs
    //DDRD = 0b11111110; //PORTD (RX on PD0)
	DDRB  |= _BV(LED_PIN);
	//ADCSRA |= _BV(ADEN);
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

//General short delays
void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}


uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}