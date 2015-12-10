/*
 * main.c
 *
 * GNU License:
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* Which analog pin we want to read from.  The pins are labeled "ADC0"
 * "ADC1" etc on the pin out in the data sheet.  In this case ADC_PIN
 * being 0 means we want to use ADC0.  On the ATmega328P this is also
 * the same as pin PC0 */
 
#define ADC_PIN			0
#define ADC_NEAR		125
#define ADC_FAR			55
#define	LED_PIN			PB0
#define	BUZZ			PB2
#define LED0			PD5			
#define LED1			PD4
#define LED2			PD7
#define	INT_LED			PB1


/* This function just keeps the reading code out of the loop itself.
 * It takes the analog pin number as a parameter and returns the
 * analog reading on that pin as a result.
 *
 * Look for its definition below main. */
 
//Function Declarations 
uint16_t adc_read(uint8_t adcx);
void ioinit(void);      //Initializes IO
void delay_ms(uint16_t x); //General purpose delay
void Wait(){
	uint16_t i;
	for(i=0;i<50;i++)
	{
		_delay_loop_2(0);
		_delay_loop_2(0);
		_delay_loop_2(0);
	}
}

int main(void){
	
	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
	// PD2 (PCINT0 pin) is now an input - input from button
		
	EICRA &= ~(1 << ISC01);    // set INT0 to trigger on any state change
	EICRA |= (1 << ISC00);
	
	EIMSK |= (1 << INT0);     // Turns on INT0

	DDRD |= (1 << DDD6) | //servo
			(1 << DDD7) | //red LED			
			(1 << DDD5) | //blue LED
			(1 << DDD4);  //green LED

	DDRB |= (1 << DDB2) | //buzzer
			(1 << DDB1);  //debug LED
	

	OCR0A = 128;
	// set PWM for 50% duty cycle

	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// set fast PWM Mode

	ioinit();			
	sei();                    // turn on interrupts
	PORTB |= _BV(LED2);
	PORTB |= _BV(LED1);
	PORTB |= _BV(LED0);
		
	//intialize pwm to off
	TCCR0A &= ~(1 << COM0A1);// set none-inverting mode
	TCCR0B &= ~(1 << CS02);
	for (;;){
		if(adc_read(ADC_PIN) > ADC_NEAR) //close distance object
		{
			PORTD |= _BV(LED0);			//turn red LED on
			PORTD &= ~_BV(LED1);		//confirms other two LEDs are off
			PORTD &= ~_BV(LED2);
			PORTB |= _BV(BUZZ);			//turns buzzer on
		}
		else if ((adc_read(ADC_PIN) < ADC_NEAR) && (adc_read(ADC_PIN) > ADC_FAR)) //medium distance object
		{
			PORTD &= ~_BV(LED0);		//turn off other 2 LEDs and buzzer
			PORTD |= _BV(LED1);			//turns on blue LED
			PORTD &= ~_BV(LED2);
			PORTB &= ~_BV(BUZZ);
		}
		else if ((adc_read(ADC_PIN) < ADC_FAR)) //no object in sight
		{
			PORTD &= ~_BV(LED0);
			PORTD &= ~_BV(LED1);
			PORTD |= _BV(LED2);		//turn on green LED
			PORTB &= ~_BV(BUZZ);	//turn off other two LEDs and buzzer
		}

	}

}

void ioinit (void)
{

	DDRB  |= _BV(LED_PIN);
	
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
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

//interrupt service routine
//checks voltage level on pin PD2
//if voltage is high it turns on PWM and a debug LED which is non populated on board
ISR (INT0_vect)
{
	//PORTB ^= _BV(LED);
	//checks value on PD2 and if high sets the PWM prescaleer and turns on PWM
	//motor will begin to spin
	if (PIND & (1<<PD2)){		
		TCCR0A |= (1 << COM0A1);// set none-inverting mode
		TCCR0B |= (1 << CS01);	//500 Hz PWM
		PORTB |= _BV(INT_LED);
	
	//when voltage on PD2 is low, the PWM will be turn off
	//motor will stop spinning and LED will turn off
	} else {
		// set none-inverting mode
		TCCR0A &= ~(1 << COM0A1);// set none-inverting mode
		TCCR0B &= ~(1 << CS01);
		PORTB &= ~_BV(INT_LED);
		
	}
	
	_delay_ms(250); //debounce loop for safety
}