/*@****************************** RCS HEADER ********************************/
/*
 * $RCSfile: lut.c,v $
 * $Author: chris $
 * $Date: 2007-10-07 09:58:09 $
 * $Revision: 1.1 $
 *
 */ 

//@********************************* lut.c ***********************************

/*
 * Light Up $THING  --  LED lightshow using RGB LED with software PWM
 *
 * The goal is to create a coloured light that cyles gradually through random 
 * colours, but changes colour instantly at the press of a switch.
 *
 * Why?   My kids had this $2 toy pen that did this with 3 SMD LEDs and some lithium batteries.
 *        They broke it almost instantly.
 *
 * Since the ATtiny13 only has one timer with 2 pwm channels, we do the PWM logic in 
 * software, using a regular tick from the hardware timer0.
 *
 * If I could get the ATtiny[248]5 in Oz (3 PWM channels), I'd use that instead...
 *
 * Theory of operation
 *
 * Lowest level:
 *     3 independent PWM channels generate waveforms that drive an RGB LED.
 *     The 3 PWM duty cycle values define the colour.
 * Colour change level:
 *     Every CYCLERATEx PWM cycles, the duty cycle of channel X changes by one bit.
 *     If a button is pressed, the duty cycle of one channel is changed to a new random value.
 * 
 */ 

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/*@****************************** constants *********************************/
/* 
 *@@ pin assignments
 * 
 */

#ifdef __AVR_ATmega168__
#error mega168 not implemented yet
#elif defined(__AVR_ATtiny2313__)
#error tiny2313 not implemented yet
#elif defined(__AVR_ATtiny13__)
/* 
*@@ Pin assignments for ATtiny13
* 
*/

// hello-world LED is PB0
#define HWL_DDR    DDRB
#define HWL_PORT   PORTB
#define HWL_PIN    PINB
#define HWL_MASK   _BV(PB0)

// push-button-input is PB1 (INT0)
#define PBI_DDR    DDRB
#define PBI_PORT   PORTB
#define PBI_PIN    PINB
#define PBI_MASK   _BV(PB1)

// soft-PWM output A is PB2, pin 7
#define PWMA_DDR  DDRB
#define PWMA_PORT PORTB
#define PWMA_PIN  PINB
#define PWMA_MASK _BV(PB2)
// soft-PWM output B is PB3, pin 2
#define PWMB_DDR  DDRB
#define PWMB_PORT PORTB
#define PWMB_PIN  PINB
#define PWMB_MASK _BV(PB3)
// soft-PWM output C is PB4, pin 3
#define PWMC_DDR  DDRB
#define PWMC_PORT PORTB
#define PWMC_PIN  PINB
#define PWMC_MASK _BV(PB4)

#else
#error unsupported chip
#endif



/*@********************************* data ***********************************/
/* 
 *@@ register-bound variables
 * 
 */

 // top value for pseudo-timers.  
 // Don't necessarily need 8-bit pwm for LEDs, might get away with, say,
 // 5-bits
 // 
 // Lowering this value gives a faster PWM frequency
 //
#define PWM_TOP 255
register uint8_t pwm_cnt_top asm("r2"); 

// counter values for the pwm pseudo-timers.
// We /could/ get away with a single timer, but by running three
// pseudo-timers 'out of phase' (different initial values), peak 
// current draw may be reduced.
//
// We simulate the AVR hardware timers' "fast PWM" mode.
//
// Counter starts at zero and counts to TOP, then goes back to 0
// Overflow event (pseudo-interrupt) occurs when timer passes TOP
// Output pin is set at 0, and cleared when CNT==OC
//
register uint8_t pwm_cnt_a asm("r7");
register uint8_t pwm_cnt_b asm("r8");
register uint8_t pwm_cnt_c asm("r9");

// output-compare values for the 3 soft-PWM channels
//
// these are the current 'duty cycle' values for the PWM waveforms, 
// i.e. the LED brightness 0==always off, TOP==full on
//
register uint8_t pwm_oc_a asm("r4");
register uint8_t pwm_oc_b asm("r5");
register uint8_t pwm_oc_c asm("r6");


/* 
 *@@ RAM variables 
 */

// debounce interval - number of PWM cycles for which input is ignored 
//                     after a button press
//
// At button press, button interrupt is disabled and debounce_ctr set to
// DEBOUNCE_INTERVAL
//
// At Every overflow event, debounce_ctr is decremented.  When it reaches
// zero, the button interrupt is re-enabled
//
#define DEBOUNCE_INTERVAL 32
uint8_t debounce_ctr;


// cycle rate values for the pwm channels.
// the duty cycle (pwm_oc_N) will change by dcc_delta_N every dcc_interval_N
// overflows
//
// This produces a shifting colour display, esp when the values are
// different for each channel.
// 
// A button press should modify one or more of these values to produce a new
// colour
//
// dcc = duty cycle change
//
uint8_t dcc_interval_a = 1;
uint8_t dcc_ctr_a;
int8_t dcc_delta_a = 1;

uint8_t dcc_interval_b = 1;
uint8_t dcc_ctr_b;
int8_t dcc_delta_b = 2;

uint8_t dcc_interval_c = 1;
uint8_t dcc_ctr_c;
int8_t dcc_delta_c = -3;

/*@********************** Interrupt Service routines ************************/

/* 
 *@@ Button-press event
 */

ISR(INT0_vect)
{
	/* 
	 * We got a falling edge on the INT0 pin, i.e. button press
	 *
	 * Do a "colour change" procedure
	 */

	/*
	 * Disable further INT0 events for N  PWM cycles.
	 * This is done to aid debouncing of the pushbutton input.
	 */
	GIMSK &= _BV(INT0);
	debounce_ctr = DEBOUNCE_INTERVAL;

	/*
	 * Randomly change the phase and value of one of the PWM channels.  
	 *
	 * This should yield an interesting colour variation
	 **/
	switch (TCNT0&0x03) {
	case 0x00:
		// randomly change colour of channel one
		pwm_oc_a ^= TCNT0;
		if (TCNT0&0x01) 
			dcc_delta_a <<= 1;
		else 
			dcc_delta_a >>= 1;
		if (dcc_delta_a == 0) 
			dcc_delta_a = 1;
		break;
	case 0x01:
		// randomly change colour of channel two
		pwm_oc_b ^= TCNT0;
		if (TCNT0&0x01) 
			dcc_delta_b <<= 1;
		else 
			dcc_delta_b >>= 1;
		if (dcc_delta_b == 0) 
			dcc_delta_b = 1;
		break;
	default:
		// randomly change colour of channel three
		pwm_oc_c ^= TCNT0;
		if (TCNT0&0x01) 
			dcc_delta_c <<= 1;
		else 
			dcc_delta_c >>= 1;
		if (dcc_delta_c == 0) 
			dcc_delta_c = 1;
		break;
		
	}
	
	
}


/* 
 *@@ Hardware timer overflow event 
 */
ISR(TIM0_OVF_vect)	
{
	int oc_new;


	/* 
	 * We got a timer overflow interrupt.  Time to bump our software-PWM
	 * counters, and test the output compares.
	 *
	 * We simulate non-inverting fast PWM mode.  OC pin is set at BOTTOM
	 * and cleared at OC match.  So PWM value 0 is "always off", PWM
	 * value 255 is (almost) always on.
	 */
	if (pwm_cnt_a == pwm_cnt_top) {
		if (pwm_oc_a) 
			PWMA_PORT |= PWMA_MASK;
		pwm_cnt_a = 0;
	}
	else {
		++pwm_cnt_a;
		if (pwm_cnt_a == pwm_oc_a) {
			PWMA_PORT &= ~PWMA_MASK;
		}
	}
		
	++pwm_cnt_b;
	if (pwm_cnt_b > pwm_cnt_top) pwm_cnt_b = 0;
	if (pwm_cnt_b == pwm_oc_b) {
		PWMB_PORT &= ~PWMB_MASK;
	}
	else if (pwm_cnt_b == 0) {
		PWMB_PORT |= PWMB_MASK;
	}

	++pwm_cnt_c;
	if (pwm_cnt_c > pwm_cnt_top) pwm_cnt_c = 0;
	if (pwm_cnt_c == pwm_oc_c) {
		PWMC_PORT &= ~PWMC_MASK;
	}
	else if (pwm_cnt_c == 0) {
		PWMC_PORT |= PWMC_MASK;
	}

	/* 
	 * Check if its time to change colours
	 *
	 * We do this check at soft-timer overflow, every time pwm_cnt_N
	 * wraps to 0
	 *
	 */
	if (pwm_cnt_a == 0) {
		++dcc_ctr_a;
		if (dcc_ctr_a >= dcc_interval_a) {
			// 
			// time to bump the duty cycle for this channel by a
			// small bit
			//
			dcc_ctr_a = 0;
			HWL_PIN|=HWL_MASK;
			oc_new = pwm_oc_a + dcc_delta_a;
			if (oc_new <= 0) {
				oc_new = 1;
				dcc_delta_a = -dcc_delta_a;
			}
			else if (oc_new > pwm_cnt_top) {
				oc_new = pwm_cnt_top;
				dcc_delta_a = -dcc_delta_a;
			}
			pwm_oc_a = oc_new;
			
		}
	}

	if (pwm_cnt_b == 0) {
		++dcc_ctr_b;
		if (dcc_ctr_b > dcc_interval_b) {
			dcc_ctr_b = 0;
			oc_new = pwm_oc_b + dcc_delta_b;
			if (oc_new <= 0) {
				oc_new = 1;
				dcc_delta_b = -dcc_delta_b;
			}
			else if (oc_new > pwm_cnt_top) {
				oc_new = pwm_cnt_top;
				dcc_delta_b = -dcc_delta_b;
			}
			pwm_oc_b = oc_new;
		}
	}
	
	if (pwm_cnt_c == 0) {
		++dcc_ctr_c;
		if (dcc_ctr_c > dcc_interval_c) {
			dcc_ctr_c = 0;
			oc_new = pwm_oc_c + dcc_delta_c;
			if (oc_new <= 0) {
				oc_new = 1;
				dcc_delta_c = -dcc_delta_c;
			}
			else if (oc_new > pwm_cnt_top) {
				oc_new = pwm_cnt_top;
				dcc_delta_c = -dcc_delta_c;
			}
			pwm_oc_c = oc_new;
		}
	}
	
#if 0
	/* 
	 * Check if it's time to reenable pushbutton interrupts
	 *
	 * We disable interrupts for a while after each button press, for debouncing
	 */
	if (debounce_ctr) {
		--debounce_ctr;
		if (debounce_ctr==0) {
			// re-enable interrupts
			GIMSK |= _BV(INT0);
		}
	}
#endif
	
}

#if 0
ISR(TIM0_COMPA)
{
}

ISR(TIM0_COMPB)
{

}
#endif

/*@******************************* IO Init **********************************/
void
ioinit (void)			/* Note [6] */
{
	/* 
	 * Hello, World
	 */
	HWL_DDR|=HWL_MASK;
	HWL_PORT|=HWL_MASK;
	
	/* 
	 * Set up pseudo-PWM OC pins as outputs 
	 *
	 * we know these are all the same port, but what the heck.
	 */
	PWMA_DDR|=PWMA_MASK;
	PWMB_DDR|=PWMB_MASK;
	PWMC_DDR|=PWMC_MASK;

	/* 
	 * Configure the push-button pin as an input with internal pullup
	 * Enable the low-edge interrupt on the push-button pin.
	 *
	 */
#if 0
	PBI_DDR &= ~PBI_MASK; // input (default state after reset, actually)
	PBI_PORT |= PBI_MASK; // internal pull-up resistor enabled
	MCUCR |= _BV(ISC01);  // interrupt on falling edge of INT0
	GIMSK |= _BV(INT0);
#endif
	
	
	/* 
	 * Set up the timer 0 with the hardware PWM disabled (which is the
	 * default), in "normal" free running mode.  We will use the overflow
	 * interrupt to drive the software PWM.
	 */
	
	/* Enable timer 0 overflow interrupt. */
	TIMSK0 = _BV(TOIE0);

	/* 
	 * Start timer 0.
	 *
	 * Select clock divisor 1 (8MHz), which with default TOP of 255
	 * gives overflow interrupts at a rate of about 32kHz.
	 *
	 * Clock=8MHz and TOP=255 gives an soft-PWM frequency of around 128Hz.
	 *
	 * If a higher frequency is needed, the hardware timer's TOP can be 
	 * lowered using CTC mode (mode 2).
	 *
	 * Fastest possible soft-pwm could be achieved by running in a tight
	 * delay loop instead of using tiner interrupts.
	 *
	 * Note: The rapidly changing hardware timer's TCNT value will make
	 * a nice pseudo-random number when someone presses the colour 
	 * change button.
	 */
	TCCR0B = _BV(CS00); // IO clock divisor = clk/1
	
	sei ();
}

int
main (void)
{
	ioinit ();

	/* run the PWM counters out of phase to 
	 * maybe lower peak current draw 
	 */
	pwm_cnt_a=0x00;
	pwm_cnt_a=0x60;
	pwm_cnt_a=0xB0;
	

	/* set up initial pwm values */
	pwm_oc_a	= 0x00;
	pwm_oc_b	= 0x00;
	pwm_oc_c	= 0x00;
	pwm_cnt_top	= PWM_TOP;
	
	
	/* loop forever, the interrupts are doing the rest */
	for (;;)
		sleep_mode();

	return (0);
}


//@****************************** end of lut.c *******************************

