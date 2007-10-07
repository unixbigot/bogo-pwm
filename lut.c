/*@****************************** RCS HEADER ********************************/
/*
 * $RCSfile: lut.c,v $
 * $Author: chris $
 * $Date: 2007-10-07 11:47:15 $
 * $Revision: 1.3 $
 *
 */ 
//@********************************* lut.c ***********************************
/*
 * Light Up $THING  --  LED lightshow using RGB LED with software PWM
 *
 * The goal is to create a coloured light that cyles gradually through random 
 * colours, but changes colour instantly at the press of a switch.
 *
 * Why?   My kids had this $2 toy pen that did this with 3 SMD LEDs and some coin-cells.
 *        They broke it almost instantly, plus, they wrote on themselves with the pen.
 *
 * Since the ATtiny13 only has one timer with 2 pwm channels, we do the PWM logic in 
 * software, using a regular tick from the hardware timer0.
 *
 * If I could get the ATtiny[248]5 easily in Oz (3 PWM channels), I'd probly just use 
 * that instead...
 *
 * THEORY OF OPERATION:
 *
 * Lowest level:
 *     3 independent PWM channels generate waveforms that drive an RGB LED.
 *     The 3 PWM duty cycle values define the colour.
 *     (I use 3 channels rather than 1ch with 3 OC pins, out of some voodoo idea that
 *     running the channels out-of-phase will give more even current draw)
 * Colour change level:
 *     Every CYCLERATEx PWM cycles, the duty cycle of channel X changes by a small amount.
 *     If a button is pressed, the cycle-rate of one channel is changed to a new random value.
 * 
 */ 

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*@**************************** forward decls *******************************/

void pwm_setup(uint8_t ch_id, uint8_t pin, uint8_t cnt, uint8_t oc, uint8_t mut_interval, int8_t mut_delta);

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
// this is used for testing, and omitted in deployment
// note, if enabling this, disable button input that shares same pin
#if 0
#define HWL_DDR    DDRB
#define HWL_PORT   PORTB
#define HWL_PIN    PINB
#define HWL_MASK   _BV(PB0)
#endif

// soft-power input is PB0
// We use a pin-change interrupt to wake from sleep 
// Shares pin with HWL, so do not enable if HWL in use
#if 0 && !defined(HWL_PIN)
#define PWR_DDR    DDRB
#define PWR_PORT   PORTB
#define PWR_PIN    PINB
#define PWR_MASK   _BV(PB0)
#define PWR_INTSETUP() PCMSK|=PWR_MASK;
#define PWR_INTON()  PCMSK|=PWR_MASK; GIMSK|=_BV(PCIE)
#define PWR_INTOFF() PCMSK&=~PWR_MASK;
#else
#define PWR_INTSETUP() 
#define PWR_INTON()  
#define PWR_INTOFF() 
#endif

// push-button-input is PB1 (INT0)
// We use an edge-triggered interrupt to detect button press
#define PBI_DDR    DDRB
#define PBI_PORT   PORTB
#define PBI_PIN    PINB
#define PBI_MASK   _BV(PB1)
#ifdef PBI_BIN
#define PBI_INTSETUP() 	MCUCR |= _BV(ISC01)
#define PBI_INTON()  GIMSK|=_BV(INT0)
#define PBI_INTOFF() GIMSK&=~_BV(INT0)
#else
#define PBI_INTSETUP() 	
#define PBI_INTON()  
#define PBI_INTOFF() 
#endif

// soft-PWM output A is PB2, pin 7
// soft-PWM output B is PB3, pin 2
// soft-PWM output C is PB4, pin 3
#define PWM_DDR  DDRB
#define PWM_PORT PORTB
#define PWM_PIN  PINB


#else
#error unsupported chip
#endif



/*@********************************* data ***********************************/
 /* 
 *@@ register-bound variables
 * 
 */
// Counter limits
//
// pwm_cnt_top = top value for pseudo-timers.  
//
//	Don't necessarily need 8-bit pwm for LEDs, 
//	might get away with, say, 5-bits (32 brightness levels)
// 
//	Lowering this value gives a faster PWM frequency, 
//	at expense of less colour resolution
//
// pwm_oc_max = maximum LED brightness
//
//	If using high-brightness LEDs, they may be TOO bright.
//
//	Either use a higher resistor value, or change PWM_OC_MAX
//	to prevent running LEDs at full-brightness
//
#define PWM_TOP		0xFF
#define PWM_OC_MAX	0xFF
#if (PWM_OC_MAX>PWM_TOP)
#error Invalid PWM_OC_MAX
#endif

/*register*/ uint8_t pwm_cnt_top /*asm("r2")*/; 
/*register*/ uint8_t pwm_oc_max /*asm("r3")*/;

/* 
 *@@ RAM variables 
 */

// counter values for the pwm pseudo-timers.
//
// We /could/ get away with a single counter and merely use
// three separate OC values, but by running three complete 
// pseudo-timers 'out of phase' (different initial cnt values), 
// peak  current draw may be reduced.
//
// That is, when duty cycle is <1/3 the 3 LEDs will never be on 
// at same time.
//
// We simulate the AVR hardware timers' "inverted fast PWM" mode.
// (Inverted because I am using a common-anode LED array)
//
// Counter starts at zero and counts to TOP, then goes back to 0
// Overflow event (pseudo-interrupt) occurs when timer passes TOP
// Output pin is set at 0, and cleared when CNT==OC
//
// pwm_cnt  = the PWM counter value, ranges from 0 to pwm_cnt_top
// pwm_oc   = the PWM output-compare value (duty cycle) [0..pwm_oc_max]
// 
// mut_interval = mutate OC every N overflows
// mut_delta    = value to add to OC at mutation time
// mut_ctr      = number of overflows since last mutation
//

typedef struct _pwm_ctr_s 
{
	uint8_t pwm_cnt;
	uint8_t pwm_oc;
	uint8_t pin_mask; // bitmask in PWM_PORT for this channel

	// colour cycle mode
	uint8_t mut_interval; // change only every N overflows (0=do not change)
	int8_t mut_delta;     // value to add to OC after each change interval
	uint8_t mut_ctr;      // number of overflows since last change
} pwm_ch_t;

#define CH_RED 0
#define CH_GRN 1
#define CH_BLU 2
#define CH_MAX 3 

pwm_ch_t SPWM[CH_MAX];
	

//@@@ power state
//
// Set to 0 when we go to deep-sleep mode
//
#ifdef PWR_PIN
uint8_t pwr_state = 1;
uint8_t pwr_down = 0;
#endif

//@@@ debounce interval - number of PWM cycles for which input is ignored 
//                        after any button press
//
// At button press, button interrupt is disabled and debounce_ctr set to
// DEBOUNCE_INTERVAL
//
// At Every overflow event, debounce_ctr is decremented.  When it reaches
// zero, the button interrupt is re-enabled
//
#define DEBOUNCE_INTERVAL 32
uint8_t debounce_ctr;

/*@******************************** MACROS **********************************/

#define HANG 	cli(); for(;;) _delay_ms(100);

/*@***************************** subroutines ********************************/
/*@@---------------------- initialization routines -------------------------*/
/* 
 *@@@ ioinit - set up input/output pin configurations
 */
void
ioinit (void)			/* Note [6] */
{
	/* 
	 * Configure the push-buttons pin as input with internal pullup
	 *
	 * INT0 pin will use low-level interrupt
	 * Other pins will use pin-change interrupt
	 *
	 */
#ifdef PWR_PIN
	pwr_state = 1;        // we are currently [ON].  Duh.
	PWR_DDR &= ~PWR_MASK; // input (default state after reset, actually)
	PWR_PORT |= PWR_MASK; // internal pull-up resistor enabled
	PWR_INTSETUP();
	PWR_INTON();
#endif

#ifdef PBI_PIN
	PBI_DDR &= ~PBI_MASK; // input (default state after reset, actually)
	PBI_PORT |= PBI_MASK; // internal pull-up resistor enabled
	PBI_INTSETUP();
	PBI_INTON();
#endif
	
	/* 
	 * Set up the timer 0 with the hardware PWM disabled (which is the
	 * default), in "normal" free running mode.  We will use the overflow
	 * interrupt to drive the software PWM.
	 *
	 * Select clock divisor 1 (8MHz), which with default TOP of 255
	 * gives overflow interrupts at a rate of about 32kHz.
	 *
	 * Interrupts at 32kHz and pwm_cnt_top=255 gives an soft-PWM frequency of around 128Hz.
	 *
	 * If a higher frequency is needed, the hardware timer's TOP can be 
	 * lowered using CTC mode (mode 2), or pwm_cnt_top can be changed.
	 *
	 * Fastest possible soft-pwm could be achieved by running in a tight
	 * delay loop instead of using timer interrupts.
	 *
	 * Note: The rapidly changing hardware timer's TCNT value will make
	 * a nice pseudo-random number when someone presses the colour 
	 * change button.
	 */

	/* Enable timer 0 overflow interrupt. */
	TIMSK0 = _BV(TOIE0);
	/* Start timer 0. */
	TCCR0B = _BV(CS00); // divisor==1
	//TCCR0B = _BV(CS01); // divisor==8
	//TCCR0B = _BV(CS01)|_BV(CS00); // divisor==256
	
}

/* 
 *@@@ hwl_init - hello_world pin setup
 * 
 * Control a simple on-off led used as a debugging aid.
 *
 * Not present in deployed version
 */
#ifdef HWL_PIN
#define HWL_ON() HWL_PORT|=HWL_MASK
#define HWL_OFF() HWL_PORT&=~HWL_MASK
#define HWL_FLIP() HWL_PIN|=HWL_MASK
inline void hwl_init() 
{
	HWL_DDR|=HWL_MASK;
	HWL_ON();
}
#else
#define HWL_ON() 
#define HWL_OFF() 
#define HWL_FLIP()
#define hwl_init()
#endif

/* 
 *@@@ pwminit - set up software pwm counters
 * 
 */
void pwminit(void) 
{
	pwm_cnt_top	= PWM_TOP;    // PWM resolution vs frequency
	pwm_oc_max	= PWM_OC_MAX; // sprag max brightness for safety

	/* 
	 * Set up initial pwm state
	 *
	 * run the PWM counters out of phase to maybe lower peak current draw 
	 *
	 * If all 3 leds are at less than 1/3 brightness, they will never be 
	 * on at the same time!
	 *
	 * Start with all LEDs off (duty cycle 0%)
	 *
	 * For each channel SPWM[i] the LED brightness (pwm_oc) will change
	 * by mu_delta once every mut_interval overflows
	 *
	 * This produces a shifting colour display, esp when the values are
	 * different for each channel.
	 * 
	 * A button press should modify one or more of these values to produce a new
	 * colour
	 *
	 */
	pwm_setup(CH_RED, PB2, 0x00, 0x20, 2, -1);
	pwm_setup(CH_GRN, PB4, 0x55, 0x10, 3, 1);
	pwm_setup(CH_BLU, PB3, 0xAA, 0x30, 1, -1);
}

/*@@---------------------- application subroutines -------------------------*/
/* 
 *@@@ pwm_setup - initialize pwm table entry
 * 
 */
void pwm_setup(uint8_t ch_id, uint8_t pin, uint8_t cnt, uint8_t oc, uint8_t mut_interval, int8_t mut_delta)
{
	pwm_ch_t *pc = &(SPWM[ch_id]);
	
	pc->pwm_cnt = cnt;
	pc->pwm_oc = oc;
	pc->pin_mask = _BV(pin);
	
	pc->mut_interval = mut_interval;
	pc->mut_delta = mut_delta;
	pc->mut_ctr = 0;
	
	PWM_DDR |= pc->pin_mask;   // configure pin as output
	PWM_PORT |= pc->pin_mask;  // LED off (sink mode)
}

/* 
 *@@@ pwm_overflow - handle overflow of a software-pwm channel's counter
 * 
 */
void pwm_overflow(uint8_t ch_id) 
{
	int16_t oc_new;
	int8_t delta_new = 0;
	pwm_ch_t *pc = &(SPWM[ch_id]);
	
	/* 
	 * The counter for SPWM channel 'i' has overflowed.  Do 'colour drift' action
	 *
	 */
	if (pc->mut_interval == 0)
		return;
	
	pc->mut_ctr++;
	if (pc->mut_ctr >= pc->mut_interval) 
	{
		// 
		// time to bump the duty cycle for this channel by a
		// small bit
		//
		pc->mut_ctr = 0;
		oc_new = pc->pwm_oc;
		oc_new += pc->mut_delta;

		if (oc_new <= 0) {
			oc_new = 1;
			delta_new = 0 - pc->mut_delta;
		}
		else if (oc_new > pwm_oc_max) {
			oc_new = pwm_oc_max;
			delta_new = 0 - pc->mut_delta;
		}
		pc->pwm_oc = oc_new & 0xFF;
		if (delta_new != 0)
			pc->mut_delta = delta_new;
	}
}

/* 
 *@@@ colour_jump - make a discontinuous colour change, and alter rate of colour cycling
 */
 void colour_jump(void)
{
	uint8_t i, tc;
	pwm_ch_t *pc;

	// Use (TCNT0 mod 3) as a random value to determine which LED to mogrify
	// Don't actually care about real division, so use bogo-mod
	tc = TCNT0&0x03; 
	while (tc>=CH_MAX) tc-=CH_MAX;
	
	/*
	 * Turn off all LEDs, and randomly change the drift-rate of one of the PWM channels.  
	 *
	 * This should yield a new colour cycle with a different variation.
	 *
	 * This routine is also called at power-down.
	 **/
	for (i=0; i<CH_MAX; i++) {
		pc = &(SPWM[i]);

		PWM_PORT |= pc->pin_mask;   // Pin high => LEDs off (sink mode)
		pc->pwm_oc = 0;

		if (i!=tc) 
			continue;

		// Use high-bit of TCNT0 as a random value to determine which LED to mogrify

		if (TCNT0 & 0x80) 
			pc->mut_delta >>= 1;
		else
			pc->mut_delta <<= 1;
	
		if (pc->mut_delta == 0) 
			pc->mut_delta = 1;
	}
}

/*@********************** Interrupt Service routines ************************/
/* 
 *@@ Hardware timer overflow event 
 */
ISR(TIM0_OVF_vect)	
{
	uint8_t i;
	uint8_t debounce_check = 0;
	pwm_ch_t *pc;

	/* 
	 * We got a timer overflow interrupt.  Time to bump our software-PWM
	 * counters, and test the output compares.
	 *
	 * We simulate inverting fast PWM mode (common-anode LEDs, in sink mode).  
	 *
	 * OC pin is cleared at BOTTOM (led ON) and set  at OC match (LED off).  
	 *
	 * So PWM value 0 is "always off", PWM value 255 is (almost) always on.
	 */

	for (i=0; i<CH_MAX; i++) 
	{
		pc = &(SPWM[i]);
		
		//PWM_PIN |= pc->pin_mask;

		++pc->pwm_cnt;
		if (pc->pwm_cnt > pwm_cnt_top) pc->pwm_cnt = 0;
		if (pc->pwm_cnt == pc->pwm_oc) {
			PWM_PORT |= pc->pin_mask;	// set output pin (LED off) when ctr==OC
		}
		else if (pc->pwm_cnt == 0) {
			PWM_PORT &= ~(pc->pin_mask); // clear output pin (LED on) when ctr==BOTTOM
			pwm_overflow(i);
			if (i==0) debounce_check = 1;  // use CH0 overflow
						       // to clock debounce timer
		}
	}

	/* 
	 * Check if it's time to reenable pushbutton interrupts
	 *
	 * We disable interrupts for a while after each button press, for debouncing
	 */
	if (debounce_check && debounce_ctr && (--debounce_ctr == 0))
	{
		// re-enable interrupts
		PWR_INTON();
		PBI_INTON();
	}
}

/* 
 *@@ PCINT0 - Power button (via general-purpose pin change) interrupt
 * 
 * The only pin we ever enable for PCINT0 is the power button, so we don't need to 
 * check /which/ pin triggered the interrupt.
 */
#ifdef PWR_PIN
ISR(PCINT0_vect) 
{
	/* 
	 * something changed on a monitored pin.
	 *
	 * In this application, this can only be the power button
	 *
	 * Power button is active-low, we go to power-off at the RELEASE event,
	 * cos powering off at the PRESS event would power us right back on again when
	 * released!
	 *
	 * If power is OFF, we'll take whatever event we can get 
	 */
	if (pwr_state == 1) 
	{
		if ((PWR_PIN & PWR_MASK) == 0) 
		{
			// button pressed, ignore (wait for release event)
			pwr_down = 1;
			return;
		}
		if (!pwr_down) {
			// FIXME: I suspect spurious initial interrupt.
			// Can't happen, but seems to?
			return;
		}
		pwr_down=0;

		/* 
		 * Turn off (deep sleep mode)
		 */
		colour_jump(); // ALL LEDs off
		pwr_state = 0;
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_mode();
		
		// FALL THRU to power-on case
	}
	
	if (pwr_state == 0) // not an else-case!
	{
		/* 
		 * Turn on (wake from sleep)
		 *
		 * We (probably) went to sleep from inside the power-button interrupt (this routine), 
		 * and we were awoken by another instance of the same interrupt, so we
		 * would actually execute part of this function TWICE on powerup
		 *
		 * (disabling pin-change interrupts via debounce counter will avert the 2nd case)
		 */
		pwr_state = 1;
		set_sleep_mode(SLEEP_MODE_IDLE);
		PWR_INTOFF();
		debounce_ctr = 0xFF; // maximum debounce for power events
	}
}
#endif

/* 
 *@@ Button-press event
 */
#ifdef PBI_PIN
ISR(INT0_vect)
{
	char i;
	
	/* 
	 * We got a negative-edge on the INT0 pin, i.e. colour button press
	 *
	 * Do a "colour change" procedure
	 */	
	colour_jump();
	for (i=0;i<5;i++) _delay_ms(100);

	/*
	 * Disable further INT0 events for DEBOUNCE_INTERVAL soft-PWM cycles.
	 *
	 * This is done to aid debouncing of the pushbutton inputs.
	 */
	PBI_INTOFF();
	debounce_ctr = DEBOUNCE_INTERVAL;
}
#endif


/*@********************************* main ***********************************/
int
main (void)
{
	/* 
	 * Hello, World
	 */
	hwl_init();

	/* set up application IO resources */
	ioinit ();
	pwminit();

	// Enable interrupts, go!
	sei();
	
	/* loop forever, the interrupts are doing the rest */
	for (;;)
		sleep_mode();

	return (0);
}

//@****************************** end of lut.c *******************************
