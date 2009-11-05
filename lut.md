This project illustrates how to use an ATTiny microcontroller and an
RGB LED to create a coloured light that cyles continuously through
random colours, but changes colour dramatically at the press of a
switch. 

Why?  My kids had this $2 novelty pen that did this employing a teensy
circuit along with 3x3mm LEDs and some alkaline coin-cells.  They
broke it almost instantly, losing some critical small parts.

"Daddy, my light-up pen broke!  WAAAAAAAAAAAAAAHHH!!!"  

Having deprived themselves of the captivating coloured lights, they
then drew on the walls with the
instrument-formerly-known-as-the-light-up-pen.  Then they experimented
with strangling each other with the pen's narrow ribbon lanyard.  I
decided to replicate the pretty light without the other problems.

This circuit produces coloured light by using an 8-pin ATTiny13
microcontroller and Pulse-width-modulation
(PWM) to vary the brightness of three LEDs (Red, Green and Blue) that
are closely located, or share a common package.   I used a
common-anode 5mm RGB led, which looks just like a normal 5mm LED but
has 4 leads instead of two.

Since the ATtiny13 only has one timer with 2 PWM channels, it cannot
control 3 PWM channels in hardware.  Instead, I do the PWM logic in
software, timing the pulse modulation via a regular tick (interrupt)
from the ATTiny's hardware timer.  (Nowadays you can get the ATTiny45
and friends which have 3 PWM channels, but I could not obtain them
when I first built the prototype for my middle child a couple of years
ago).   My youngest child is now 2, so I dug this project out of the
mothball bunker for her to play with.

WARNING: modern multi-colour LEDs can be PAINFULLY bright.  If
children will play with this, choose one of the following: ensure your
LED is not a superbright, or use a diffuse led, or diffuse a clear one
(eg cover it with half a table tennis ball, or enclose it in resin or wax.
You could also use higher-value resistors to limit current, or lower
PWM_OC_MAX to limit the PWM duty-cycle (LED brightness) via software.

* Pulse-Width Modulation provides colour choice

The microcontroller fuses are configured to run it from the 8MHz
internal clock.   AVR factory default has a divide-by-8 fuse enabled
giving 1MHz, so remember to change that.

The high frequency interrupt from a hardware timer triggers an
interrupt service routine (rougly 32000 times per second) which
increments each of 3 separate 8 bit counters.  These counters, and
corresponding "Output Compare" (OC) values are used to generate 3
Pulse-Width-Modulated waveforms, with each counter being associated
with a particular output pin.  Every time a counter reaches zero, the
corresponding pin is turned on.  When the counter reaches its OC value
it is turned off.  When the counter reaches its TOP value (255) it
wraps around to zero and the cycle repeates.

The effect of this is that each of the 3 output pins is driven with a
square-wave with a wave-legnth of 256 timer ticks (about 1/128 of a
second), which is equivalent to a frequency of 128Hz
(cycles-per-second).  The OC value controls the "duty-cycle" (ratio of
on-period to off-period) of the wave, with 0 giving always off, 255
always on, and any intermediate value giving a wave that is on for
x/255 of a cycle and off for the remainder of each cycle.  If an
electric light is powered by such a square wave, with a frequency of
more than about 50Hz, the human eye does not percieve any
flickering---instead changes in the duty cycle are perceived as
changes in brightness (this is how most household dimmers work).  With
3 leds in Red, Green and Blue, and the ability to vary the perceived
brighness of each, you can produce many different colours, exactly as is done
in modern TVs and projectors.

So using 3 PWM outputs, we have a coloured light that we can set to
any brightness and colour we choose within an achievable range.

BTW, it's not strictly necessary to use separate 3 counters.  A single counter
could be used with 3 different OC values to control each output pin
independently.  I chose to implement 3 distinct channels rather than 1
channel with 3 Output-compare values, out of some voodoo idea that
running the channels out-of-phase will lower the peak current draw and
give slightly longer battery life.

* Colour change by drifting PWM parameters

A coloured light isn't very interesting.

For each of the 3 channels, at the end of every PWM cycle (when the
counter reaches 255), the duty cycle of the particular channel changes
by a small amount.  This causes the colour to change.  The rate that
each of the PWM channels changes duty-cycle is chosen semi-randomly
such that an ever changing progression of colour is produced.


* User interface

A pulsing coloured light is nice, but when you're 2, a BUTTON TO PUSH
is best of all.

If the CHANGE button is pressed, the LEDs are reset to all off (OCx=0), and
the cycle-rate of one channel is changed to a new random value, giving
an abrupt change of colour, followed by a new and different
progression of colour changes.    Small children love this and will
sit and frob the button for ages, before they break it or lose it.
Then you can make them something else.

A second button acts as an "on/off switch" (actually triggering a low-power sleep
mode, and waking from same).   

Both buttons trigger interrupts via the pin-change interrupt facility.
The buttons are "debounced" in software, by masking further interrupts
for a certain interval after any button interrupt.

* Hardware

I prototyped this on a solderless breadboard, and programmed the
ATTiny with an AVRUSB500v2 programmer
(http://www.tuxgraphics.org/electronics/200705/article07052.shtml),
which has the useful advantage of using a 1x5 connector instead of the
more common 2x3, allowing simple in-circuit programming of
breadboarded AVRs.

You can use any simple voltage regulator (or none).  I used an MCP1700
low-dropout regulator, which works off 3 watch cells.   A 78L05 and a
9v battery works fine too.

Once I had the software working, I built a soldered prototype on
proto-board.    I'd show you a picture but I think it's probably under my
son's pillow right now.

This circuit is so simple it's hardly worth bothering with a PCB.
I suppose an SMD version could be made not much larger than the coin
cells that power it, giving some interesting swallowing-hazard fun to be
had.
