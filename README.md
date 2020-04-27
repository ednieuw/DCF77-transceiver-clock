# DCFtiny-clock
This clock is still in development</p>
<p> <This clock is still in development</p>
<p><img alt="DCFklok" height="450" src="DCFklok.jpg" /></p>
This clock uses the DCF77-signal and display the received 
bits.</br>
The software makes use of an algorithm that collects the readings in the loop 
and does not use interrupts.
In this way it collects 10,000 - 16,000 0 or 1 readings.
The advantage of this methods ids that is smoothens spikes in the signal. It is 
smaller than a library and can be easily debugged.
It is not as good as the DCF77 library of Thijs Ellenbaas. It receives 80% of 
the signals while the DCF77 library scores 90%.
When both libraries are used together the efficiency&nbsp; is &gt;95%.
The DCFtiny algorithm was developed to drive the LEDs of the clock. The clock 
contains three rings of 8, 24 and 60 LEDs.
The outer ring displays the received bits per minute. blue = 0 and red = 1.
The middle ring display the signal efficiency per hour and the current hour with 
a white LED.
the inner ring display various parameters.
On top of the clock is a TM1637 display to show the current time and a MAX7219 
is used to show the pulse with in msec the time as it is received and the 
efficiency as a percentage of the DCF algorithms.
The time is transmitted in the air with a HC12 transceiver.
With a Bluetooth module the clock can be operated with a phone.
More info here:
<a href="https://ednieuw.home.xs4all.nl/Woordklok/index.html">
https://ednieuw.home.xs4all.nl/Woordklok/index.html</a></p>
