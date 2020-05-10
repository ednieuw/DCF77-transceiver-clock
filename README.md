# DCFtiny-clock
This clock is still in development</p>
<p> <This clock is still in development</p>
<p><img alt="DCFklok" height="450" src="DCFklok.jpg" /></p>
This clock decodes the DCF77-signal and displays the received 
bits.</br>
The software makes use of an algorithm that collects the readings in the loop 
and does not use interrupts.</br>
In this program the processor is able to collect, in it spare time, over 30,000 per second of 0 or 1 readings.</br>
Therefore no delays can be used. The program demonstrates a method to avoid delays. </br>
An advantage of this method is it smoothens spikes in the signal.</br>
The routine is smaller than the DCF library and can be easily debugged.</br>
the effixiency is comparable with the DCF77 library of Thijs Ellenbaas.
with good reception both methods decode over 99%.</br>
Both libraries can be uses together increasing the efficiency even further.</br>
The DCFtiny algorithm was developed to drive the LEDs of the clock. The clock 
contains three rings of 8, 24 and 60 LEDs.</br>
The outer ring displays the received bits per minute. blue = 0 and red = 1.
The middle ring display the signal efficiency per hour and the current hour with 
a white LED.</br>
The inner ring displays various parameters.</br>
On top of the clock is a TM1637 display to show the current time.<br>
A MAX7219 display is used to show the pulsewidth in msec, the time as it is decoded by the algorithm<br>
and the decoding efficiency in time as a percentage.</br>
The time is transmitted in the air with a HC12 transceiver.</br>
With a Bluetooth module the clock can be operated with a phone.</br>
More info here:</br>
<a href="https://ednieuw.home.xs4all.nl/Woordklok/index.html">
https://ednieuw.home.xs4all.nl/Woordklok/index.html</a></p>
