This is a small piece of embedded software written for (almost) any two USARTs AVR ATMega microcontroller.

It must be connected to KME Nevo (Pro) LPG ECU via USART1 and to PC/PPC/Android via COM cable or Bluetooth adapter on USART0.

It will automatically connect to KME Nevo ECU and start reading current data (both LPG and OBD in case of using Nevo Pro or, 
probably, Nevo Plus with OBD adapter).
It will immediately begin calculation of fuel consumption for both petrol and LPG. And fuel tank remainings.
It will supply all those values to PC via simple data request.
It will detect the connection of a genuine KME Nevo software and switch to "transparent" mode acting as a proxy between 
KME Nevo software and the ECU.
It will still detect the data required for fuel consumption calculations while KME Nevo software communication is active.

The work I've done reversing the KME Nevo protocol is completely legal here in Russia. As long as I'm not doing it for 
commercial purposes. Which I don't do, obviously.

I'll publish the protocol documentation and protocol simulator software if anyone are interested.

The code is licensed under... hmm... let's say MIT License:

The MIT License (MIT)

Copyright (c) 2015 Dmitry 'Creat0r' Bobrik

Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

There're some headers from AVRLib included which are licensed under GPL though...