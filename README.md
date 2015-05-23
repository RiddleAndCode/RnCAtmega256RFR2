# RnCAtmega256RFR2
The power of WPAN 802.15.4, ZigBee, 6LowPan and Thread.

Riddle&Code's Atmega256RFR2 Board Manager packagage for the Arduino IDE!

### Atmega256RFR2 Xplained Pro as universal, Arduino-compatible WPAN 802.15.4, ZigBee, 6LowPan, Thread board.
For a price slightly below a classical XBee Pro Series 2 one can get with the Atmega256RFR2 Xplained Pro board a perfect SOC solution packaging a power AVR MCU with an excellent RF chip. We've tested many Arduino-ZigBee combinations available on the market like the Pinoccio, Zigduino, Libelium including Xbee and Xbee shield, Arduino Uno with Xbee and Xbee shield and some more "esoteric solutions". So far the Xplained Pro delivered the most flexible and satisfying results. It's a remarkable versatile device.

---

#### How to use the Arduino IDE to develop with and for the Atmega256RFR2 Xplained Pro?

As the *Xplained Pro* doesn't come with an *FTDI chip* or an *Atmega16u2* to enable direct USB to serial programming of the board an ISP programmer ha to be used to upload the Arduino bootloader and the Arduino code. This has been tested with the easiest available ISP tools on the market, the Arduino ISP, an Arduino UNO, a Digispark and finally and USBTinyISP. All of them work fabulous.


The necessary ICSP pins on the Xplain Pro are as follows:

Atmega2RFR2-XPRO board to USPTinyISP programmer

1. EXT 5 : Pin 16 : PB2 : MOSI
2. EXT 5 : Pin 17 : PB3 : MISO
3. EXT 5 : Pin 18 : PB1 : SCK
4. EXT 5 : Pin 19 : GND : GND
5. EXT 5 : Pin 20 : VCC : VCC
6. SPARE SIGNALS : Pin RSTN : RST

<br /><br />
**Then with the Arduino IDE select from the Tools menu**

1. Select 'Board' -> 'RiddleAndCode Boards' -> '256RFR2XPRO'
2. Don't select a 'Port'
3. But select 'Tools' -> 'Programmer' -> 'USBTinyISP' ( or any other programmer you use!)

<br /><br />
**Please keep in mind that one has to use two commands from Sketch menu within the Arduino IDE**

1. 'Sketch' -> 'Verify / Compile'
2. 'Sketch' -> 'Upload Using Programmer'
