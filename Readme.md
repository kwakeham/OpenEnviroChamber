# Parts
- Arduino Leonardo (Uno or similar should work)
- Heater Power Fet: PSMN1R1-30PL (1.3mohm) or similar
- Fan Power Fet: P16NF06L or similar
- Heatsinks: Ohmite EV-T220-38E This mounts
- 2x XT60 connector pairs
- 2x "JST" 2 pin (I know JST is a brand but that's how they come up on Aliexpress or Jungle site)
- Adafruit MAX31855 Breadout Board (or similar but the mount won't line up)
- 2 x Bar magnets that fit in a 59 x 10mm trench (I think they were imperial)

- Top and Enclosure 3d printed
- M3 Inserts for the arduino mount
- 2 x 2mm self tapping screws for the MAX31855
- 12v Powersupply (min 5 amps)
- 50W PTC heater with fan (Jungle site)
- SSR for the compressor

- Thick Silicone wire for the 12v, enough to handle 5A but more for safety

# schematic
![Image of schematic] (https://github.com/kwakeham/OpenEnviroChamber/blob/master/Fritzing/EnvironmentalChamber_sm.jpg)

# Notes
I soldered wires to the back of the Arduino and modules because the push wires tend to be unreliable. I can't have a device that uses high power 12v and 120v AC going on the fritz. I suggest you do the same. Means debuging is easier