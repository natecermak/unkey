# Acoustic Underwater Text-based Communication Device (Name TBD)

## Purpose/Motivation
Divers often need to communicate with one another or with a dive boat, but typically can only communicate when in visual range of each other. A simple means to communicate with a bandwidth of text messaging (~40 wpm) over distances of several hundred meters could potentially save lives when if other divers or the dive boat can be made aware of and respond to unexpected conditions/situations.

## Communication scheme overview
Acoustic communication is basically the only feasible scheme underwater, and hence has been the standard for underwater communication since at least the early 1900s. Electromagnetic signals penetrate very poorly underwater, so convential means of communication (cellular communication, radio/walkie-talkies, bluetooth, wifi) are not feasible. Optical signaling is sometimes feasible (e.g. with flashlights), but relies on line of sight and good water clarity, and has limited range due to scattering and absorption underwater. However, if one has ever heard dolphins or whales underwater, they can attest to the immense distances these signals can travel, greatly exceeding the distance one can see underwater.

Underwater environments (a) add a lot of noise and (b) distort signals due to reflections.

Here, we use a **FM/AM/OFDM** scheme to send bits.

## Subsystems:

### Keyboard
To get around difficulties with mechanical keys, we chose to use magnetic sensors and a magnet-bearing stylus. Underwater, mechanical keys have several failure modes:
 - They can break in such a way that allows water into the device, and
 - As one goes underwater, pressure increases, which eventually can press keys on its own, at which point buttons are stuck in the 'pressed' state, and basically useless.
We chose to use omnipolar Hall-effect switches, which have a digital output that changes from high to low when the magnetic flux through the device increases beyond some threshold. We rapidly read out every key's on/off state using an array of daisy-chained shift registers, an approach commonly used in other keyboards. When the user brings the magnet-tipped stylus near a sensor "key", the microcontroller can read the change in the Hall-effect switch output.

Users typically prefer some tactile feedback when they hit a key, and this system doesn't naturally provide that. As such, we experimented with adding a soft (low coercivity) magnetic material (1-2mm mild steel ball bearings) above the Hall-effect sensor, to provide a small pull-in magnetic force when the stylus is brought close to the key. This provides tacticle sensation to the user that they have actually touched a key. It's essential that this material is magnetizable so that it attracts the stylus, but also that it cannot develop a permanent magnetic moment. If it were to develop a permanent magnetic moment, it would turn the Hall-effect sensor on all the time and the key would be stuck "on".

Keyboard layout:
---------- Default, no modifier -------
1   2   3   4   5   6   7   8   9   0
q   w   e   r   t   y   u   i   o   p
  a   s   d   f   g   h   j   k   l   DEL
CAP z   x   c   v   b   n   m   ?   BCK
SYM      SPACESPACESPACESP   ,   .    RET
(plus 6 arrow keys, SEND)

---------- SYM modifier ---------------
!   @   #   $   %   ^   &   *   (   )
`   ~   -   _   =   +   :   ;   '   "
  [   ]   {   }   |   \   /   <   >   DEL
CAP z   x   c   v   b   n   m   ?   BCK
SYM      SPACESPACESPACESP   ,   .    RET

key count is:
    row 1: 10 keys
    row 2: 10 keys
    row 3: 10 keys
    row 4: 10 keys
    row 5: 5 keys
    off-grid: 6 arrow keys, 1 SEND key
Total: 52 keys

Non-character keys: 6 arrow keys, SEND, BCK, DEL, CAP, SYM

Future work could use open-drain hall-effect sensors in a grid to give more precise hit-boxes (e.g. a 2x2 grid of AH1925-HK4-7, wired in parallel, for each key). However, this would be significantly more expensive, as it would require roughly 4x as many switches.

### Microcontroller
We use a Teensy 4.1 microcontroller due to ease of development. Essential features this microcontroller provides are:
 - rapid (~100 ksps) ADC readout via DMA
 - SPI peripheral for driving a TFT LCD scree
n - DMA-compatible SPI peripheral for driving a DAC
 - I2C peripheral for controlling an external switch matrix
However, many other microcontrollers would likely work fine for this system.

### Battery
We use a rechargeable lithium ion battery, which provides a nominal 3.7V input.

### Transducer
We use a 45mm x 45mm x 1.4mm square piezo plate from Steminc (SMPL45W45T14111) as a transducer. This piezo is spec'd as having a capacitance of 5.4nF +- 20%. It's made from material SM111, which has a piezo d33 constant of 320e-12 C/N.

We made some measurements of the impedance at various frequencies:
    - Frequency   Real      Imaginary     (Capacitance, Q)
    -  0.1 kHz:   570    -  103.5k imag   (15.3nF, Q=182)
    -  1.0 kHz:    45    -   10.42k imag  (15.3nF, Q=236)
    - 10.0 kHz:     2.9  -    1.03k imag  (15.4nF, Q=356)
    - 40.0 kHz:     2.25 -       63 imag  (63.0nF, Q=28)
    -100.0 kHz:    -0.09 -      132 imag  (12nF,   Q=1350)

Overall, this seems pretty discrepant with their claim of 5.4nF. Given two parallel plates of this size, separated by 1.4mm, and a dielectric constant of 1400 (also from the [SM111 materials page](http://www.steminc.com/piezo/PZ_property.asp)), the calculated capacitance should be 17.9nF, closer to our measured value than their spec.

We also purchased several other transducers to try out, but have not yet used/characterized ther others:
- SMD25T14F15111 - Piezo Ceramic Disc 25x1.4mm R 1.5 Mhz          1.9 nF
- SMD50T21F1000R - Piezo Electric Disc 50x2.1mm R 1 MHz           10 nF

### Transducer driver

### Transducer sensor
We use a charge amplifier circuit with a configurable gain.

### Sensor data processing
We use Goertzel filters to extract amplitude and phase at a fixed set of frequencies.

### Screen
TBD.

### Enclosure
TBD.

## Version history:
v1.2 PCB changes:
 - [X] switch from teensy 4.1 to t4 for space
 - [X] add battery monitor circuitry
 - [X] replicate LiIon charger circuit on it https://github.com/adafruit/Adafruit-USB-LiIon-LiPoly-Charger-PCB/tree/master
 - [X] switch shift registers to qfn
 - [X] redo keyboard layout
 - [X] add 40-pin ZIF connector for 3.2" TFT from buydisplay.com
 - [X] make all passives 0402 for consistency
 - [X] add explicit space for screen
 - [X] add explicit space for battery
 - [X] add explicit space for Qi charger coil + PCB
 - [X] pull LAT down on MCP4, or just wire it out -- cant leave it floating
 - [X] CE on 5V boost wont work! needs to be always enabled, and drive a switch
 - [X] figure out FMARK and RD on ZIF connector for display: tested with them floating and display still works, so non-essential

v1.3 pcb
 - [X] split board into two separate parts with explicit design for overall assembly. This is for two reasons:
    1. so that I can do single-sided assembly on both pieces, but then have some parts sticking up (keyboard) and some facing down (teensy/driver electronics)
    2. it makes the face of the screen flush with the face of the magnetic switches.
 - [X] fix TFT connector pin numbering (it's backwards from the connector)

TODO:
 - [ ] make an attempt at a BOM

