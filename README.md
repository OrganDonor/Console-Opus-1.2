# Console-Opus-1.2

Arduino code for the improved 12-button console for Opus 1 with Organelle.

## Hardware Configuration

The improved console contains an Arduino MEGA 2560 controller board. Three Sparkfun MIDI breakout boards (BOB-09598) are connected to Serial1, Serial2, and Serial3. Twelve momentary pushbuttons with integral LED lights are connected to individual pins on the Mega.

The inputs of Serial1 and Serial2 are to be connected, through MIDI breakout boards, to two standard MIDI keyboards. Serial3 is to be connected, through another MIDI breakout board, to the MIDI input and output of a USB MIDI adapter connected to the Raspberry Pi in the Organelle. 

The output of Serial2 is connected through a long umbilical cable to the windchest, which contains two daisy-chained MIDI-to-Parallel converter boards, MTP-8 from J-Omega Electronics, which are in turn connected to the air valves from Peterson Electro-Musical Products, Inc.

## Console Functional Description

The console's job is to combine the two MIDI streams from the two keyboards (manuals) into a single MIDI stream for the MTP8 converters. It limits the number of concurrent notes played, because the MTP8 boards have a limited current sinking capability. The MTP8 boards are protected by external fuses, but in order to avoid blowing fuses, the console software must limit the number of notes the organ tries to play simultaneously.

MIDI messages from the keyboards are also relayed to the Organelle for processing, unless this function is switched off by the Organelle using a custom defined SysEx MIDI command.

A big pipe organ will have lots of stops (which connect ranks of pipes to manuals) and couplers (which either cross-connect ranks or connect ranks with octave offsets). Organ Donor Opus 1 has just two ranks of pipes (8' and 4') and two manuals (called Great and Swell), and its stops and couplers are implemented using the twelve pushbuttons. The pushbuttons do not have text labels, but they are mounted on a panel laser engraved with an explanatory diagram.

The pushbuttons appear in four groups of three. Each group of three controls the mapping of one manual onto one rank of pipes. The center pushbutton in each group (a "stop") is the master, so if it is off, there is no mapping from that manual to that keyboard. If it is turned on, that rank will sound the note played on that manual. In addition, if the right pushbutton in the group is turned on, that rank will sound the note an octave higher than the note played (an "octave coupler"), and if the left pushbutton in the group is turned on, that rank will sound the note an octave lower than the note played (a "suboctave coupler).

This twelve-button arrangement of stops and couplers is somewhat more flexible than the eight-button arrangement on the original ten-button console. The hope is that it will also be easier to explain to users, especially with the nice diagram engraved on the panel.

The improved console doesn't have any button equivalent to the MIDI button on the original console. Instead, decisions about whether an external MIDI device is used (and which device if there are several) are left up to the Organelle (the Raspberry Pi in the other compartment of the console furniture). MIDI commands arriving from the Organelle are always relayed to the MTP8 converters.

## Dependencies

We use the standard MIDI library for Arduino, https://github.com/FortySevenEffects/arduino_midi_library/
