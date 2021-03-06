#include <MIDI.h>
// MIDI Controller for Organ Donor Opus 1.2
// Two manuals and two ranks, control panel with Organelle (12 buttons)
// Requires Arduino MEGA 2560 board and Arduino >=1.5.8 (or at least not 1.0.6)
// 2014-10-07 ptw created.
// For change history, see https://github.com/OrganDonor/Console-Opus-1.2

// Pin assignments for the control console:
#define PIN_GREAT4MAIN  39
#define PIN_GREAT4SUB   41
#define PIN_GREAT4SUPER 37
#define PIN_SWELL4MAIN  27
#define PIN_SWELL4SUB   29
#define PIN_SWELL4SUPER 25
#define PIN_GREAT8MAIN  45
#define PIN_GREAT8SUB   47
#define PIN_GREAT8SUPER 43
#define PIN_SWELL8MAIN  33
#define PIN_SWELL8SUB   35
#define PIN_SWELL8SUPER 31

#define PIN_GREAT4MAIN_LED  38
#define PIN_GREAT4SUB_LED   40
#define PIN_GREAT4SUPER_LED 36
#define PIN_SWELL4MAIN_LED  26
#define PIN_SWELL4SUB_LED   28
#define PIN_SWELL4SUPER_LED 24
#define PIN_GREAT8MAIN_LED  44
#define PIN_GREAT8SUB_LED   46
#define PIN_GREAT8SUPER_LED 42
#define PIN_SWELL8MAIN_LED  32
#define PIN_SWELL8SUB_LED   34
#define PIN_SWELL8SUPER_LED 30

#define PIN_DEBUG_ENABLE  2
//boolean debugMode;

// Control console is a bunch of switches, which are read into these flags.
boolean flag4GreatMain, flag4SwellMain, flag8GreatMain, flag8SwellMain;	// enable each rank on each manual
boolean flag4GreatSub, flag4SwellSub;		// sub-octave couplers for 4' rank
boolean flag4GreatSuper, flag4SwellSuper;	// super-octave couplers for 4' rank
boolean flag8GreatSub, flag8SwellSub;		// sub-octave couplers for 8' rank
boolean flag8GreatSuper, flag8SwellSuper;	// super-octave couplers for 8' rank

// Other flags can be set by a SysEx command from the Organelle
byte    flagMidi = 1;		// enable external MIDI input (0 means no external MIDI;
                    		//                             1 means map everything to rank 1;
                        	//                             2 means map channel 1 and 2 to rank 0 and 1.
                            //                             Defaults to 1.
boolean flagKBecho = true;  // enable keyboard echo to the Organelle (defaults to enabled)
boolean flagGhostBuster = true;	// enable constant stream of note_off to silent phantom notes
// etc.
//
// SysEx format:
//  Byte#   Value     Meaning
//    0       F0      Start of SysEx command, defined by MIDI standard
//    1       7D      Manufacturer code reserved for "educational use"
//    2       55      my command code for setting the flags
//    3     0,1,2     flagMidi
//    4    0 or 1     flagKBecho
//	  5    0 or 1     flagGhostBuster
//    etc. for more flags
//    N       F7      End of SysEx command, defined by MIDI standard

#define SYSEX_CMD_SETFLAGS 0x55
#define SYSEX_BASE_SIZE    4      // size of a SysEx with just the command code in it
#define SYSEX_CMD_OFFSET   2      // index of the command code
#define SYSEX_DATA_OFFSET  3      // index of first byte after the command code

#define SYSEX_CMD_FLASH    0x50     // debug feature to flash the console from Organelle

// When we relay key events to the Organelle, we include information about which manual
// was hit by using different channel numbers for the two manuals.
#define GREAT_CHANNEL 2
#define SWELL_CHANNEL 1

// There are three hardware MIDI shields connected, to Serial1 and Serial2 and Serial3.
// Each has the MIDI IN and OUT connectors installed, but not the THRU connector (for
// the Opus 1.2 console).
// The IN connectors on Serial1 and Serial2 are wired up to the two manuals (keyboards),
// which are traditionally called Great and Swell.
// The IN and OUT connectors on Serial3 are connected to a USB MIDI adapter on the
// Organelle (Raspberry Pi). The Organelle can generate its own MIDI commands to the
// console and can also manage external MIDI sources. The output to the Organelle, when
// enabled, echoes all notes played on the local keyboards.
// The OUT connector on Serial1 is the output to the optional kiosk.
// The OUT connector on Serial2 is the output to the Organ Donor windchest (two J-Omega
// MTPs chained).
//
// In addition to the three MIDI shields, the default debug serial port (Serial) was once
// connected through a voltage level converter, bidirectionally, to the serial port
// on the Organelle's Raspberry Pi. This could be used for debug data, but the intent
// was to use it as another MIDI-like port, though it is not electrically MIDI, and it
// turns out that the Raspberry Pi's serial port can't easily do the standard MIDI
// baud rate. In any case, we don't use this anymore. We leave Serial free for programming
// via a USB cable connected to the Arduino's USB port.

#define  midiOrgan  midi2   // output to the MTPs
#define  midiKiosk  midi1   // output to the optional kiosk

#define  midiGreat  midi1		// input from lower keyboard
#define  midiSwell  midi2		// input from upper keyboard
#define  midiExt    midi3		// input from external MIDI controller (now the Organelle)
#define	 midiElle   midi3		// output to the Raspberry Pi (Organelle)

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midi1)
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midi2)
MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, midi3)


// The kiosk is configured to handle input on only a single MIDI channel. We will send
// both ranks to that channel, and accept that this won't be correct when both ranks
// are being played independently.
#define  KIOSK_CHANNEL  1

// Each rank supports exactly 61 notes.
#define  RANKS            2
#define  RANK0  0      // 8' rank
#define  RANK1  1      // 4' rank

// Ranks are 0-based (so we can use array indexing) but MIDI Channels start at 1.
#define  channelForRank(x)   (x+1)
#define  rankForChannel(x)   (x-1)

// But we limit how many are sounded simultaneously over all ranks
#define  MaxAllowedNotes  24

// Keep track of all the ways each note has been requested on,
// as a bitmap in a byte for each possible MIDI pitch.
#define  NOTE_PRIME       0x01
#define  NOTE_SUB         0x02
#define  NOTE_SUPER       0x04
#define  NOTE_RANK_PRIME  0x08
#define  NOTE_RANK_SUB    0x10
#define  NOTE_RANK_SUPER  0x20
// We'll use 128 bytes here even though only 61 are valid. Easier.
byte noteIsOn[RANKS][128];

// Count of how many notes are currently active on the organ.
byte noteCount = 0;

// Initialization or Panic: turn everything off right now.
void allOff(void)
{
  byte rank, pitch;
  
  for (pitch=0; pitch < 128; pitch++)
  {
    for (rank=0; rank < RANKS; rank++)
    {
      midiOrgan.sendNoteOff(pitch, 0, channelForRank(rank));
      noteIsOn[rank][pitch] = 0;
    }
  }
  
  noteCount = 0;
}

/*
// send a redundant Note Off command to each note that isn't supposed to be playing right now.
// Never used this, for fear of the big blob of commands all going out at once and displacing
// real notes. Switched to incrementalReOff() instead.
void reOff(void)
{
  byte rank, pitch;
  
  for (pitch=0; pitch < 128; pitch++)
  {
    for (rank=0; rank < RANKS; rank++)
    {
      if (0 == noteIsOn[rank][pitch])
      {
        midiOrgan.sendNoteOff(pitch, 0, channelForRank(rank));
      }
    }
  }
}
*/

// Send a redundant Note Off command to a note if it isn't supposed to be playing right now.
// Eventually send one to all such notes, thus unsticking any notes that got stuck on
// downstream of here (i.e., in the MTP8 boards). 
void incrementalReOff(void)
{
  static byte rank = 0;
  static byte pitch = 0;
  
  if (!flagGhostBuster) {
  	return;
  }
  
  // Don't turn off a note that's supposed to be on!
  // Just skip this round in that case, no need to search for a note to turn off.
  if (!noteIsOn[rank][pitch])
  {
    midiOrgan.sendNoteOff(pitch, 0, channelForRank(rank));
  }
  
  // Go on to the next note, wrapping around.
  if (++rank >= RANKS)
  {
    rank = 0;
    if (++pitch >= 128)
    {
      pitch = 0;
    }
  }
}

// All note playing flows through requestOn and requestOff. They are responsible for two main
// things:
// 1. combining all the different possible ways a note can be requested
// (directly or through sub-octave or super-octave coupler, any of which can be from
//  the default manual or from the other manual).
//
// 2. keeping track of how many notes are being played, so as to limit the current pulled through
// the Peterson valvles and not blow any fuses.
//
// Notes already playing are not re-played. This may be sort of appropriate for the pipe organ,
// but would not be good for the general case.
void requestOn(byte flag, byte rank, byte pitch)
{
  byte oldReq, newReq;
  
  oldReq = noteIsOn[rank][pitch];
  newReq = oldReq | flag;
  
  if (oldReq == 0  && newReq != 0)
    if (noteCount < MaxAllowedNotes)
    {
      noteCount++;
      midiOrgan.sendNoteOn(pitch, 127, channelForRank(rank));
      midiKiosk.sendNoteOn(pitch, 127, KIOSK_CHANNEL);
      noteIsOn[rank][pitch] = newReq;
    }
}

void requestOff(byte flag, byte rank, byte pitch)
{
  byte oldReq, newReq;

  oldReq = noteIsOn[rank][pitch];
  newReq = oldReq & ~flag;
  
  if (oldReq != 0  &&  newReq == 0)
  {
    midiOrgan.sendNoteOff(pitch, 0, channelForRank(rank));
    midiKiosk.sendNoteOff(pitch, 0, KIOSK_CHANNEL);
    noteCount--;    
    noteIsOn[rank][pitch] = newReq;
  }
}

// These handlers are called by the MIDI library when the corresponding
// message arrives from one of the manuals. They are responsible for mapping
// the keypress into one or more note requests, depending on the current
// settings of the stops and couplers on the control panel.
void handleGreatNoteOn(byte channel, byte pitch, byte velocity)
{
//  if (debugMode) {
//    Serial.print("Great note ");
//    Serial.println(pitch);
//  }

  // Let the Organelle know about this event
  if (flagKBecho)
  {
    midiElle.sendNoteOn(pitch, velocity, GREAT_CHANNEL);
  }
  
  // If we are playing the 8' rank with the Great manual,
  if (flag8GreatMain)
  {
    // First, handle the normal note for this manual
    requestOn(NOTE_PRIME, RANK0, pitch);
    
    // Then check for sub-octave and super-octave couplers
    if (flag8GreatSub && pitch > 12)
      requestOn(NOTE_SUB, RANK0, pitch-12);
    if (flag8GreatSuper && pitch <= 115)
      requestOn(NOTE_SUPER, RANK0, pitch+12);
  }
  
  // And, if we are playing the 4' rank with the Great manual, (cross coupled)
  if (flag4GreatMain)
  {
    requestOn(NOTE_RANK_PRIME, RANK1, pitch);
    
    if (flag4GreatSub && pitch > 12)
      requestOn(NOTE_RANK_SUB, RANK1, pitch-12);
    if (flag4GreatSuper && pitch <= 115)
      requestOn(NOTE_RANK_SUPER, RANK1, pitch+12);
  }
}

void handleGreatNoteOff(byte channel, byte pitch, byte velocity)
{
  // Let the Organelle know about this event
  if (flagKBecho)
  {
    midiElle.sendNoteOff(pitch, velocity, GREAT_CHANNEL);
  }
  
  // First, handle the normal note for this manual
  requestOff(NOTE_PRIME, RANK0, pitch);
  
  // Turn off all optional couplers, disregarding flag states.
  // This is in case the flag state has changed: we don't want orphan notes left on!
  requestOff(NOTE_RANK_PRIME, RANK1, pitch);

  if (pitch > 12)
  {
    requestOff(NOTE_SUB, RANK0, pitch-12);
    requestOff(NOTE_RANK_SUB, RANK1, pitch-12);
  }
  
  if (pitch <= 115)
  {
    requestOff(NOTE_SUPER, RANK0, pitch+12);
    requestOff(NOTE_RANK_SUPER, RANK1, pitch+12);
  }

}

void handleGreatPitchBend(byte channel, int bend)
{  
  // Let the Organelle know about this event
  if (flagKBecho)
  {
  	midiElle.sendPitchBend(bend, GREAT_CHANNEL);
  }
}

void handleGreatControlChange(byte channel, byte number, byte value)
{
  // Let the Organelle know about this event
  if (flagKBecho)
  {
    midiElle.sendControlChange(number, value, GREAT_CHANNEL);
  }
  
}

void handleSwellNoteOn(byte channel, byte pitch, byte velocity)
{
//  if (debugMode) {
//    Serial.print("Swell note ");
//    Serial.println(pitch);
//  }

  // Let the Organelle know about this event
  if (flagKBecho)
  {
    midiElle.sendNoteOn(pitch, velocity, SWELL_CHANNEL);
  }
  
  // If we are playing the 4' rank with the Swell manual,
  if (flag4SwellMain)
  {
    // First, handle the normal note for this manual
    requestOn(NOTE_PRIME, RANK1, pitch);
    
    // Then check for sub-octave and super-octave couplers
    if (flag4SwellSub && pitch > 12)
      requestOn(NOTE_SUB, RANK1, pitch-12);
    if (flag4SwellSuper && pitch <= 115)
      requestOn(NOTE_SUPER, RANK1, pitch+12);
  }
  
  // And, if we are playing the 8' rank with the Swell manual, (cross coupled)
  if (flag8SwellMain)
  {
    requestOn(NOTE_RANK_PRIME, RANK0, pitch);
    
    if (flag8SwellSub && pitch > 12)
      requestOn(NOTE_RANK_SUB, RANK0, pitch-12);
    if (flag8SwellSuper && pitch <= 115)
      requestOn(NOTE_RANK_SUPER, RANK0, pitch+12);
  }
}

void handleSwellNoteOff(byte channel, byte pitch, byte velocity)
{
  // Let the Organelle know about this event
  if (flagKBecho)
  {
    midiElle.sendNoteOff(pitch, velocity, SWELL_CHANNEL);
  }
  
    // First, handle the normal note for this manual
  requestOff(NOTE_PRIME, RANK1, pitch);
  
  // Turn off all optional couplers, disregarding flag states.
  // This is in case the flag state has changed: we don't want orphan notes left on!
  requestOff(NOTE_RANK_PRIME, RANK0, pitch);

  if (pitch > 12)
  {
    requestOff(NOTE_SUB, RANK1, pitch-12);
    requestOff(NOTE_RANK_SUB, RANK0, pitch-12);
  }
  
  if (pitch <= 115)
  {
    requestOff(NOTE_SUPER, RANK1, pitch+12);
    requestOff(NOTE_RANK_SUPER, RANK0, pitch+12);
  }

}

void handleSwellPitchBend(byte channel, int bend)
{
  // Let the Organelle know about this event
  if (flagKBecho)
  {
  	midiElle.sendPitchBend(bend, SWELL_CHANNEL);
  }
}

void handleSwellControlChange(byte channel, byte number, byte value)
{
  // Let the Organelle know about this event
  if (flagKBecho)
  {
    midiElle.sendControlChange(number, value, SWELL_CHANNEL);
  }
  
}

// External notes are passed through to the windchest, according to the
// setting of the value of flagMidi.
// flagMidi = 0 means no passthrough.
// flagMidi = 1 means all notes go out to rank 1 (4' rank).
// flagMidi = 2 means notes go out according to the channel they came in on:
// 		channel 1 = rank 0 = 8' rank
//		channel 2 = rank 1 = 4' rank
//		channel >2 = no passthrough
// It's the Organelle's responsibility to make sure no external notes are playing when
// flagMidi changed; they might get stuck on.
void handleExtNoteOn(byte channel, byte pitch, byte velocity)
{
	byte rank;
	
	switch(flagMidi)
	{
		case 0:
			return;
		
		case 1:
			rank = 1;
			break;
			
		case 2:
			if (channel == 1)
				rank = 0;
			else if (channel == 2)
				rank = 1;
			else
				return;
			break;
		default:
			return;
	}
		
	if (velocity != 0)
	{
		requestOn(NOTE_PRIME, rank, pitch);
	}
	else
	{
		requestOff(NOTE_PRIME, rank, pitch);
	}
}


void handleExtNoteOff(byte channel, byte pitch, byte unused_velocity)
{
	byte rank;
	
	switch(flagMidi)
	{
		case 0:
			return;
		
		case 1:
			rank = 1;
			break;
			
		case 2:
			if (channel == 1)
				rank = 0;
			else if (channel == 2)
				rank = 1;
			else
				return;
			break;
		default:
			return;
	}
	
    requestOff(NOTE_PRIME, rank, pitch);
}

void handleElleSysEx(byte *inData, unsigned int inSize)
{
  if ((inSize >= SYSEX_BASE_SIZE) && (inData[SYSEX_CMD_OFFSET] == SYSEX_CMD_SETFLAGS)) {
    if (inSize >= SYSEX_BASE_SIZE+1) {
      flagMidi = inData[SYSEX_DATA_OFFSET];
    }
    if (inSize >= SYSEX_BASE_SIZE+2) {
      flagKBecho = inData[SYSEX_DATA_OFFSET+1];
    }
    if (inSize >= SYSEX_BASE_SIZE+2) {
      flagGhostBuster = inData[SYSEX_DATA_OFFSET+2];
    }
    // can add more bytes of SysEx data, compatibly.
  }

  if ((inSize >= SYSEX_BASE_SIZE) && (inData[SYSEX_CMD_OFFSET] == SYSEX_CMD_FLASH)) {
    flash();      // it isn't good practice to do something slow in the handler, but OK for debug
  }

  // can add other command codes. 
}
// The control panel tells us what the mapping should be between manual keypresses
// and organ pipes sounded. It consists of 12 push-push buttons, in four groups of three.
// Each group of three has one main button, which enables that manual to that rank.
// Subordinate to the main button, there is a button for suboctave coupling and one
// for superoctave coupling.
#define NUM_BUTTONS 12
uint8_t buttons[NUM_BUTTONS] = {
          PIN_GREAT4MAIN, PIN_GREAT4SUB, PIN_GREAT4SUPER,
          PIN_SWELL4MAIN, PIN_SWELL4SUB, PIN_SWELL4SUPER,
          PIN_GREAT8MAIN, PIN_GREAT8SUB, PIN_GREAT8SUPER,
          PIN_SWELL8MAIN, PIN_SWELL8SUB, PIN_SWELL8SUPER
          };

byte on_off[NUM_BUTTONS] = {0,0,0, 1,0,0, 1,0,0, 0,0,0};    // default values on powerup
byte magic_keydown;

#define DEBOUNCE  10    // milliseconds

void check_buttons()
{
  static byte prev_state[NUM_BUTTONS];
  static byte curr_state[NUM_BUTTONS];
  static byte pressed[NUM_BUTTONS];
  static unsigned long last_time = 0;
  static byte magic_keypress;
  byte i;

  if ((last_time + DEBOUNCE) > millis()) {
    // too soon, wait for debounce interval to elapse
    return;
  }
  
  last_time = millis();

  for (i=0; i < NUM_BUTTONS; i++) {
    curr_state[i] = digitalRead(buttons[i]);

    if (curr_state[i] == prev_state[i]) {
      // two samples in a row agree!
      if ((pressed[i] == LOW) && (curr_state[i] == LOW)) {
        // it's a button-down!
        on_off[i] = !on_off[i];
      }

      pressed[i] = !curr_state[i];  // active-low buttons!
    }

    prev_state[i] = curr_state[i];      
  }

  if (pressed[2] && pressed[7]) {
    if (!magic_keypress) {
      magic_keypress = HIGH;
      magic_keydown = HIGH;
    }
  }
  else {
    magic_keypress = LOW;
  }
}

// This function looks at the current state of all the controls and updates.
void pollControlPanel()
{

  check_buttons();

  if (magic_keydown) {
    magic_keydown = LOW;
    allOff();
    flash();
  }
  
  flag4GreatMain  = on_off[0];
  flag4GreatSub   = on_off[1];
  flag4GreatSuper = on_off[2];
  
  flag4SwellMain  = on_off[3];
  flag4SwellSub   = on_off[4];
  flag4SwellSuper = on_off[5];
  
  flag8GreatMain  = on_off[6];
  flag8GreatSub   = on_off[7];
  flag8GreatSuper = on_off[8];
  
  flag8SwellMain  = on_off[9];
  flag8SwellSub   = on_off[10];
  flag8SwellSuper = on_off[11];
  
  // The switches are illuminated; echo the switch state to its LED
  digitalWrite(PIN_GREAT4MAIN_LED, flag4GreatMain);
  digitalWrite(PIN_SWELL4MAIN_LED, flag4SwellMain);
  digitalWrite(PIN_GREAT8MAIN_LED, flag8GreatMain);
  digitalWrite(PIN_SWELL8MAIN_LED, flag8SwellMain);
  digitalWrite(PIN_GREAT4SUB_LED, flag4GreatSub);
  digitalWrite(PIN_SWELL4SUB_LED, flag4SwellSub);
  digitalWrite(PIN_GREAT4SUPER_LED, flag4GreatSuper);
  digitalWrite(PIN_SWELL4SUPER_LED, flag4SwellSuper);
  digitalWrite(PIN_GREAT8SUB_LED, flag8GreatSub);
  digitalWrite(PIN_SWELL8SUB_LED, flag8SwellSub);
  digitalWrite(PIN_GREAT8SUPER_LED, flag8GreatSuper);
  digitalWrite(PIN_SWELL8SUPER_LED, flag8SwellSuper);
}

void flash()
{
   digitalWrite(PIN_GREAT4MAIN_LED, HIGH);
   digitalWrite(PIN_SWELL4MAIN_LED, HIGH);
   digitalWrite(PIN_GREAT8MAIN_LED, HIGH);
   digitalWrite(PIN_SWELL8MAIN_LED, HIGH);
   digitalWrite(PIN_GREAT4SUB_LED, HIGH);
   digitalWrite(PIN_SWELL4SUB_LED, HIGH);
   digitalWrite(PIN_GREAT4SUPER_LED, HIGH);
   digitalWrite(PIN_SWELL4SUPER_LED, HIGH);
   digitalWrite(PIN_GREAT8SUB_LED, HIGH);
   digitalWrite(PIN_SWELL8SUB_LED, HIGH);
   digitalWrite(PIN_GREAT8SUPER_LED, HIGH);
   digitalWrite(PIN_SWELL8SUPER_LED, HIGH);

   delay(100);

   digitalWrite(PIN_GREAT4MAIN_LED, LOW);
   digitalWrite(PIN_SWELL4MAIN_LED, LOW);
   digitalWrite(PIN_GREAT8MAIN_LED, LOW);
   digitalWrite(PIN_SWELL8MAIN_LED, LOW);
   digitalWrite(PIN_GREAT4SUB_LED, LOW);
   digitalWrite(PIN_SWELL4SUB_LED, LOW);
   digitalWrite(PIN_GREAT4SUPER_LED, LOW);
   digitalWrite(PIN_SWELL4SUPER_LED, LOW);
   digitalWrite(PIN_GREAT8SUB_LED, LOW);
   digitalWrite(PIN_SWELL8SUB_LED, LOW);
   digitalWrite(PIN_GREAT8SUPER_LED, LOW);
   digitalWrite(PIN_SWELL8SUPER_LED, LOW);

   delay(100);
   
}

void setup()
{

  // Check what mode the default serial port will be used in.
 // pinMode(PIN_DEBUG_ENABLE, INPUT_PULLUP);
 // debugMode = !digitalRead(PIN_DEBUG_ENABLE); // ground to enable debug
 // if (debugMode) {
    Serial.begin(115200);
    Serial.println("Opus 1.3 MIDI Controller");
 // }
  
  pinMode(PIN_GREAT4MAIN, INPUT_PULLUP);
  pinMode(PIN_GREAT4SUB, INPUT_PULLUP);
  pinMode(PIN_GREAT4SUPER, INPUT_PULLUP);
  pinMode(PIN_SWELL4MAIN, INPUT_PULLUP);
  pinMode(PIN_SWELL4SUB, INPUT_PULLUP);
  pinMode(PIN_SWELL4SUPER, INPUT_PULLUP);
  pinMode(PIN_GREAT8MAIN, INPUT_PULLUP);
  pinMode(PIN_GREAT8SUB, INPUT_PULLUP);
  pinMode(PIN_GREAT8SUPER, INPUT_PULLUP);
  pinMode(PIN_SWELL8MAIN, INPUT_PULLUP);
  pinMode(PIN_SWELL8SUB, INPUT_PULLUP);
  pinMode(PIN_SWELL8SUPER, INPUT_PULLUP);

  pinMode(PIN_GREAT4MAIN_LED, OUTPUT);
  pinMode(PIN_GREAT4SUB_LED, OUTPUT);
  pinMode(PIN_GREAT4SUPER_LED, OUTPUT);
  pinMode(PIN_SWELL4MAIN_LED, OUTPUT);
  pinMode(PIN_SWELL4SUB_LED, OUTPUT);
  pinMode(PIN_SWELL4SUPER_LED, OUTPUT);
  pinMode(PIN_GREAT8MAIN_LED, OUTPUT);
  pinMode(PIN_GREAT8SUB_LED, OUTPUT);
  pinMode(PIN_GREAT8SUPER_LED, OUTPUT);
  pinMode(PIN_SWELL8MAIN_LED, OUTPUT);
  pinMode(PIN_SWELL8SUB_LED, OUTPUT);
  pinMode(PIN_SWELL8SUPER_LED, OUTPUT);
  
  // All processing is handled in the receive callbacks for NoteOn and NoteOff.
  midiGreat.setHandleNoteOn(handleGreatNoteOn);
  midiGreat.setHandleNoteOff(handleGreatNoteOff);
  midiGreat.setHandlePitchBend(handleGreatPitchBend);
  midiGreat.setHandleControlChange(handleGreatControlChange);
  midiSwell.setHandleNoteOn(handleSwellNoteOn);
  midiSwell.setHandleNoteOff(handleSwellNoteOff);
  midiSwell.setHandlePitchBend(handleSwellPitchBend);
  midiSwell.setHandleControlChange(handleSwellControlChange);
  midiExt.setHandleNoteOn(handleExtNoteOn);
  midiExt.setHandleNoteOff(handleExtNoteOff);
  //if (!debugMode) {
  //  midiElle.setHandleNoteOn(handleElleNoteOn);
  //  midiElle.setHandleNoteOff(handleElleNoteOff);
    midiElle.setHandleSystemExclusive(handleElleSysEx);
  //}
  
  // Begin receive processing. Listen to all channels so we don't care about keyboard configuration.
  midiGreat.begin(MIDI_CHANNEL_OMNI);
  midiSwell.begin(MIDI_CHANNEL_OMNI);
  midiExt.begin(MIDI_CHANNEL_OMNI);
  //if (!debugMode) {
  //  midiElle.begin(MIDI_CHANNEL_OMNI);
  //}
  
  // We don't want automatic echoing of MIDI messages.
  midiGreat.turnThruOff();
  midiSwell.turnThruOff();
  midiExt.turnThruOff();
  //if (!debugMode) {
  //  midiElle.turnThruOff();
  //}
  
  // Cancel anything going on right now
  allOff();
  flash();flash();flash();
  
  // debug
  pollControlPanel();
}

void loop()
{
  static unsigned long lasttick = millis();
  
  pollControlPanel();
  midiGreat.read();
  midiSwell.read();
  midiExt.read();
  //if (!debugMode) {
  //  midiElle.read();
  //}

  if (millis() > lasttick + 5)
  {
    lasttick = millis();
    incrementalReOff();
  }

}

