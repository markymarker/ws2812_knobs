// Author: Mark Fletcher
//   Date: 2022-04-01

/* Description:
 *     A ws2812 strip is illuminated based on inputs from 2 knobs. One knob
 * controls the hue of illumination, and the other knob controls the position
 * of illumination. The point at the position will be illuminated with the
 * selected hue at full brightness, and when the point is moved, a trail will
 * be left that gradually fades away.
 *
 * There are additional parameters controlled by pushbutton inputs. One button
 * is used to cycle between different methods of moving the light point. The
 * other button is used to cycle through different delay times so that the strip
 * animation can be slowed down to better see what's going on.
 *
 * Debug level 1 is recommended, as it gives a good indication of the settings
 * directly on the strip itself without being overly intrusive.
 */



#include <FastLED.h>
/* ?? FASTLED_USING_NAMESPACE ?? */



//-----------------------------------//
//--- CONFIGURATION AND CONSTANTS ---//

// Set debug level with M_DEBUG
// 0: Debug off
// 1: Debug display on lights (see note on UNRESERVED_START below)
// 2: Debug display on serial monitor (+ slower default operation)
#define M_DEBUG 1

// LED-related constants
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define DATA_PIN 2
#define MAX_BRIGHTNESS 128
#define STATUS_BRIGHTNESS 64
const uint16_t NUM_LEDS = 128;

#if M_DEBUG == 1
	/* Bottom area of led strip is reserved for debug display. The constant
	 * below should equal the number of status leds + 1 for divider.
	 * Legend:
	 *   0: Hue change
	 *   1: Position target change
	 *   2: Position change
	 *   3: Inactive led fade change
	 *   4: Delay time setting
	 *   5: Move style setting
	 *
	 *   UNRESERVED_START - 1: Divider (white, pulses when updating)
	 */
	#define UNRESERVED_START 7
#else
	#define UNRESERVED_START 0
#endif

// Input-related constants
#define KNOB_POS A3
#define KNOB_HUE A5
#define BUTTON_MOVE 4
#define BUTTON_DELAY 8

#define DELAY_MAX 1024
#define DEBOUNCE_MILLIS 20

// Status constants
typedef uint8_t status_t;  // Set C_RES to MSB value of type
#define CHANGED_RESERVED 0x80
#define CHANGED_HUE 0x01
#define CHANGED_TAR 0x02
#define CHANGED_MOV 0x04
#define CHANGED_FAD 0x08



//---------------//
//--- GLOBALS ---//

uint8_t hue;
uint16_t position = UNRESERVED_START;
uint16_t position_target;
CRGB leds[NUM_LEDS];

// Movement style to use when shifting position
// 0: (default) Fill area between and jump to target
// 1: Step position toward target with each frame
// 2: Jump to target with no in-between
const uint8_t move_style_count = 3;
uint8_t move_style = 0;
CRGB move_style_status = CRGB::Black;

// Delay to use at end of main loop
uint16_t delay_time = 0;
CRGB delay_time_status = CRGB::Black;



//---------------------------//
//--- FUNCTION PROTOTYPES ---//

void handleButtonMove(void);
void handleButtonDelay(void);

status_t logicHue(void);
status_t logicTarg(void);
status_t logicMove(void);
status_t logicFade(void);



//------------------------------//
//--- STRUCTS AND MISC TYPES ---//

// Struct of parameters for button debouncing
typedef struct {
	      uint8_t pin;            // Input pin number
	unsigned long lastPressTime;  // Initialize to 0
	          int lastState;      // Init to HIGH for pullup, LOW for pulldown
	          int trigger_on;     // Set to LOW for pullup, HIGH for pulldown
} buttonTracker;



//----------------------//
//--- MAIN FUNCTIONS ---//

void setup(void){
	// "Power up safety delay" -- seeing 3000 used here commonly
	delay(2666);

#if M_DEBUG == 2
	delay_time = 1000;
	delay_time_status = CHSV(map(delay_time, 0, DELAY_MAX, 0, 224), 255, STATUS_BRIGHTNESS);

	Serial.begin(9600);
	while(!Serial){;}
#endif

	pinMode(DATA_PIN, OUTPUT);

	pinMode(KNOB_HUE, INPUT);
	pinMode(KNOB_POS, INPUT);
	pinMode(BUTTON_MOVE, INPUT_PULLUP);
	pinMode(BUTTON_DELAY, INPUT_PULLUP);

	FastLED
		.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
		.setCorrection(TypicalLEDStrip)
//		.setBrightness(MAX_BRIGHTNESS)
	;

	fill_solid(leds, NUM_LEDS, CRGB::Black);

#if M_DEBUG == 1
	leds[UNRESERVED_START - 1] = CHSV(0, 0, STATUS_BRIGHTNESS);
#endif

	FastLED.show();
}


void loop(void){
	status_t changed = 0;

	handleButtonMove();
	handleButtonDelay();

	changed |= logicHue();
	changed |= logicTarg();
	changed |= logicMove();
	changed |= logicFade();

#if M_DEBUG == 1
	// Set status lights and check if they need updating
	CRGB sleds[UNRESERVED_START - 1];
	memcpy(sleds, leds, (UNRESERVED_START - 1) * sizeof(leds[0]));

	leds[0] = CHSV( 96, 255, ((changed & CHANGED_HUE) > 0) * STATUS_BRIGHTNESS);
	leds[1] = CHSV( 64, 255, ((changed & CHANGED_TAR) > 0) * STATUS_BRIGHTNESS);
	leds[2] = CHSV(192, 255, ((changed & CHANGED_MOV) > 0) * STATUS_BRIGHTNESS);
	leds[3] = CHSV(  0, 255, ((changed & CHANGED_FAD) > 0) * STATUS_BRIGHTNESS);
	leds[4] = delay_time_status;
	leds[5] = move_style_status;

	for(uint8_t j = 0; j < UNRESERVED_START - 1; ++j){
		if(sleds[j] != leds[j]){
			changed |= CHANGED_RESERVED;
			break;
		}
	}
#endif

	if(changed){
#if M_DEBUG == 1
		leds[UNRESERVED_START - 1] = CHSV(0, 0, beatsin16(120, STATUS_BRIGHTNESS - 40, STATUS_BRIGHTNESS));
#elif M_DEBUG == 2
		char str[50];
		sprintf(str, "updating (%u) p.%u h.%u T+%lu", changed, position, hue, millis());
		Serial.println(str);
		delay(10);  // Delay after serial, before FLED show for reasons
#endif

		if(position != position_target) leds[position_target] = CHSV(hue, 128, 42);
		leds[position] = CHSV(hue, 255, MAX_BRIGHTNESS);
		FastLED.show();
	}

	delay(delay_time);
}



//-----------------------//
//--- INPUT FUNCTIONS ---//

/**
 * Generic button handling function.
 * Takes in a buttonTracker struct and uses and sets values therein to read
 * buttons and apply debouncing. Returns TRUE if a debounced button press
 * occured, FALSE otherwise. Will only return TRUE once per detected button
 * press.
 */
bool handleButton(buttonTracker * params){
	int bval = digitalRead(params->pin);

	// State changed - set timer and track new state
	if(bval != params->lastState){
		params->lastPressTime = millis();
		params->lastState = bval;
		return false;
	}

	// No state change and no timer set - do nothing
	if(params->lastPressTime == 0) return false;
	// Timer set, but not time to trigger yet - do nothing
	if(millis() - params->lastPressTime <= DEBOUNCE_MILLIS) return false;

	// Debounce time elapsed and state steady - apply current bval
	params->lastPressTime = 0;
	return bval == params->trigger_on;
}


/**
 * Handler for the movement style button.
 * Cycles through the movement styles when the relevant button is pressed and
 * sets the related status light to the corresponding color.
 */
void handleButtonMove(void){
	static buttonTracker params = {
		.pin = BUTTON_MOVE,
		.lastPressTime = 0,
		.lastState = HIGH,
		.trigger_on = LOW
	};

	if(handleButton(&params)){
		move_style = (move_style + 1) % move_style_count;

		switch(move_style){
		// 1: Step with frame
		case 1: move_style_status = CHSV(32, 255, STATUS_BRIGHTNESS); break;
		// 2: Jump to target
		case 2: move_style_status = CHSV(128, 255, STATUS_BRIGHTNESS); break;
		// default: Fill and jump
		default: move_style_status = CRGB::Black;
		}
	}
}


/**
 * Handler for the delay time button.
 * Increases the delay when the relevant button is pressed. The delay will be
 * stepped up by doubling, up to the DELAY_MAX value. Once DELAY_MAX is
 * exceeded, the delay will wrap back to zero.
 */
void handleButtonDelay(void){
	static buttonTracker params = {
		.pin = BUTTON_DELAY,
		.lastPressTime = 0,
		.lastState = HIGH,
		.trigger_on = LOW
	};

	if(handleButton(&params)){
		delay_time = (delay_time == 0) ? 8 : delay_time * 2;
		if(delay_time > DELAY_MAX) delay_time = 0;
		delay_time_status = (delay_time == 0)
			? CHSV(0, 0, 0)
			: CHSV(map(delay_time, 0, DELAY_MAX, 0, 224), 255, STATUS_BRIGHTNESS)
		;
	}
}



//-----------------------//
//--- LOGIC FUNCTIONS ---//

/**
 * Logic for hue control.
 * Reads the value of the hue knob's pin and determines if a change has been
 * made and what value to set for the hue if so. Returns the CHANGED_HUE status
 * code if a change was made or 0 if no change.
 */
status_t logicHue(void){
	static uint16_t last_hue = -1;

	uint16_t hue_reading = analogRead(KNOB_HUE);
	if(hue_reading == last_hue) return 0;

	uint8_t hpre = hue;
	hue = map(hue_reading, 0, 1024, 0, 255);
	last_hue = hue_reading;

	// Only register change if hue itself changed
	return (hpre != hue) * CHANGED_HUE;
}


/**
 * Logic for position target control.
 * Reads the value of the position knob's pin and determines if a change has
 * been made and what value to set for the position target if so. Returns the
 * CHANGED_TAR status code if a change was made or 0 if no change.
 */
status_t logicTarg(void){
	static uint16_t last_pos = -1;

	uint16_t pos_reading  = analogRead(KNOB_POS);
	if(pos_reading == last_pos) return 0;

	uint16_t ppre = position_target;
	position_target = map(pos_reading, 0, 1024, UNRESERVED_START, NUM_LEDS);
	last_pos = pos_reading;

	// Only register change if pos itself changed
	return (ppre != position_target) * CHANGED_TAR;
}


/**
 * Logic for moving active led.
 * Will adjust the value in the position global as needed to get the active led
 * to the correct place. The way the move happens will depend on the currently
 * set move style.  Returns the CHANGED_MOV status code if any movement
 * occurred.
 */
status_t logicMove(void){
	if(position == position_target) return 0;

#if DEBUG == 2
	// If doing serial debug, just jump to position
	position = position_target;
	return CHANGED_MOV;
#else
	switch(move_style){
	case 1:
		// Move one step toward position target with each frame
		if(position < position_target) ++position;
		else --position;
	break;

	case 2:
		// Jump position to target with no in-between
		position = position_target;
	break;

	default:
		// Fill space between position and target and jump to target
		uint16_t start, finish;
		if(position < position_target){
			start = position;
			finish = position_target;
		} else {
			start = position_target;
			finish = position;
		}
		fill_solid(leds + start, finish - start, CHSV(hue, 255, MAX_BRIGHTNESS));
		position = position_target;
	}

	return CHANGED_MOV;
#endif
}


/**
 * Logic for fading inactive leds.
 * Will go through the leds array and apply a fade to each entry, skipping the
 * led at the active position. Returns the CHANGED_FAD status code if any leds
 * values were changed or 0 if no values actually changed.
 */
status_t logicFade(void){
	// TODO: Consider using fade_whatever(leds, range, amt) to do setting more
	// efficiently (?) and then detect change another way

	status_t changed = 0;

	for(uint16_t j = UNRESERVED_START; j < NUM_LEDS; ++j){
		// Eliminate branch if possible
		// Supposedly, AVR BP will assume branch not taken, 1 cycle penalty when wrong
		if(j == position) continue;

		CRGB initial = leds[j];

#if M_DEBUG != 2
		leds[j].fadeToBlackBy(16);
#else
		leds[j].fadeToBlackBy(175);

		if(initial != leds[j]){
			Serial.print(j); Serial.print(": "); Serial.println(initial != leds[j]);
		}
		status_t cpre = changed;
#endif

		changed |= (initial != leds[j]);

#if M_DEBUG == 2
		if(cpre != changed){
			Serial.print("changed (j="); Serial.print(j);
			Serial.print(") pre: "); Serial.print(cpre);
			Serial.print("  :: post: "); Serial.println(changed);
		}
#endif
	}

	return changed * CHANGED_FAD;
}


