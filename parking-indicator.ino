/* Garage Depth Sensor
    By Karen Waldenmeyer
    https://github.com/kwaldz
    
    The purpose of this program is to ensure that a single car in our household parks deep enough into our
    garage to avoid any mishaps.
    I have a hatchback so it's important for me to know when I'm in far enough to clear the garage door,
    far enough in to open my hatch with the garage door down,
    and any further than that.

    Hardware:
    - HC-SR04 ultrasonic sensor + 5V power source (shared)
    - APA102 Addressable LED strip, 1m 30LEDs/m + 5V power source (shared)
    - GOSEPP Light Sensor Module for Arduino
    - Arduino Mega 2560

	Stipulations:
	- The light sensor's purpose to ensure LEDs aren't constantly staying on - this is my janky way of accomplishing that
	- Right now this only works with a single sensor. My plan is to obtain another sensor and rework this code
	such that I can simultaneously display 2 cars' depth sensor info.
*/

#include "FastLED.h"
#include "NewPing.h"

#define LIGHT_SENSOR          A0 //Light Sensor is connected to A0 of Arduino
#define MIN_LIGHT_THRESHOLD   23 //Setting a min light threshold set for garage door light @ nightime

// HC-SR04 Setup
#define TRIG_PIN              48
#define ECHO_PIN              49
NewPing sensor(TRIG_PIN, ECHO_PIN, 10000);

// defines distance variables - ints in cms
long distance;
long ms;
int tooclose =                121;
int inopenabletrunk =         137;
int incannotopentrunk =       162;

//LED Strip constants
#define NUM_LEDS              30
#define DATA_PIN              30
#define CLOCK_PIN             31
#define LED_TYPE              APA102
#define COLOR_ORDER           BGR
#define BRIGHTNESS            40
#define FRAMES_PER_SECOND     120
CRGBArray<NUM_LEDS> leds;
CRGB color;
int numLedsToLight = 0;

void setup() {
  delay(2000); // 2 second delay for recovery
  Serial.begin(9600); // Starts the serial communication
  //Set up light sensor
  pinMode(LIGHT_SENSOR, INPUT);
  //LED initialization
  FastLED.addLeds<LED_TYPE, DATA_PIN, CLOCK_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  // limit my draw to 1A at 5v of power draw
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 1000);
}
void loop() {
  //Get Light Reading - everything is dependent on it
  int lux = analogRead(LIGHT_SENSOR);
  Serial.print("Lux: ");
  Serial.println(lux);

  if (lux > MIN_LIGHT_THRESHOLD) { //Measured light must be above the min threshold
    ms = 0;
    ms = sensor.ping_median(5); //get median reading for every 5 pings, helps reduce sensor noise
    distance = sensor.convert_cm(ms); //convert ms to to cm
    Serial.print("RAW Distance: ");
    Serial.println(distance);

    //This calls a linear map function I found that calculate the total pixels to be lit.
    //May eventually wanna add a constrain() here because sensor readings do tend to be weird.
    numLedsToLight = linearmap(distance, tooclose, incannotopentrunk, 0, (NUM_LEDS - 1));

    //My own silly constrain()
    if (numLedsToLight < 0) {
      numLedsToLight = 0;
    }
    if (numLedsToLight >= (NUM_LEDS - 1)) {
      numLedsToLight = (NUM_LEDS - 1);
    }

    //Color logic based on depth measurements
    if (distance < tooclose) {
      color = CRGB::Red;
    } else if (distance >= tooclose && distance <=  inopenabletrunk) {
      color = CRGB::Green;
    } else if (distance > inopenabletrunk && distance < incannotopentrunk) {
      color = CRGB::Yellow;
    } else if (distance >= incannotopentrunk) {
      color = CRGB::Red;
    } else {
      color = CRGB::Black;
    }

    //Actual fills & show
    leds(0, numLedsToLight).fill_solid(color);
    leds(numLedsToLight + 1, NUM_LEDS).fill_solid(CRGB::Black);
    FastLED.show();

  } else { //light is below threshold
    
    // Now turn the LED off, then pause for 5 secs
    leds.fill_solid(CRGB::Black);
    FastLED.show();
    delay(5000);
  }

}

long linearmap(long x, long in_min, long in_max, long out_min, long out_max)
{
  if ( x == in_max)
    return out_max;
  else if (out_min < out_max)
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min) + out_min;
  else
    return (x - in_min) * (out_max - out_min - 1) / (in_max - in_min) + out_min;
}


