//Add libraries
  #include <Wire.h>
  #include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU8702
  #include <Adafruit_SSD1306.h>
  #include <RunningAverage.h>

//variables for scale 
  NAU7802 myScale; //Create instance of the NAU7802 class
  float scale_offset = 0;
  float scale_slope = 1;
  float scale_g;
  int interval_meas = 100;
  zero_offset = 0;

//Create an array to take average of weights. This helps smooth out jitter.
  RunningAverage SW(10);
  
//for SSD1306 I2C OLED Display
  #define SCREEN_WIDTH 128 //lower if RAM malfunctions
  #define SCREEN_HEIGHT 64
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// variables for timer
  unsigned long previousMillis = 0; // For storing previous timestep
  unsigned long previousMs_meas = 0; // For storing previous timestep
  int interval_display = 1000;        // speed the sketch runs (ms)
  int count = 0;

//variables for push-button control
#define buttonPin 5        // analog input pin to use as a digital input


void setup()
{
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(100); //Qwiic Scale is capable of running at 400kHz if desired

  if (myScale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Scale detected!");

  myScale.setSampleRate(NAU7802_SPS_320); //Increase to max sample rate
  //myScale.calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel 

        //OLED setup
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C )) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  //set additional display parameters
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text

    // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.write("Running Setup");display.display();

  SW.clear();

  // Set pushbutton input pin
   pinMode(buttonPin, INPUT);
   digitalWrite(buttonPin, HIGH );
}

void loop()
{

  //--Check the elapsed time    
    unsigned long currentMillis = millis(); 

  if ((currentMillis - previousMs_meas) >= interval_meas) {
    previousMs_meas = currentMillis;
    float scaleRaw = myScale.getReading();
    scale_g = scaleRaw * scale_slope + scale_offset;
    SW.addValue(scale_g);
  }

  if ((currentMillis - previousMillis) >= interval_display) {
      // ..If yes, save current time.  Then update the LED pin and LED state.
    previousMillis = currentMillis;  //reset previous time for interval
    float avgWeight = 0;

    Serial.print("AvgWeight: ");Serial.println(SW.getAverage(), 2); //Print 2 decimal places
    Serial.print("Mass");Serial.println(scale_g);
    
    int minute = floor((currentMillis - previousMillis)/(1000*60));
    int second = floor((currentMillis - previousMillis)/(60) - minute*60);
    
    display.clearDisplay(); display.setCursor(0, 0);
    display.write("Wt;");display.print(SW.getAverage(), 2);display.println();
    display.write("Timer: ");display.print(minute);display.write(":"); display.println(second);
    display.display();
  }
  
}

//=================================================
// Events to trigger

//zero the balance
void clickEvent() {
   //zero_offset = average
}
void doubleClickEvent() {
   ledVal2 = !ledVal2;
   digitalWrite(ledPin2, ledVal2);
}
void holdEvent() {
   ledVal3 = !ledVal3;
   digitalWrite(ledPin3, ledVal3);
}
void longHoldEvent() {
   ledVal4 = !ledVal4;
   digitalWrite(ledPin4, ledVal4);
}
//=================================================
//  MULTI-CLICK:  One Button, Multiple Events

// Button timing variables
int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250;            // max ms between clicks for a double click event
int holdTime = 1000;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 3000;    // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

int checkButton() {    
   int event = 0;
   buttonVal = digitalRead(buttonPin);
   // Button pressed down
   if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
   {
       downTime = millis();
       ignoreUp = false;
       waitForUp = false;
       singleOK = true;
       holdEventPast = false;
       longHoldEventPast = false;
       if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
       else  DConUp = false;
       DCwaiting = false;
   }
   // Button released
   else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
   {        
       if (not ignoreUp)
       {
           upTime = millis();
           if (DConUp == false) DCwaiting = true;
           else
           {
               event = 2;
               DConUp = false;
               DCwaiting = false;
               singleOK = false;
           }
       }
   }
   // Test for normal click event: DCgap expired
   if ( buttonVal == HIGH && (millis()-upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
   {
       event = 1;
       DCwaiting = false;
   }
   // Test for hold
   if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
       // Trigger "normal" hold
       if (not holdEventPast)
       {
           event = 3;
           waitForUp = true;
           ignoreUp = true;
           DConUp = false;
           DCwaiting = false;
           //downTime = millis();
           holdEventPast = true;
       }
       // Trigger "long" hold
       if ((millis() - downTime) >= longHoldTime)
       {
           if (not longHoldEventPast)
           {
               event = 4;
               longHoldEventPast = true;
           }
       }
   }
   buttonLast = buttonVal;
   return event;
}
