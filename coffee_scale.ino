//Add libraries
  #include <Wire.h>
  #include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU8702
  #include <Adafruit_SSD1306.h>

//variables for scale 
  NAU7802 myScale; //Create instance of the NAU7802 class
  float scale_slope = 0.001;
  float scale_g;
  float zero_offset = 0;
  int n_array = 50;
  float mass_array[50];
  float raw_offset;
  float mass_avg;
  float sum_g;
  float sum_raw = 0;
  
//for SSD1306 I2C OLED Display
  #define SCREEN_WIDTH 128 //lower if RAM malfunctions
  #define SCREEN_HEIGHT 64
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// variables for timer
  unsigned long previousMillis = 0; // For storing previous timestep
  unsigned long previousMs_meas = 0; // For storing previous timestep
  unsigned long timerMillis = 0; // For storing previous timestep
  int interval_display = 500;        // speed the sketch runs (ms)
  int interval_meas = 10;
  int count = 0;
  bool timer = false;

//variables for push-button control
int buttonPin = 12;        // analog input pin to use as a digital input
int b;

void setup()
{

  Serial.begin(115200);

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

  raw_offset = zero();
  
        //OLED setup
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C )) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  //set additional display parameters
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text

    // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.write("Running Setup");display.display();

  // Set pushbutton input pin
   pinMode(buttonPin, INPUT_PULLUP);
   digitalWrite(buttonPin, HIGH); 

}

float zero(){
  float sum_raw = 0;
  float raw;
  for(int i = 0; i < 50; ++i){
    //measure mass
    raw = floor(myScale.getReading()/1000);
    sum_raw += raw * 1000;
    delay(40);
  }

  delay(100);

  sum_raw = 0;
  for(int i = 0; i < 50; ++i){
    //measure mass
    raw = floor(myScale.getReading()/1000);
    sum_raw += raw * 1000;
    delay(40);
  }
  
  delay(100);

  sum_raw = 0;
  for(int i = 0; i < (1000); ++i){
    //measure mass
    raw = floor(myScale.getReading()/1000);
    sum_raw += raw * 1000;
  }
  //calculate average
  raw_offset = sum_raw / 1000;
  
  return raw_offset;
}


void loop()
{
  
  //--Check the elapsed time    
  unsigned long currentMillis = millis(); 

  //check pushbutton status
  b = checkButton();
   if (b == 1) raw_offset = clickEvent();
   if (b == 2) doubleClickEvent();
   if (b == 3) timer = holdEvent(timer);
   if (b == 4) longHoldEvent();

  if ((currentMillis - previousMs_meas) >= interval_meas) {
    previousMs_meas = currentMillis;
    sum_g = 0;
    
    //populate array with new mass data
    for(int i = 0; i < (n_array-1); ++i){
      //measure mass
      float scaleRaw = floor(myScale.getReading()/1000) ;
      float scaleClean = (scaleRaw * 1000) - raw_offset;
      scale_g = scaleClean * scale_slope;
      mass_array[i] = scale_g ;
      sum_g += scale_g;
    }
    //calculate average
    mass_avg = sum_g / n_array;
  }

  if ((currentMillis - previousMillis) >= interval_display) {
      // ..If yes, save current time.  Then update the LED pin and LED state.
    previousMillis = currentMillis;  //reset previous time for interval

    if(timer == true){
      timerMillis = timerMillis;
    }else{
      timerMillis = currentMillis;
    }

    int minute = floor((currentMillis - timerMillis)/(1000*60));
    int second = floor((currentMillis - timerMillis)/(1000) - minute*60);

    Serial.print("AvgWeight: ");Serial.println(mass_avg, 2); //Print 2 decimal places
    Serial.print("timer:  ");Serial.println(timer);

    display.clearDisplay(); 
    display.setCursor(40,0);display.setTextSize(1);display.write("MASS (g)");
    display.setCursor(35,10); display.setTextSize(2);display.print(mass_avg, 1);
    display.setCursor(30,35);display.setTextSize(1);display.write("TIMER (mm:ss) ");
    display.setCursor(40,45);display.setTextSize(2);display.print(minute);display.write(":"); display.print(second);
    display.display();
  }
  

}

//=================================================
// Events to trigger

//zero the balance
float clickEvent() {
   Serial.println();
   Serial.println("click event");
   Serial.println();
   float clck_zero = zero();
   return clck_zero;
   delay(500);
}
void doubleClickEvent() {

}
bool holdEvent(bool clck_timer) {
  Serial.println();
  Serial.println("hold event");
  Serial.println();
  if(clck_timer == true){
    clck_timer = false;
  }else{
    clck_timer = true;
  }
  return clck_timer;
}
void longHoldEvent() {

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



