// Need to include the SoftTimer library
// Need a rotary encoder, FM Chip, and an OLED display

#include <SoftTimer.h>

#include <Adafruit_SSD1306.h>

/*
* RDA5807M Radio
*
* Sketch illustrates how to use some of the basic commands in the
* RDA5807M Library. The sketch will start the RDA5807M in West-FM mode
* (87-108MHz) and then wait for user commands via the serial port, buttons,
* or Potentiometer.
* More information on the RDA5807M chip can be found in the datasheet.
*
* HARDWARE SETUP:
* This sketch assumes you are using the RRD-102 (V2.0) breakout board.
*
* The board should b\e connected to a 3.3V Arduino's I2C interface.
* Alternatively, a 5V Arduino can be used if a proper I2C level translator is
* used (see the README for an example).
* You will need an audio amplifier as the RDA5807M is too weak to directly drive
* a pair of headphones. Immediate candidates would be a pair of active
* (multimedia) speakers that you're probably already using with your computer.
* You will also need a proper antenna connected to breakout board. Luckily
* for you, this is a very forgiving chip when it comes to "proper" antennas,
* so for FM only you will be able to get away with just a 2ft (60cm) length of
* wire connected to the FM antenna pad on the shield. Decent results can
* probably be obtained using a 6" breadboard jumper wire too.
*
* USING THE SKETCH:
* Once you've connected the RRD-102 to your Arduino board (and antenna(s), as
* appropriate), connect the Arduino to your computer, select the corresponding
* board and COM port from the Tools menu and upload the sketch. After the sketch
* has been updated, open the serial terminal using a 9600 baud speed. The sketch
* accepts single character commands (just enter the character and press 'send').
* Here is a list of the acceptable commands:
*   v/V     - decrease/increase the volume
*   s/S     - seek down/up with band wrap-around
*   m/M     - mute/unmute audio output
*   f       - display currently tuned frequency
*   q       - display RSSI for currently tuned station
*   t       - display decoded status register
*   ?       - display this list
*
*/

//Due to a bug in Arduino, this needs to be included here too/first
#include <Wire.h>

//Add the RDA5807M Library to the sketch.
#include <RDA5807M.h>

//Create an instance of the RDA5807M named radio
RDA5807M radio;

//Other variables we will use below
char command;
word status, frequency;

//OLED
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Button Variables
const int seekUp = 3;
const int seekDown = 2;
int seekUpState = 0;
int seekDownState = 0;
const int volUp = 9;
const int volDown = 8;
int volUpState = 0;
int volDownState = 0;

int currentFreq = 0;

//Tuning Variables
//float tunedFreqInt = 0;
//float theFreq = 0;
//float prevFreq = 0;
//int val = 0;
//int mod;
float newFreq = 10150;

//Encoder Pins
#define outputA 6
#define outputB 7

int counter = 0; 
int aState;
int aLastState;  

SoftTimer timer;

void setup()
{
  //Create a serial connection
  Serial.begin(9600);

  timer.every(3000, callbackShowStatus);

  //Button modes
  pinMode(seekUp, INPUT);
  pinMode(seekDown, INPUT);
  pinMode(volUp, INPUT);
  pinMode(volDown, INPUT);
  
  //Encoder Pin Modes
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  
  //Initialize the radio to the West-FM band. (see RDA5807M_BAND_* constants).
  //The mode will set the proper receiver bandwidth.
  radio.begin(RDA5807M_BAND_WEST);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);
  // Clear the buffer.
  display.clearDisplay();
  radio.setFrequency(newFreq);
  delay(500);
  showStation();
  showStatus();
}

//Callback function to run every 5 seconds with timer.every()
boolean callbackShowStatus(EventBase* evt) {
  showStatus();
  return false;
}

//Function to show during Seek on OLED
void showSeeking() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Seeking");
  display.setCursor(24, 10);
  display.print("...");
  display.display();
}

//Funtion to show the station on OLED
void showStation() {
  //float station = radio.getFrequency();
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(98,16);
  display.print("MHz");
  display.setCursor(24, 16);
  display.setTextSize(2);
  display.print(newFreq / 100);
  display.display();
}

//Function to show FM Status on OLED
void showStatus() {
  status = radio.getRegister(RDA5807M_REG_STATUS);
  if(status & RDA5807M_STATUS_ST) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(24, 0);
    display.print("Stereo FM");
    display.display();
  }
}

boolean debounceButton(boolean state, int buttonPin)
{
  boolean stateNow = digitalRead(buttonPin);
  if(state!=stateNow)
  {
    delay(50);
    stateNow = digitalRead(buttonPin);
  }
  return stateNow;
  
}

            
void loop()
{
  //Update the showdisplay
  timer.update(); 
  
  //Read the button status
  seekUpState = digitalRead(seekUp);
  seekDownState = digitalRead(seekDown);

  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  //currentFreq = radio.getFrequency();
  currentFreq = newFreq;
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       //counter ++;
       newFreq = currentFreq + 20;
       radio.setFrequency(newFreq);
       //delay(50);
       showStation();
       //delay(100);
       //showStatus();
     } else {
       //counter --;
       newFreq = currentFreq - 20;
       radio.setFrequency(newFreq);
       //delay(50);
       showStation();
       //delay(50);
       //showStatus();
     }
     //Serial.print("Position: ");
     //Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state

  //Button's pressed
  if (seekUpState == HIGH) {
      showSeeking();
      //Serial.println(F("Seeking up with band wrap-around"));
      //Serial.flush();
      radio.seekUp();
      delay(2000);
      newFreq = radio.getFrequency();
      showStation();
      showStatus();
  } else if (seekDownState == HIGH) {
    showSeeking();
    //Serial.println(F("Seeking down with band wrap-around"));
    //Serial.flush();
    radio.seekDown();
    delay(2000);
    newFreq = radio.getFrequency();
    showStation();
    showStatus();
  } else if (debounceButton(volUpState, volUp) == HIGH && volUpState == LOW) {
    radio.volumeUp();
  } else if (debounceButton(volDownState, volDown) == HIGH && volDownState == LOW) {
    radio.volumeDown();
  }

  //Wait until a character comes in on the Serial port.
  if(Serial.available()){
    //Decide what to do based on the character received.
    command = Serial.read();
    switch(command){
      case 'v':
        if(radio.volumeDown()) Serial.println(F("Volume decreased"));
        else Serial.println(F("ERROR: already at minimum volume"));
        Serial.flush();
        break;
      case 'V':
        if(radio.volumeUp()) Serial.println(F("Volume increased"));
        else Serial.println(F("ERROR: already at maximum volume"));
        Serial.flush();
        break;
      case 's':
        showSeeking();
        Serial.println(F("Seeking down with band wrap-around"));
        Serial.flush();
        radio.seekDown();
        delay(2000);
        showStation();
        showStatus();
        break;
      case 'S':
        showSeeking();
        Serial.println(F("Seeking up with band wrap-around"));
        Serial.flush();
        radio.seekUp();
        delay(2000);
        showStation();
        showStatus();
        break;
      case 'm':
        radio.mute();
        Serial.println(F("Audio muted"));
        Serial.flush();
        break;
      case 'M':
        radio.unMute();
        Serial.println(F("Audio unmuted"));
        Serial.flush();
        break;
      case 'f':
        frequency = radio.getFrequency();
        Serial.print("Raw Frequency: ");
        Serial.println(frequency);
        Serial.print(F("Currently tuned to "));
        Serial.print(frequency / 100);
        Serial.print(".");
        Serial.print(frequency % 100);
        Serial.println(F("MHz FM"));
        Serial.flush();
        break;
      case 'q': 
        Serial.print(F("RSSI = "));
        Serial.print(radio.getRSSI());
        Serial.println("dBuV");
        Serial.flush();
        break;
      case 't':
        status = radio.getRegister(RDA5807M_REG_STATUS);
        Serial.println(F("Status register {"));
        if(status & RDA5807M_STATUS_RDSR)
            Serial.println(F("* RDS Group Ready"));
        if(status & RDA5807M_STATUS_STC)
            Serial.println(F("* Seek/Tune Complete"));
        if(status & RDA5807M_STATUS_SF)
            Serial.println(F("* Seek Failed"));
        if(status & RDA5807M_STATUS_RDSS)
            Serial.println(F("* RDS Decoder Synchronized"));
        if(status & RDA5807M_STATUS_BLKE)
            Serial.println(F("* RDS Block E Found"));
        if(status & RDA5807M_STATUS_ST)
            Serial.println(F("* Stereo Reception"));
        Serial.println("}");
        Serial.flush();
        break;
      case '?':
        Serial.println(F("Available commands:"));
        Serial.println(F("* v/V     - decrease/increase the volume"));
        Serial.println(F("* s/S     - seek down/up with band wrap-around"));
        Serial.println(F("* m/M     - mute/unmute audio output"));
        Serial.println(F("* f       - display currently tuned frequency"));
        Serial.println(F("* q       - display RSSI for current station"));
        Serial.println(F("* t       - display decoded status register"));
        Serial.println(F("* ?       - display this list"));
        Serial.flush();
        break;
    }
  }
}
