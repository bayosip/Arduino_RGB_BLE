#include <SPI.h>
#include <string.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


#define LED_R 						3
#define LED_G 						5
#define LED_B 						6
#define BTTN_ONOFF  				2
#define	NUM_COLORS					7
#define BUFSIZE     				128
#define ADC 						255
#define RSSI	 					"AT+BLEGETRSSI"
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define FACTORYRESET_ENABLE      	true
#define NEXT						"next"
#define PREVIOUS					"back"
#define ONOFF						"on/off"
#define BRIGHT						"bright"
#define DIM							"dim"
#define HEXA            "#"

float colors[NUM_COLORS][3] = {{1.0, 1.0, 1.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0},
  {0.0, 1.0, 0.0}, {0.0, 1.0, 1.0},
  {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}
};

int currentColor =		0;
volatile int ledState = 0;
int fadeAmount = 		5;    // how many points to fade the LED by
int brightness = 		ADC; // how bright the LED is
// Check for user input


/* The service information */
int32_t ledServiceId;
int32_t ledONOFFCharId;
int32_t ledStatusCharId;
uint32_t i;

void fadeOff(int color);
void lightUpLed(int i, int br);
void lightUpLed_(int port, int decVal);
void turnOnOffLed (int state);
void checkButtonOnOFF (void);
void fadeOn (int color);
void configureBLE(void);
void error(const __FlashStringHelper*err);
void talkBLE(void);
void updateLedStatus(void);
void changeToNextColor (void);
void changeToPreviousColor (void);
void dimLights(void);
void brightenLights(void);
boolean arrayIncludeElement(char *input, char element);
int hexadecimalToDecimal(String hexVal);
void processInstructionSet(String &str);

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small Flash print helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  configBLE();
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
}

void configBLE (void) {

  boolean success;
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit UART DATA mode Example"));
  Serial.println(F("-------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset!"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* disable verbose mode */
  ble.verbose(false);

  Serial.println(F("Setting device name to Osi_p BLE-LED`"));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Osi_p BLE-LED Controller" )) ) {
    error(F("Could not set device name!"));
  }

  Serial.println(F("Clearing previous Custom Service"));
  success = ble.sendCommandCheckOK( F("AT+GATTCLEAR"));
  if (! success) {
    error(F("Could not Clear service"));
  }

  Serial.println(F("Adding the LED status service definition(UUID = fe-33-e6-80-e2-0b-11-e5-aa-34-00-02-a5-d5-c5-1b): "));
  success = ble.sendCommandCheckOK( F("AT+GATTADDSERVICE=UUID128=fe-33-e6-80-e2-0b-11-e5-aa-34-00-02-a5-d5-c5-1b"));
  if (! success) {
    error(F("Could not add LED Status service"));
  }

  /* Add the LED status characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the LED Control characteristic (UUID = 0x0002): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x0002, PROPERTIES=0x08, MIN_LEN=1, VALUE=0"), &ledONOFFCharId);
  if (! success) {
    error(F("Could not add LED Status characteristic"));
  }

  /* Add the LED status characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the LED Status characteristic (UUID = 0x0003): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x0003, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=20, VALUE=Off"), &ledStatusCharId);
  if (! success) {
    error(F("Could not add LED Status characteristic"));
  }

  Serial.println(F("Please use app to connect arduino."));
  Serial.println(F("Awaiting connection..."));
  Serial.println();

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  // Set module to DATA mode
  Serial.println( F("Device connected: Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);


  Serial.println(F("******************************"));
}

void readBleRssi(void) {
  int n;
  // check response stastus
  // if (ble.available()) {
  ble.println(RSSI);
  ble.waitForOK();
  //}
  //	return i;
}

void talkBLE(void) {
  char n, inputs[BUFSIZE + 1];
  String str;

  // Echo received data
  if (ble.available()) {
    n = ble.readline(inputs, BUFSIZE + 1);
    ble.waitForOK();
    inputs[n] = 0;

    // display to user
    Serial.print("Receiving: ");
    
    if (inputs != NULL)
      Serial.println(inputs);
      str = String(inputs);
      processInstructionSet(str);
      return;
  }
}

void loop() {

  //readBleRssi();
  talkBLE();

}

void processInstructionSet(String &instruct){

  while (instruct != NULL) {
    Serial.println("instructons are:");
    Serial.println(instruct);
    if (instruct.equalsIgnoreCase( ONOFF)) {
      ledState = !ledState;
      Serial.println(F("Turn on or off LED"));
      turnOnOffLed(ledState);
      updateLedStatus();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }

    else if (instruct.equalsIgnoreCase("on")) {
      ledState = 1;
      Serial.println(F("Turn on LED"));
      turnOnOffLed(ledState);
      updateLedStatus();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }

    else if (instruct.equalsIgnoreCase("off")) {
      ledState = 0;
      Serial.println(F("Turn off LED"));
      turnOnOffLed(ledState);
      updateLedStatus();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }

    else if ( instruct.equalsIgnoreCase( NEXT)) {
      Serial.println(F("Change to next color"));
      changeToNextColor ();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }

    else if ( instruct.equalsIgnoreCase( PREVIOUS)) {
      Serial.println(F("Change to previous color"));
      changeToPreviousColor ();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }

    else if ( instruct.equalsIgnoreCase( DIM)) {
      Serial.println(F("Dim lights"));
      dimLights();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }

    else if ( instruct.equalsIgnoreCase(BRIGHT)) {
      Serial.println(F("Brighten lights"));
      brightenLights();
      Serial.println(F("Waiting for device instruction...."));
      break;
    }
    else {

      if (instruct.charAt(0) == '#'){
        lightUpLed_(LED_R, hexadecimalToDecimal(instruct.substring(1,3)));
         lightUpLed_(LED_G, hexadecimalToDecimal(instruct.substring(3,5)));
          lightUpLed_(LED_B, hexadecimalToDecimal(instruct.substring(5)));
          break;
      }
      else{
        Serial.println(F("Instuction Error!"));
        Serial.println(F("Waiting for device instruction...."));
        break;
      }
    }
  }
}

void updateLedStatus(void) {
  if (ledState == 1) {
    ble.print( F("AT+GATTCHAR=") );
    ble.println( ledStatusCharId );
    ble.print( F(", LED is: "));
    ble.println(F("On"));
  }
  else {
    ble.print( F("AT+GATTCHAR=") );
    ble.println( ledStatusCharId );
    ble.print( F(", LED is: "));
    ble.println(F("Off"));
  }
}

void changeToNextColor (void) {
  if (ledState == 1) {
    fadeOff(currentColor);
    if (++currentColor == NUM_COLORS) currentColor = 0;
    fadeOn(currentColor);
  }
  else {
    Serial.println(F("LED not on!!!"));
  }
}

void changeToPreviousColor (void) {
  if (ledState == 1) {
    fadeOff(currentColor);
    if (--currentColor < 0 ) currentColor = 6;
    fadeOn(currentColor);
  }
  else {
    Serial.println(F("LED not on!!!"));
  }
}

void dimLights(void) {
  if (ledState == 1) {
    if(brightness > 1) {
      lightUpLed(currentColor, brightness);
      brightness = brightness >> 1;
      // break of at lowest intensity
      if (brightness == 1)return;
    }
  }
  else {
    Serial.println(F("LED not on!!!"));
  }
}

void brightenLights(void) {
  if (ledState == 1) {
    if(brightness < ADC) {
      lightUpLed(currentColor, brightness);
      brightness = brightness << 1;

      // break off at max intensity:
      if (brightness == ADC) return;
    }
  }
  else {
    Serial.println(F("LED not on!!!"));
  }
}

void fadeOn (int color) {

  while (brightness < ADC) {
    lightUpLed(color, brightness);
    brightness = brightness + fadeAmount;

    // break out once at full brightness:
    if (brightness == ADC) {
      break;
    }
    // wait for 20 milliseconds to see the dimming effect
    delay(25);
  }
}

void fadeOff(int color) {
  while (brightness > 0) {
    lightUpLed(color, brightness);
    brightness = brightness - fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness == 0) {
      break;
    }
    // wait for 20 milliseconds to see the dimming effect
    delay(25);
  }
}

void lightUpLed_(int port, int decVal){
  analogWrite(port, decVal);
}

void lightUpLed(int i, int br) {

  analogWrite(LED_R, (colors[i][0])*br);
  analogWrite(LED_G, (colors[i][1])*br);
  analogWrite(LED_B, (colors[i][2])*br);
}

void turnOnOffLed (int state) {

  analogWrite(LED_R, (colors[currentColor][0]) * (brightness * state));
  analogWrite(LED_G, (colors[currentColor][1]) * (brightness * state));
  analogWrite(LED_B, (colors[currentColor][2]) * (brightness * state));
}

int hexadecimalToDecimal(String hexVal) 
{    
    Serial.println(F("Hex is: "));
    Serial.println(hexVal);
    int len = hexVal.length(); 
      
    // Initializing base value to 1, i.e 16^0 
    int base = 1; 
      
    int dec_val = 0; 
      
    // Extracting characters as digits from last character 
    for (int i=len-1; i>=0; i--) 
    {    
        // if character lies in '0'-'9', converting  
        // it to integral 0-9 by subtracting 48 from 
        // ASCII value. 
        if (hexVal.charAt(i)>='0' && hexVal.charAt(i)<='9') 
        { 
            dec_val += (hexVal.charAt(i) - 48)*base; 
                  
            // incrementing base by power 
            base = base * 16; 
        } 
  
        // if character lies in 'A'-'F' , converting  
        // it to integral 10 - 15 by subtracting 55  
        // from ASCII value 
        else if (hexVal.charAt(i)>='A' && hexVal.charAt(i)<='F') 
        { 
            dec_val += (hexVal.charAt(i) - 55)*base; 
          
            // incrementing base by power 
            base = base*16; 
        } 
    }  
    return dec_val; 
} 
