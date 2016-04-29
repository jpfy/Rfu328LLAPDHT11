//Send DHT11 or DHT22 using LLAP to Computer
//I then use node-red to then convert messages to MQTT
//It uses the RFU328 very low power mode ATSM3 allowing batteries to last months a time

RFU328 can be found at wirelessthings.net



/////////////////////////////////////////////////////////////////////////////
//
// LLAP very low power example - RFu-328 only
//
///////////////////////////////////////////////////////////////////////////////
//
// Target:       Ciseco RFu-328
// version:      0.1
// date:         25 August 2013
//
// copyright/left, limitations to use etc. statement to go here
//
//////////////////////////////////////////////////////////////////////////////
//
// Reads the voltage at A0 e.g. for moisture sensor
// Connections:
// Probes connected between GND and Analog 0 (A0)
// 10kOhm resistor connected between pin 9 and Analog 0 (A0)
//
/////////////////////////////////////////////////////////////////////////////
//
// RFu-328 specific AT commands (requires firmware RFu-328 V0.84 or better)
//    The following RFu specific AT commands are supported in V0.84 or later
//		Added a new sleep mode (ATSM3) Wake after timed interval
//			The duration is specified by a new command ATSD, units are mS
//			and the range is 32bit (max approx. 49 days)
//				e.g. ATSD5265C00 is one day (hex 5265C00 is decimal 86400000)
//				ATSD36EE80 is one hour
//				ATSD493E0 is five minutes
//		This sleep mode can be used to wake the 328 after a timed interval
//			by connecting RFu-328 pin 7 to pin 19, D3(INT1) or 20, D2(INT0).
//		Using this technique means that a RFu-328 can sleep for a timed
//			period consuming about 0.6uA, instead of using the AVR328 watchdog (for timing)
//			which uses around 7uA.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <LLAPSerial.h>
#include <DHT.h>

#define DEVICEID "R3"	// this is the LLAP device ID

#define DHTPIN 5     // what I/O the DHT-22 data pin is connected to
#define DHTTYPE DHT11   // DHT   (AM2302)

// Connect pin 7 to 19
// Connect pin 1 (on the left) of the sensor to 3.3V
// Connect pin 2 of the sensor to whatever your DHTPIN is. pin 18 on rfu328
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

// Battery monitoring
#define BATTERY_READ_INTERVAL 10
int batteryCountDown = BATTERY_READ_INTERVAL;
/** readVcc - Read the current battery voltage
 * @return Vcc in millivolts
 */
int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
//  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vcc in millivolts
}
// end of battery monitoring code

/////////////////////////////////////////////////////////
// SRF AT command handling
/////////////////////////////////////////////////////////

uint8_t setupSRF()	// set Sleep mode 2
{
  if (!enterCommandMode())	// if failed once then try again
	{
		if (!enterCommandMode()) return 1;
	}
//Set time in sleepmode using ATSD followed by hex version of milliseconds
	
  //if (!sendCommand("ATSD49E30")) return 2;	// 5 minutes
  //if (!sendCommand("ATSD4E20")) return 2;	// 20 seconds
  //if (!sendCommand("ATSD1388")) return 2;	// 5 seconds
  // if (!sendCommand("ATSD3E8")) return 2;	// 1 second
  	if (!sendCommand("ATSD1D4C0")) return 2; //2 minutes
  	if (!sendCommand("ATSM3")) return 3;
  	if (!sendCommand("ATDN")) return 3;
  return 5;
}
//End of sleep mode

uint8_t enterCommandMode()
{
  delay(1200);
  Serial.print("+++");
  delay(500);
  while (Serial.available()) Serial.read();  // flush serial in
  delay(500);
  return checkOK(500);
}

uint8_t sendCommand(char* lpszCommand)
{
  Serial.print(lpszCommand);
  Serial.write('\r');
  return checkOK(100);
}

uint8_t checkOK(int timeout)
{
  uint32_t time = millis();
  while (millis() - time < timeout)
  {
    if (Serial.available() >= 3)
    {
      if (Serial.read() != 'O') continue;
      if (Serial.read() != 'K') continue;
      if (Serial.read() != '\r') continue;
      return 1;
    }
  }
  return 0;
}


void setup()
{
  Serial.begin(115200);         // Start the serial port

  LLAP.init(DEVICEID);                  // Initialise the LLAPSerial library and the device identity

  pinMode(8, OUTPUT);           // pin 8 controls the radio
  digitalWrite(8, HIGH);        // select the radio

  pinMode(4, OUTPUT);           // pin 4 controls the radio sleep 
  
  digitalWrite(4, LOW);        // wake the radio

  delay(400);						// let everything start up

  // Initialise countdown number of sleeps before sending battery reading
  batteryCountDown = BATTERY_READ_INTERVAL;
  // Wait for it to be initialised
  delay(200);
  // set up sleep mode 3 (low = awake)
  uint8_t val;
  while ((val = setupSRF()) != 5)
  {
	  LLAP.sendInt("ERR",val); // Diagnostic
	  delay(5000);	// try again in 5 seconds
  }

  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);	// No voltage to the sensor

	LLAP.sendMessage(F("STARTED"));  // send the usual "started message
}

void loop()
{
	  // Determine if we need to send a battery voltage reading or a distance reading
	  if( --batteryCountDown <= 0 ) {
	    int mV = readVcc();
	    LLAP.sendIntWithDP("B", mV, 3 );
	    batteryCountDown = BATTERY_READ_INTERVAL;
	    delay(20);
	  }

	digitalWrite(9,HIGH);
	delay(500);// provide voltage for the sensor
		dht.begin();
				int h = dht.readHumidity() * 10;
				int t = dht.readTemperature() * 10;
		 		// check if returns are valid, if they are NaN (not a number) then something went wrong!
				if (isnan(t) || isnan(h)) {
					LLAP.sendMessage(F("ERROR"));
				} else {
					LLAP.sendIntWithDP("HUM",h,1);
					//delay(100);
					LLAP.sendIntWithDP("TMP",t,1);
				}
				digitalWrite(9,LOW);

	pinMode(4, INPUT);        // sleep the radio
	LLAP.sleep(2, RISING, false);		// sleep until woken on pin 2, no pullup (low power)
	pinMode(4, OUTPUT);        // wake the radio
	//Might connect power to pin 9 and set pin high to supply voltage

 }

