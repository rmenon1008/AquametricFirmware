STARTUP(cellular_credentials_set("hologram", "", "", NULL));

#include <JsonParserGeneratorRK.h>
#include <math.h>

const int    THERMISTOR_PIN     = A5;
const int    SAMPLE_NUMBER      = 5;            // Constants for thermistor readings
const double BALANCE_RESISTOR   = 97200.0;
const double BETA               = 3974.0;
const double ROOM_TEMP          = 298.15;
const double RESISTOR_ROOM_TEMP = 100000.0;

double MAX_ADC            = 4095.0;

const int TRIG_PIN = D5;
const int ECHO_PIN = D6;

int sleepTime = 1;
bool updateAvailable = false;

SYSTEM_THREAD(ENABLED);                         // Allows the device to take sample measurements while attempting to make a cloud connection
SYSTEM_MODE(SEMI_AUTOMATIC);

ApplicationWatchdog wd(300000, shutdown);       // Turns off the device if measurement is not taken successfully after 5 minutes

FuelGauge fuel;

int runtime;

bool firstRun = true;

void setup(){
    Particle.subscribe("hook-response/liveConfig", myHandler, MY_DEVICES);
    Particle.connect();
    //Serial.begin(9600);
    RGB.control(true); 
    RGB.brightness(3);
    RGB.control(false);
    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D7, OUTPUT);
}

void loop() {
    runtime = millis();
    
    if(firstRun){
        digitalWrite(D7, HIGH);
        delay(250);
        digitalWrite(D7, LOW);
    }
    
    digitalWrite(D2, HIGH);
    digitalWrite(D3, HIGH);
    delay(500);
    JsonWriterStatic<256> doc;                  // Assembles sensor data into JSON string to be uploaded
	{
		JsonWriterAutoObject obj(&doc);

		doc.insertKeyValue("id", "002");
		doc.insertKeyValue("battery", fuel.getVCell());
		doc.insertKeyValue("stage", getRange());
		doc.insertKeyValue("temp", getTemp());
		doc.insertKeyValue("conductivity", getCond());
	}
	delay(2000);
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);
    
    while(!Particle.connected()){               // Waits here until device establishes a cloud connection
        if(firstRun){
            delay(900);
            digitalWrite(D7, HIGH);
            delay(200);
            digitalWrite(D7, LOW);
        }
        else{
            delay(1000);
        }
    }
    
    Particle.publish("liveConfig", PRIVATE);
    delay(3000);
    
    if(updateAvailable){
        digitalWrite(D7, HIGH);
        Particle.publish("ready_for_update");
        delay(120000);
    }
    
    Particle.publish("Measurement", doc.getBuffer());   // Publishes sensor measurement and sleeps
    shutdown();
    
    
}

void myHandler(const char *event, const char *data) {
    JsonParserStatic<1024, 10> parser;
    parser.addString(data);
    if (parser.parse()) {
		sleepTime = parser.getReference().key("002").key("update_freq").valueInt();
		updateAvailable = parser.getReference().key("002").key("ota_update").valueBool();
	}
}

void shutdown(){                                // Runs when a measurement is taken, or if the device fails to connect in time
    
    for(int i = 0; i < 7; i++){
        if(firstRun){
            digitalWrite(D7, HIGH);
            delay(50);
            digitalWrite(D7, LOW);
            delay(50);
        }
        else{
            delay(100);
        }
    }
    firstRun = false;
    runtime = millis() - runtime;
    System.sleep(SLEEP_MODE_SOFTPOWEROFF, (sleepTime*60)-(runtime/1000)-6);
}

int getRange(){
    
    int sum = 0;
    
    for(int i = 0; i < 5; i++){                 // Takes 5 ultrasonic measurements and returns the average in milliseconds
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        sum += rdPulseIn(ECHO_PIN, HIGH, 37)/5.82;
        delay(250);
    }
    
    return sum/5;

}

double getCond(){
    double sum = 0;
    
    for (int i = 0; i < 5; i++){
        sum += map(analogRead(A4),0.0,4095.0,0.0,1.0);     // Take 5 conductivity measurements and return average as value from 0 to 1
    }
    return sum/5;
}

double getTemp() 
{
    
  MAX_ADC = 1240.90*3.3;
  double rThermistor = 0;
  double tKelvin     = 0;
  double tCelsius    = 0;
  double adcAverage  = 0;
  int    adcSamples[SAMPLE_NUMBER];
  
  for (int i = 0; i < SAMPLE_NUMBER; i++) 
  {
    adcSamples[i] = analogRead(THERMISTOR_PIN);
    delay(10);
  }

  for (int i = 0; i < SAMPLE_NUMBER; i++) 
  {
    adcAverage += adcSamples[i];                // Take the average of 5 temperature readings
  }
  adcAverage /= SAMPLE_NUMBER;

  rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / adcAverage) - 1);
  tKelvin = (BETA * ROOM_TEMP) / 
            (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));
  tCelsius = tKelvin - 273.15;
  return tCelsius;                              // Return the temperature in Celsius
}
/*
double getTurb() 
{
    int sum = 0;
    
    for (int i = 0; i < 5; i++){
        sum += map(analogRead(A2),0.0,4095.0,0.0,1.0);     // Take 5 conductivity measurements and return average as value from 0 to 1
    }
    return sum/5;
}*/

unsigned long rdPulseIn(int pin, int value, int timeout) { // Keeps track of microseconds between sending and receieving a signal from the ultrasonic sensor
    
    unsigned long now = micros();
    while(pinReadFast(pin) == value) {
        if (micros() - now > (timeout*1000)) {
            return 0;
        }
    }
    
    now = micros();
    while (pinReadFast(pin) != value) {
        if (micros() - now > (timeout*1000)) { 
            return 0;
        }
    }
    
    now = micros();
    while (pinReadFast(pin) == value) {
        if (micros() - now > (timeout*1000)) {
            return 0;
        }
    }
    return micros() - now;
}
