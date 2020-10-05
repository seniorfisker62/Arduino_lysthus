#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};

// Ethernet UDP setup
unsigned int localPort = 8888;
EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
int led = 9;

// Temperature setup
int Sens1_clockPin = 3;  // pin used for clock sensor 1  lysthus temp
int Sens1_dataPin  = 2;  // pin used for data sensor 1   lysthus hum
int Sens2_clockPin = 7;  // pin used for clock sensor 2  ude temp
int Sens2_dataPin  = 6;  // pin used for data sensor 2   ude hum
int Sensor1 = 1;
int Sensor2 = 2;
int lamp = 9;

float temperature[3] = {0}; 
float humidity[3] = {0};
char tempstr[12]= {0};
unsigned long nextMeasure = 0;
unsigned long measureTime = 10000; 
static int nextSensor = Sensor1;

void setup() 
{ 
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while (!Serial) 
	{
	; // wait for serial port to connect. Needed for native USB port only
	}

	pinMode(led, OUTPUT);
	// start the Ethernet connection:
	Serial.println("Initialize Ethernet with DHCP:");
	if (Ethernet.begin(mac) == 0) {
		Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
		Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
	nextMeasure = millis() + measureTime;
	pinMode(lamp, OUTPUT);
  }
  // print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  Udp.begin(localPort);
}

// temperature and humidity measurement
//-------------------------------------
float getTemperature(int datapin, int clockpin)
{
  //Return Temperature in Celsius
  SHT_sendCommand(B00000011, datapin, clockpin);
  SHT_waitForResult(datapin);

  int val = SHT_getData(datapin, clockpin);
  SHT_skipCrc(datapin, clockpin);
  return (float)val * 0.01 - 40; //convert to celsius
}

float getHumidity(int datapin, int clockpin)
{
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, datapin, clockpin);
  SHT_waitForResult(datapin);
  int val = SHT_getData(datapin, clockpin);
  SHT_skipCrc(datapin, clockpin);
  return -4.0 + 0.0405 * val + -0.0000028 * val * val; 
}


void SHT_sendCommand(int command, int datapin, int clockpin)
{
  // send a command to the SHTx sensor
  // transmission start
  pinMode(datapin, OUTPUT);
  pinMode(clockpin, OUTPUT);
  digitalWrite(datapin, HIGH);
  digitalWrite(clockpin, HIGH);
  digitalWrite(datapin, LOW);
  digitalWrite(clockpin, LOW);
  digitalWrite(clockpin, HIGH);
  digitalWrite(datapin, HIGH);
  digitalWrite(clockpin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(datapin, clockpin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockpin, HIGH);
  pinMode(datapin, INPUT);

  if (digitalRead(datapin)) Serial.println("ACK error 0");
  digitalWrite(clockpin, LOW);
  if (!digitalRead(datapin)) Serial.println("ACK error 1");
}


void SHT_waitForResult(int datapin)
{
  // wait for the SHTx answer
  pinMode(datapin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i)
  {
    delay(2);
    ack = digitalRead(datapin);
    if (ack == LOW) break;
  }
  if (ack == HIGH) Serial.println("ACK error 2");
}

int SHT_getData(int datapin, int clockpin)
{
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(datapin, INPUT);
  pinMode(clockpin, OUTPUT);
  byte MSB = shiftIn(datapin, clockpin, MSBFIRST);

  // send the required ACK
  pinMode(datapin, OUTPUT);
  digitalWrite(datapin, HIGH);
  digitalWrite(datapin, LOW);
  digitalWrite(clockpin, HIGH);
  digitalWrite(clockpin, LOW);

  // get the LSB (less significant bits)
  pinMode(datapin, INPUT);
  byte LSB = shiftIn(datapin, clockpin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

void SHT_skipCrc(int datapin, int clockpin)
{
  // skip CRC data from the SHTx sensor
  pinMode(datapin, OUTPUT);
  pinMode(clockpin, OUTPUT);
  digitalWrite(datapin, HIGH);
  digitalWrite(clockpin, HIGH);
  digitalWrite(clockpin, LOW);
}

void measure(int sensor,int datapin, int clockpin)
{
	temperature[sensor] = getTemperature(datapin, clockpin);
  	humidity[sensor] = getHumidity(datapin, clockpin);	
}

// end of temperature and humidity measurement
//--------------------------------------------

void ReceiveMessage()
{
	int packetSize = Udp.parsePacket();
	  
	if(packetSize)
	{
		for(int i=0;i<UDP_TX_PACKET_MAX_SIZE;i++) packetBuffer[i] = 0;

		Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
		char valIn[packetSize];
  		for(int i = 0; i < packetSize; i++)
		{  
			valIn[i] = packetBuffer[i];
		}
		String theVal = valIn;
		int strIndex = theVal.length();

		if(theVal.substring(0,1) == "1") // temperature sensor 1
		{     		
			dtostrf(temperature[Sensor1], 6, 2, tempstr);			
		}
		else if(theVal.substring(0,1) == "2") // humidity sensor 1
		{		
			dtostrf(humidity[Sensor1], 6, 2, tempstr);			
		}    
		else if(theVal.substring(0,1) == "3") // temperature sensor 2
		{		
			dtostrf(temperature[Sensor2], 6, 2, tempstr);			
		}
		else if(theVal.substring(0,1) == "4") // humidity sensor 2
		{
			dtostrf(humidity[Sensor2], 6, 2, tempstr);
		}
		else if(theVal.substring(0,1) == "5") // lamp ON
		{
			digitalWrite(led, HIGH);
			strcpy(tempstr, "1");			
		}
		else if(theVal.substring(0,1) == "6") // lamp OFF
		{
			digitalWrite(led, LOW);
			strcpy(tempstr, "0");
		}
		else
		{				
			strcpy(tempstr, "error");			
		}
		Udp.beginPacket(Udp.remoteIP(), 8888);
		Udp.write(tempstr);
		Udp.endPacket();		
	} 
}
   

void loop() 
{
	static int cnt = 0;
	switch (Ethernet.maintain()) 
	{
		case 1:
			Serial.println("Error: renewed fail");
			break;

		case 2:      
			Serial.println("Renewed success");      
			Serial.print("My IP address: ");
			Serial.println(Ethernet.localIP());
			break;

		case 3:      
			Serial.println("Error: rebind fail");
			break;

		case 4:      
			Serial.println("Rebind success");      
			Serial.print("My IP address: ");
			Serial.println(Ethernet.localIP());
			break;

		default:      
			break;
	}
  
	ReceiveMessage(); 

	static int firsttime = 1;
	if (firsttime || (millis() >= nextMeasure))
	{
		nextMeasure = millis() + measureTime;
		
		if (nextSensor == Sensor1)
		{		
			measure(Sensor1,Sens1_dataPin,Sens1_clockPin);
			nextSensor = Sensor2;
		}else
		{
			measure(Sensor2,Sens2_dataPin,Sens2_clockPin);
			nextSensor = Sensor1;	
			firsttime = 0;			
		}		
	}
}		
