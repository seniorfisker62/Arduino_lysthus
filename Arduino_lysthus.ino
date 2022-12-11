#include <SPI.h>
#include <Ethernet.h>

byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x03
};

// Ethernet UDP setup
unsigned int localPort = 5000; //8889
EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

// Temperature setup
int Sens1_clockPin = 3;  // pin used for clock sensor 1  lysthus temp
int Sens1_dataPin  = 2;  // pin used for data sensor 1   lysthus hum
int Sens2_clockPin = 7;  // pin used for clock sensor 2  ude temp
int Sens2_dataPin  = 6;  // pin used for data sensor 2   ude hum
int Sensor1 = 1;
int Sensor2 = 2;

// digital setup
const int off = 0;
const int on = 1;
int lampe_output = 5;

float temperature[3] = {0}; 
float humidity[3] = {0};
char tempstr[12]= {0};
unsigned long nextMeasure = 0;
unsigned long measureTime = 10000; 
static int nextSensor = Sensor1;

static unsigned long firsttime = 0;

void setup() 
{ 
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while (!Serial) 
	{
	; // wait for serial port to connect. Needed for native USB port only
	}
	
	// start the Ethernet connection:
	Serial.println("Initialize Ethernet with DHCP:");
	if (Ethernet.begin(mac) == 0) 
	{
		Serial.println("Failed to configure Ethernet using DHCP");
		if (Ethernet.hardwareStatus() == EthernetNoHardware) 
		{
			Serial.println("Ethernet shield not found");
		} else if (Ethernet.linkStatus() == LinkOFF) 
		{
			Serial.println("Ethernet cable is not connected.");
		}
		// no point in carrying on
		while (true) 
		{
			delay(1);
		}			
	}
	// print your local IP address:
	Serial.print("My IP address: ");
	Serial.println(Ethernet.localIP());
	pinMode(lampe_output, OUTPUT);	
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

 // if (digitalRead(datapin)) Serial.println("ACK error 0");
  digitalWrite(clockpin, LOW);
 // if (!digitalRead(datapin)) Serial.println("ACK error 1");
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
		strcpy(tempstr, "" );		

		int msg = theVal.substring(0,1).toInt();

		switch (msg)
		{
			case 1:	{	dtostrf(temperature[Sensor1], 6, 2, tempstr);					
						break;
					}

			case 2:	{	dtostrf(humidity[Sensor1], 6, 2, tempstr);					
						break;
					}
			case 3:	{	dtostrf(temperature[Sensor2], 6, 2, tempstr);					
						break;
					}
			case 4:	{	dtostrf(humidity[Sensor2], 6, 2, tempstr);					
						break;
					}
			case 5:	{							
						digitalWrite(lampe_output, LOW);
						strcpy(tempstr, "5" );					
						break;
					}
			case 6:	{	digitalWrite(lampe_output, HIGH);
						strcpy(tempstr, "6" );								
						break;
					}
			default:{
						strcpy(tempstr, "error");
						break;
					} 
		} // switch 

		Udp.beginPacket(Udp.remoteIP(),Udp.remotePort() );
		Udp.write(tempstr);	
		Udp.endPacket();
	} 
}
  

void loop() 
{	
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
