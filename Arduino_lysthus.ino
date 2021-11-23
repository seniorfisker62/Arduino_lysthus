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

// Temperature setup
int Sens1_clockPin = 3;  // pin used for clock sensor 1  lysthus temp
int Sens1_dataPin  = 2;  // pin used for data sensor 1   lysthus hum
int Sens2_clockPin = 7;  // pin used for clock sensor 2  ude temp
int Sens2_dataPin  = 6;  // pin used for data sensor 2   ude hum
int Sensor1 = 1;
int Sensor2 = 2;
// digital setup
int blaeser_input = 8;
int olievarme_input = 9;
int lampe_output = 5;

float temperature[3] = {0}; 
float humidity[3] = {0};
char tempstr[12]= {0};
unsigned long nextMeasure = 0;
unsigned long measureTime = 10000; 
static int nextSensor = Sensor1;

const int off = 0;
const int on = 1;
const int chgon = 2;
const int chgoff = 3;

const unsigned long maxmilli = 4294967296;
 
int blaeserstate = off;
int olievarmestate = off;
unsigned long blaeserstart = 0;
unsigned long olievarmestart = 0;
unsigned long varmetemp = 0;
unsigned long blaesertime = 0;
unsigned long olievarmetime = 0;
unsigned long lastms = 0;

unsigned long blaeserchgtimer = 0;
unsigned int olievarmechgtimer = 0;
unsigned int chgtime = 2000;
unsigned int lampestate = 0;

void setup() 
{ 
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	while (!Serial) 
	{
	; // wait for serial port to connect. Needed for native USB port only
	}
	
	// start the Ethernet connection:
	Serial.println("Lysthus controller");
	Serial.println("Initialize Ethernet with DHCP:");
	if (Ethernet.begin(mac) == 0) 
	{
		Serial.println("Failed to configure Ethernet using DHCP");
		if (Ethernet.hardwareStatus() == EthernetNoHardware) 
		{
			Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
		} else if (Ethernet.linkStatus() == LinkOFF) 
		{
			Serial.println("Ethernet cable is not connected.");
		}
		// no point in carrying on, so do nothing forevermore:
		while (true) 
		{
			delay(1);
		}	
	}
	// print your local IP address:
	Serial.print("My IP address: ");
	Serial.println(Ethernet.localIP());
	nextMeasure = millis() + measureTime;
	pinMode(blaeser_input, INPUT);
	pinMode(olievarme_input, INPUT);
	pinMode(lampe_output, OUTPUT);
	Udp.begin(localPort);	  
	// turn off output
	digitalWrite(lampe_output, HIGH);
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
		strcpy(tempstr, "" );
		
		if((theVal.substring(0,1) == "1") && (strIndex == 5)) // temperature sensor 1
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
		else if(theVal.substring(0,1) == "9") // blæser tid
		{
			dtostrf(blaesertime, 8, 0, tempstr);
			blaesertime = 0;						
		}
		else if((theVal.substring(0,1) == "1") && (theVal.substring(1,2) == "0")) // olieovn tid
		{
			dtostrf(olievarmetime, 8, 0, tempstr);
			olievarmetime = 0;
		}		
		else if(theVal.substring(0,1) == "5")
		{
			// turn on output
			digitalWrite(lampe_output, LOW);
			dtostrf(5, 8, 0, tempstr);
		}
		else if(theVal.substring(0,1) == "6")
		{
			// turn of output
			digitalWrite(lampe_output, HIGH);
			dtostrf(6, 8, 0, tempstr);
		}
		else
		{				
			Serial.println("error");
			strcpy(tempstr, "error");			
		}		
		
		Udp.beginPacket(Udp.remoteIP(), localPort);
		Udp.write(tempstr);
		Udp.endPacket();				
	} 
}
   
void checkblaeser()
{
	switch (blaeserstate)
	{
		case off:
				{
					if (digitalRead(blaeser_input))
					{	
						blaeserstart = millis();
						blaeserstate = chgon;
						blaeserchgtimer = (millis() + chgtime);
						Serial.println("blaeser chg on");
					}
					break;
				}

		case chgon:
				{
					if(millis() > blaeserchgtimer) 
					{	
						Serial.println("blaeser on");
						blaeserstate = on;
					}	
					break;
				}

		case chgoff:
				{
					if(millis() > blaeserchgtimer) 
					{			
						Serial.println("blaeser off");
						blaeserstate = off;	
					}	
					break;
				}

		case on:
				{	
					if (!digitalRead(blaeser_input))	
					{
						if (millis() < blaeserstart)
						{             
							varmetemp = (maxmilli - blaeserstart) + millis();							
						}else
						{
							varmetemp = millis() - blaeserstart;                    			
						}
						blaesertime += varmetemp;
						blaeserchgtimer = (millis() + chgtime);
						blaeserstate = chgoff;
						Serial.print("blaeser chg off : ");
						Serial.println(blaesertime / 1000);
					}	
					break;	
				}		
	}	
}   

void checkolieovn()
{
	switch (olievarmestate)
	{
		case off:
				{
					if (digitalRead(olievarme_input))
					{	
						olievarmestart = millis();
						olievarmechgtimer = (millis() + chgtime);
						olievarmestate = chgon;
						Serial.println("olievarmer chg on");
					}
					break;
				}
					
		case chgon:
				{
					if(millis() > olievarmechgtimer) 
					{						
						olievarmestate = on;
						Serial.println("olievarmer on");
					}	
					break;
				}

		case chgoff:
				{
					if(millis() > olievarmechgtimer) 
					{						
						Serial.println("olievarmer off");
						olievarmestate = off;	
					}	
					break;
				}	

		case on:
				{	
					if (!digitalRead(olievarme_input))	
					{
						if (millis() < olievarmestart)
						{             
							varmetemp = (maxmilli - olievarmestart) + millis();							
						}else
						{
							varmetemp = millis() - olievarmestart;                    			
						}
						olievarmetime += varmetemp;
						olievarmechgtimer = (millis() + chgtime);
						olievarmestate = chgoff;
						Serial.print("olievarmer chg off : ");
						Serial.println(olievarmetime / 1000);
					}	
					break;	
				}		
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
	
	// check if the millis() has been reset - happens after apr. 50 days
	if (millis() < lastms)
	{
		nextMeasure = 0; // force new measurement
		
	}
	lastms = millis();
	
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
	
	checkblaeser();
	checkolieovn();	

}		
