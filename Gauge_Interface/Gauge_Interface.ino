// See Gauge_Disp for comments on redundant structures, both were the same program at one time. I'm not going to re-comment this whole program right now.
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <SoftwareSerial.h>
//create object
EasyTransferI2C ETrx;
EasyTransferI2C ETtx;
int ran;
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int request;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO. Seems like this is limited to 28 bytes?
  //If you need more space, use Serial.
  int AFR;
  int MRP;
  int RPM;
  int IAT;
  int IDC;
  int TPS;
  int KCK;
  byte LKC;
  byte MAF;
  byte FUL;
  byte GER;
  byte SPD;
  int OIL;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

#define sendSerial Serial1 //Remaps SoftSerial port used on Due to Serial1 for Micro
//SoftwareSerial sendSerial(3, 2); //Rx, Tx - For ECU Data
SoftwareSerial auxSerial(9, 8); //Rx, Tx - For Wideband Serial Stream

//ECU variables
byte ECUbytes[30];//must be at least as large as data received or will crash.
byte pollECUbytes[82];//must be at least as large as data sent or will crash.

int pollECUlength = 0;//size of packet to xmit (populated by packet builder)
int readECUlength = 6;//size of packet to receive, initialized with header and checksum count.

boolean dataSent = false;//Indicates whether data request packet has been sent to ECU
boolean dataReady = false;//Indicates whether valid data has been recieved from the ECU stream.
boolean readingAFR = false;//Indicates that it is now trying to read AFR values.
boolean readingECU = false;//Indicates that it is now trying to read ECU values.

boolean diagnostics=true; //Enables dev features for diags and testing


// All of these variable declarations need to be cleaned up and organized. Some probably aren't even used, or are used and shouldn't be.
long AFRTimer=0;
long timeAFR = 0;
long timeScan = 0;
long sScan = 1000; //number of milliseconds to wait for ECU data.
long timeProc = 0;
long sProc = 90;

float AFR=22.8;
float MRP=0.0;
int RPM=0;
int IAT=0;
int IDC=0;
int TPS=0;
float KCK=0.0;
byte LKC;
byte MAF;
byte FUL;
byte GER;
byte SPD;
byte LAT;
byte YAW;
byte DIF;
int OIL;

void setup(){
  Serial.begin(57600); //USB port for diagnostics
  Wire.onReceive(receive);
  Wire.begin(9);
  TWBR=0x0C; // (DEC = 12)
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc. 
 ETrx.begin(details(rxdata), &Wire);
 ETtx.begin(details(txdata), &Wire);
   
  
  sendSerial.begin(4800); //SSM uses 4800 8N1 baud rate (OBD ELM/CAN Adapter uses 9600)
  auxSerial.begin(19200); //2nd Serial Port for WidebandO2 or Terminal Passthrough
  
        
 
 buildPkt(); // Calls the subroutine to assemble the request packet.
  
}

void loop() {
  if(ETrx.receiveData()){
      if(rxdata.request==1){ //This means we have a data request from the display.
        delay(1); //May need a delay here to prevent response from interfering with request.
        ETtx.sendData(8); //Send response data to display.
        rxdata.request=0; //Reset request flag
      }
    
  }
  
  if (auxSerial.available() >= 6){ //If at least 6 bytes are waiting from the Wideband, process the data.
            timeAFR=millis(); //For testing
            readAFR();
            Serial.print(F("AFRead:")); //For testing
            Serial.println(millis()-timeAFR); //For testing
        }
  
  if (!dataSent){ //Sends ECU request packet when dataSent flag is false
                Serial.print("R!");  //For testing
		while (sendSerial.available() > 0) sendSerial.read(); //Clears out input buffer? This may be a waste of time, remove it later to see if it breaks anything
		writeSSM(pollECUbytes, pollECUlength); //Send Data Request to ECU
		dataSent = true; //Set flag showing that data was sent and to expect a response
	}

	

	if (dataSent && !dataReady){ //If the data was sent, but no response yet
		if (millis() > timeScan){ //This resets dataSent flag after a timout period.
			dataSent = false;
                        timeScan = millis() + sScan;
		}
		if (sendSerial.available() >= readECUlength){ //Watches for enough data to make a packet
			readECU(); //Reads the data.
                        timeScan = millis() + sScan; //Resets request timeout.
		}
	}
        
        if (diagnostics) processData(); //Bypasses data checks for bench testing.
        
        if (dataReady) { //If the data was received and the packet was good, process the information.
		processData();
                timeProc = millis() + sProc; //Stray?
		dataReady = false; //Tell loop to look for the next data packet.

	}
  
}

void receive(int numBytes) {} //For EZ-Transfer

void buildPkt(){
	
	byte ECUheader[7] = { 128, 16, 240, 0, 168, 01, 00 };//Begin,Begin,Begin,Length,Address(single),(00=request/response, 01=stream),Space for CheckSum

	byte params[21][4] = { // These are the addressed that will be added to the request packet. Order here will be order of data in response
                        //MPG Calculation = (([MPH]/3600)/(([MAF]/[AFR])/2880))
		        //{0,0,36,1},//MRP This address was limited to 20psi
			{ 255, 77, 208, 4 },//MRP1
			{ 255, 77, 209, 3 },//MRP2
			{ 255, 77, 210, 2 },//MRP3
			{ 255, 77, 211, 1 },//MRP4
			{ 0, 0, 18, 1 },//IAT
			{ 0, 0, 32, 1 },//IPW
			{ 0, 0, 14, 2 },//RPM1
			{ 0, 0, 15, 1 },//RPM2
			{ 0, 0, 21, 1 },//TPS
			{ 255, 88, 204, 4 },//FBKC1
			{ 255, 88, 205, 3 },//FBKC2
			{ 255, 88, 206, 2 },//FBKC3
			{ 255, 88, 207, 1 },//FBKC4
                        {   0,  1, 153, 1 },//LKC ((x*0.25)-32)
                        {   0,  0,  13, 1 },//MAF (x/100)
                        {   0,  0,  46, 1 },//Fuel Level V (x/50)
                        {   0,  0,  74, 1 },//Gear (x+1)
                        {   0,  0,  16, 1 },//Speed (x*0.621371192)
                        //{   0,  1, 241, 1 },//Lateral G m/sec2? (x*1.0862), Doesn't work on '06
                        //{   0,  1, 242, 1 },//Yaw deg/s (x*0.19118), Doesn't work on '06
                        //{   0,  1, 243, 1 },//DCCD (x), Doesn't work on '06


	};

	byte ECUcheck = 0;//Checksum
	int pointECUbytes = 0;//pointer for xmit packet assembly
	byte sumBytes = 0;//accumulates bytes for checksum

	/* This part is hard to read, but it assembles the data packet to request information from the ECU.
	 I was getting sick of pre-assembling packets in EXCEL.I'll try to add lots of comments so that I know what I did here later.
	 Make sure that the destination arrays are large enough to contain the packets or weird stuff happens (usually crashes).*/
	ECUheader[3] = (sizeof(ECUheader) - 5 + sizeof(params) - sizeof(params) / 4); //Calculates packet length byte and inserts it into header array.
	pollECUlength = ECUheader[3] + 5; //Calculates the total number of bytes in the packet for use by the sendSSM subroutine.
	for (int i = 0; i < (sizeof(ECUheader) - 1); i++){ //Goes through the header (except for the last byte where I tacked on the checksum) 
		ECUcheck += ECUheader[i]; //adds up bytes for checksum calculation
		pollECUbytes[pointECUbytes] = ECUheader[i]; //puts header bytes into ECU request array
		pointECUbytes++; //increments counter for ECU request array
	}
	int numParams = sizeof(params) / 4; //gets number of 4-byte parameter addresses specified in params array. 
	for (int i = 0; i < numParams; i++){ //loops through parameters
		for (int j = 0; j < sizeof(params[numParams]) - 1; j++){ //loops through first 3 bytes in each parameter, 4th byte isn't currently used.
			ECUcheck += (params[i][j]); //adds parameter bytes for checksum calculation
			pollECUbytes[pointECUbytes] = params[i][j]; //puts parameter bytes into ECU request array
			pointECUbytes++; //increments counter for ECU request array
		}
	}
	ECUheader[sizeof(ECUheader)] = CheckSum(ECUcheck); //Takes the last byte in the header array (skipped previously), calculates the checksum of the packet, and inserts it there.
	pollECUbytes[pointECUbytes] = ECUheader[sizeof(ECUheader)]; //Copies the checksum to the ECU request array to complete the packet.
	pointECUbytes++; //increments counter for ECU request array
	readECUlength += numParams; //This variable tells the program how many response bytes it should expect back from the ECU.

	for (int i = 0; i < pollECUlength; i++){//Displays assembled packet on startup screen.
		Serial.print(pollECUbytes[i]);
		Serial.print(" ");
	}
	Serial.println();
	Serial.print(F("S:")); //Displays necessary request packet array size on startup screen.
	Serial.println(pollECUlength);
	Serial.print(F("R:")); //Displays necessary response packet array size on startup screen.
	Serial.println(readECUlength);
}

void readAFR(){
	byte AFRpacketStart[] = { 178, 130, 67, 19 }; //This is the sequence we look for in the datastream from the LC-1 wideband, data is the two bytes after this pattern.
	readingAFR = false;
	int bytesFound = 0;
	int thisByte = 0;
	int AFRloop = 0;
	byte AFRByte1 = 0;
	byte AFRByte2 = 0;
	byte AFRloopLimit = 70; //WAAAAY Excessive
	byte AFRloopDelay = 1;
        long AFRDiag=millis();

	while (bytesFound < 4 && AFRloop < AFRloopLimit){ //Look for byte sequence in buffer that matches known header pattern
		if (auxSerial.available()){
			thisByte = auxSerial.read();
			if (thisByte == AFRpacketStart[bytesFound]){
				bytesFound++;
			}
			else{
				if (bytesFound == 2 && thisByte == 83){// Use this to set Sensor Warmup Flag
					txdata.AFR = 225; //Values are going directly to int so they are just x10.
					return;
				}
				if (bytesFound == 2 && thisByte == 79){//Use this to set Calibration Request Flag
					txdata.AFR = 226;
					return;
				}
				if (bytesFound == 2 && thisByte == 91){//Use this to set ERROR Flag.
					txdata.AFR = 227;
					return;
				}
				bytesFound = 0; //If a functional pattern wasn't found, reset the byte counter and keep looking.
				AFRloop++;
			}
		}
		else{
			delay(AFRloopDelay);
			AFRloop++;
		}
	}

	while (bytesFound < 6 && AFRloop < AFRloopLimit){ //The pattern was found, so now grab the two bytes that contain the AFR value.
                Serial.print(F("F@:")); //for testing
                Serial.println(millis()-AFRDiag);//for testing
		AFRTimer=millis();//for testing
                if (bytesFound == 4){ // If we get here, it means the header bytes were found.
			if (auxSerial.available()){ //Get first byte
				AFRByte1 = auxSerial.read();
				bytesFound = 5;
				AFR = AFRByte1 * 128;
			}
			else{
				delay(AFRloopDelay);
				AFRloop++;
			}
		}
		if (bytesFound == 5){
			if (auxSerial.available()){ //Get second byte
				AFRByte2 = auxSerial.read();
				bytesFound = 6;
				AFR = AFR + AFRByte2;
			}
			else{
				delay(AFRloopDelay);
				AFRloop++;
			}
		}
	}

	if (bytesFound == 6){ //Convert the raw data to something usable.
		AFR = ((AFR + 500) * 147) / 10000;
		if (AFR > 22.3)AFR = 22.3; //Constrain values to what the display uses.
		if (AFR < 9.0)AFR = 9.0;
                readingAFR=false; //I don't think this flag is used anymore?
                txdata.AFR=AFR*10; // Convert to integer for xfer.
		return;
	}
	else{
		AFR = 22.4; //Special value for a bad reading
                readingAFR=false;
                txdata.AFR=AFR*10;
                Serial.print(F("AFRTO:")); //for testing
                Serial.println(bytesFound); //for testing
		return;
	}
}

void readECU(){
	int bytesFound = 0;
	int thisByte = 0;
	int sumB = 616; //Starting value is sum of header bytes except readSize
	int ECUloop = 0;
	byte ECUloopLimit = 20;
	byte readECUstart[3] = { 128, 240, 16 };
	byte readSize = 0;
	int loopDelay = 1;
	while (bytesFound < 3 && ECUloop < ECUloopLimit){ //Look for byte sequence in buffer that matches known header pattern
		thisByte = sendSerial.read();
                Serial.print("Srch:");Serial.println(thisByte); //for testing
		if (thisByte == readECUstart[bytesFound]){
			bytesFound++;
		}
		else{
			delay(loopDelay);
			ECUloop++;
			bytesFound = 0;
		}
	}
        if(bytesFound==3) Serial.println("Header Found");
	while (bytesFound == 3 && ECUloop < ECUloopLimit){ //Get size byte after header
		if (sendSerial.available()){
			readSize = sendSerial.read();
                        Serial.print("PktSz:");Serial.println(readSize); //for testing
			bytesFound = 4;
			sumB = sumB + readSize; //Include size byte in checksum
		}
		else{
			delay(loopDelay);
			ECUloop++;
		}
	}

	while (bytesFound == 4 && ECUloop < ECUloopLimit){ //Get last header byte, not used for anything right now
		if (sendSerial.available()){
			thisByte = sendSerial.read();
                        Serial.print("EndHd:");Serial.println(thisByte); //for testing
			bytesFound = 5;
		}
		else{
			delay(loopDelay);
			ECUloop++;
		}

	}

	while (bytesFound < readSize + 4 && ECUloop < ECUloopLimit){ //Now just read the rest of the packets into ECU Array (the next readSize-1 bytes are data, so add 5 to readSize to compensate for bytesFound==5, then subtract 1 for the checkSum).

		if (sendSerial.available()){
			ECUbytes[bytesFound - 5] = sendSerial.read();
                         Serial.print("Byte");Serial.print(bytesFound-5);Serial.println(ECUbytes[bytesFound - 5]); //for testing
			sumB = sumB + ECUbytes[bytesFound - 5];
			bytesFound++;
		}
		else{
			delay(loopDelay);
			ECUloop++;
		}
	}

	while (bytesFound == readSize + 4 && ECUloop < ECUloopLimit){ //Get the last byte which should be the checkSum
		if (sendSerial.available()){
			thisByte = sendSerial.read();
			bytesFound++;
		}
		else{
			delay(loopDelay);
			ECUloop++;
		}
	}
	if (bytesFound == readSize + 5 && CheckSum(sumB) == thisByte){ //This checks if the data is good.
		Serial.println("ECU DATA OK"); //for testing
                readingECU = false;
		dataReady = true;
		bytesFound = 0;
		return;
	}
	else{
		Serial.print("ECU Data Bad:"); //for testing
                Serial.print(CheckSum(sumB)); //for testing
                Serial.print("/"); //for testing
                Serial.println(thisByte); //for testing
                readingECU = false;
		dataReady = false;
		bytesFound = 0;
		return;
	}
}

byte CheckSum(byte sum) { // returns the 8 least significant bits of an input byte
	byte counter = 0;
	byte power = 1;
	for (byte n = 0; n < 8; n++) {
		counter += bitRead(sum, n) * power;
		power = power * 2;
	}
	return counter;
}

void processData(){
	int paraNum = 0; //Keeps track of which parameter we are accessing, first one is at position 0

	// interpret ECU data
        int MRPbyte1 = ECUbytes[paraNum];
	paraNum++;
	int MRPbyte2 = ECUbytes[paraNum];
	paraNum++;
	int MRPbyte3 = ECUbytes[paraNum];
	paraNum++;
	int MRPbyte4 = ECUbytes[paraNum];
	paraNum++;

	int IATbyte = ECUbytes[paraNum];
	paraNum++;

	int IPWbyte = ECUbytes[paraNum];
	paraNum++;

	int RPMbyte1 = ECUbytes[paraNum];
	paraNum++;
	int RPMbyte2 = ECUbytes[paraNum];
	paraNum++;

	int TPSbyte = ECUbytes[paraNum];
	paraNum++;

	int KCKbyte1 = ECUbytes[paraNum];
	paraNum++;
	int KCKbyte2 = ECUbytes[paraNum];
	paraNum++;
	int KCKbyte3 = ECUbytes[paraNum];
	paraNum++;
	int KCKbyte4 = ECUbytes[paraNum];
	paraNum++;

        txdata.LKC = ECUbytes[paraNum];
	paraNum++;
        
        txdata.MAF = ECUbytes[paraNum];
	paraNum++;
       
        txdata.FUL = ECUbytes[paraNum];
	paraNum++;

        txdata.GER = ECUbytes[paraNum];
	paraNum++;
        
        txdata.SPD = ECUbytes[paraNum];
	paraNum++;
        
        
	//Translates raw bytes into usable values, remember that AFR comes from a different source.
	float MRP = computeMRP(MRPbyte1, MRPbyte2, MRPbyte3, MRPbyte4);
	int IAT = computeIAT(IATbyte);
	int IDC = computeIDC(IPWbyte, RPMbyte1, RPMbyte2);
	long RPM = computeRPM(RPMbyte1, RPMbyte2);
	int TPS = computeTPS(TPSbyte);
	float KCK = computeKCK(KCKbyte1, KCKbyte2, KCKbyte3, KCKbyte4);
        txdata.OIL=analogRead(A0);
        Serial.print("OIL:");
        Serial.println(txdata.OIL);
        
	if(diagnostics){ //Generates Test Values, uses too much memory, remove when not dev.
            float mover=random(100)/100.00;
            AFR= 9.0 + ((mover)*13.3);
            txdata.AFR=AFR*10;
            MRP= -11.0 + ((mover)*31.0); 
            RPM= (mover) * 7000;
            txdata.LKC=255*mover;
            txdata.MAF=255*mover;
            txdata.FUL=255*mover;
            txdata.GER=255*mover;
            txdata.SPD=255*mover;
	}
        
        //Assigns data to variables that will be transmitted to display.
        txdata.MRP=MRP*10;
        txdata.IAT=IAT;
        txdata.IDC=IDC;
        txdata.RPM=RPM;
        txdata.TPS=TPS;
        txdata.KCK=KCK*10;

}

void writeSSM(byte data[], byte length) {   //Sends data request packet to the ECU
	for (byte x = 0; x < length; x++) {
		sendSerial.write(data[x]);
                sendSerial.read(); //K-line Serial interface causes echo, this removes each echo byte from the buffer as it's created (probably not necessary)
	}
}

int computeIAT(int IAT)
{
	//int temperature = IAT - 40; //What is this metric shit?
	int temperature = 32 + 9 * (IAT - 40) / 5; //That's better! :)
	//if (IAT < -100 || IAT>300) IAT = 0;
	return temperature;
}

float computeMRP(int boost1, int boost2, int boost3, int boost4)
{
              /*This next one is a tricky little bastard. Converting 4-byte values into usable numbers is a royal pain in the ass.
        This little beauty magically links two memory registers together so that anything put in one can be read as the other.
        Just populate the conv.asBytes[] array in reverse order, and then read the results as conv.asFloat.*/
        union{
        	byte asBytes[4];
        	float asFloat;
        } conv;
	conv.asBytes[3] = boost1;
	conv.asBytes[2] = boost2;
	conv.asBytes[1] = boost3;
	conv.asBytes[0] = boost4;
	float mrp = (conv.asFloat / 51.715);
	//if (mrp < -15 || mrp>30) mrp = 0;
	return mrp;
}


float computeKCK(int fbkc1, int fbkc2, int fbkc3, int fbkc4){
              /*This next one is a tricky little bastard. Converting 4-byte values into usable numbers is a royal pain in the ass.
        This little beauty magically links two memory registers together so that anything put in one can be read as the other.
        Just populate the conv.asBytes[] array in reverse order, and then read the results as conv.asFloat.*/
        union{
        	byte asBytes[4];
        	float asFloat;
        } conv;
	conv.asBytes[3] = fbkc1;
	conv.asBytes[2] = fbkc2;
	conv.asBytes[1] = fbkc3;
	conv.asBytes[0] = fbkc4;
	float kck = conv.asFloat;
	//if (kck < -10 || kck>1) kck = 0;
	return kck;
}

int computeIDC(int IPW, int rpm1, int rpm2)
{
	float rpm = ((rpm1 * 256) + rpm2) / 4;
	float IPWcor = IPW * 256 / 1000;
	int IDC = (IPWcor * rpm) / (1200);
	//if (IDC < 0 || IDC>120) IDC = 0;
	return IDC;
}


int computeRPM(int rpm1, int rpm2) //OEM Tach reads a bit high?? The equation is right as far as I can tell.
{
	float rpm = ((rpm1 * 256) + rpm2) / 4;
	//if (rpm < 0 || rpm>9999) rpm = 0;
	return rpm;
}

float computeTPS(int TPS)
{
	int tps = TPS * 100 / 255;
	//if (tps < 0 || tps>100) tps = 0;
	return tps;
}

/*

Table of standard addresses for 2006 USDM Subaru Impreza WRX STi. (Theoretical since it includes unsupported stuff).
For instance the ECU doesn't actually get any DCCD data even if it could. And it isn't a diesel either.
This is just for quick reference.
								
P1	0x000007	0	0	7	Engine Load (Relative)	%	x*100/255	8
P2	0x000008	0	0	8	Coolant Temperature	F	32+9*(x-40)/5	8
P3	0x000009	0	0	9	A/F Correction #1	%	(x-128)*100/128	8
P4	0x00000A	0	0	10	A/F Learning #1	%	(x-128)*100/128	8
P5	0x00000B	0	0	11	A/F Correction #2	%	(x-128)*100/128	8
P6	0x00000C	0	0	12	A/F Learning #2	%	(x-128)*100/128	8
P7	0x00000D	0	0	13	Boost Pressure actual	psi	x*37/255	8
P8	0x00000E	0	0	14	Engine Speed	rpm	x/4	16
P9	0x000010	0	0	16	Vehicle Speed	mph	x*0.621371192	8
P10	0x000011	0	0	17	Ignition Total Timing	degrees	(x-128)/2	8
P11	0x000012	0	0	18	Intake Air Temperature	F	32+9*(x-40)/5	8
P12	0x000013	0	0	19	Mass Airflow	g/s	x/100	16
P13	0x000015	0	0	21	Throttle Opening Angle	%	x*100/255	8
P14	0x000016	0	0	22	Front O2 Sensor #1	V	x/200	16
P15	0x000018	0	0	24	Rear O2 Sensor	V	x/200	16
P16	0x00001A	0	0	26	Front O2 Sensor #2	V	x/200	16
P17	0x00001C	0	0	28	Battery Voltage	V	x*8/100	8
P18	0x00001D	0	0	29	Mass Airflow Sensor Voltage	V	x/50	8
P19	0x00001E	0	0	30	Throttle Sensor Voltage	V	x/50	8
P20	0x00001F	0	0	31	Differential Pressure Sensor Voltage	V	x/50	8
P21	0x000020	0	0	32	Fuel Injector #1 Pulse Width	ms	x*256/1000	8
P22	0x000021	0	0	33	Fuel Injector #2 Pulse Width	ms	x*256/1000	8
P23	0x000022	0	0	34	Knock Correction Advance	degrees	(x-128)/2	8
P24	0x000023	0	0	35	Atmospheric Pressure	psi	x*37/255	8
P25	0x000024	0	0	36	Manifold Relative Pressure	psi	(x-128)*37/255	8
P26	0x000025	0	0	37	Pressure Differential Sensor	psi	(x-128)*37/255	8
P27	0x000026	0	0	38	Fuel Tank Pressure	psi	(x-128)*35/10000	8
P28	0x000027	0	0	39	CO Adjustment	V	x/50	8
P29	0x000028	0	0	40	Learned Ignition Timing	degrees	(x-128)/2	8
P30	0x000029	0	0	41	Accelerator Pedal Angle	%	x*100/255	8
P31	0x00002A	0	0	42	Fuel Temperature	F	32+9*(x-40)/5	8
P32	0x00002B	0	0	43	Front O2 Heater Current #1	A	x*1004/25600	8
P33	0x00002C	0	0	44	Rear O2 Heater Current	A	x*1004/25600	8
P34	0x00002D	0	0	45	Front O2 Heater Current #2	A	x*1004/25600	8
P35	0x00002E	0	0	46	Fuel Level	V	x/50	8
P36	0x000030	0	0	48	Primary Wastegate Duty Cycle	%	x*100/255	8
P37	0x000031	0	0	49	Secondary Wastegate Duty Cycle	%	x*100/255	8
P38	0x000032	0	0	50	CPC Valve Duty Ratio	%	x*100/255	8
P39	0x000033	0	0	51	Tumble Valve Position Sensor Right	V	x/50	8
P40	0x000034	0	0	52	Tumble Valve Position Sensor Left	V	x/50	8
P41	0x000035	0	0	53	Idle Speed Control Valve Duty Ratio	%	x/2	8
P42	0x000036	0	0	54	A/F Lean Correction	%	x*100/255	8
P43	0x000037	0	0	55	A/F Heater Duty	%	x*100/255	8
P44	0x000038	0	0	56	Idle Speed Control Valve Step	steps	x	8
P45	0x000039	0	0	57	Number of Exh. Gas Recirc. Steps	steps	x	8
P46	0x00003A	0	0	58	Alternator Duty	%	x	8
P47	0x00003B	0	0	59	Fuel Pump Duty	%	x*100/255	8
P48	0x00003C	0	0	60	Intake VVT Advance Angle Right	degrees	x-50	8
P49	0x00003D	0	0	61	Intake VVT Advance Angle Left	degrees	x-50	8
P50	0x00003E	0	0	62	Intake OCV Duty Right	%	x*100/255	8
P51	0x00003F	0	0	63	Intake OCV Duty Left	%	x*100/255	8
P52	0x000040	0	0	64	Intake OCV Current Right	mA	x/32	8
P53	0x000041	0	0	65	Intake OCV Current Left	mA	x/32	8
P54	0x000042	0	0	66	A/F Sensor #1 Current	mA	(x-128)/8	8
P55	0x000043	0	0	67	A/F Sensor #2 Current	mA	(x-128)/8	8
P56	0x000044	0	0	68	A/F Sensor #1 Resistance	ohms	x	8
P57	0x000045	0	0	69	A/F Sensor #2 Resistance	ohms	x	8
P58	0x000046	0	0	70	A/F Sensor #1	Lambda	x/128	8
P59	0x000047	0	0	71	A/F Sensor #2	Lambda	x/128	8
P60	0x00004A	0	0	74	Gear Position	gear	x+1	8
P61	0x000053	0	0	83	A/F Sensor #1 Heater Current	A	x/10	8
P62	0x000054	0	0	84	A/F Sensor #2 Heater Current	A	x/10	8
P63	0x0000CE	0	0	206	Roughness Monitor Cylinder #1	misfire count	x	8
P64	0x0000CF	0	0	207	Roughness Monitor Cylinder #2	misfire count	x	8
P65	0x0000D0	0	0	208	A/F Correction #3 (16-bit ECU)	%	(x-128)*100/128	8
P66	0x0000D1	0	0	209	A/F Learning #3	%	(x-128)*100/128	8
P67	0x0000D2	0	0	210	Rear O2 Heater Voltage	V	x/50	8
P68	0x0000D3	0	0	211	A/F Adjustment Voltage	V	x/50	8
P69	0x0000D8	0	0	216	Roughness Monitor Cylinder #3	misfire count	x	8
P70	0x0000D9	0	0	217	Roughness Monitor Cylinder #4	misfire count	x	8
P71	0x0000FA	0	0	250	Throttle Motor Duty	%	(x-128)*100/128	8
P72	0x0000FB	0	0	251	Throttle Motor Voltage	V	x*8/100	8
P73	0x000100	0	1	0	Sub Throttle Sensor	V	x/50	8
P74	0x000101	0	1	1	Main Throttle Sensor	V	x/50	8
P75	0x000102	0	1	2	Sub Accelerator Sensor	V	x/50	8
P76	0x000103	0	1	3	Main Accelerator Sensor	V	x/50	8
P77	0x000104	0	1	4	Brake Booster Pressure	psi	x*37/255	8
P78	0x000105	0	1	5	Fuel Pressure (High)	psi	x/25*145.0377	8
P79	0x000106	0	1	6	Exhaust Gas Temperature	F	32+9*(x+40)	8
P80	0x000108	0	1	8	Cold Start Injector (Air Pump)	ms	x*256/1000	8
P81	0x000109	0	1	9	SCV Step	steps	x	8
P82	0x00010A	0	1	10	Memorised Cruise Speed	mph	x*0.621371192	8
P83	0x000118	0	1	24	Exhaust VVT Advance Angle Right	degrees	x-50	8
P84	0x000119	0	1	25	Exhaust VVT Advance Angle Left	degrees	x-50	8
P85	0x00011A	0	1	26	Exhaust OCV Duty Right	%	x*100/255	8
P86	0x00011B	0	1	27	Exhaust OCV Duty Left	%	x*100/255	8
P87	0x00011C	0	1	28	Exhaust OCV Current Right	mA	x*32	8
P88	0x00011D	0	1	29	Exhaust OCV Current Left	mA	x*32	8
P89	0x0000D0	0	0	208	A/F Correction #3 (32-bit ECU)	%	(x*.078125)-5	8
P90	0x0000F9	0	0	249	IAM	multiplier	x/16	8
P91	0x000199	0	1	153	Fine Learning Knock Correction	degrees	(x*0.25)-32	8
P92	0x00002F	0	0	47	Radiator Fan Control	%	x	8
P93	0x000048	0	0	72	Front Wheel Speed	mph	x*0.621371192	8
P94	0x000049	0	0	73	ATF Temperature	index	x	8
P95	0x00004B	0	0	75	Line Pressure Duty Ratio	%	x/2	8
P96	0x00004C	0	0	76	Lock Up Duty Ratio	%	x/2	8
P97	0x00004D	0	0	77	Transfer Duty Ratio	%	x/2	8
P98	0x00004E	0	0	78	Throttle Sensor Voltage	V	x/45	8
P99	0x00004F	0	0	79	Turbine Revolution Speed	rpm	x*32	8
P100	0x000050	0	0	80	Brake Clutch Duty Ratio	%	x/2	8
P101	0x000051	0	0	81	Rear Wheel Speed	mph	x*0.621371192	8
P102	0x000052	0	0	82	Manifold Pressure Sensor Voltage	V	x/50	8
P103	0x000055	0	0	85	Lateral G Sensor Voltage	V	x/50	8
P104	0x000056	0	0	86	ATF Temperature	F	32+9*(x-50)/5	8
P105	0x000057	0	0	87	Low Clutch Duty	%	x/2	8
P106	0x000058	0	0	88	High Clutch Duty	%	x/2	8
P107	0x000059	0	0	89	Load and Reverse Brake (L and RB) Duty	%	x/2	8
P108	0x00005A	0	0	90	ATF Temperature 2	F	32+9*(x-50)/5	8
P109	0x00005B	0	0	91	Voltage Center Differential Switch	V	x/51	8
P110	0x00005C	0	0	92	AT Turbine Speed 1	rpm	x*32	8
P111	0x00005D	0	0	93	AT Turbine Speed 2	rpm	x*32	8
P112	0x00005E	0	0	94	Center Differential Real Current	A	x/32	8
P113	0x00005F	0	0	95	Center Differential Indicate Current	A	x/32	8
P114	0x00016A	0	1	106	SI-Drive Mode	index	x	8
P115	0x00016B	0	1	107	Throttle Sensor Closed Voltage	V	x/50	8
P116	0x000107	0	1	7	Exhaust Gas Temperature 2	F	32+9*(x*5+200)/5	8
P117	0x00010B	0	1	11	Air/Fuel Correction #4	%	(x-64)/128*10	8
P118	0x00010C	0	1	12	Air/Fuel Learning #4	%	(x-128)/128*100	8
P119	0x00010D	0	1	13	Fuel Level Sensor Resistance	ohms	x*4/2	8
P120	0x00010E	0	1	14	Estimated odometer	miles	x*1.242742384	16
P121	0x000172	0	1	114	Fuel Tank Air Pressure	psi	x*14.50377	16
P122	0x000113	0	1	19	Oil Temperature	F	32+9*(x-40)/5	8
P123	0x000114	0	1	20	Oil Switching Solenoid Valve (OSV) Duty (Right)	%	x/255*100	8
P124	0x000115	0	1	21	Oil Switching Solenoid Valve (OSV) Duty (Left)	%	x/255*100	8
P125	0x000116	0	1	22	Oil Switching Solenoid Valve (OSV) Current (Right)	mA	x*32	8
P126	0x000117	0	1	23	Oil Switching Solenoid Valve (OSV) Current (Left)	mA	x*32	8
P127	0x00011E	0	1	30	VVL Lift Mode	raw ecu value	x	8
P128	0x000140	0	1	64	H and LR/C Solenoid Valve Current	A	x/255	8
P129	0x000141	0	1	65	D/C Solenoid Valve Current	A	x/255	8
P130	0x000142	0	1	66	F/B Solenoid Valve Current	A	x/255	8
P131	0x000143	0	1	67	I/C Solenoid Valve Current	A	x/255	8
P132	0x000144	0	1	68	P/L Solenoid Valve Current	A	x/255	8
P133	0x000145	0	1	69	L/U Solenoid Valve Current	A	x/255	8
P134	0x000146	0	1	70	AWD Solenoid Valve Current	A	x/255	8
P135	0x000147	0	1	71	Yaw Rate Sensor Voltage	V	x/51	8
P136	0x000148	0	1	72	H and LR/C Solenoid Valve Pressure	psi	x*1.450377	8
P137	0x000149	0	1	73	D/C Solenoid Valve Pressure	psi	x*1.450377	8
P138	0x00014A	0	1	74	F/B Solenoid Valve Pressure	psi	x*1.450377	8
P139	0x00014B	0	1	75	I/C Solenoid Valve Pressure	psi	x*1.450377	8
P140	0x00014C	0	1	76	P/L Solenoid Valve Pressure	psi	x*1.450377	8
P141	0x00014D	0	1	77	L/U Solenoid Valve Pressure	psi	x*1.450377	8
P142	0x00014E	0	1	78	AWD Solenoid Valve Pressure	psi	x*1.450377	8
P143	0x00014F	0	1	79	Yaw Rate and G Sensor Reference Voltage	V	x/51	8
P144	0x00013C	0	1	60	Wheel Speed Front Right	mph	x*0.621371192	8
P145	0x00013D	0	1	61	Wheel Speed Front Left	mph	x*0.621371192	8
P146	0x00013E	0	1	62	Wheel Speed Rear Right	mph	x*0.621371192	8
P147	0x00013F	0	1	63	Wheel Speed Rear Left	mph	x*0.621371192	8
P148	0x00015A	0	1	90	Steering Angle Sensor	degrees	x	16
P149	0x000185	0	1	133	Fwd/B Solenoid Valve Current	A	x/255	8
P150	0x000186	0	1	134	Fwd/B Solenoid Valve Target Pressure	psi	x*1.450377	8
P151	0x0000EF	0	0	239	Roughness Monitor Cylinder #5	misfire count	x	8
P152	0x0000F8	0	0	248	Roughness Monitor Cylinder #6	misfire count	x	8
P153	0x0000F9	0	0	249	Learned Ignition Timing Correction	degrees	x/16	8
P154	0x00019A	0	1	154	Fuel Tank Pressure	psi	(x-128)*0.007251885	8
P155	0x0001E1	0	1	225	Main Injection Period	degrees KW	x/5-15	8
P156	0x0001E2	0	1	226	Final Injection Amount	mm3/st	x/256	16
P157	0x0001E4	0	1	228	Number of Times Injected	count	x	8
P158	0x0001E5	0	1	229	Target Intake Manifold Pressure	psi	x*0.1450377	8
P159	0x0001E6	0	1	230	Target Intake Air Amount	mg/cyl	x*10	8
P160	0x0001E7	0	1	231	Air Mass	mg/cyl	x*10	8
P161	0x0001E8	0	1	232	Exhaust Gas Recirculation (EGR) Target Valve Opening Angle	degrees	x-50	8
P162	0x0001E9	0	1	233	Exhaust Gas Recirculation (EGR) Valve Opening Angle	degrees	x-50	8
P163	0x0001EA	0	1	234	Exhaust Gas Recirculation (EGR) Duty	%	x	8
P164	0x0001EB	0	1	235	Common Rail Target Pressure	psi	x*145.0377	8
P165	0x0001EC	0	1	236	Common Rail Pressure	psi	x*145.0377	8
P166	0x0001ED	0	1	237	Intake Air Temperature (combined)	F	32+9*(x-40)/5	8
P167	0x0001EE	0	1	238	Target Engine Speed	rpm	x/4	16
P168	0x0001F0	0	1	240	Boost Pressure Feedback	%	x-128	8
P169	0x0001F5	0	1	245	Electric Power Steering Current	A	x	8
P170	0x0001F6	0	1	246	Target Fuel Pump Current	mA	x	16
P171	0x0001F1	0	1	241	Yaw Rate	degrees/s	x*0.19118	8
P172	0x0001F2	0	1	242	Lateral G	m/s2	x*1.0862	8
P173	0x0001F3	0	1	243	Drivers Control Center Differential (DCCD) Torque Allocation	raw ecu value	x	8
P174	0x0001F4	0	1	244	Drivers Control Center Differential (DCCD) Mode	raw ecu value	x	8
P175	0x0001F8	0	1	248	Actual Fuel Pump Current	mA	x	16
P176	0x0001FA	0	1	250	Mileage after Injector Learning	miles	x*3.10685596	16
P177	0x000204	0	2	4	Mileage after Injector Replacement	miles	x*3.10685596	16
P178	0x000270	0	2	112	Interior Heater	steps	x	8
P179	0x00025D	0	2	93	Quantity Correction Cylinder #1	ms	(x-100)/100	8
P180	0x00025E	0	2	94	Quantity Correction Cylinder #2	ms	(x-100)/100	8
P181	0x00025F	0	2	95	Quantity Correction Cylinder #3	ms	(x-100)/100	8
P182	0x000260	0	2	96	Quantity Correction Cylinder #4	ms	(x-100)/100	8
P183	0x000271	0	2	113	Battery Current	A	x-128	8
P184	0x000273	0	2	115	Battery Temperature	F	32+9*(x-40)/5	8
P185	0x000272	0	2	114	Alternator Control Mode	index	x	8
P186	0x000275	0	2	117	Cumulative Ash Ratio	%	x	8
P187	0x000276	0	2	118	Pressure Difference between Diesel Particulate Filter (DPF) Inlet and Outlet	psi	x*14.50377	8
P188	0x000277	0	2	119	Exhaust Gas Temperature at Catalyst Inlet	F	32+9*(x*5-40)/5	8
P189	0x000278	0	2	120	Exhaust Gas Temperature at Diesel Particulate Filter (DPF) Inlet	F	32+9*(x*5-40)/5	8
P190	0x000279	0	2	121	Estimated Catalyst Temperature	F	32+9*(x*5-40)/5	8
P191	0x00027A	0	2	122	Estimated Temperature of the Diesel Particulate Filter (DPF)	F	32+9*(x*5-40)/5	8
P192	0x00027B	0	2	123	Soot Accumulation Ratio	%	x	8
P193	0x00027C	0	2	124	Oil Dilution Ratio	%	x	8
P194	0x000293	0	2	147	Front-Rear Wheel Rotation Ratio	%	x/128	8
P195	0x000294	0	2	148	ABS/VDC Front Wheel Mean Wheel Speed	mph	x*143/255	8
P196	0x000295	0	2	149	ABS/VDC Rear Wheel Mean Wheel Speed	mph	x*143/255	8
P197	0x000296	0	2	150	Automatic Transmission Fluid (ATF) Deterioration Degree	%	x*40/13107	16
P198	0x000298	0	2	152	Accumulated Count of Overspeed Instances (Very High RPM)	Time	x	8
P199	0x000299	0	2	153	Accumulated Count of Overspeed Instances (High RPM)	Time	x	8
P204	0x00021F	0	2	31	Actual Common Rail Pressure (Time Synchronized)	psi	x*145.0377	8
P205	0x00029A	0	2	154	Estimated Distance to Oil Change	miles	x*62	8
P206	0x00029B	0	2	155	Running Distance since last Diesel Particulate Filter (DPF) Regeneration	miles	x*0.621371192	16
P207	0x00029D	0	2	157	Diesel Particulate Filter (DPF) Regeneration Count	Times	x	16
P208	0x00023D	0	2	61	Micro-Quantity-Injection Final Learning Value 1-1	ms	(x-128)/200	8
P209	0x00023E	0	2	62	Micro-Quantity-Injection Final Learning Value 1-2	ms	(x-128)/200	8
P210	0x00023F	0	2	63	Micro-Quantity-Injection Final Learning Value 1-3	ms	(x-128)/200	8
P211	0x000240	0	2	64	Micro-Quantity-Injection Final Learning Value 1-4	ms	(x-128)/200	8
P212	0x000241	0	2	65	Micro-Quantity-Injection Final Learning Value 2-1	ms	(x-128)/200	8
P213	0x000242	0	2	66	Micro-Quantity-Injection Final Learning Value 2-2	ms	(x-128)/200	8
P214	0x000243	0	2	67	Micro-Quantity-Injection Final Learning Value 2-3	ms	(x-128)/200	8
P215	0x000244	0	2	68	Micro-Quantity-Injection Final Learning Value 2-4	ms	(x-128)/200	8
P216	0x000245	0	2	69	Micro-Quantity-Injection Final Learning Value 3-1	ms	(x-128)/200	8
P217	0x000246	0	2	70	Micro-Quantity-Injection Final Learning Value 3-2	ms	(x-128)/200	8
P218	0x000247	0	2	71	Micro-Quantity-Injection Final Learning Value 3-3	ms	(x-128)/200	8
P219	0x000248	0	2	72	Micro-Quantity-Injection Final Learning Value 3-4	ms	(x-128)/200	8
P220	0x000249	0	2	73	Micro-Quantity-Injection Final Learning Value 4-1	ms	(x-128)/200	8
P221	0x00024A	0	2	74	Micro-Quantity-Injection Final Learning Value 4-2	ms	(x-128)/200	8
P222	0x00024B	0	2	75	Micro-Quantity-Injection Final Learning Value 4-3	ms	(x-128)/200	8
P223	0x00024C	0	2	76	Micro-Quantity-Injection Final Learning Value 4-4	ms	(x-128)/200	8
P224	0x00024D	0	2	77	Micro-Quantity-Injection Final Learning Value 5-1	ms	(x-128)/200	8
P225	0x00024E	0	2	78	Micro-Quantity-Injection Final Learning Value 5-2	ms	(x-128)/200	8
P226	0x00024F	0	2	79	Micro-Quantity-Injection Final Learning Value 5-3	ms	(x-128)/200	8
P227	0x000250	0	2	80	Micro-Quantity-Injection Final Learning Value 5-4	ms	(x-128)/200	8
P228	0x000238	0	2	56	Individual Pump Difference Learning Value	mA	x-1000	16
P229	0x000257	0	2	87	Final Main Injection Period	ms	x/1000	16
E31	0xFF2570	255	37	112	IAM (4-byte)*	multiplier	x	32
E32	0xFF4F58	255	79	88	Engine Load (4-Byte)*	g/rev	x	32
E33	0xFF6B79	255	107	121	CL/OL Fueling*	status	x+6	8
E34	0xFF4BBC	255	75	188	Turbo Dynamics Integral (4-byte)*	absolute %	x	32
E35	0xFF4BB0	255	75	176	Boost Error*	psi	x*0.01933677	32
E36	0xFF4BB4	255	75	180	Target Boost (4-byte)*	psi relative sea level	(x-760)*0.01933677	32
E37	0xFF4BB8	255	75	184	Turbo Dynamics Proportional (4-byte)*	absolute %	x	32
E38	0xFF4E5C	255	78	92	Throttle Plate Opening Angle (4-byte)*	%	x/.84	32
E39	0xFF58CC	255	88	204	Feedback Knock Correction (4-byte)*	degrees	x	32
E40	0xFF5920	255	89	32	Knock Correction Advance (IAM only)*	degrees	x	32
E41	0xFF5938	255	89	56	Fine Learning Knock Correction (4-byte)*	degrees	x	32
E42	0xFF58DC	255	88	220	Map Ratio (Primary)*	multiplier	x	32
E44	0xFF251C	255	37	28	A/F Learning #1 A (Stored)*	%	x*100	32
E45	0xFF2524	255	37	36	A/F Learning #1 B (Stored)*	%	x*100	32
E46	0xFF252C	255	37	44	A/F Learning #1 C (Stored)*	%	x*100	32
E47	0xFF2534	255	37	52	A/F Learning #1 D (Stored)*	%	x*100	32
E48	0xFF5494	255	84	148	A/F Learning #1 (4-byte)*	%	x*100	32
E49	0xFF5C3C	255	92	60	Idle Speed Map Selection*	raw ecu value	x	8
E50	0xFF56D4	255	86	212	Fuel Injector #1 Latency (4-byte)*	ms	x*.001	32
E51	0xFF4DCC	255	77	204	Manifold Absolute Pressure (4-byte)*	psi absolute	x*0.01933677	32
E52	0xFF4DCC	255	77	204	Manifold Relative Sea Level Pressure (4-byte)*	psi relative sea level	(x-760)*0.01933677	32
E53	0xFF57F0	255	87	240	Ignition Base Timing*	degrees	x	32
E54	0xFF4E60	255	78	96	Tip-in Throttle*	%	x	32
E55	0xFF5610	255	86	16	Tip-in Enrichment (Last Calculated)*	raw ecu value	x	32
E56	0xFF5AC4	255	90	196	Requested Torque*	raw ecu value	x	32
E57	0xFF5AC0	255	90	192	Target Throttle Plate Position*	%	x/.84	32
E58	0xFF593E	255	89	62	Fine Learning Table Offset*	index position	x+1	8
E59	0xFF516D	255	81	109	Gear (Calculated)*	position	x	8
E60	0xFF56C0	255	86	192	Fuel Injector #1 Pulse Width (4-byte)*	ms	x*.001	32
E61	0xFF549D	255	84	157	A/F Learning Airflow Range (Current)*	offset	x+1	8
E70	0xFF4BD8	255	75	216	Primary Wastegate Duty Maximum* (4-byte)*	%	x	32
E77	0xFF4BD8	255	75	216	Primary Wastegate Duty Maximum* (4-byte)*	%	x	32
E81	0xFF534C	255	83	76	A/F Correction #1 (4-byte)*	%	(x*100)-100	32
E84	0xFF54F4	255	84	244	Primary Open Loop Map Enrichment (4-byte)*	estimated AFR	14.7/(1+x)	32
E91	0xFF5028	255	80	40	A/F Sensor #1 (4-byte)*	estimated AFR	x*14.7	32
E113	0xFF4DD0	255	77	208	Manifold Relative Pressure (4-byte)*	psi relative	x*0.01933677	32
E115	0xFF52D0	255	82	208	Primary Enrichment Final (4-byte)*	estimated AFR	14.7/(1+x)	32
E116	0xFF591C	255	89	28	Knock Correction Advance Primary Map Ratio*	multiplier	x	32
E117	0xFF6100	255	97	0	Map Ratio (Alternate)*	multiplier	x	32
E118	0xFF5918	255	89	24	Knock Correction Advance Max Primary*	degrees	x	32
E119	0xFF5924	255	89	36	Knock Correction Advance Additive*	degrees	x	32
E121	0xFF5394	255	83	148	Closed Loop Fueling Target (4-byte)*	estimated AFR	x*14.7	32
E123	0xFF52A0	255	82	160	Final Fueling Base (4-byte)*	estimated AFR	14.7/x	32
						
*/

