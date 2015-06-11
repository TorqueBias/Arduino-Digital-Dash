
#include <Fat16.h>
#include <Fat16Config.h>
#include <Fat16mainpage.h>
#include <Fat16util.h>
#include <SdCard.h>
#include <SdInfo.h>
#include <EasyTransferI2C.h>
#include <Wire.h>
#include "RTClib.h"
#include <PDQ_FastPin.h>
#include <SPI.h>
#include <PDQ_ILI9341_config.h>
#include <PDQ_ILI9341.h>
#include <PDQ_GFX.h>
#include <Adafruit_FT6206.h>
#define TFT_DC 9
#define TFT_CS 10
Adafruit_FT6206 ctp = Adafruit_FT6206();

// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
//#define ILI9341_GREY 0x2104 // Dark grey 16 bit colour, I though this was too light.
//#define ILI9341_GREY 0x3186 // Dark grey 16 bit colour, Still too light.
#define ILI9341_GREY 0x73E8 // Dark grey 16 bit colour

//Create objects to transfer data between Arduinos
EasyTransferI2C ETrx;
EasyTransferI2C ETtx; 

PDQ_ILI9341 lcd = PDQ_ILI9341();//Create display object

#define CHIP_SELECT     SS  // SD chip select pin

SdCard card; //Define SD Card object
Fat16 file; //Define log file object


//SoftwareSerial sendSerial(3, 2, true); //Rx, Tx - For ECU Data ***Inverted Signal for easy diagnostics***


RTC_DS1307 rtc; //Define Real Time Clock object

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  //For I2C, this may be limited to about 28 bytes??? Consider serial if you need more data.
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

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int request;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

//LCD size values, this has been slowly getting phased out.
int charS = 1;
int charH;
int charW;
char bgColor = ILI9341_BLACK;
char fgColor = ILI9341_WHITE;



//Misc flags for misc things...
boolean menu = false;//Indicates that the menu is the current screen.
boolean gauges = true;//Indicates that the gauges are being displayed.
boolean time = true;//determines wheter the time should display, or just be calculated when updateTime is called.
boolean refresh = true;//When true, triggers subroutines to ignore history data and re-write all values.
boolean logData = true;//Sets whether data will be recorded to a log file on SD Card or not.
boolean useCard = true;//Used to disable card related functions
boolean sdPresent = true;//Changes to false if there is a problem detecting the SD Card.
boolean diagnostics = false;//Used at various stages to enable/disable diagnostic features.


//Holds previous values of readings and warnings so that the program can be smart about updating the screen.
//These will just waste memory if you don't need them.
int pIAT = -99;
int pMRP = -990;
int pRPM = -99;
int pAFR = -990;
//int pOIL=0;
//int pTPS = -99;
//float pKCK = -99;
//int pIDC = -99;

//These hold timing values for items that update on intervals.
long timeClock = 0;
long tIAT = 0;
long tMRP = 0;
long tRPM = 0;
long tAFR = 0;
//long tKCK = 0;
//int tOIL=0;

//These set the update frequency for items that update on intervals.
long sClock = 2000; //number of milliseconds between clock updates (These can be slow too, disabled seconds display and only update every 2 seconds.)
int sIAT = 500;
int sMRP = 50;
int sRPM = 50;
long sAFR = 90;
//int sKCK = 50;
//int sOIL=0;


//These hold the floating point values after they are passed into the program as integers or bytes
float fMRP;
float fAFR;
float fKCK;

// These hold previous warning values to prevent getting spammed with warnings.
int pWarnMRP = 0;
int pWarnAFR = 0;
int pWarnKCK = 0;
int pWarnIDC = 0;

//Defines where warning lines appear on display.
int warnLines[2] = { 11, 13 };
int warnLine = 10;
int warning = 0;

//Timekeeping variables
int hunTime = 0;
int compTime = 0;
byte hourTime = 0;
byte minTime = 0;
byte secTime = 0;

//Tracks timing of gauge display for FPS calculations, can be removed when development is done.
long gTime;
long gCounter;


void setup()
{
	//Fire up the ports and devices
	Serial.begin(57600); //for diagnostics and programming interface (USB)
	lcd.begin(); //somehow, this must begin before sendSerial or else ECU comms fail?
	Wire.onReceive(receive); //Part of EZ-Transfer Library
        Wire.begin(8); //Starts I2C as device 8
        TWBR=0x0C; // (DEC = 12), Turns up I2C clock from 100 to 400?
        
        //Start the EZ-Transfer library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
        //"&Wire" specifies I2C
        ETrx.begin(details(rxdata), &Wire);
        ETtx.begin(details(txdata), &Wire);
	rtc.begin();//Starts the Real Time Clock

        //Initialize TFT Screen
	clearDisplay();
	charS = 1;
	colors();

	//initialilze touchscreen
	lcd.print(F("TS "));
	if (!ctp.begin(40)) {  // pass in 'sensitivity' coefficient
		lcd.print(F("ERR"));
		while (1); // Jams Program so that it won't continue if touchscreen not working
	}
	lcd.println(F("OK"));

	//Setup RTC Clock (Clock seems to always be a few minutes off? Look into that later)
	//rtc.adjust(DateTime(__DATE__, __TIME__));//in order to update the time without pulling the battery, uncomment this code, upload, then recomment the code and re-upload the code
	updateLCD(0, 2, 0);
	lcd.print(F("RTC "));
	if (!rtc.isrunning()) {
		//if(diagnostics) Serial.println(F("RTC is NOT running!"));
		rtc.adjust(DateTime(__DATE__, __TIME__)); // following line sets the RTC to the date & time this sketch was compiled
		lcd.println(F("ERR"));
	}
	else{
		lcd.println(F("OK"));
	}

	// initialize the SD card
	sdPresent = (card.begin(4));  //Does the SD Card initialize?
	if (useCard && sdPresent){ //If it does, and the option is enabled, display OK
		updateLCD(0, 3, 0);
		lcd.print(F("SD "));
		if (sdPresent){
			lcd.println(F("OK,"));
		}
		else{
			lcd.println(F("ERR,"));
			useCard = false;
			logData = false;
		}
	}

	// initialize a FAT16 volume on SD Card if option enabled.
	if (useCard && sdPresent){
		updateLCD(14, 3, 0);
		lcd.print(F("FAT "));
		if (!Fat16::init(&card))
		{
			lcd.println(F("ERR"));
			useCard = false;
			logData = false;
		}
		else
		{
			lcd.println(F("OK"));
		}

		/*Find available File Name and Create a new file
		Stores 100 log files on the SD Card but can't handle running out of filenames. I need to make this where it can rotate the logs, low priority.
		Maybe a menu item or warning to clear the logs instead? Also want to add a seperate file for warnings?
                Sometimes data gets corrupted when ignitiion turned off, I plan to stop the logging when motor is at idle to prevent this*/
		char name[] = "LOG00.CSV";
		for (uint8_t i = 0; i < 100; i++) {
			//name[3] = i / 100 + '0'; //Tried adding 3rd digit. Starts using invalid characters. Ditching this for now.
			name[3] = i / 10 + '0';
			name[4] = i % 10 + '0';
			/*O_CREAT - create the file if it does not exist
			O_EXCL - fail if the file exists
			O_WRITE - open for write only*/
			if (file.open(name, O_CREAT | O_EXCL | O_WRITE))break;
		}
		updateLCD(0, 4, 0);
		lcd.print(F("File "));
		if ((!file.isOpen()) || (!file.sync())){//Check to see if file was created and opened.
			lcd.println(F("ERR"));
		}
		else{
			lcd.println(name);
			file.writeError = false;
		}
	}
        if (logData){//Inserts Column Headings in Log File
		time = false; updateTime(); //First Column Heading is time in MS with current time as heading prefix
		file.print(hourTime); file.print(F("-")); file.print(minTime); file.print(F("-")); file.print(secTime); file.print(F("-")); //Insert Time into first column heading
		file.println(F("MS,MRP,IAT,IDC,RPM,TPS,FBKC,AFR,WARN")); //Insert column headings into file.
	}
	

	updateLCD(0, 7, 0);
	charS = 1;
	colors();

	delay(2000); //Pause to view startup diagnostics
	clearDisplay(); //Clear Screen
	setupGauges();  //Draw Gauge Template
	refresh = true; //Flags subroutines to bypass optimization and re-draw elements.
}// End of void setup

void loop()
{
      
	if (ctp.touched()){ //Check to see if the touchscreen has been touched.
		processTouch();
	}

	if ((millis() - timeClock) > sClock){ //Updates clock every sClock interval.
		timeClock = millis();
		if (gauges) time = true;
		updateTime();
		//refresh = false;
	}
        
        txdata.request = 1; //Sets request value for Interface Arduino, right now only using #1
        ETtx.sendData(9); //Sends data request to Interface Arduino
        txdata.request = 0; //Resets request value	
        
        if(diagnostics) { //Bypasses check for received data for bench testing.
          processData();
        }
        
        if(ETrx.receiveData()){
          //This is how you access the variables from EZ-Transfer. [name of the group].[variable name]
            //These valuse are passed in as integers, we will now convert them back to floats.
            fMRP=rxdata.MRP/10.0;
            fKCK=rxdata.KCK/10.0;
            fAFR=rxdata.AFR/10.0;
            
            //Now that we have new data, lets go use it!
            processData();
      
  }

}




void processTouch(){
	TS_Point p = ctp.getPoint();
	if (!(p.x == 0) && !(p.y == 0)){
		p.y = map(p.y, 0, 320, 320, 0);
		int tzone[] = { 0, 0 };// This and the next 3 lines convert coordinates into array that represents 16 buttons on a 4x4 grid.
		tzone[0] = int((p.y / 80) + .5) + 1;
		tzone[1] = int((p.x / 60) + .5) + 1;
		if ((tzone[0] == 4 && tzone[1] == 1) && menu == false){ //Goes to Menu Screen
			clearDisplay();
			setupMenu();
			menu = true;
			gauges = false;
			time = false;
		}
		else if ((tzone[0] == 4 && tzone[1] == 1) && menu == true){ //Goes back to Gauge Screen
			clearDisplay();
			setupGauges();
			menu = false;
			gauges = true;
			time = true;
			refresh = true;
		}
	}
}




void processData(){
  
        if(diagnostics){ //Generates Test Values for Bench Testing, takes up more memory than you'd think, remove later.
            float mover=random(100)/100.00;
            rxdata.MRP= -110 + ((mover)*310); 
            rxdata.RPM= (mover) * 7000;
            rxdata.AFR= 90 + ((mover)*133);
            rxdata.LKC=255*mover;
            rxdata.MAF=255*mover;
            rxdata.FUL=255*mover;
            rxdata.GER=255*mover;
            rxdata.SPD=255*mover;
	}
        
        
	// Runs data through some checks and generates warnings if there are any. (Lean, Knock, Overboost, etc...)
	checkWarn(rxdata.AFR, rxdata.IAT, rxdata.IDC, rxdata.MRP, rxdata.RPM, rxdata.TPS, rxdata.KCK);//Looks for patterns that indicate a problem.
	
        //Writes CSV data to SD Card
        if (logData){
                file.print(millis());
		file.print(F(","));
		file.print(fMRP);
		file.print(F(","));
		file.print(rxdata.IAT);
		file.print(F(","));
		file.print(rxdata.IDC);
		file.print(F(","));
		file.print(rxdata.RPM);
		file.print(F(","));
		file.print(rxdata.TPS);
		file.print(F(","));
		file.print(fKCK);
		file.print(F(","));
		file.print(fAFR);
		file.print(F(","));
		file.print(warning);
		//Finishes record in file
		file.println();
		file.sync();
	}

        //Draws and updates gauges on screen.
	if (gauges) updateGauges(rxdata.AFR, rxdata.IAT, rxdata.IDC, rxdata.MRP, rxdata.RPM, rxdata.TPS, rxdata.KCK);//Updates Display
        
        //Draws and updates menu on screen.
        if (menu) updateMenu();
        
        //Resets warning flag
	warning = 0;


}


//Checks for warning indicators, LOGGING SLOWS DOWN WHILE WARNING!
//Make sure that you filter warnings so that they don't spam the screen
//During the alert. Probably need more conditions such as a timer for readings that lag such as AFR.
void checkWarn(int AFR, int IAT, int IDC, int MRP, int RPM, int TPS, int KCK){
	if (MRP > 190 && MRP > pWarnMRP){ //Overboost
		warning = 1;
		time = false; updateTime();
		if (warnLine < warnLines[1]){
			warnLine++;
		}
		else{
			warnLine = warnLines[0];
		}
		pWarnMRP = MRP;
		charS = 2; fgColor = ILI9341_BLACK; bgColor = ILI9341_YELLOW; colors();
		updateLCD(0, warnLine, 0);
		lcd.print(F("MRP:"));
		lcd.print(fMRP);
		updateLCD(11, warnLine, 0);
		lcd.print(RPM);
		charS = 1; fgColor = ILI9341_WHITE; bgColor = ILI9341_BLACK; colors();
	}
	if (MRP > 100 && AFR > 116 && AFR > pWarnAFR){ //Lean under boost
		warning = 2;
		time = false; updateTime();
		if (warnLine < warnLines[1]){
			warnLine++;
		}
		else{
			warnLine = warnLines[0];
		}
		pWarnAFR = AFR;
		charS = 2; fgColor = ILI9341_BLACK; bgColor = ILI9341_YELLOW; colors();
		updateLCD(0, warnLine, 0);
		lcd.print(F("AFR:"));
		lcd.print(fAFR);
		updateLCD(11, warnLine, 0);
		lcd.print(RPM);
		updateLCD(17, warnLine, 0);
		lcd.print(fMRP);
		charS = 1; fgColor = ILI9341_WHITE; bgColor = ILI9341_BLACK; colors();
	}
	if (KCK < 0 && MRP>50 && KCK < pWarnKCK){ //Knock Detected
		warning = 3;
		time = false; updateTime();
		if (warnLine < warnLines[1]){
			warnLine++;
		}
		else{
			warnLine = warnLines[0];
		}
		pWarnKCK = KCK;
		charS = 2;
		if (KCK < 2){ fgColor = ILI9341_BLACK; bgColor = ILI9341_YELLOW; }// Sets Warning Colors
		if (KCK >= 2){ fgColor = ILI9341_BLACK; bgColor = ILI9341_RED; }
		colors();
		updateLCD(0, warnLine, 0);
		lcd.print(F("KCK:"));
		lcd.print(fKCK);
		updateLCD(11, warnLine, 0);
		lcd.print(RPM);
		updateLCD(17, warnLine, 0);
		lcd.print(fMRP);
		charS = 1; fgColor = ILI9341_WHITE; bgColor = ILI9341_BLACK; colors();
	}
	if (IDC > 95 && IDC > pWarnIDC){ //Injector Duty High
		warning = 4;
		time = false; updateTime();
		if (warnLine < warnLines[1]){
			warnLine++;
		}
		else{
			warnLine = warnLines[0];
		}
		pWarnIDC = IDC;
		charS = 2;
		if (IDC < 99){ fgColor = ILI9341_BLACK; bgColor = ILI9341_YELLOW; }// Sets Warning Colors
		if (IDC >= 99){ fgColor = ILI9341_BLACK; bgColor = ILI9341_RED; }
		colors();
		updateLCD(0, warnLine, 0);
		lcd.print(F("IDC:"));
		lcd.print(IDC);
		updateLCD(11, warnLine, 0);
		lcd.print(RPM);
		updateLCD(17, warnLine, 0);
		lcd.print(fMRP);
		charS = 1; fgColor = ILI9341_WHITE; bgColor = ILI9341_BLACK; colors();
	}
}

void updateGauges(int AFR, int IAT, int IDC, int MRP, int RPM, int TPS, int KCK){
	if (refresh){// Changes previous values so that they will redraw on this cycle. 
		pIAT = 0;
		//pIDC = 0;
		pMRP = -105;
		pRPM = 0;
		//pTPS = 0;
		pAFR = 0;
		//pKCK = 0;
	}

	//Corrects the color settings if I messed them up elsewhere, sloppy programming, clean this up later.
        if ((bgColor != ILI9341_BLACK) || (fgColor != ILI9341_WHITE) || (charS !=1)){// Fixes the screen colors for the gauges
		bgColor = ILI9341_BLACK;
		fgColor = ILI9341_WHITE;
                charS=1;
                colors();
	}
        
        //ringMeter(reading, prev reading, range1,range2,xpos,ypos,radius,label,data width, decimals, color scheme, Gauge Type, big indicator)
        
        /* //3-Medium Gauges
	if ((AFR != pAFR) || refresh){
		if (AFR >= 90 && AFR <= 223){
                        ringMeter(AFR, pAFR,105,150,213,10,52," AFR",4,1,4,2,113);
                        pAFR = AFR;
		}
                
	}
        
	if (RPM != pRPM || refresh){
                ringMeter(RPM, pRPM, 750,7000,1,10,52," RPM",4,0,4,1,6500);
                pRPM = RPM; tRPM = millis();
	}
	if (MRP != pMRP || refresh){
                ringMeter(MRP, pMRP, -110,200,107,10,52,"BOOST",5,1,4,1,175);
		pMRP = MRP; tMRP = millis();
	}
        */
        
         //2-Large Gauges
	
        
	if ((AFR != pAFR) || refresh){ //Only do the work if the data has changed, or refresh flag is set.
		if (AFR >= 90 && AFR <= 223){ // Values outside this range indicate non-operational info such as Warmup.
                        ringMeter(AFR, pAFR,105,160,1,10,78," AFR",4,1,4,2,114); //Draw the Gague
                        pAFR = AFR; //Reset the previous value for next time
		}
                
	}
	if (MRP != pMRP || refresh){
                ringMeter(MRP, pMRP, -110,200,160,10,78,"BOOST",5,1,4,1,175);
		pMRP = MRP; tMRP = millis();
	}
        
        
	if ((millis()>tIAT && IAT != pIAT) || refresh){ //Only updates on interval, data changes slowly.
                lcd.setCursor(15*charW,0);
                lcd.print(IAT); //No meter, just display the data.
		pIAT = IAT; 
                tIAT = millis()+sIAT; //Reset the timer.
	}
	refresh = false; //We assume all has been redrawn by now. This causes blanks sometimes on interval based items. Minor, but needs a re-think.

        gCounter++; //Increment counter to track screen update speed.
        
        // Puts speed counter on screen
        lcd.setCursor(30*6,0);
        lcd.print(gCounter/(millis()/1000));
                
}


void colors() //Made to save some typing during dev. 
{
	lcd.setTextSize(charS);
	lcd.setTextColor(fgColor, bgColor);
}

void updateLCD(int column, int row, int stringL){ //Uses Screen Definition Values to place cursor and/or clear blocks before printing text. (column,row,Blocks to Clear)
	charH = ((7 * charS) + charS);
	charW = ((5 * charS) + charS);
	lcd.setCursor(column*charW, row*charH);
	if (!(stringL == 0)) lcd.fillRect(column*charW, row*charH, stringL * charW, charH, bgColor);
}




void clearDisplay() //This clears the screen, resets the basic parameters, sets Text Color, and redraws Gauges. Some of this will be moved to seperate subroutines later.
{
	lcd.fillScreen(bgColor);
	lcd.setRotation(1);
	lcd.setTextSize(charS);
	lcd.setTextColor(fgColor, bgColor);
	updateLCD(0, 0, 0);
}


void updateTime() //Grabs and Displays the current time if hour/minute/sec has changed. Returns fase if time hasn't changed.
{
	DateTime now = rtc.now();
	if (time){
		charS = 1;
		colors();
		compTime = hourTime;
		if (refresh) compTime=0;
                hourTime = now.hour();
		if (hourTime != compTime){
			updateLCD(0, 0, 0);
			if (hourTime < 10) lcd.print("0");
			lcd.print(hourTime);
		}
		compTime = minTime;
                if (refresh) compTime=0;
		minTime = now.minute();
		if (minTime != compTime){
			updateLCD(3, 0, 0);
			if (minTime < 10) lcd.print("0");
			lcd.print(minTime);
		}
		compTime = secTime;
		secTime = now.second();
	}else{
		hourTime = now.hour();
		minTime = now.minute();
		secTime = now.second();
	}
}


void setupGauges() // This draws the gauge Template. I want to make this graphical later with images from the SD Card. Now it's quick and dirty for testing.
{
	charS = 1;
	fgColor = ILI9341_WHITE;
	bgColor = ILI9341_BLACK;
	colors();
	updateLCD(0, 0, 0);
	lcd.print(F("  :  ")); //For Clock
	updateLCD(19, 0, 0);
	lcd.print(F("degF  FPS:")); //Units for IAT
	updateLCD(10, 0, 0);
	lcd.print(F("IAT:"));
}


void setupMenu(){ //This draws the Menu Template
	charS = 1;
	colors();
	updateLCD(0, 2, 0);
	lcd.println(F("AFR"));
	lcd.println(F("MRP"));
	lcd.println(F("RPM"));
	lcd.println(F("IAT"));
	lcd.println(F("IDC"));
	lcd.println(F("TPS"));
	lcd.println(F("KCK"));
	lcd.println(F("LKC"));
	lcd.println(F("MAF"));
	lcd.println(F("FUL"));
	lcd.println(F("GER"));
	lcd.println(F("SPD"));
	lcd.println(F("OIL"));
	charS = 2;
	bgColor = ILI9341_WHITE;
	fgColor = ILI9341_BLACK;
	colors();
	updateLCD(22, 1, 4);
	lcd.print(F("EXIT"));
        charS = 1;
	fgColor = ILI9341_WHITE;
	bgColor = ILI9341_BLACK;
	colors();

}

void updateMenu(){ //Fills in menu screen with data
	updateLCD(5, 2, 0);lcd.print(rxdata.AFR);
	updateLCD(5, 3, 0);lcd.print(rxdata.MRP);
	updateLCD(5, 4, 0);lcd.print(rxdata.RPM);
	updateLCD(5, 5, 0);lcd.print(rxdata.IAT);
	updateLCD(5, 6, 0);lcd.print(rxdata.IDC);
	updateLCD(5, 7, 0);lcd.print(rxdata.TPS);
	updateLCD(5, 8, 0);lcd.print(rxdata.KCK);
	updateLCD(5, 9, 0);lcd.print(rxdata.LKC);
	updateLCD(5, 10, 0);lcd.print(rxdata.MAF);
	updateLCD(5, 11, 0);lcd.print(rxdata.FUL);
	updateLCD(5, 12, 0);lcd.print(rxdata.GER);
	updateLCD(5, 13, 0);lcd.print(rxdata.SPD);
	updateLCD(5, 14, 0);lcd.print(rxdata.OIL);
  
}



//  Draw the meter on the screen, returns x coord of righthand side
int ringMeter(int value, int pValue, int vmin, int vmax, int x, int y, int r, char *units, int fWidth, int decs, byte scheme, int gType, int indicator)
{
  // Minimum value of r is about 52 before value text intrudes on ring, right size for 3-wide.
  x += r; y += r;   // Calculate coords of centre of ring
  r -= 2; //Added an outer circle, this moves the segments in a bit so they don't draw over it.
  
  if (value == pValue && !refresh) return x + r; //Only draws if new value different from previous value, need to remove this redundancy from updateGauges()
  
  int w=r/4; //Sets inner radius of segments
  
  int angle = 150;  // Half the sweep angle of meter (300 degrees)
  int pAngle = 150;
  int iAngle = 150;
  
  
  byte indColor=ILI9341_GREY; //Initializes segment colors
  int text_colour = 0; // To hold the text color
  
 
  
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  int pv = map(pValue, vmin, vmax, -pAngle, pAngle); // Map the previous value to an angle v
  int ind = map(indicator, vmin, vmax, -iAngle, iAngle); // Map Indicator value to an angle v
  
  byte seg = 6; // Segments are 5 degrees wide = 60 segments for 300 degrees
  byte inc = 12; // Draw segments every 5 degrees, increase to 10 for segmented ring
  
  int minAngle;
  int maxAngle;
  
  if (v<pv) { // decide which values are maximums and minimums, buff them by segment size.
    minAngle=v-seg;
    maxAngle=pv+seg;
  }else{
    minAngle=pv-seg;
    maxAngle=v+seg;
  }
  
  if (refresh){ //if the refresh flag is set, set max and min to full sweep.
    minAngle = -angle;
    maxAngle = angle;
    lcd.drawCircle(x,y,r+2,ILI9341_WHITE); //Outer ring of gauge
  }  

  // Draw colour blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) { //Sweeps through each segment
    
    if (i >= (ind-seg) && i <= (ind+seg)){ //Changes width of segment when it is at indicator value.
          w = r / 2;    // Width of segment is 1/3 of radius
        }else{
          w = r / 4;    // Width of segment is 1/4 of radius
        }
        
    if(i >= (minAngle-seg) && i <= (maxAngle+seg)){ //Only re-draws segments between new value and previous value for SPEED!
        
        int colour = 0; // Choose colour from scheme
        switch (scheme) {
          case 0: colour = ILI9341_RED; break; // Fixed colour
          case 1: colour = ILI9341_GREEN; break; // Fixed colour
          case 2: colour = ILI9341_BLUE; break; // Fixed colour
          case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
          case 4: colour = rainbow(map(i, -angle, angle, 63, 127)); break; // Green to red (high temperature etc)
          case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
          default: colour = ILI9341_BLUE; break; // Fixed colour
        }
     
        // Calculate pair of coordinates for segment start
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (r - w) + x;
        uint16_t y0 = sy * (r - w) + y;
        uint16_t x1 = sx * r + x;
        uint16_t y1 = sy * r + y;
    
        // Calculate pair of coordinates for segment end
        float sx2 = cos((i + seg - 90) * 0.0174532925);
        float sy2 = sin((i + seg - 90) * 0.0174532925);
        int x2 = sx2 * (r - w) + x;
        int y2 = sy2 * (r - w) + y;
        int x3 = sx2 * r + x;
        int y3 = sy2 * r + y;
    
        // Fill in coloured segments with 2 triangles. Type 1 is a sweep, Type 2 is individual segments.
        // Type 2 is much faster. Can change drawTriangle to fillTriangle for solid segments instead of
        // Wireframe, but cuts speed in half.
        if ((gType==1 && i < v + seg ) || (gType==2 && (i >= v - seg && i <= v + seg))) { 
          lcd.drawTriangle(x0, y0, x1, y1, x2, y2, colour);
          lcd.drawTriangle(x1, y1, x2, y2, x3, y3, colour);
          text_colour = colour; // Save the last colour drawn for text color later.
        }
        else if ((gType==1 && i >= v + seg ) || (gType==2 && (i >= pv - seg && i <= pv + seg)) || refresh)// Fill in blank segments
        {
          if (i >= (ind-seg) && i <= (ind+seg)){ //make big indicator 
            indColor=ILI9341_WHITE; //This is supposed to be white, but it's blue instead???    
          }else{
            indColor=ILI9341_GREY;    
          }
          
          lcd.drawTriangle(x0, y0, x1, y1, x2, y2, indColor);
          lcd.drawTriangle(x1, y1, x2, y2, x3, y3, indColor);
          
        }
    }
  }
  
  
  
  // Set the text colour to default
  fgColor=ILI9341_WHITE;bgColor=ILI9341_BLACK;charS=1;colors(); //Sets Text Color
  
  // Uncomment next line to set the text colour to the last segment value!
  //fgColor=text_colour;bgColor=ILI9341_BLACK;charS=1;colors(); //Sets Text Color
  
  // Print value, if the meter is large then use big font 6, othewise use 4
  int xPosition=x-36;
  int yPosition=(y-12);
  
  lcd.setCursor(xPosition,yPosition);
  
  float lvalue;
  if (decs==0){ //If there are decimals specified, numbers are stored as integers and must be "re-floated".
    lvalue=value; 
  }else{
    lvalue=value/(decs*10.0);
  }
  
  lcd.setTextSize(2);
  //Add leading spaces to keep numbers from jumping around when lengths change (somewhat).
  if (value >= 0 && value < 100) lcd.print("  ");
  if ((value <0 && value > -100) || value >= 100) lcd.print(" ");
  
  lcd.print(lvalue,decs); //Print the data value in the center.
  lcd.setTextSize(1);

  if(refresh){ // Print units at bottom
    lcd.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    lcd.setCursor(xPosition+24,yPosition+(r));
    lcd.print(units);
  }
  
  // Calculate and return right hand side x coordinate
  return x + r + 2;
  
}

// Return a 16 bit rainbow colour
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}

void receive(int numBytes) {}

