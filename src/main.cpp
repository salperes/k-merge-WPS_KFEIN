// WPS MODULE

#include <SparkFunLSM303C.h>

#include <Arduino.h>
#include <Wire.h>
#include "LowPower.h"

#include <RH_RF95.h>


#include <SPI.h>
#include <eeprom.h>
#include <SoftwareSerial.h>
#include "D:\Dropbox\Smart Parking\VS Solution\WPS Soluttion\WPS\ConfigFile.h"
//#include "C:\Users\salpe\Dropbox\smart parking\VS Solution\WPS Soluttion\WPS\ConfigFile.h"
//#include <SimpleKalmanFilter.h>

#include "wps.h"


//****************************************************************
//****************************************************************
//****************   DEĞİŞKEN TANIMLAMALARI   ********************
//****************************************************************
//****************************************************************

sWPSperiodical wpsdata;
sCoordData coordinatordata;



//#define TimeConstant 1250 // YAKLAŞIK 35sn
//#define TimeConstant 1070 // YAKLAŞIK 30sn

// BLE VARIABLES
#define bleenablepin 6
uint8_t blecounter = 0;
uint8_t bletimeout;
bool bleok = false;
bool blesleep = false;
uint8_t BLE_TX_POWER = 2; //Default value
#define BLE_POWER_PIN 3
#define RXpin 8
#define TXpin 9
SoftwareSerial blemodule(RXpin, TXpin);


// DETECTION ALGORITHM VARIABLES
uint8_t VECTOR_VARIANCE_LIMIT = def_VECTOR_VARIANCE_LIMIT;
uint8_t VECTOR_VARIANCE_BASE = def_VECTOR_VARIANCE_BASE;
uint8_t MIN_RSSI_DIFF = def_MIN_RSSI_DIFF;
uint8_t MAX_RSSI_DIFF = def_MAX_RSSI_DIFF;
bool CHECK_RSSI = def_CHECK_RSSI;
uint8_t VEHICLE_DETECTION_COUNT = def_VEHICLE_DETECTION_COUNT;
int8_t LASTRSSI, PREVRSSI;										//BURAYI 8 bit olarak tekrar değiştir...
uint8_t VEHICLE_DETECTION_COUNTER = 0;
uint8_t DIFFMULTIPLIER = defDIFFMULTIPLIER;
float Vector = 0;

// MAGNETORMETER VARIABLES
uint8_t GAIN = 0;
uint8_t MAG_FRQ = def_MAG_FRQ;
uint8_t MAG_OMXY = def_MAG_OMXY;
uint8_t MAG_OMZ = def_MAG_OMZ;
uint8_t MAGVal = 0; //MAG_FRQ, MAG_OMXY ve MAG_OMZ ile türetilen değer...

// RADIO VARIABLES
uint8_t RADIO_TX_POWER = def_RADIO_TX_POWER;


// SENSOR DEVICE VARIABLES
int VOLTAGE_OFFSET = 0;
uint8_t WPS_SLEEP_COUNT = def_WPS_SLEEP_COUNT;
uint8_t ErrorCode = 0;
uint32_t Sequence = 0;
uint8_t CompassType = 0;
uint8_t HWVersion = HW_VERSION;
uint8_t Vehicle = 0;
uint8_t COORDINATOR_ADDRESS = def_COORDINATOR_ADDRESS;
uint8_t WPS_ADDRESS = def_WPS_ADDRESS;
bool InitialState = STATE_EMPTY;
bool VarianceOrDiff = DIFFCHK;
bool CONFIG_APPLIED = false;
bool VehiclePacketSent = false; // SLEEP CYCLE TAMAMLANMADAN VEHICLE STATUS PAKEDİ YOLLANDI İSE SAYACI SIFIRLAMAK İÇİN KULLANILIYOR
								// AKSİ DURUMDA VEHICLE STATUS PAKEDİNDEN HEMEN SONRA PERIODICAL PAKEDİ YOLLUYOR. PİL ÖMRÜ İÇİN GEREKLİ


void(*resetFunc) (void) = 0;//declare reset function at address 0


// Singleton instance of the radio driver
RH_RF95 driver;

//RH_RF95 rf95;
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above

//RHReliableDatagram manager(driver, WPS_ADDRESS);
////RHReliableDatagram *manager = NULL;

/////////////////////////////////////////
////	HMC 5983L TANIMLAMALARI /////////
/////////////////////////////////////////
#define Xp 4
#define Yp 8
#define Zp 6
#define X 3
#define Y 7
#define Z 5

/////////////////////////////////////////
////	LSM303C   TANIMLAMALARI /////////
/////////////////////////////////////////

LSM303C lsm303c;
//Adafruit_LSM303 LSM303;		// LSM303

///////////////////
// FILTER VARIABLES
///////////////////
float AdaptiveBaseEmp[5] = { 0,0,0,0,0 };								//added @ Version 3.03
float AdaptiveBaseBsy[5] = { 0,0,0,0,0 };								//added @ Version 3.03
float AdaptiveBaseEmpty = 0;											//added @ Version 3.03
float AdaptiveBaseBusy = 0;												//added @ Version 3.03

uint8_t MovAvgnE = 0;
uint8_t MovAvgnB = 0;
uint8_t AdaptiveBaseBusyStabilationFactor = 0;
uint8_t IndexCounterEmpty, IndexCounterBusy;

float VectorPrev = 0;
float VectorPrev0 = 0;
float VectorDiff = 0;


float VarianceDiffEmpty = 0;
float VarianceDiffBusy = 0;

float TempVector = 0;
float VectorFilter[2] = { 0,0 };
uint8_t VectorFilterCounter = 0;



void BleWakeUp();
bool CheckBT();
void BleSleep();
void VoltageCalibration();
void RF95_setup();
int16_t RSSIdiff();
void PusulaINIT(bool poweron);
void TakeAvarage(uint8_t Times);
void Data_Prepare();
void axisreadings();
void Debug_Yaz();
void VoltageMeasurement();
void RadioSendPacket(uint8_t Type);
void SetBLEPower();
int HMC_Read(byte Axis);
void LotEmptyCheck();
void LotBusyCheck();
void ApplyConfig();
void SendConfigOK();
void ReCheckBT();
void Init_HMC(uint8_t TempSensor, uint8_t Sample_Rate, uint8_t Freq_Rate, uint8_t Gain_Rate);


//###########################################################################################
//###########################################################################################
//#####################                                               #######################
//#####################          SETUP BÖLÜMÜ                         #######################
//#####################                                               #######################
//###########################################################################################
//###########################################################################################
void setup() {
	//VoltageCalibration();
	// POWER ON BLE
	pinMode(BLE_POWER_PIN, OUTPUT);
	digitalWrite(BLE_POWER_PIN, HIGH);
	//pinMode(RXpin, OUTPUT);
	pinMode(TXpin, OUTPUT);
	//digitalWrite(TXpin,LOW)

	// SET ERROR TO NO ERROR
	ErrorCode = 0; // ERROR CODE = 0; no errors

				   // START SERIAL INTERFACE
	Serial.begin(115200);

	// START BLE INTERFACE
	blemodule.begin(9600);

	//#####################         BLE CHECK BÖLÜMÜ                         #######################
	delay(500);
	BleWakeUp();
	blemodule.flush(); //bledump();
	bleok = CheckBT();
	if (!bleok)
	{
		ErrorCode = ErrorCode | 2;
	}
	BleSleep();

	// BLE TX POWER AND LORA TX POWER SETTINGS 
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	/*
	uint8_t tttemp = 0;
	tttemp = 2;
	tttemp = tttemp << 5;
	tttemp = tttemp | RADIO_TX_POWER;
	EEPROM.write(8, tttemp);
	EEPROM.write(19,3);
	EEPROM.write(17, 2); //LORA CHANNEL

	EEPROM.write(7, VEHICLE_DETECTION_COUNT);
	delay(200);
	EEPROM.write(23, VECTOR_VARIANCE_BASE);
	delay(200);
	EEPROM.write(22, CHECK_RSSI);
	delay(200);
	EEPROM.write(24, MIN_RSSI_DIFF);
	delay(200);
	EEPROM.write(20, MAX_RSSI_DIFF);
	MAGVal = MAG_FRQ << 4 | MAG_OMXY << 2 | MAG_OMZ;
	EEPROM.write(21, MAGVal);
	delay (200);
	EEPROM.write(29, VarianceOrDiff);
	delay(200);
	EEPROM.write(30,InitialState);
	delay(200);
	EEPROM.write(29, DIFFCHK);
	delay(200);
	EEPROM.write(31, DIFFMULTIPLIER);
	Serial.println("EEPROM Correction done...");
	*/
	//EEPROM.write(6, 5); //var limit
	//EEPROM.write(23, 2); //var base
	//EEPROM.write(31, DIFFMULTIPLIER);
	//EEPROM.write(17, 2);
	//EEPROM.write(4, 3); 1.9Gauss for HMC5983
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	
	delay(8000); // MANYETİK RESETTEN SONRA UZAKLAŞMAK İÇİN SÜRE


	wpsdata.wpsdyn.nw_id = def_NETWORK_ID;
	wpsdata.wpssta.lorachannel = BASE_CHANNEL;

	if (EEPROM.read(1) != 2)
	{
		EEPROM.write(0, wpsdata.wpsdyn.nw_id);
		delay(200);
		EEPROM.write(1, 2); //2 EEPROM'DAKİ VERİLERİ KULLAN DEMEK....
		delay(200);
		EEPROM.write(2, COORDINATOR_ADDRESS);
		delay(200);
		EEPROM.write(3, WPS_ADDRESS);
		delay(200);
		if (HWVersion == 1 || HWVersion == 2)
		{
			EEPROM.write(4, GAIN_400);
		}
		else
		{
			EEPROM.write(4, GAIN_4);
		}
		delay(200);
		EEPROM.write(5, WPS_SLEEP_COUNT);
		delay(200);
		EEPROM.write(6, VECTOR_VARIANCE_LIMIT);
		delay(200);
		EEPROM.write(7, VEHICLE_DETECTION_COUNT);
		delay(200);

		// BLE TX POWER AND LORA TX POWER SETTINGS 
		uint8_t tempvalue = 0;
		tempvalue = BLE_TX_POWER;
		tempvalue = tempvalue << 5;
		tempvalue = tempvalue | RADIO_TX_POWER;
		EEPROM.write(8, tempvalue);

		delay(200);
		VoltageCalibration();
		delay(200);

		EEPROM.write(17, wpsdata.wpssta.lorachannel);
		delay(200);

		EEPROM.write(18, HW_VERSION);
		delay(200);
		

		EEPROM.write(19, def_BLE_TIMEOUT);
		delay(200);

		EEPROM.write(20, MAX_RSSI_DIFF);
		delay(200);

		MAGVal = MAG_FRQ << 4 | MAG_OMXY << 2 | MAG_OMZ;
		EEPROM.write(21, MAGVal);

		delay(200);
		EEPROM.write(22, CHECK_RSSI);
		
		delay(200);
		EEPROM.write(23, VECTOR_VARIANCE_BASE);
		
		delay(200);
		EEPROM.write(29, VarianceOrDiff);
				
		delay(200);
		EEPROM.write(30, InitialState);

		delay(200);
		EEPROM.write(31, DIFFMULTIPLIER);
	}

	// READ SETTINGS FROM EEPROM
	// 0	
	wpsdata.wpsdyn.nw_id = EEPROM.read(0);
	// 1 - EEPROM CHECK...
	// 2 - COORDINATOR ADDRESS
	COORDINATOR_ADDRESS = EEPROM.read(2);
	// 3 - WPS_ADDRESS
	WPS_ADDRESS = EEPROM.read(3);
	// 4 - GAIN
	GAIN = EEPROM.read(4);
	// 5 - SLEEP COUNT
	WPS_SLEEP_COUNT = EEPROM.read(5);
	// 6 - VECTOR_VARIANCE_LIMIT
	VECTOR_VARIANCE_LIMIT = EEPROM.read(6);
	// 7 - VEHICLE_DETECTION_COUNT 
	VEHICLE_DETECTION_COUNT = EEPROM.read(7);
	// 8 - BLE AND RADIO POWER
	uint8_t tempvalue = EEPROM.read(8);
	RADIO_TX_POWER = 0b00011111 & tempvalue;
	BLE_TX_POWER = (0b01100000 & tempvalue) >> 5;
	if (RADIO_TX_POWER > 23) RADIO_TX_POWER = 23;
	if (RADIO_TX_POWER < 5) RADIO_TX_POWER = 5;
	// 9-10
	VOLTAGE_OFFSET = (EEPROM.read(9) << 8) | EEPROM.read(10);
	// 11-12-13-14-15-16 SERIAL 
	wpsdata.wpssta.serialmsb = ((uint16_t)EEPROM.read(11)) << 8 | EEPROM.read(12);
	wpsdata.wpssta.serialble = (((uint32_t)EEPROM.read(13)) << 24) | (((uint32_t)EEPROM.read(14)) << 16) | (((uint32_t)EEPROM.read(15)) << 8) | EEPROM.read(16);
	// 17
	wpsdata.wpssta.lorachannel = EEPROM.read(17);
	RF95_setup();
	delay(200);
	RSSIdiff();
	Sequence = 1;
	// 18
	CompassType = 2;							//LSM303
	if (EEPROM.read(18) <= 2) CompassType = 1;	//HMC5983

	// 19
	bletimeout = EEPROM.read(19);				// BLE timeout (Mins)
	// 20 MAX_RSSI_DIFF
	MAX_RSSI_DIFF = EEPROM.read(20);
	// 21 ve 29
	MAGVal = EEPROM.read(21);
	MAG_FRQ = (MAGVal & 0b01110000) >> 4;
	MAG_OMXY = (MAGVal & 0b00001100) >> 2;
	MAG_OMZ = (MAGVal & 0b00000011);
	VarianceOrDiff = EEPROM.read(29);
	wpsdata.wpssta.mag_varordiff = VarianceOrDiff << 7 | MAGVal;
	// 22
	VECTOR_VARIANCE_BASE = EEPROM.read(23);
	// 23
	CHECK_RSSI = EEPROM.read(22);
	// 24
	MIN_RSSI_DIFF = EEPROM.read(24);
	// 25-28 Boş

	// 29 Yukarıda okundu

	// 30 
	InitialState = EEPROM.read(30);
	Vehicle = InitialState;
	//Serial.print("dbg initstate="); Serial.println(InitialState);
	// 31 
	DIFFMULTIPLIER = EEPROM.read(31);



	//#####################          I2C SETUP BÖLÜMÜ                         #######################
	Wire.begin();

	PusulaINIT(MAG_ON);
	TakeAvarage(10);
	PusulaINIT(MAG_OFF);

	
	
	// WPS DATA PREPARE
	//data 00		NW_ID Yukarıda EEPROM.read(0) ile oluşturuldu.
	//data 01		FLAG TYPE for first send
	wpsdata.wpsdyn.flagtype= FLAG_PERIODICAL;
	//data 2-4		SEQUENCE NUMBER calculated
	//data 5-6		WPS TEMP VALUE calculated
	//data 7-8		BATTERY VALUE calculated
	//data 9		LAST RSSI calculated
	//data 10		ERROR CODE & VEHICLE DETECTED caculated
	wpsdata.wpsdyn.error_vehicle = Vehicle << 7 | ErrorCode;
	//data 11		RETRY & VARIANCE calculated
	//data 12-15	Vector calculated
	//data 16-17	Versions
	wpsdata.wpssta.versions = ((uint16_t)((uint16_t)EEPROM.read(18) << 4 | SW_VERSION_MAJOR)) << 8 | SW_VERSION_MINOR;
	//data 18-23	SERIAL	YUKARIDA ALINIYOR (// 11-12-13-14-15-16 SERIAL)
	//data 24		MAG PARAMETERS	YUKARIDA ALINIYOR (// 21 ve 29)
	//data 25		VECTOR LIMITS Data_Prepare() içinde
	//data 26		RSSI LIMITS Data_Prepare() içinde
	//data 27		BLE & RADIO POWER & CHECK_RSSI Data_Prepare() içinde
	//data 28		VEHICLE D.C. & WPSSLEEP COUNT Data_Prepare() içinde
	//data 29		LORA CHANNEL YUKARIDA ALINIYOR (// 17)
	//data 30		DIFFMULTIPLIER & BLE TIMEOUT & SENSOR GAIN Data_Prepare() içinde
	//data 31		CheckSum to be calculated
	   

	Data_Prepare();
	axisreadings();
	Debug_Yaz();

}

//###########################################################################################
//###########################################################################################
//#####################                                               #######################
//#####################                 LOOP  BÖLÜMÜ                  #######################
//#####################                                               #######################
//###########################################################################################
//###########################################################################################
void loop() {

	//axisreadings();
	VoltageMeasurement();
	uint16_t sleeptimer = ((uint16_t)WPS_SLEEP_COUNT)*SleepCycleFactor;
	
	if (Sequence <= 5)
	{
		RadioSendPacket(FLAG_PERIODICAL);
		sleeptimer = 15 * SleepCycleFactor;
	}
	else if (Sequence % 10 == 0)
	{
		RadioSendPacket(FLAG_PERIODICAL);
	}
	else
	{
		RadioSendPacket(FLAG_INTER);
	}

	//if(!VehiclePacketSent) RadioSendPacket(FLAG_PERIODICAL); // BİR ÖNCE VEHICLE STATUS PAKEDİ YOLLANMAMIŞ İSE PERIODICAL PAKEDİ YOLLA
	//if (VehiclePacketSent) VehiclePacketSent = false;

	delay(100);
	driver.sleep();
	

	// FIRST 5 SEQUENCE PERIODICAL TIMEOUT APROX. 10mins. DUE TO QUICK CONFIG CHANGE

	
	for (uint16_t i = 0; i < sleeptimer; i++)

		//for (int i = 0; i < 15; i++) //TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST

	{
		//Serial.println("Enter sleep state...");
		
		//PusulaINIT(MAG_OFF); delay(20);									//added @ Version 3.03
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
		//LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);//TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST

		if (!blesleep)
		{
			//Serial.print("blecnt: "); Serial.println(blecounter);
			blecounter++;
			//Serial.println(millis());
		}

		if (blecounter > (bletimeout*7.5 / 2))
		{
			blecounter = 0;
			BleSleep();
		}
		
		//delay(50); PusulaINIT(MAG_ON); delay(50);									//added @ Version 3.03
		axisreadings();
		
	}

	//Serial.println("Wake");
	driver.setModeIdle();
}

void bledump()
{
	char c;
	while (blemodule.available())
	{
		c = blemodule.read();
		Serial.print(c);
	}
	Serial.println();
}

bool CheckBT()
{
	//delay(1000);
	//blemodule.print("AT"); delay(500);
	//bledump();


	//blemodule.flush();
	blemodule.print(F("AT"));
	delay(500);
	//Serial.println("BLE error check");
	if (blemodule.read() == 79 && blemodule.read() == 75)
	{
		bledump();
		delay(200);
		blemodule.print(F("AT+IBEA?"));
		delay(200);

		for (int i = 0; i < 7; i++)
		{
			Serial.print((char)blemodule.read());
		}
		Serial.println();
		char c = blemodule.read();

		if (c != '1' || EEPROM.read(1) != 2)
		{
			//SET BLE MODE TO iBeacon and Name to kmerge
			char c[2];
			uint8_t i = 0;
			uint8_t j = 0;
			blemodule.print(F("AT+ADDR?"));
			delay(200);
			bledump();

			blemodule.print(F("AT+ADDR?"));
			delay(200);
			for (i = 0; i < 8; i++) Serial.print((char)blemodule.read());
			i = 0;

			for (j = 0; j < 6; j++)
			{
				for (i = 0; i < 2; i++)	c[i] = blemodule.read();
				EEPROM.write((11 + j), CharToBin(c[0], c[1]));
			}
			bledump();

			delay(200);

			char beaconnumber[] = "AT+MARJ0x0000";
			char buffer[2];
			uint8_t buff = 0;

			// MARJ -- BYTE 1 < SERIAL 3rd BYTE
			buff = EEPROM.read(13);
			if (buff > 9)
			{
				itoa(buff, buffer, 16);
			}
			else
			{
				buffer[0] = 48;
				buffer[1] = 48 + buff;
			}
			beaconnumber[9] = toCapital(buffer[0]);
			beaconnumber[10] = toCapital(buffer[1]);

			// MARJ -- BYTE 0 < SERIAL 4th BYTE

			buff = EEPROM.read(14);
			if (buff > 9)
			{
				itoa(buff, buffer, 16);
			}
			else
			{
				buffer[0] = 48;
				buffer[1] = 48 + buff;
			}

			beaconnumber[11] = toCapital(buffer[0]);
			beaconnumber[12] = toCapital(buffer[1]);

			//Serial.println(beaconnumber);
			blemodule.print(beaconnumber); Serial.println();
			delay(200); bledump();
			Serial.println(beaconnumber);

			beaconnumber[4] = 'I';
			beaconnumber[5] = 'N';
			beaconnumber[6] = 'O';

			// MINO -- BYTE 1 < SERIAL 5th BYTE
			buff = EEPROM.read(15);
			if (buff > 9)
			{
				itoa(buff, buffer, 16);
			}
			else
			{
				buffer[0] = 48;
				buffer[1] = 48 + buff;
			}
			beaconnumber[9] = toCapital(buffer[0]);
			beaconnumber[10] = toCapital(buffer[1]);

			// MINO -- BYTE 0 < SERIAL 6th BYTE
			buff = EEPROM.read(16);
			if (buff > 9)
			{
				itoa(buff, buffer, 16);
			}
			else
			{
				buffer[0] = 48;
				buffer[1] = 48 + buff;
			}
			beaconnumber[11] = toCapital(buffer[0]);
			beaconnumber[12] = toCapital(buffer[1]);

			Serial.println(beaconnumber);
			blemodule.print(beaconnumber); Serial.println();
			delay(200); bledump();

			blemodule.print(F("AT+ADVI5")); Serial.println();
			delay(200); bledump();
			blemodule.print(F("AT+NAMEkmerge")); Serial.println();
			delay(200); bledump();
			blemodule.print(F("AT+ADTY3"));	Serial.println();
			delay(200); bledump();
			blemodule.print(F("AT+IBEA1"));	Serial.println();
			delay(200); bledump();
			blemodule.print(F("AT+DELO2"));	Serial.println();
			delay(200); bledump();
			blemodule.print(F("AT+PWRM1"));	Serial.println();
			delay(200); bledump();

			SetBLEPower();

			Serial.println();

			bledump();

			blemodule.print(F("AT+RESET"));
			delay(600);
			bledump();
		}
		delay(200); bledump();
		blemodule.print(F("AT+POWE?"));
		delay(200);
		for (int i = 0; i < 7; i++)
		{
			blemodule.read();
		}
		Serial.println();
		BLE_TX_POWER = blemodule.read() - 48;
		Serial.print(F("BLE PRW : "));
		Serial.println(BLE_TX_POWER);
		return true;
	}
	return false;
}

void Data_Prepare()
{
	//data 25
	if (VECTOR_VARIANCE_BASE > 7) VECTOR_VARIANCE_BASE = 7;
	wpsdata.wpssta.varlimits = VECTOR_VARIANCE_LIMIT << 3 | VECTOR_VARIANCE_BASE;

	//data 26
	if (MIN_RSSI_DIFF > 7) MIN_RSSI_DIFF = 7;
	if (MAX_RSSI_DIFF > 31) MAX_RSSI_DIFF = 31;
	wpsdata.wpssta.rssilimits = MAX_RSSI_DIFF << 3 | MIN_RSSI_DIFF;

	//data 27
	if (CHECK_RSSI)
	{
		wpsdata.wpssta.blepw_radiopw_chkrssi = 0b10000000 | BLE_TX_POWER << 5 | RADIO_TX_POWER;
	}
	else
	{
		wpsdata.wpssta.blepw_radiopw_chkrssi = 0b00000000 | BLE_TX_POWER << 5 | RADIO_TX_POWER;
	}

	//data 28
	wpsdata.wpssta.vehicledc_sleepc = VEHICLE_DETECTION_COUNT << 6 | WPS_SLEEP_COUNT;
	   	
	//data 30
	wpsdata.wpssta.diffmultp_bleto_gain = bletimeout << 4 | GAIN << 2 | DIFFMULTIPLIER;


	if (VarianceOrDiff == VARIANCECHK)
	{
		VarianceDiffEmpty = AdaptiveBaseEmpty * VECTOR_VARIANCE_BASE / 100;
		VarianceDiffBusy = AdaptiveBaseEmpty * VECTOR_VARIANCE_LIMIT / 100;
		//empty_variance = VECTOR_VARIANCE_BASE;
		//busy_variance = VECTOR_VARIANCE_LIMIT;
	}
	else
	{
		VarianceDiffEmpty = VECTOR_VARIANCE_BASE * 10 * (DIFFMULTIPLIER + 1);
		VarianceDiffBusy = VECTOR_VARIANCE_LIMIT * 10 * (DIFFMULTIPLIER + 1);
	}
}

void axisreadings()
{
	PusulaINIT(MAG_ON); delay(50);
	float tempX, tempY, tempZ, tmpVect;

	if (CompassType == 1)
	{
		tempX = (float)HMC_Read(X);
		tempY = (float)HMC_Read(Y);
		tempZ = (float)HMC_Read(Z);
		
		Wire.beginTransmission(HMC_Address);
		Wire.write(0x31);
		Wire.endTransmission();

		// HMC üzerinden sıcaklık değerini al...
		Wire.requestFrom(HMC_Address, 2);
		//data 5-6
		wpsdata.wpsdyn.temp = (((uint16_t)Wire.read()) << 8) | Wire.read();
	}

	if (CompassType == 2)
	{
		tempX = (float)lsm303c.readMagX();
		tempY = (float)lsm303c.readMagY();
		tempZ = (float)lsm303c.readMagZ();
		//data 5-6
		wpsdata.wpsdyn.temp = lsm303c.readTempValue();
	}

	tmpVect = sqrtf(tempX*tempX + tempY * tempY + tempZ * tempZ);

	if (Vector == 0)
	{
		ErrorCode = ErrorCode | 0b00000001;
	}
	else
	{
		ErrorCode = ErrorCode & 0b11111110;
	}


	float tempvector = 0;
	//float empty_variance = 0;
	//float busy_variance = 0;
	bool JumpVector = false;
	int Sloope = -1;
	
	// VECTOR MOV_AVG FILTER
	VectorFilter[VectorFilterCounter%2] = tmpVect;
	VectorFilterCounter++;
	Vector = (VectorFilter[0] + VectorFilter[1]) / 2;
	   	

	

	if (abs(Vector - VectorPrev0) > VarianceDiffEmpty)
	{
		JumpVector = true;
		Serial.println(F("Jump"));
		if (Vector > VectorPrev0) Sloope = 1;
	}
	else
	{
		Serial.println(F("Steady"));
	}
	VectorPrev0 = VectorPrev;
	VectorPrev = Vector;


	if ((abs(Vector - AdaptiveBaseEmpty) <= VarianceDiffEmpty) && InitialState == STATE_EMPTY)
	{
		Serial.println(F("Empty"));
		AdaptiveBaseBusy = 0;
		
		if (!JumpVector)
		{
			AdaptiveBaseEmp[IndexCounterEmpty%5] = Vector;
			AdaptiveBaseBsy[IndexCounterEmpty%5] = Vector;
			//VectorDiff = 0;
			IndexCounterEmpty++;
			AdaptiveBaseEmpty = (AdaptiveBaseEmp[0] + AdaptiveBaseEmp[1] + AdaptiveBaseEmp[2] + AdaptiveBaseEmp[3] + AdaptiveBaseEmp[4]) / 5;
		}
		LotEmptyCheck();
	}

	if ((abs(Vector - AdaptiveBaseEmpty) > VarianceDiffBusy)|| InitialState==STATE_BUSY)
	{
		Serial.println(F("Busy"));
		if (!JumpVector)
		{
			AdaptiveBaseBsy[IndexCounterBusy%5] = Vector;
			IndexCounterBusy++;
			//if (IndexCounterBusy == 5) IndexCounterBusy = 0;
			AdaptiveBaseBusy = (AdaptiveBaseBsy[0] + AdaptiveBaseBsy[1] + AdaptiveBaseBsy[2] + AdaptiveBaseBsy[3] + AdaptiveBaseBsy[4]) / 5;

			// BLE KAPATILDIKTAN SONRA Adaptive Base Line Busy çalışmaya başlayacak....
			if (AdaptiveBaseBusyStabilationFactor > (6 + (bletimeout*7.5 / 2)))
			{
				AdaptiveBaseEmpty = AdaptiveBaseBusy - VectorDiff;
				AdaptiveBaseEmp[0] = AdaptiveBaseEmpty; AdaptiveBaseEmp[1] = AdaptiveBaseEmpty; AdaptiveBaseEmp[2] = AdaptiveBaseEmpty; AdaptiveBaseEmp[3] = AdaptiveBaseEmpty; AdaptiveBaseEmp[4] = AdaptiveBaseEmpty;
			}
			else
			{
				AdaptiveBaseBusyStabilationFactor++;
				VectorDiff = AdaptiveBaseBusy - AdaptiveBaseEmpty;
			}
		}
		else
		{
			//INITIAL STATE BUSY İSE AŞAĞIDAKİLER İLE EMPTY KONUMUNA GETİR
			if (InitialState == STATE_BUSY)
			{
				InitialState = STATE_EMPTY;
				AdaptiveBaseEmpty = tmpVect;
				AdaptiveBaseEmp[0] = AdaptiveBaseEmpty; AdaptiveBaseEmp[1] = AdaptiveBaseEmpty; AdaptiveBaseEmp[2] = AdaptiveBaseEmpty; AdaptiveBaseEmp[3] = AdaptiveBaseEmpty; AdaptiveBaseEmp[4] = AdaptiveBaseEmpty;
				EEPROM.write(30, InitialState); 
			}
		}
		tempvector = abs(((AdaptiveBaseBusy - VectorDiff) - Vector) * 100 / (AdaptiveBaseBusy - VectorDiff));
		LotBusyCheck();
	}


	//data 11 Variance
	if (tempvector > 15) tempvector = 15;
	wpsdata.wpsdyn.retry_variance = ((uint8_t)tempvector << 4) | (0b00001111 & wpsdata.wpsdyn.retry_variance);
	

	PusulaINIT(MAG_OFF);

	Serial.print(F("% : ")); Serial.print(tempvector); Serial.print(F("  - Vector : ")); Serial.print(Vector);
	Serial.print(F("  - ABE : ")); Serial.print(AdaptiveBaseEmpty); Serial.print(F("  - ABB :")); Serial.print(AdaptiveBaseBusy); Serial.print(F("  - Diff :")); Serial.println(VectorDiff);
	delay(50);
}

void LotBusyCheck()
{
	int16_t rssidiff = 0;
	if (Vehicle == 0) //EĞER ARAÇ VAR BİLGİSİ =0 İSE ARAÇ DETECTION COUNT KADAR ÜST ÜSTE VAR MI?
	{
		VEHICLE_DETECTION_COUNTER++;
		if (VEHICLE_DETECTION_COUNTER > VEHICLE_DETECTION_COUNT)
		{
			if (CHECK_RSSI)											//Version 3.02
			{														//Version 3.02
				rssidiff = RSSIdiff();								//Version 3.02
				if (rssidiff >= 255) rssidiff = MAX_RSSI_DIFF; 		//Version 3.02
			}														//Version 3.02
			else rssidiff = MAX_RSSI_DIFF;							//Version 3.02
																	//Version 3.02
			if (rssidiff >= MAX_RSSI_DIFF)							//Version 3.02
			{														//Version 3.02
																	//ARAÇ VARSA PAKET YOLLA....
				VEHICLE_DETECTION_COUNTER = VEHICLE_DETECTION_COUNT;

				// Pil voltajı ölç
				VoltageMeasurement();

				//BLE MODULÜNÜ AÇ
				BleWakeUp();
				ReCheckBT();
				BleSleep();

				// LoRA pakedi içerisine araç var bilgisi ekle
				Vehicle = 1;
				Serial.println(F("DBG Lora idle"));

				//LoRA aç
				driver.setModeIdle();
				Serial.println(F("DBG Lora send"));
				//Paket yolla
				RadioSendPacket(FLAG_VEHICLE_STATUS_CHANGE);
				VehiclePacketSent = true;
				//LoRA sleep
				driver.sleep();
				//BLE TIMER ÇALIŞTIR

				BleWakeUp();
				
			}														//Version 3.02
			else VEHICLE_DETECTION_COUNTER--;								//Version 3.02
		}
	}
}

void LotEmptyCheck()
{

	if ((Vehicle == true)) //Version 3.00
	{
		int16_t rssidiff = 0;
		if (CHECK_RSSI)													//Version 3.00
		{																//Version 3.00
			rssidiff = RSSIdiff();										//Version 3.00
			if (rssidiff == 255) rssidiff = MIN_RSSI_DIFF;				//Version 3.00
		}																//Version 3.00
		else rssidiff = MIN_RSSI_DIFF;//CHECK_RSSI = FALSE İSE MIN_RSSI_DIFF ile aynı sayıtı ata.... //Version 3.00

		if (rssidiff <= MIN_RSSI_DIFF)									//Version 3.00
		{																//Version 3.00
			VEHICLE_DETECTION_COUNTER = 0;
			Vehicle = 0;
			VectorDiff = 0;
			AdaptiveBaseBusy = 0;
			AdaptiveBaseBsy[0] = 0; AdaptiveBaseBsy[1] = 0; AdaptiveBaseBsy[2] = 0; AdaptiveBaseBsy[3] = 0; AdaptiveBaseBsy[4] = 0;
			MovAvgnB = 0;
			AdaptiveBaseBusyStabilationFactor = 0;

			driver.setModeIdle();
			VoltageMeasurement();
			RadioSendPacket(FLAG_VEHICLE_STATUS_CHANGE);
			VehiclePacketSent = true;
			BleSleep();
			driver.sleep();
		}																//Version 3.00
	}
	else if (VEHICLE_DETECTION_COUNTER != 0)	VEHICLE_DETECTION_COUNTER--;//Version 3.01 
}

void Debug_Yaz()
{
	Serial.print(F("Seq ID:"));
	Serial.println(Sequence);

	Serial.print(F("Vector0:"));
	Serial.println(AdaptiveBaseEmpty);

	Serial.print(F("Vector :"));
	Serial.println(Vector);

	Serial.print(F("WPS   :"));
	Serial.println(WPS_ADDRESS);

	Serial.print(F("SERIAL:"));
	Serial.print(wpsdata.wpssta.serialmsb, HEX); Serial.print(F(":"));
	Serial.println(wpsdata.wpssta.serialble, HEX);
	
	Serial.print(F("SLEEP :"));
	Serial.println(WPS_SLEEP_COUNT);

	Serial.print(F("VARIANC:"));
	Serial.print(wpsdata.wpsdyn.retry_variance>>4);
	Serial.print(F("-"));
	Serial.println(VECTOR_VARIANCE_LIMIT);
	Serial.print(F("BLE PWR:"));
	Serial.println(BLE_TX_POWER);
	Serial.print(F("TEMP  :"));
	uint8_t t1 = (uint8_t)(wpsdata.wpsdyn.temp >> 8);
	uint8_t t2 = (uint8_t)(wpsdata.wpsdyn.temp);

	if (CompassType < 2)
	{
		Serial.println((float)((int16_t)t1*256+t2) / 128 + 25);
	}
	else
	{
		Serial.println((float)((int16_t)t1 * 256 + t2) / 8 + 25);
	}

	Serial.print(F("HWver :")); Serial.println((uint8_t)(wpsdata.wpssta.versions >> 12));
}

void RadioSendPacket(uint8_t packet_type)
{
	wpsdata.wpsdyn.flagtype= packet_type;
	
	//data 2-4
	wpsdata.wpsdyn.sequence3= Sequence >> 16;
	wpsdata.wpsdyn.sequence2 = Sequence >> 8;
	wpsdata.wpsdyn.sequence1 = Sequence;

	//data 10
	wpsdata.wpsdyn.error_vehicle = Vehicle << 7 | ErrorCode;

	//float tvector = 0;

	//data 12-15
	if (Sequence == 1)
	{
		wpsdata.wpsdyn.vector = AdaptiveBaseEmpty;
		if (InitialState) wpsdata.wpsdyn.vector = AdaptiveBaseBusy;
	}
	else
	{
		wpsdata.wpsdyn.vector = Vector;
	}
		
	uint8_t RetryCount = 0;

SendPacketLine:
	//data 11
	wpsdata.wpsdyn.retry_variance = (0b11110000 & wpsdata.wpsdyn.retry_variance) | (RetryCount + 1);

	
	wpsdata.wpssta.checksum = wpsdata.wpsdyn.nw_id;

	driver.setModeRx();
	driver.waitCAD();
	driver.setModeTx();
	if (packet_type == FLAG_INTER) driver.send((uint8_t*)&wpsdata.wpsdyn, sizeof(wpsdata.wpsdyn));
	else driver.send((uint8_t*)&wpsdata, sizeof(wpsdata));

	driver.waitPacketSent();
	
	driver.setModeRx();
	
	//Serial.println("Sent...wait for ACK");
	
	// Now wait for a reply from the COORDINATOR
	if (!driver.waitAvailableTimeout(8000))
	{
		Serial.println(F("retry"));
		delay(random(200, 1000)); // Version 3.00
		RetryCount++;

		if (RetryCount < 10)
		{
			goto SendPacketLine;
		}
		else
		{
			Serial.println(F("No resp."));
			ErrorCode = ErrorCode | 4; //TURN ON BIT 2. TRANSMISSION FAILED....
			goto EndOfStory;
		}
	}
	else
	{
		ErrorCode = ErrorCode & ~4; //TURN OFF BIT 2. IF CURRENT TRANSMISSION SUCCEDED.

		uint8_t tmplen = sizeof(coordinatordata);
		
		
		//GELEN PAKET VAR..... İNCELENECEK
		wpsdata.wpsdyn.rssi = (uint8_t)(-1 * driver.lastRssi());
		if (driver.recv((uint8_t*)&coordinatordata, &tmplen))
		{
			if (coordinatordata.nw_id == wpsdata.wpsdyn.nw_id)
			{

				switch (coordinatordata.flagtype)
				{
				case FLAG_ACK_OK: {Serial.println(F("ACK_OK rcv"));  break; }

				case FLAG_RETRANSMIT:
				{
					if (RetryCount < 10)
					{
						Serial.println(F("re_tx"));
						RetryCount++;
						goto SendPacketLine;
					}
					else
					{
						Serial.println(F("tx lim."));
						goto EndOfStory;
					}
					break;
				}
				case FLAG_NEW_CONFIGURATION: //{Serial.print("Change Request"); ApplyConfig(); break; }
				{
					Serial.println(F("CH_RQS rcv"));
					ApplyConfig();
					break;
				}
				}
			}
		}

		// TÜM ALINANLAR DOĞRU İSE FIN PAKEDİ YOLLA...
		wpsdata.wpsdyn.flagtype= FLAG_FIN;
		if (!CONFIG_APPLIED)
		{
			driver.setModeTx();
			driver.send((uint8_t*)&wpsdata, sizeof(wpsdata));
			//driver.send(data_to_coordinator, sizeof(data_to_coordinator));
			driver.waitPacketSent();
			Serial.println(F("Send FIN"));
			delay(50);
		}
		// CONFIG_APPLIED COORDINATOR TARAFINDAN SendConfigOK() İÇERİSİNDE ALINMAMIŞSA
		// TRUE OLARAK BIRAKILMIŞTI. PAKET ALINDI BİLGİSİ SONRASI BURADA FALSE YAPILACAK
		CONFIG_APPLIED = false;
		driver.sleep();
	}

EndOfStory:
	//Serial.println("EndOfStory");
	Sequence++;
	if (Sequence >= 16777215) Sequence = 1;

}

void ApplyConfig()
{
	bool RF95Reset = false;
	if (coordinatordata.changerq != 0)
	{


		if ((coordinatordata.changerq & 0b00000100) >> 2)
		{
			//coorddata 05
			if ((coordinatordata.changeflag & 0b00000001))
			{
				MAGVal = coordinatordata.mag_varordiff & 0b01111111;
				MAG_FRQ = (MAGVal & 0b01110000) >> 4;
				MAG_OMXY = (MAGVal & 0b00001100) >> 2;
				MAG_OMZ = (MAGVal & 0b00000011);
				EEPROM.write(21, MAGVal);

				VarianceOrDiff = (coordinatordata.mag_varordiff & 0b10000000) >> 7;
				EEPROM.write(29, VarianceOrDiff);
				wpsdata.wpssta.mag_varordiff = coordinatordata.mag_varordiff;
				CONFIG_APPLIED = true;
				Data_Prepare();
			}

			//coorddata 06
			if ((coordinatordata.changeflag & 0b00000010) >> 1)
			{
				wpsdata.wpssta.varlimits = coordinatordata.varlimits;
				VECTOR_VARIANCE_BASE = coordinatordata.varlimits & 0b00000111;
				VECTOR_VARIANCE_LIMIT = (coordinatordata.varlimits & 0b11111000) >> 3;
				EEPROM.write(23, VECTOR_VARIANCE_BASE);
				delay(20);
				EEPROM.write(6, VECTOR_VARIANCE_LIMIT);
				CONFIG_APPLIED = true;
				Data_Prepare();
			}

			//coorddata 07
			if ((coordinatordata.changeflag & 0b00000100) >> 2)
			{
				wpsdata.wpssta.rssilimits = coordinatordata.rssilimits;
				MIN_RSSI_DIFF = coordinatordata.rssilimits & 0b00000111;
				MAX_RSSI_DIFF = (coordinatordata.rssilimits & 0b11111000) >> 3;
				EEPROM.write(20, MAX_RSSI_DIFF);
				EEPROM.write(24, MIN_RSSI_DIFF);
				CONFIG_APPLIED = true;
				Data_Prepare();
			}

			//coorddata 08
			if ((coordinatordata.changeflag & 0b00001000) >> 3)
			{
				wpsdata.wpssta.blepw_radiopw_chkrssi = coordinatordata.blepw_radiopw_chkrssi;
				RADIO_TX_POWER = coordinatordata.blepw_radiopw_chkrssi & 0b00011111;
				BLE_TX_POWER = (coordinatordata.blepw_radiopw_chkrssi & 0b01100000) >> 5;
				CHECK_RSSI = (coordinatordata.blepw_radiopw_chkrssi & 0b10000000) >> 7;

				if (RADIO_TX_POWER > 23) RADIO_TX_POWER = 23;
				if (RADIO_TX_POWER < 5) RADIO_TX_POWER = 5;

				EEPROM.write(8, (coordinatordata.blepw_radiopw_chkrssi & 0b01111111));

				BleWakeUp();
				delay(50);
				SetBLEPower();
				BleSleep();

				EEPROM.write(22, CHECK_RSSI);

				driver.setTxPower(RADIO_TX_POWER, false);
				CONFIG_APPLIED = true;
				RF95Reset = true;
			}

			//coorddata 09 
			if ((coordinatordata.changeflag & 0b00010000) >> 4)
			{
				wpsdata.wpssta.vehicledc_sleepc = coordinatordata.vehicledc_sleepc;
				WPS_SLEEP_COUNT = coordinatordata.vehicledc_sleepc & 0b00111111;
				VEHICLE_DETECTION_COUNT = (coordinatordata.vehicledc_sleepc & 0b11000000) >> 6;
				EEPROM.write(5, WPS_SLEEP_COUNT);
				delay(20);
				EEPROM.write(7, VEHICLE_DETECTION_COUNT);
				CONFIG_APPLIED = true;
			}

			//coorddata 10
			if ((coordinatordata.changeflag & 0b00100000) >> 5)
			{
				wpsdata.wpssta.lorachannel = coordinatordata.lorachannel;
				EEPROM.write(17, wpsdata.wpssta.lorachannel);
				CONFIG_APPLIED = true;
				RF95Reset = true;
			}

			//coorddata 11
			if ((coordinatordata.changeflag & 0b01000000) >> 6)
			{
				wpsdata.wpssta.diffmultp_bleto_gain = coordinatordata.diffmultp_bleto_gain;
				DIFFMULTIPLIER = coordinatordata.diffmultp_bleto_gain & 0b00000011;
				GAIN = (coordinatordata.diffmultp_bleto_gain & 0b00001100) >> 2;
				bletimeout = (coordinatordata.diffmultp_bleto_gain & 0b11110000) >> 4;

				EEPROM.write(4, GAIN);
				delay(20);
				EEPROM.write(19, bletimeout);
				delay(20);
				EEPROM.write(31, DIFFMULTIPLIER);
				
				CONFIG_APPLIED = true;
			}

			//coorddata 12
			if ((coordinatordata.changeflag & 0b10000000) >> 7)
			{
				if ((coordinatordata.changerq & 0b00010000) >> 4 == 0)
				{
					WPS_ADDRESS = coordinatordata.wpsaddress;
					EEPROM.write(3, WPS_ADDRESS);
				}
				else
				{
					wpsdata.wpsdyn.nw_id = coordinatordata.wpsaddress;
					EEPROM.write(0, wpsdata.wpsdyn.nw_id);
				}
				CONFIG_APPLIED = true;
				RF95Reset = true;
			}
		}

		if (coordinatordata.changerq == 4)	SendConfigOK(); //SEND CONFIGOK IF ONLY CHANGE PACKET (Not with any RESET flag)

		// Vector0 Reset
		if ((coordinatordata.changerq & 0b00000001))
		{
			if ((coordinatordata.changerq & 0b00001000) >> 3) InitialState = STATE_BUSY;
			CONFIG_APPLIED = true;
			TakeAvarage(3);
			SendConfigOK();
		}

		// Module Reset
		if ((coordinatordata.changerq & 0b00000010) >> 1)
		{
			if ((coordinatordata.changerq & 0b00001000) >> 3) EEPROM.write(30, STATE_BUSY);
			CONFIG_APPLIED = true;
			SendConfigOK();
			delay(100);
			resetFunc();

		}
	}
		   	 
	if (RF95Reset) RF95_setup();

}

void SendConfigOK()
{
	wpsdata.wpsdyn.flagtype= FLAG_CONFIGURATION_SET;
	//Serial.println("Set To TX");
	driver.setModeRx();
	driver.waitCAD();
	driver.setModeTx();
	driver.send((uint8_t*)&wpsdata, sizeof(wpsdata));
	//driver.send(data_to_coordinator, sizeof(data_to_coordinator));
	driver.waitPacketSent();
	//	Serial.println("Send CHG OK");
	delay(20);
}

void TakeAvarage(uint8_t Iteration)
{
	float SumX = 0;
	float SumY = 0;
	float SumZ = 0;

	if (CompassType == 1)
	{
		for (int i = 0; i < Iteration; i++)
		{
			SumX += (float)HMC_Read(X);
			SumY += (float)HMC_Read(Y);
			SumZ += (float)HMC_Read(Z);
			delay(1000);
		}
	}

	if (CompassType == 2)
	{
		for (int i = 0; i < Iteration; i++)
		{
			SumX += lsm303c.readMagX();
			SumY += lsm303c.readMagY();
			SumZ += lsm303c.readMagZ();
			delay(1000);
		}
	}

	
	if (InitialState == STATE_EMPTY)
	{
		AdaptiveBaseEmpty = sqrtf(sq((float)(SumX / Iteration)) + sq((float)(SumY / Iteration)) + sq((float)(SumZ / Iteration)));

		VectorFilter[0] = AdaptiveBaseEmpty;
		VectorFilter[1] = AdaptiveBaseEmpty;
	
		AdaptiveBaseEmp[0] = AdaptiveBaseEmpty;
		AdaptiveBaseEmp[1] = AdaptiveBaseEmpty;
		AdaptiveBaseEmp[2] = AdaptiveBaseEmpty;
		AdaptiveBaseEmp[3] = AdaptiveBaseEmpty;
		AdaptiveBaseEmp[4] = AdaptiveBaseEmpty;
		AdaptiveBaseEmpty = AdaptiveBaseEmpty;
		VectorPrev = AdaptiveBaseEmpty;
	}
	else
	{
		AdaptiveBaseBusy = sqrtf(sq((float)(SumX / Iteration)) + sq((float)(SumY / Iteration)) + sq((float)(SumZ / Iteration)));

		VectorFilter[0] = AdaptiveBaseBusy;
		VectorFilter[1] = AdaptiveBaseBusy;
	
		AdaptiveBaseEmp[0] = AdaptiveBaseBusy;
		AdaptiveBaseEmp[1] = AdaptiveBaseBusy;
		AdaptiveBaseEmp[2] = AdaptiveBaseBusy;
		AdaptiveBaseEmp[3] = AdaptiveBaseBusy;
		AdaptiveBaseEmp[4] = AdaptiveBaseBusy;
		AdaptiveBaseEmpty = AdaptiveBaseBusy;
		VectorPrev = AdaptiveBaseBusy;
	}
}

void PusulaINIT(bool poweron)
{

	if (CompassType == 1)
	{
		uint8_t tmpGain = 0;
		switch (GAIN)
		{
		case (1): tmpGain = GAIN_130; break;
		case (2): tmpGain = GAIN_190; break;
		case (3): tmpGain = GAIN_250; break;
		default: tmpGain = GAIN_088; break;
		}

		Init_HMC(Temp_Sensor_Enable, Sample_x4, Rate_3, tmpGain);
	}

	if (CompassType == 2)
	{
		uint8_t tmpGain = 0;
		uint8_t tmpMAGFRQ = 0;
		uint8_t tmpMAGOMZ = 0;
		uint8_t tmpMAGOMXY = 0;
		uint8_t mag_power = 0;
		if (poweron) mag_power = 4;

		switch (GAIN)
		{
		case (1): tmpGain = GAIN_8; break;
		case (2): tmpGain = GAIN_12; break;
		case (3): tmpGain = GAIN_16; break;
		default: tmpGain = GAIN_4; break;
		}

		switch (MAG_FRQ)
		{
		case (0): tmpMAGFRQ = 0x00; break;
		case (1): tmpMAGFRQ = 0x04; break;
		case (2): tmpMAGFRQ = 0x08; break;
		case (3): tmpMAGFRQ = 0x0C; break;
		case (4): tmpMAGFRQ = 0x10; break;
		case (5): tmpMAGFRQ = 0x14; break;
		case (6): tmpMAGFRQ = 0x18; break;
		case (7): tmpMAGFRQ = 0x1C; break;
		}
		switch (MAG_OMXY)
		{
		case (0): tmpMAGOMXY = 0x00; break;
		case (1): tmpMAGOMXY = 0x20; break;
		case (2): tmpMAGOMXY = 0x40; break;
		case (3): tmpMAGOMXY = 0x60; break;
		}
		switch (MAG_OMZ)
		{
		case (0): tmpMAGOMZ = 0x00; break;
		case (1): tmpMAGOMZ = 0x04; break;
		case (2): tmpMAGOMZ = 0x08; break;
		case (3): tmpMAGOMZ = 0x0C; break;
		}

		//if (lsm303c.begin(GAIN) != IMU_SUCCESS)	ErrorCode = ErrorCode | 1;
		

		bool pusulainit = lsm303c.begin(// Default to I2C bus
			MODE_I2C,
			// Initialize magnetometer output data rate to 0.625 Hz (turn on device)
			//GÜÇ TÜKETİMİ İÇİN BURAYI TEKRAR KONTROL ET. 5 VEYA 10 Hz YAPIP KONTROL ET.
			tmpMAGFRQ,
			// Initialize magnetic field full scale to +/-16 gauss
			tmpGain,//MAG_FS_4_Ga,
					// Enabling block data updating
			MAG_BDU_ENABLE,
			// Initialize magnetometer X/Y axes ouput data rate to high-perf mode
			//MAG_OMXY_HIGH_PERFORMANCE,
			tmpMAGOMXY,
			// Initialize magnetometer Z axis performance mode
			//MAG_OMZ_HIGH_PERFORMANCE,
			tmpMAGOMZ,
			// Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
			mag_power,
			//MAG_MD_CONTINUOUS,
			//MAG_MD_SINGLE,
			// Initialize acceleration full scale to +/-2g
			ACC_FS_2g,
			// Enable block data updating
			ACC_BDU_ENABLE,
			// Enable X, Y, and Z accelerometer axes
			ACC_X_ENABLE | ACC_Y_ENABLE | ACC_Z_ENABLE,
			// Initialize accelerometer output data rate to 100 Hz (turn on device)
			//ACC_ODR_100_Hz
			ACC_ODR_POWER_DOWN,
			//TEMP SENSÖRÜ AÇ
			MAG_TEMP_EN_ENABLE
		);

		if (poweron)
		{
			if (pusulainit != IMU_SUCCESS)	ErrorCode = ErrorCode | 1;
		}
		
	}

	delay(100);
}

void RF95_setup()
{
	if (driver.init())
	{
		Serial.println(F("Radio OK"));
	}
	else
	{
		Serial.println(F("Radio Bad"));
	}
	driver.setModemConfig(5);
	Serial.print(F("Set power:"));
	Serial.println(RADIO_TX_POWER);
	driver.setTxPower(RADIO_TX_POWER, false);

	Serial.print(F("Set WPS Adr:"));
	Serial.println(WPS_ADDRESS);
	driver.setThisAddress(WPS_ADDRESS);
	driver.setHeaderFrom(WPS_ADDRESS);

	Serial.print(F("Set TO Adr:"));
	Serial.println(COORDINATOR_ADDRESS);
	driver.setHeaderTo(COORDINATOR_ADDRESS);

	Serial.print(F("Set frq		:"));
	if (wpsdata.wpssta.lorachannel < 64)
	{
		Serial.println(BASE_FREQ433 + ((float)wpsdata.wpssta.lorachannel)*0.2);
		driver.setFrequency(BASE_FREQ433 + ((float)wpsdata.wpssta.lorachannel)*0.2);

	}
	else
	{
		Serial.println(BASE_FREQ865 + ((float)(64 - wpsdata.wpssta.lorachannel))*0.3);
		driver.setFrequency(BASE_FREQ865 + ((float)(64 - wpsdata.wpssta.lorachannel))*0.3);
	}

}

int16_t RSSIdiff()
{
	uint8_t ping_packet[2];
	ping_packet[0] = wpsdata.wpsdyn.nw_id;
	ping_packet[1] = FLAG_PING;
	//Serial.println("1658");
	// SEND PACKET TO COORDINATOR
	driver.setModeRx();
	delay(100);
	//Serial.println("CAD");
	driver.waitCAD(); //Serial.println("1662");
					  //Serial.println("Set To TX");
	driver.setModeTx();// Serial.println("1664");
					   //Serial.println("sending");
	driver.send(ping_packet, sizeof(ping_packet));
	//Serial.println("wait");
	driver.waitPacketSent();
	//driver.clearRxBuf();
	driver.setModeRx();
	// Now wait for a reply from the COORDINATOR

	if (!driver.waitAvailableTimeout(8000))
	{
		LASTRSSI = 0;
		return 255;
	}
	else
	{
		uint8_t tmplen = sizeof(ping_packet);
		if (driver.recv(ping_packet, &tmplen))
		{
			if (ping_packet[0] == wpsdata.wpsdyn.nw_id && ping_packet[1] == FLAG_PING && driver.headerFrom() == COORDINATOR_ADDRESS)
			{
				LASTRSSI = (uint8_t)(-1* driver.lastRssi());
				wpsdata.wpsdyn.rssi = LASTRSSI;			// SNR FROM WPS MSB
				
				if (Sequence == 0 || PREVRSSI == 255)
				{
					PREVRSSI = LASTRSSI;
				}
			}
			else
			{
				LASTRSSI = 0;
				return 255;
			}

		}
	}
	Serial.print(F("Lst RSSI:")); Serial.println(LASTRSSI);
	delay(200);
	return abs(LASTRSSI - PREVRSSI);
}

int HMC_Read(byte Axis)
{
	int Result = 0;
	Wire.beginTransmission(HMC_Address);
	Wire.write(0x02);
	// FOR SINGLE MODE 01, CONTINIOUS MODE 00
	Wire.write(0x00);
	Wire.endTransmission();
	delay(20);

	// Move modules the resiger pointer to one of the axis data registers

	Wire.beginTransmission(HMC_Address);
	Wire.write(Axis);
	Wire.endTransmission();

	// Read the data from registers (there are two 8 bit registers for each axis) 
	Wire.requestFrom(HMC_Address, 2);

	uint8_t LeftByte = Wire.read();
	uint8_t RightByte = Wire.read();
	Result = (((uint16_t)LeftByte) << 8) | RightByte;
	return Result;
}

void Init_HMC(uint8_t TempSensor, uint8_t Sample_Rate, uint8_t Freq_Rate, uint8_t Gain_Rate)
{
	// Set the module to 8x averaging and 15Hz measurement rate 



	uint8_t Register00 = 0;

	//Register00 = Temp_Sensor_Enable | Sample_x4 | Rate_3;
	Register00 = TempSensor | Sample_Rate | Freq_Rate;
	//Serial.println(Register00, HEX);



	Wire.beginTransmission(HMC_Address);
	Wire.write(0x00);
	Wire.write(Register00);

	// Set a gain of 5 


	Wire.write(0x01);
	Wire.write(Gain_Rate);
	Wire.endTransmission();
}

void SetBLEPower()
{
	switch (BLE_TX_POWER)
	{
	case 0:
		blemodule.print(F("AT+POWE0"));
		break;
	case 1:
		blemodule.print(F("AT+POWE1"));
		break;
	case 2:
		blemodule.print(F("AT+POWE2"));
		break;
	case 3:
		blemodule.print(F("AT+POWE3"));
		break;
	}
	Serial.println();
	delay(200); bledump();
	blemodule.print(F("AT+RESET"));
	delay(600);
	bledump();
}


void ReCheckBT()
{
	delay(100);
	blemodule.flush();
	blemodule.print(F("AT"));
	delay(200);
	if (blemodule.read() == 79 && blemodule.read() == 75)
	{
		Serial.println(F("BLE OK"));
		ErrorCode = ErrorCode & ~2;
	}
	else
	{
		Serial.println(F("BLE ERR"));
		ErrorCode = ErrorCode | 2;
	}
}

void BleSleep()
{
	Serial.println(F("BLE SLP"));

	delay(100);
	bledump();

	digitalWrite(BLE_POWER_PIN, LOW);
	digitalWrite(TXpin, LOW);
	pinMode(RXpin, OUTPUT);
	digitalWrite(RXpin, LOW);
	//digitalWrite(RXpin, LOW);
	blesleep = true;
}

void BleWakeUp()
{
	Serial.println(F("BLE WKE"));
	digitalWrite(BLE_POWER_PIN, HIGH);

	pinMode(RXpin, INPUT);
	digitalWrite(TXpin, HIGH);
	delay(500);

	blemodule.print(F("AT"));
	delay(100);
	bledump();
	blesleep = false;


}

void VoltageMeasurement()
{
	wpsdata.wpsdyn.voltage = analogRead(VoltageMeasurePIN) + VOLTAGE_OFFSET;
}

void VoltageCalibration()
{
	// BU KOD BİR DEFA SADECE İLK ENERJİ VERİLDİĞİNDE ÇALIŞACAK. İŞLETİM BOYUNCA BİR DAHA ÇALIŞTIRILMAYACAK
	// KALİBRASYON HASSAS GÜÇ KAYNAĞI ÜZERİNDEN 4.20 V VERİLEREK YAPILACAKTIR
	//Serial.println("Vin Calibr.");
	//Serial.println("4.2V REQ");
	uint16_t AnalogValue = 0;
	analogRead(VoltageMeasurePIN);
	delay(200);
	for (int i = 0; i < 5; i++)
	{
		AnalogValue = AnalogValue + (analogRead(VoltageMeasurePIN));
		Serial.println(AnalogValue);
		delay(200);
	}

	AnalogValue = AnalogValue / 5; // 5 DEĞERİN ORTALAMASINI AL
	VOLTAGE_OFFSET = 652 - AnalogValue;//652
									   //if (VOLTAGE_OFFSET < 0) VOLTAGE_OFFSET = 0;
	
	EEPROM.write(9, VOLTAGE_OFFSET >> 8);
	EEPROM.write(10, (uint8_t)VOLTAGE_OFFSET);
	delay(100);
}


/*
char toCapital(char buf)
{
	if (buf > 71) buf -= 32;
	if (buf < 48) buf = 48;
	return buf;
}

uint8_t CharToBin(uint8_t Char1, uint8_t Char2)
{
	uint8_t Decimal = 0;

	if (Char1 > 64)
	{
		Char1 = Char1 - 55;
	}
	else
	{
		Char1 = Char1 - 48;
	}
	if (Char2 > 64)
	{
		Char2 = Char2 - 55;
	}
	else
	{
		Char2 = Char2 - 48;
	}

	Decimal = Char1 * 16 + Char2;
	return Decimal;
}
*/