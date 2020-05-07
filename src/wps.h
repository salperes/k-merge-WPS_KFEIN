#ifndef _wps_h
#define _wps_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#endif


#pragma pack(1)
struct sWPSDataDyn
{
	// DYNAMIC VALUES (Bytes 00-15)
	uint8_t nw_id;
	uint8_t flagtype;
	uint8_t sequence3;
	uint8_t sequence2;
	uint8_t sequence1;
	uint16_t temp;
	uint16_t voltage;
	uint8_t rssi;
	uint8_t error_vehicle;
	uint8_t retry_variance;
	float vector;
};
#pragma pack()

#pragma pack(1)
struct sWPSDataSta
{
	// STATIC VALUES (Bytes 16-30)
	uint16_t versions;
	uint32_t serialble;
	uint16_t serialmsb;
	uint8_t mag_varordiff;
	uint8_t varlimits;
	uint8_t rssilimits;
	uint8_t blepw_radiopw_chkrssi;
	uint8_t vehicledc_sleepc;
	uint8_t lorachannel;
	uint8_t diffmultp_bleto_gain;
	uint8_t checksum;
};
#pragma pack()

#pragma pack(1)
struct sWPSperiodical
{
	sWPSDataDyn wpsdyn;
	sWPSDataSta wpssta;
};
#pragma pack()


#pragma pack(1)
struct sCoordData
{
	// DYNAMIC VALUES (Bytes 00-03)
	uint8_t nw_id;
	uint8_t flagtype;
	uint8_t rssi;
	uint8_t changerq;
	uint8_t changeflag;
	// CHANGE PARAMETERS (Bytes 16-30)
	uint8_t mag_varordiff;
	uint8_t varlimits;
	uint8_t rssilimits;
	uint8_t blepw_radiopw_chkrssi;
	uint8_t vehicledc_sleepc;
	uint8_t lorachannel;
	uint8_t diffmultp_bleto_gain;
	uint8_t wpsaddress;
};
#pragma pack()


char toCapital(char buf);
uint8_t CharToBin(uint8_t Char1, uint8_t Char2);

