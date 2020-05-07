#include "wps.h"
#include <eeprom.h>

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

uint16_t VoltageCalibration(uint16_t VoltageMeasurePIN)
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
	uint16_t VOLTAGE_OFFSET = 652 - AnalogValue;//652
									   //if (VOLTAGE_OFFSET < 0) VOLTAGE_OFFSET = 0;
	
	EEPROM.write(9, VOLTAGE_OFFSET >> 8);
	EEPROM.write(10, (uint8_t)VOLTAGE_OFFSET);
	delay(100);
	return VOLTAGE_OFFSET;
}