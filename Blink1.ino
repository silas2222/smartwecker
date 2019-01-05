#include <Wire.h>
#include <DS3231.h>
#include <EEPROM.h>
#include <IRremote.h>
#include <LiquidCrystal.h>
/*
* BUZZER: + an PIN; - an GND
*
* IRREC (FRONT = SENSORSEITE): G-5V-D
*
* SDA = A4
* SCL = A5
*/

//PINS
#define SBUTTONPIN  8
#define OBUTTONPIN  9
#define IRRECPIN    6
#define BUZZERPIN   7
#define RS          12
#define EN          11
#define D4          5
#define D5          4
#define D6          3
#define D7          2
#define LED LED_BUILTIN


//EEPROM Speicher Adressen (0-1024)
#define ADR_WECKSTART	0
#define ADR_WECKENDE	4


/*
class CUhr;
class CGyroskop;
class CAlarm;
class COutput;
*/



class CUhr
{
private: //Variablen
	DS3231 m_clock;
	RTCDateTime m_dt;
	int m_weckstart;
	int m_weckende;
	bool m_bAlarmState;

public: //Methoden
	CUhr()
	{
		//Uhrzeitmodul starten
		m_clock.begin();
		//clock.setDateTime(__DATE__, __TIME__);

		//Speicher auslesen
		EEPROM.get(ADR_WECKSTART, m_weckstart);
		EEPROM.get(ADR_WECKENDE, m_weckende);
	}

	void Update()
	{
		//Uhrzeit aktualisieren
		m_dt = m_clock.getDateTime();

		//Weckzeitraum erreicht?
	}

	bool isWaketime()
	{ //Vergleicht Uhrzeit mit Weckzeit und gibt boolschen Wert zurück
		if (m_dt.hour * 60 + m_dt.minute >= m_weckstart && m_dt.hour * 60 + m_dt.minute <= m_weckende)
			return true;
		else
			return false;
	}

	void setDateTime()
	{

	}

	void setWeckzeit(int weckstart, int weckende)
	{
		if (m_weckstart != weckstart || m_weckende != weckende)
		{ //Weckzeit hat sich geändert
			m_weckstart = weckstart;
			m_weckende = weckende;
			EEPROM.put(ADR_WECKSTART, weckstart);
			EEPROM.put(ADR_WECKENDE, weckende);
		}
	}

};

class CGyroskop
{
private: //Variablen
	double m_acX, m_acY, m_acZ, m_tmpC, m_gyX, m_gyY, m_gyZ;
public: //Methoden
	CGyroskop()
	{
		
	}

	void Update()
	{
		const int MPU_addr = 0x69;  // I2C Adresse für MPU-6050
		const double GySens = 32767.0;
		const double AcSens = 16383.0;

		int16_t rAcX, rAcY, rAcZ, rTmp, rGyX, rGyY, rGyZ;

		Wire.beginTransmission(MPU_addr);
		Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
		Wire.endTransmission(false);
		Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
		rAcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
		rAcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		rAcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		rTmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		rGyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		rGyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		rGyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
		Wire.endTransmission(true);

		m_acX = rAcX / AcSens;
		m_acY = rAcY / AcSens;
		m_acZ = rAcZ / AcSens;
		m_tmpC = rTmp / 340.00 + 36.53; //equation for temperature in degrees C from datasheet
		m_gyX = rGyX / GySens;
		m_gyY = rGyY / GySens;
		m_gyZ = rGyZ / GySens;
	}

	bool isMoving()
	{ //Bewegung erkannt
		if (m_gyX < -0.01 || m_gyX > 0.01 || m_gyY < -0.01 || m_gyY > 0.01 || m_gyZ < -0.01 || m_gyZ > 0.01)
			return true;
		else
			return false;
	}
};

class COutput
{
private: //Variablen
	LiquidCrystal *m_pDisplay;
public: //Methoden
	COutput()
	{
		//Speicher reservieren, Adresse in Zeiger speichern
		m_pDisplay = new LiquidCrystal(RS, EN, D4, D5, D6, D7);

		//Display aktivieren
		m_pDisplay->begin(16, 2);
	}

	void displayText(String str, uint8_t cursorx = -1, uint8_t cursory = -1)
	{
		if (cursorx >= 0 && cursory >= 0)
			m_pDisplay->setCursor(cursorx, cursory);

		m_pDisplay->print(str);
	}

	void displayTime(int hour, int minute, uint8_t cursorx = -1, uint8_t cursory = -1, int second = -1, int millisecond = -1)
	{
		if (cursorx >= 0 && cursory >= 0)
			m_pDisplay->setCursor(cursorx, cursory);

		//Zeit auf Display ausgeben, ggfs. 0er hinzufügen
		if (hour < 10)
			m_pDisplay->print("0");
		m_pDisplay->print(hour);

		m_pDisplay->print(":");

		if (minute < 10)
			m_pDisplay->print("0");
		m_pDisplay->print(minute);

		if (second >= 0)
		{
			m_pDisplay->print(":");

			if (second < 10)
				m_pDisplay->print("0");
			m_pDisplay->print(second);
		}

		if (millisecond >= 0)
		{
			m_pDisplay->print(":");

			if (millisecond < 10)
				m_pDisplay->print("0");
			m_pDisplay->print(millisecond);
		}
	}

	void displayTime(int minuteFormat, uint8_t cursorx = -1, uint8_t cursory = -1)
	{
		//Zeit ist in Minuten(von 0 Uhr) angegeben und muss in Stunden und Minuten zerteilt werden
		int hour = minuteFormat / 60;
		int minute = minuteFormat % 60;

		//An überladene Funktion weitergeben
		displayTime(hour, minute, cursorx, cursory);
	}


	void activebeep(int hightime, int lowtime = 1, int repeats = 1)
	{ //Erzeugt Ton mit aktivem Buzzer
		for (int i = 0; i<repeats; i++)
		{
			digitalWrite(BUZZERPIN, HIGH);
			delay(hightime);
			digitalWrite(BUZZERPIN, LOW);
			delay(lowtime);
		}
	}
};

enum {
	IR0 = 0, IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8, IR9,
	IRMINUS, IRPLUS, IREQ, IRPAUSE
};

class CInput
{
private: //Variablen
	IRrecv *m_pIrrec;
public: //Methoden
	CInput()
	{
		//Speicher reservieren, Adresse in Zeiger speichern
		m_pIrrec = new IRrecv(IRRECPIN);

		//Infrarot Receiver starten
		m_pIrrec->enableIRIn();
	}

	void Update()
	{
		//Wenn Snoozebutton gedrückt wird, für 5 Minuten alarm ausschalten
		static unsigned long tsnooze = 0;
		if (digitalRead(SBUTTONPIN) == LOW)
		{
			tsnooze = millis();
			snooze = true;
			alarm = false;
		}
		if (snooze && tsnooze + 300000 <= millis())
		{
			snooze = false;
			alarm = true;
		}

		//Funktionen
	}

	byte getIR()
	{ //Fragt Infrarotsensor ab und gibt 
		decode_results results;
		if (m_pIrrec->decode(&results))
		{
			//Signal erkannt
			switch (results.value)
			{
			case 0xFFA25D: Serial.println("CH-");		break;
			case 0xFF629D: Serial.println("CH");		break;
			case 0xFFE21D: Serial.println("CH+");		break;

			case 0xFF22DD: Serial.println("VOL-");		break;
			case 0xFF02FD: Serial.println("VOL+");		break;
			case 0xFFC23D: return IRPAUSE;

			case 0xFFE01F: return IRMINUS;
			case 0xFFA857: return IRPLUS;
			case 0xFF906F: return IREQ;

			case 0xFF6897: return 0;
			case 0xFF9867: break; //+100
			case 0xFFB04F: break; //+200

			case 0xFF30CF: return 1;
			case 0xFF18E7: return 2;
			case 0xFF7A85: return 3;

			case 0xFF10EF: return 4;
			case 0xFF38C7: return 5;
			case 0xFF5AA5: return 6;

			case 0xFF42BD: return 7;
			case 0xFF4AB5: return 8;
			case 0xFF52AD: return 9;

			case 0xFFFFFFFF: Serial.println(" REPEAT"); break;

			default:
				Serial.println("WTF WAS GEHT HIER AB");
			}// End Case

			m_pIrrec->resume();
		}
	}
};


class CAlarm
{
private: //Variablen
	CUhr *m_pUhr;
	COutput *m_pOutput;
	CGyroskop *m_pGyro;
	
	bool m_alarm;
	bool m_snooze;
	bool m_wasalarmed;

public: //Methoden
	CAlarm(CUhr *pUhr, COutput *pOutput, CGyroskop *pGyro)
	{
		m_pUhr = pUhr;
		m_pOutput = pOutput;
		m_pGyro = pGyro;


		m_alarm = false;
		m_snooze = false;
		m_wasalarmed = false;
	}

	void Update()
	{
		if (m_pGyro->isMoving() && !m_wasalarmed)
		{
			m_alarm = true;
			m_wasalarmed = true;
		}

		if (m_alarm)
			m_pOutput->activebeep(10, 10, 10);
	}

	void Snooze()
	{

	}
};



//Globale Variablen





//Klasseninstanzen
CUhr Uhr;
CGyroskop Gyroskop;
COutput Output;

void setup()
{
	pinMode(BUZZERPIN, OUTPUT);
	pinMode(SBUTTONPIN, INPUT_PULLUP);
	pinMode(OBUTTONPIN, INPUT_PULLUP);
	

	Serial.begin(9600);
}

void loop()
{
	Uhr.Update();
	

	//Wenn Weckzeitraum erreicht
	if (Uhr.isWaketime())
	{
		//Bewegung erkannt ODER Weckzeit ist zu Ende(und noch nicht geweckt)
		if (Gyroskop.isMoving())
		{
			//Alarm einschalten
			alarm = true;
			wasalarmed = true;
		}
	}
	//Wenn der Alarm an ist soll der Buzzer rumpiepen
	if (alarm)
	{
		Output.activebeep(50, 10, 10);
		delay(1000);
	}
}


