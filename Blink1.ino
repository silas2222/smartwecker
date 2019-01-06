#include <Wire.h>
#include <DS3231.h>
#include <EEPROM.h>
#include <RCSwitch.h>
#include <IRremote.h>
#include <LiquidCrystal.h>

//PINBELEGUNG
//SDA = A4
//SCL = A5
#define D7          2
#define D6          3
#define D5          4
#define D4          5
#define IRRECPIN    6 //IRREC (FRONT = SENSORSEITE): G-5V-D
#define BUZZERPIN   7 //BUZZER: + an PIN; - an GND
#define SBUTTONPIN  8
#define OBUTTONPIN  9
#define SEND433PIN	10
#define EN          11
#define RS          12
#define LED			LED_BUILTIN


//EEPROM Speicher Adressen (0-1024)
#define ADR_WECKSTART	0
#define ADR_WECKENDE	4


class CUhr
{
private: //Variablen
	DS3231 m_clock;
	RTCDateTime m_dt;
	short m_weckstart;
	short m_weckende;

	unsigned long m_timer;

public: //Methoden
	CUhr()
	{
		m_timer = 0;

		//Uhrzeitmodul starten
		m_clock.begin();
		//m_clock.setDateTime(__DATE__, __TIME__);

		//Speicher auslesen
		EEPROM.get(ADR_WECKSTART, m_weckstart);
		EEPROM.get(ADR_WECKENDE, m_weckende);

		
		
	}

	void Update()
	{
		//deltaTime gibt die Zeit(in ms) an, welche seit dem letzen Loop vergangen sind
		static unsigned long lastMillis = 0;
		unsigned long deltaTime = millis() - lastMillis;
		lastMillis = millis();

		//Uhrzeit aktualisieren
		m_dt = m_clock.getDateTime();

		//Timer
		if (m_timer > 0)
		{
			if (m_timer < deltaTime)
				m_timer = 0;
			else
				m_timer -= deltaTime;
		}
			
	}

	bool isWaketime()
	{ //Vergleicht Uhrzeit mit Weckzeit und gibt boolschen Wert zurück
		if (m_dt.hour * 60 + m_dt.minute >= m_weckstart && m_dt.hour * 60 + m_dt.minute <= m_weckende)
			return true;
		else
			return false;
	}

	bool isWaketimeOver()
	{ //Wenn Weckende erreicht ist true
		if (m_weckende == m_dt.hour * 60 + m_dt.minute)
			return true;
		else
			return false;
	}

	RTCDateTime getDateTime()
	{
		return m_dt;
	}

	void startTimer(unsigned long sekunden)
	{
		m_timer = sekunden * 1000; //Wird in millisekunden gespeichert
	}

	bool isTimerZero()
	{
		return (m_timer == 0) ? true : false;
	}

	void setWeckzeit(short weckstart, short weckende)
	{
		if (m_weckstart != weckstart || m_weckende != weckende)
		{ //Weckzeit hat sich geändert
			m_weckstart = weckstart;
			m_weckende = weckende;
			EEPROM.put(ADR_WECKSTART, weckstart);
			EEPROM.put(ADR_WECKENDE, weckende);
		}
	}
	void setWeckzeit(short minuten)
	{
		m_weckstart += minuten;
		m_weckende += minuten;

		if (m_weckstart >= 1440)	//1440 -> 24:00 -> 00:00
			m_weckstart -= 1440;
		if (m_weckstart < 0)
			m_weckstart = 1440 + m_weckstart;

		if (m_weckende >= 1440)
			m_weckende -= 1440;
		if (m_weckende < 0)
			m_weckende = 1440 + m_weckende;

		EEPROM.put(ADR_WECKSTART, m_weckstart);
		EEPROM.put(ADR_WECKENDE, m_weckende);
	}

	void getWeckzeit(short *pweckstart, short *pweckende)
	{
		if (pweckstart == 0 || pweckende == 0)
			return;	//Nullpointer -> return

		*pweckstart = m_weckstart;
		*pweckende = m_weckende;
	}

};

class CGyroskop
{
private: //Variablen
	double m_acX, m_acY, m_acZ, m_tmpC, m_gyX, m_gyY, m_gyZ;
public: //Methoden
	CGyroskop()
	{
		Wire.begin();
		Wire.beginTransmission(0x69);
		Wire.write(0x6B);  // PWR_MGMT_1 register
		Wire.write(0);     // set to zero (wakes up the MPU-6050)
		Wire.endTransmission(true);
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
		if (m_gyX < -0.02 || m_gyX > 0.02 || m_gyY < -0.02 || m_gyY > 0.02 || m_gyZ < -0.02 || m_gyZ > 0.02)
			return true;
		else
			return false;
	}
};

class COutput
{
private: //Variablen
	CUhr *m_pUhr;
	LiquidCrystal *m_pDisplay;
	RCSwitch *m_pRCS;
public: //Methoden
	COutput(CUhr *pUhr)
	{
		m_pUhr = pUhr;

		//Speicher reservieren, Adresse in Zeiger speichern
		m_pDisplay = new LiquidCrystal(RS, EN, D4, D5, D6, D7);
		//Display aktivieren
		m_pDisplay->begin(16, 2);
		m_pDisplay->clear();

		m_pRCS = new RCSwitch();
		m_pRCS->enableTransmit(SEND433PIN);
	}

	void printLCD()
	{
		displayTime(m_pUhr->getDateTime(), 0, 0);

		short weckstart;
		short weckende;
		m_pUhr->getWeckzeit(&weckstart, &weckende); // & = Adresse der variable

		displayText("", 0, 1);
		displayTime(weckstart);
		displayText(" - ");
		displayTime(weckende);
	}

	void switchSocket(bool bOn)
	{
		static bool currentState = false;

		if (currentState == bOn)	//Bereits geschaltet
			return;
		currentState = bOn;

		//Steckdose umschalten
		m_pRCS->setProtocol(4);
		if (bOn)
			m_pRCS->send(14383122, 24); //An
		else
			m_pRCS->send(14226994, 24); //Aus
	}

	void displayText(String str, uint8_t cursorx = 100, uint8_t cursory = 100)
	{
		if (cursorx != 100 && cursory != 100)
			m_pDisplay->setCursor(cursorx, cursory);

		m_pDisplay->print(str);
	}

	
	void displayTime(int hour, int minute, uint8_t cursorx = 100, uint8_t cursory = 100, int second = -1)
	{
		if (cursorx != 100 && cursory != 100)
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
	}

	void displayTime(short minuteFormat, uint8_t cursorx = 100, uint8_t cursory = 100)
	{
		//Zeit ist in Minuten(von 0 Uhr) angegeben und muss in Stunden und Minuten zerteilt werden
		int hour = minuteFormat / 60;
		int minute = minuteFormat % 60;

		//An überladene Funktion weitergeben
		displayTime(hour, minute, cursorx, cursory);
	}

	void displayTime(RTCDateTime dt, uint8_t cursorx = 100, uint8_t cursory = 100)
	{
		//Umwandeln und an überladene Funktion weitergeben
		displayTime(dt.hour, dt.minute, cursorx, cursory, dt.second);
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
	CUhr *m_pUhr;

	IRrecv *m_pIrrec;
public: //Methoden
	CInput(CUhr *pUhr)
	{
		m_pUhr = pUhr;
		//Speicher reservieren, Adresse in Zeiger speichern
		m_pIrrec = new IRrecv(IRRECPIN);

		//Infrarot Receiver starten
		m_pIrrec->enableIRIn();
	}

	void Update()
	{
		int ir = getIR();
		Serial.println(ir);
		if (ir == IRMINUS)
			m_pUhr->setWeckzeit(-15);
		if (ir == IRPLUS)
			m_pUhr->setWeckzeit(+15);
	}

	int getIR()
	{ //Fragt Infrarotsensor ab und gibt 
		decode_results results;
		if (m_pIrrec->decode(&results))
		{
			int ret = -1;
			//Signal erkannt
			switch (results.value)
			{
			case 0xFFA25D:					Serial.println("CH-");	break;
			case 0xFF629D:					Serial.println("CH");	break;
			case 0xFFE21D:					Serial.println("CH+");	break;

			case 0xFF22DD:					Serial.println("VOL-"); break;
			case 0xFF02FD:					Serial.println("VOL+"); break;
			case 0xFFC23D: ret = IRPAUSE;	Serial.println("PAUSE");break;

			case 0xFFE01F: ret = IRMINUS;	Serial.println("-");    break;
			case 0xFFA857: ret = IRPLUS;	Serial.println("+");    break;
			case 0xFF906F: ret = IREQ;		Serial.println("EQ");   break;

			case 0xFF6897: ret = IR0;		Serial.println("0");    break;
			case 0xFF9867:					Serial.println("100+"); break;
			case 0xFFB04F:					Serial.println("200+"); break;

			case 0xFF30CF: ret = IR1;		Serial.println("1");    break;
			case 0xFF18E7: ret = IR2;		Serial.println("2");    break;
			case 0xFF7A85: ret = IR3;		Serial.println("3");    break;

			case 0xFF10EF: ret = IR4;		Serial.println("4");    break;
			case 0xFF38C7: ret = IR5;		Serial.println("5");    break;
			case 0xFF5AA5: ret = IR6;		Serial.println("6");    break;

			case 0xFF42BD: ret = IR7;		Serial.println("7");    break;
			case 0xFF4AB5: ret = IR8;		Serial.println("8");    break;
			case 0xFF52AD: ret = IR9;		Serial.println("9");    break;

			case 0xFFFFFFFF: Serial.println(" REPEAT"); break;
			}// End Case

			m_pIrrec->resume();

			return ret;
		}
		return -1;
	}

	bool isSnoozeButtonPressed()
	{
		return (digitalRead(SBUTTONPIN) == LOW) ? true : false; //Kurzform einer if-else Bedingung
	}

	bool isOffButtonPressed()
	{
		return (digitalRead(OBUTTONPIN) == LOW) ? true : false;
	}
};

class CAlarm
{
private: //Variablen
	CUhr *m_pUhr;
	CInput *m_pInput;
	COutput *m_pOutput;
	CGyroskop *m_pGyro;
	
	bool m_alarm;
	bool m_snooze;
	bool m_wasalarmed;

public: //Methoden
	CAlarm(CUhr *pUhr, CInput *pInput, COutput *pOutput, CGyroskop *pGyro)
	{
		m_pUhr = pUhr;
		m_pInput = pInput;
		m_pOutput = pOutput;
		m_pGyro = pGyro;


		m_alarm = false;
		m_snooze = false;
		m_wasalarmed = false;
	}

	void Update()
	{
		if (m_pUhr->isWaketime())
		{ //Weckzeit erreicht
			m_pOutput->switchSocket(true);	//Funksteckdose(Lampe) einschalten

			if (!m_wasalarmed && m_pGyro->isMoving() || !m_wasalarmed && m_pUhr->isWaketimeOver())
			{ //Bewegung erkannt oder Weckzeit ist vorbei
				m_alarm = true;
				m_wasalarmed = true;
			}
		}
		
		//Wenn Weckzeit vorbei, die Lampe wieder ausschalten
		if (m_pUhr->isWaketimeOver())
			m_pOutput->switchSocket(false);	//Funksteckdose(Lampe) ausschalten

		//Wenn der Alarm eingeschaltet ist (und nicht im Snooze Modus), soll ein Tonsignal ausgegeben werden
		if (m_alarm && !m_snooze)
			m_pOutput->activebeep(10, 10, 10);

		//Snooze
		if (m_pInput->isSnoozeButtonPressed()) //Snoozeknopf gedrückt
		{
			m_snooze = true;

			if (m_pUhr->isTimerZero())
				m_pUhr->startTimer(300); //5 Minütiger Timer wird gestartet
		}
		if (m_snooze && m_pUhr->isTimerZero()) //Testet ob der Timer 0 ist
			m_snooze = false;


		//Wenn Offknopf gedrückt, Alarm ausschalten
		if (m_pInput->isOffButtonPressed())
		{
			m_alarm = false;
		}

		//m_wasalarmed muss außerhalb der Weckzeit resetet werden, damit der Wecker länger als einen Tag funktioniert
		if (!m_pUhr->isWaketime() && m_wasalarmed)
		{
			m_wasalarmed = false;
		}
	}
};



void setup()
{
	pinMode(BUZZERPIN, OUTPUT);
	pinMode(SBUTTONPIN, INPUT_PULLUP);
	pinMode(OBUTTONPIN, INPUT_PULLUP);
	pinMode(SEND433PIN, OUTPUT);
	Serial.begin(9600);
}

void loop()
{
	//Statische Klasseninstanzen, bleiben bei loop vorhanden
	static CUhr Uhr;
	static CGyroskop Gyroskop;
	static COutput Output(&Uhr);
	static CInput Input(&Uhr);
	static CAlarm Alarm(&Uhr, &Input, &Output, &Gyroskop);

	Input.Update();		//Verarbeitet Infrarotsensor
	Uhr.Update();		//Aktualisiert Uhrzeit und Timer
	Gyroskop.Update();	//Aktualisiert Gyroskop

	Alarm.Update();		//Hier passiert die Magie

	Output.printLCD();	//Gibt Uhrzeit und Weckzeit auf dem Display aus
}
