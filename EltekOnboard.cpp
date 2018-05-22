//Bibliotheken einbinden

#include <Arduino.h>
#include <mcp_can.h> //aktuell für SeeedStudio Library, J.Fowler Probleme mit extended IDs (aus 1 wird 9). Codeanpassung (mcp_can.cpp) von horace3 von Github
#include <SPI.h>
#include <SimpleTimer.h>

//Konstanten und Variablen deklarieren

#define SPI_CS_PIN 10 //CS Pin
#define POTI 0 //A0 für Poti

word outputvoltage = 1162; //setze max Spannung auf 116,2V (offset = 0,1)
word outputcurrent = 260; //setze max Strom auf 32A (offset = 0,1)
word powerdemand = 1000; //setze max Leistung auf 100% (offset = 0,1)
float poti = 0;

unsigned long int sendId_broad = 0x2FF; //Broadcast, nicht extended

unsigned long int sendId_ind1 = 0x300; //Charger address = 1, individual control (offset 1 from base idetifier 2FF + 0*16 for adress)
unsigned long int sendId_ind2 = 0x310; //Charger address = 2, individual control (offset 1 from base idetifier 2FF + 1*16 for adress)
unsigned long int sendId_ind3 = 0x320; //Charger address = 3, individual control (offset 1 from base idetifier 2FF + 2*16 for adress)

unsigned long int sendId_con1 = 0x303; //Charger address = 1, configuration (offset 4 from base idetifier 2FF + 0*16 for adress)
unsigned long int sendId_con2 = 0x313; //Charger address = 2, configuration (offset 4 from base idetifier 2FF + 1*16 for adress)
unsigned long int sendId_con3 = 0x323; //Charger address = 3, configuration (offset 4 from base idetifier 2FF + 2*16 for adress)

unsigned long int recId_stat11 = 0x305; //Charger address = 1, status1 (offset 6 from base idetifier 2FF + 0*16 for adress)
unsigned long int recId_stat12 = 0x315; //Charger address = 2, status1 (offset 6 from base idetifier 2FF + 1*16 for adress)
unsigned long int recId_stat13 = 0x325; //Charger address = 3, status1 (offset 6 from base idetifier 2FF + 2*16 for adress)

unsigned long int recId_stat21 = 0x306; //Charger address = 1, status2 (offset 7 from base idetifier 2FF + 0*16 for adress)
unsigned long int recId_stat22 = 0x316; //Charger address = 2, status2 (offset 7 from base idetifier 2FF + 1*16 for adress)
unsigned long int recId_stat23 = 0x326; //Charger address = 3, status2 (offset 7 from base idetifier 2FF + 2*16 for adress)

unsigned long int recId_err1 = 0x307; //Charger address = 1, error (offset 8 from base idetifier 2FF + 0*16 for adress)
unsigned long int recId_err2 = 0x317; //Charger address = 2, error (offset 8 from base idetifier 2FF + 1*16 for adress)
unsigned long int recId_err3 = 0x327; //Charger address = 3, error (offset 8 from base idetifier 2FF + 2*16 for adress)

unsigned char voltamp[8] = {0x01, lowByte(powerdemand), highByte(powerdemand), lowByte(outputvoltage), highByte(outputvoltage), lowByte(outputcurrent), highByte(outputcurrent), 0x00};

unsigned char len = 0; //Länge emfpangener CAN Nachricht
unsigned char buf[8]; //Buffer für Daten aus CAN Nachricht
unsigned long int receiveId; //ID des Senders

//Objekte deklarieren

MCP_CAN CAN(SPI_CS_PIN); //CS Pin für SPI setzen

SimpleTimer timer1; //timer Objekt erzeugen

//Funktionen

/************************************************
** Function name:           canRead
** Descriptions:            read CAN message
*************************************************/
void canRead(){

  if(CAN_MSGAVAIL == CAN.checkReceive()){ //auf Nachrichten prüfen

    CAN.readMsgBuf(&len, buf); // read data, len: data length, buf: data buffer
    //CAN.readMsgBuf(&id, &len, buf); // read data, canId: ID  len: data length, buf: data buffer, für J.Fowler Library

    receiveId = CAN.getCanId(); //auskommentieren für J.Fowler Library

    if(receiveId == recId_stat11){ //Charger address = 1, Status 1
    //if(receiveId == recId_stat12){ //Charger address = 2, Status 1
    //if(receiveId == recId_stat13){ //Charger address = 3, Status 1

      Serial.println("CAN Daten vom Ladegerät empfangen!");

      Serial.print("CAN ID: ");
      Serial.print(receiveId, HEX); //ID ausgeben

      Serial.print(" / CAN Daten: ");
      for(int i = 0; i<len; i++){ //Daten ausgeben

        if( buf[i] < 0x10){ // führende Null wenn nur eine Ziffer
          Serial.print("0");
        }

        Serial.print(buf[i],HEX);
        Serial.print(" ");          // Leerzeichen

      }

      Serial.println(); //Absatz

      Serial.print("Ladespannung: ");
      float pv_voltage = (((float)buf[6]*256.0) + ((float)buf[5]))/10.0; //highByte/lowByte + offset
      Serial.print(pv_voltage);
      Serial.print(" V / Ladestrom: ");
      float pv_current = (((float)buf[4]*256.0) + ((float)buf[3]))/10.0; //highByte/lowByte + offset
      Serial.print(pv_current);
      Serial.print(" A / Stromaufnahme: ");
      float pv_mains = (((float)buf[2]*256.0) + ((float)buf[1]))/10.0; //highByte/lowByte + offset
      Serial.print(pv_mains);
      Serial.println(" A"); //Absatz

      switch (buf[0]) { //Statusbyte auslesen

        case B00000001: Serial.println("Status: Ruhezustand");break;
        case B00000010: Serial.println("Status: Lädt");break;
        case B00000011: Serial.println("Status: Behebarer Fehler");break;
        case B00000100: Serial.println("Status: Nicht behebarer Fehler");break;

      }

    }else if(receiveId == recId_stat21){ //Charger address = 1, Status 2
    //if(receiveId == recId_stat22){ //Charger address = 2, Status 2
    //if(receiveId == recId_stat23){ //Charger address = 3, Status 2

      Serial.println("CAN Daten vom Ladegerät empfangen!");

      Serial.print("CAN ID: ");
      Serial.print(receiveId, HEX); //ID ausgeben

      Serial.print(" / CAN Daten: ");
      for(int i = 0; i<len; i++){ //Daten ausgeben

        if( buf[i] < 0x10){ // führende Null wenn nur eine Ziffer
          Serial.print("0");
        }

        Serial.print(buf[i],HEX);
        Serial.print(" ");          // Leerzeichen

      }

      Serial.println(); //Absatz

      Serial.print("Eingangspannung: ");
      float pv_voltage_in = (((float)buf[3]*256.0) + ((float)buf[2])); //highByte/lowByte + offset
      Serial.print(pv_voltage_in);
      Serial.print(" V / Max. Leistung: ");
      float pv_powermax = (((float)buf[5]*256.0) + ((float)buf[4]))/10.0; //highByte/lowByte + offset
      Serial.print(pv_powermax);
      Serial.print(" W / Verfügbare Leistung: ");
      float pv_powerav = ((float)buf[6]/2); //highByte/lowByte + offset
      Serial.print(pv_powerav);
      Serial.print(" % / Temperatur 1: ");
      float pv_temp1 = (float)buf[0]; //highByte/lowByte + offset
      Serial.print(pv_temp1);
      Serial.print(" °C / Temperatur 1: ");
      float pv_temp2 = (float)buf[1]; //highByte/lowByte + offset
      Serial.print(pv_temp2);
      Serial.println(" °C"); //Absatz

    }else if(receiveId == recId_err1){ //Charger address = 1, Error
    //if(receiveId == recId_err2){ //Charger address = 2, Error
    //if(receiveId == recId_err3){ //Charger address = 3, Error

      Serial.println("CAN Daten vom Ladegerät empfangen!");

      Serial.print("CAN ID: ");
      Serial.print(receiveId, HEX); //ID ausgeben

      Serial.print(" / CAN Daten: ");
      for(int i = 0; i<len; i++){ //Daten ausgeben

        if( buf[i] < 0x10){ // führende Null wenn nur eine Ziffer
          Serial.print("0");
        }

        Serial.print(buf[i],HEX);
        Serial.print(" ");          // Leerzeichen

      }

      Serial.println(); //Absatz

      switch (buf[0]) { //Statusbyte auslesen

        case B00000001: Serial.print("Fehler: DC Überspannung;");
        case B00000010: Serial.print("Fehler: Übertemperatur;");
        case B00000100: Serial.print("Fehler: Eingangsspannung unzulässig;");
        case B00001000: Serial.print("Fehler: AC Überspannung;");
        case B00010000: Serial.print("Fehler: AC Unterspannung;");
        case B00100000: Serial.print("Fehler: Übertemperatur;");
        case B01000000: Serial.print("Fehler: Untertemperatur;");
        case B10000000: Serial.print("Fehler: Stromlimitierung;");
        Serial.println(); //Absatz
      }

      switch(buf[2]){

        case B00000001: Serial.print("Fehler: DC Unterspannung;");
        case B00000010: Serial.print("Fehler: Kommunikation");
        Serial.println(); //Absatz

      }

    }

  }

}

/************************************************
** Function name:           canWrite
** Descriptions:            write CAN message
*************************************************/
String canWrite(unsigned char data[8], unsigned long int id){

  byte sndStat = CAN.sendMsgBuf(id, 0, 8, data); //Nachricht senden (ID, extended Frame, Datenlänge, Daten)

  if(sndStat == CAN_OK) //Statusbyte für Übertragung
    return "CAN Nachricht erfolgreich gesendet";
  else
    return "Fehler bei Nachrichtenübertragung";

}

/************************************************
** Function name:           myTimer1
** Descriptions:            Function of timer1
*************************************************/
void myTimer1() { //zyklisch vom Timer aufgerufene Funktion

  poti = analogRead(POTI); //Wert vom Poti einlesen
  powerdemand = 1000.0*(poti/800.0); //keine 1024 erreicht, daher 800

  if(powerdemand > 1000.0){

    powerdemand = 1000.0; //begrenze auf 320 falls Ergebnis größer ist

  }

  Serial.print("Eingestellter Ladestrom: ");
  Serial.print((float)outputcurrent/10.0); //Current setpoint ausgeben
  Serial.println(" A");
  unsigned char voltamp[8] = {0x01, lowByte(powerdemand), highByte(powerdemand), lowByte(outputvoltage), highByte(outputvoltage), lowByte(outputcurrent), highByte(outputcurrent), 0x00}; //Nachricht neu generieren

  Serial.println(canWrite(voltamp, sendId_broad)); //Nachricht senden und Ergebnis ausgeben
  canRead(); //Lesefunktion aufrufen
  Serial.println(); //Absatz

}

/************************************************
** Function name:           setup
** Descriptions:            Arduino setup
*************************************************/
void setup() {

  Serial.begin(115200); //Serielle Schnittstelle starten

  while(CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)){ //CAN Bus initialisieren
 //  while(CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)){ //für Bibliothek von J.Fowler

    Serial.println("CAN Initialisierung fehlgeschlagen, Neustart");
    delay(200);

  }

  Serial.println("CAN Initialisierung erfolgreich");

  //CAN.setMode(MCP_NORMAL); //Bibliothek von J.Fowler

  timer1.setInterval(900, myTimer1); //Zeit und Funktion des Timers definieren

}

/************************************************
** Function name:           loop
** Descriptions:            Arduino loop
*************************************************/
void loop() {

  timer1.run(); //Timer starten

}
