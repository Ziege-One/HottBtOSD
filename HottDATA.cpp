/* HoTT USB/BT Telemetry , 2015 by CopterFail */

#include <Arduino.h>

//#include "defines.h"
//#include "boards.h"
//#include "globals.h"
//#include "HottBtUsb.h"

#include <SoftwareSerial.h>

#ifdef PROTOCOL_HOTT_DATA


#ifdef HOTT_DEBUG
  SoftwareSerial DEBUG_SERIAL(2, 3); // RX, TX
#endif
/* Configuration */
#define HOTT_WAIT_TIME 200		// time [ms] to wait for response or next request

/* local defines */
#define HOTT_REQUEST_GPS	1
#define HOTT_WAIT_GPS		2
#define HOTT_REQUEST_EAM	3
#define HOTT_WAIT_EAM		4
#define HOTT_REQUEST_GAM	5
#define HOTT_WAIT_GAM		6
#define HOTT_REQUEST_RX		7
#define HOTT_WAIT_RX		8
#define HOTT_IDLE		10

/* local functions */
static void vHottFormatGpsString(char *buffer, uint16_t high, uint16_t low);
static float fHottGetGpsDegree(uint16_t high, uint16_t low);
static uint32_t ui32HottGetGpsDegree(uint16_t high, uint16_t low);
static void vHottSendGpsRequest(void);
static bool bHottReadGpsResponse(void);
static void vHottSendEamRequest(void);
static bool bHottReadEamResponse(void);
static void vHottSendGamRequest(void);
static bool bHottReadGamResponse(void);
static void vHottSendReceiverRequest(void);
static bool bHottReadReceiverResponse(void);
static void vHottClean(void);
static void vUpdateGlobalData(void);
static bool bIsBtConnected(void);

/* local data */
static uint8_t crlf_count = 0;
static uint32_t lastpacketreceived = 0;
static uint16_t ui16GpsOk = 0;
static uint16_t ui16GpsFail = 0;
static uint16_t ui16EamOk = 0;
static uint16_t ui16EamFail = 0;
static uint16_t ui16GamOk = 0;
static uint16_t ui16GamFail = 0;
static uint16_t ui16RxOk = 0;
static uint16_t ui16RxFail = 0;

void uploadFont();

/* local structures */
struct
	__attribute__((__packed__))
	{
        uint8_t DummyH[5];
        uint8_t Rec_RX_S_STR;
        uint8_t Rec_RVOLT;
        uint8_t Rec_TEMP;
        uint8_t Rec_RX_dBm;
        uint8_t Rec_RX_S_QUA;
        uint8_t Rec_RLOW;
        uint16_t Rec_LOST;
        uint8_t Rec_TX_dBm;
        uint8_t Rec_Null_Byte;
        uint8_t sensorTextID;  
        //uint8_t DummyH[16];
        uint8_t flightDirection;  //#07 119 = Flightdir./dir. 1 = 2°; 0°(North), 90°(East), 180°(South), 270°(West) */
        uint16_t GPSSpeed;        //#08 8 = /GPS speed low byte 8km/h */
                                  //#09 MSB 
        uint8_t LatitudeNS;       //#10 48°39’988 #10 north = 0, south = 1*/
        uint16_t LatitudeMin;     //#11 231 0xE7 = 0x12E7 = 4839 */
                                  //#12 MSB 
        uint16_t LatitudeSec;     //#13 171 220 = 0xDC = 0x03DC =0988 */
                                  //#14 MSB 
        uint8_t longitudeEW;      //#15 000  = E= 9° 25’9360 east = 0, west = 1*/
        uint16_t longitudeMin;    //#16 150 157 = 0x9D = 0x039D = 0925 */
                                  //#17 MSB 
        uint16_t longitudeSec;    //#18 056 144 = 0x90 0x2490 = 9360*/
                                  //#19 MSB 
        uint16_t distance;        //#20 027 123 = /distance low byte 6 = 6 m */
                                  //#21 MSB 
        uint16_t altitude;        //#22 243 244 = /Altitude low byte 500 = 0m */
                                  //#23 MSB 
        uint16_t climbrate1s;     //#24 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                                  //#25 MSB 
        int8_t climbrate3s;       //#26 climb rate in m/3sec. Value of 120 = 0m/3sec
        uint8_t GPSNumSat;        //#27 GPS.Satelites (number of satelites) (1 byte) */
        uint8_t GPSFixChar;       //#28 GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
        uint8_t HomeDirection;    //#29 HomeDirection (direction from starting point to Model position) (1 byte) */
        uint8_t angleXdirection;  //#30 angle x-direction (1 byte) */
        uint8_t angleYdirection;  //#31 angle y-direction (1 byte) */
        uint8_t angleZdirection;  //#32 angle z-direction (1 byte) */
        int8_t gps_time_h;        //#33 UTC time hours
        int8_t gps_time_m;        //#34 UTC time minutes
        int8_t gps_time_s;        //#35 UTC time seconds
        int8_t gps_time_sss;      //#36 UTC time milliseconds
        int16_t msl_altitude;     //#37 mean sea level altitude
                                  //#38 MSB
        uint8_t vibration;        //#39 vibration (1 bytes) */
        uint8_t Ascii4;           //#40 00 ASCII Free Character [4] */
        uint8_t Ascii5;           //#41 00 ASCII Free Character [5] */
        uint8_t GPS_fix;          //#42 00 ASCII Free Character [6], we use it for GPS FIX */
        uint8_t version;          //#43 00 version number */
        uint8_t endByte;          //#44 0x7D Ende byte */
        uint8_t parity;           //#45 Parity Byte */
	} GPSData;

struct
	__attribute__((__packed__))
	{
			// Telegram Header:
			uint8_t ui8Start; // 0x00
			uint16_t ui16Dummy;
			uint8_t ui8Header1; // 0x0b
			uint8_t ui8Header2; // 0x00
			uint8_t ui8Header3; // 0x04
			uint8_t ui8Header4; // 0x01
			// Receiver Data:
			uint16_t ui16DummyH;
			uint16_t ui16Temp;
			uint16_t ui16LossPack;
			uint8_t ui8Strength;
			uint8_t ui8Volt;
			uint8_t ui8Dbm;
			uint8_t ui8Quality;
			uint8_t ui8LowVolt;
			uint16_t ui16DummyD;
	} ReceiverData;

struct
      __attribute__((__packed__))
      {
      uint8_t DummyH[5];
      uint8_t Rec_RX_S_STR;
      uint8_t Rec_RVOLT;
      uint8_t Rec_TEMP;
      uint8_t Rec_RX_dBm;
      uint8_t Rec_RX_S_QUA;
      uint8_t Rec_RLOW;
      uint16_t Rec_LOST;
      uint8_t Rec_TX_dBm;
      uint8_t Rec_Null_Byte;
      uint8_t sensorTextID;  
      //uint8_t DummyH[16];
      byte cell_L[7];           //#7 Volt Cell_L 1 (in 2 mV increments, 210 == 4.20 V)
                                //#8 Volt Cell_L 2 (in 2 mV increments, 210 == 4.20 V)
                                //#9 Volt Cell_L 3 (in 2 mV increments, 210 == 4.20 V)
                                //#10 Volt Cell_L 4 (in 2 mV increments, 210 == 4.20 V)
                                //#11 Volt Cell_L 5 (in 2 mV increments, 210 == 4.20 V)
                                //#12 Volt Cell_L 6 (in 2 mV increments, 210 == 4.20 V)
                                //#13 Volt Cell_L 7 (in 2 mV increments, 210 == 4.20 V)
      byte cell_H[7];           //#14 Volt Cell_L 1 (in 2 mV increments, 210 == 4.20 V)
                                //#15 Volt Cell_H 2 (in 2 mV increments, 210 == 4.20 V)
                                //#16 Volt Cell_H 3 (in 2 mV increments, 210 == 4.20 V)
                                //#17 Volt Cell_H 4 (in 2 mV increments, 210 == 4.20 V)
                                //#18 Volt Cell_H 5 (in 2 mV increments, 210 == 4.20 V)
                                //#19 Volt Cell_H 6 (in 2 mV increments, 210 == 4.20 V)
                                //#20 Volt Cell_H 7 (in 2 mV increments, 210 == 4.20 V)   
      uint16_t  Battery1;       //#21 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                                //#22 MSB 
      uint16_t  Battery2;       //#23 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                                //#24 MSB
      byte temperature1;        //#25 Temperature 1. Offset of 20. a value of 20 = 0°C
      byte temperature2;        //#26 Temperature 2. Offset of 20. a value of 20 = 0°C
      uint16_t altitude;        //#27 altitude in meters. offset of 500, 500 = 0m
                                //#28 MSB  
      uint16_t current;         //#29 current in 0.1A steps 100 == 10,0A
                                //#30 MSB current display only goes up to 99.9 A (continuous)
      uint16_t main_voltage;    //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                                //#32 MSB (Appears in GAM display right as alternate display.)
      uint16_t batt_cap;        //#33 LSB used battery capacity in 10mAh steps
                                //#34 MSB 
      uint16_t climbrate_L;     //#35 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                                //#36 MSB
      byte climbrate3s;         //#37 climb rate in m/3sec. Value of 120 = 0m/3sec
      uint16_t rpm;             //#38 RPM in 10 RPM steps. 300 = 3000rpm
                                //#39 MSB
      byte Minutes;             //#40 Electric.Minutes (Time does start, when motor current is > 3 A)
      byte Seconds;             //#41 Electric.Seconds (1 byte)
      uint16_t speed;           //#42 LSB (air?) speed in km/h(?) we are using ground speed here per default
                                //#43 MSB speed  
      byte stop_byte;           //#44 stop byte 0x7D
      byte parity;              //#45 CHECKSUM CRC/Parity (calculated dynamicaly)
      } EamData;

struct
      __attribute__((__packed__))
      {
      uint8_t DummyH[5];
      uint8_t Rec_RX_S_STR;
      uint8_t Rec_RVOLT;
      uint8_t Rec_TEMP;
      uint8_t Rec_RX_dBm;
      uint8_t Rec_RX_S_QUA;
      uint8_t Rec_RLOW;
      uint16_t Rec_LOST;
      uint8_t Rec_TX_dBm;
      uint8_t Rec_Null_Byte;
      uint8_t sensorTextID;  
      //uint8_t DummyH[16];
      byte cell[6];	        //#17 Volt Cell 1 (in 2 mV increments, 210 == 4.20 V)
	                        //#18 Volt Cell 2 (in 2 mV increments, 210 == 4.20 V)
			        //#19 Volt Cell 3 (in 2 mV increments, 210 == 4.20 V)
			        //#20 Volt Cell 4 (in 2 mV increments, 210 == 4.20 V)
			        //#21 Volt Cell 5 (in 2 mV increments, 210 == 4.20 V)
			        //#22 Volt Cell 6 (in 2 mV increments, 210 == 4.20 V)
      uint16_t  Battery1;       //#23 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
				//#24 MSB 
      uint16_t  Battery2;       //#25 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
				//#26 MSB
      byte temperature1;        //#27 Temperature 1. Offset of 20. a value of 20 = 0°C
      byte temperature2;        //#28 Temperature 2. Offset of 20. a value of 20 = 0°C
      byte fuel_procent;        //#29 Fuel capacity in %. Values 0--100
				//graphical display ranges: 0-100% with new firmwares of the radios MX12/MX20/...
      uint16_t fuel_ml;         //#30 LSB Fuel in ml scale. Full = 65535!
				//#31 MSB
      uint16_t rpm;             //#32 RPM in 10 RPM steps. 300 = 3000rpm
				//#33 MSB
      uint16_t altitude;        //#34 altitude in meters. offset of 500, 500 = 0m
				//#35 MSB
      uint16_t climbrate_L;     //#36 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
				//#37 MSB
      byte climbrate3s;         //#38 climb rate in m/3sec. Value of 120 = 0m/3sec
      uint16_t current;         //#39 current in 0.1A steps 100 == 10,0A
    				//#40 MSB current display only goes up to 99.9 A (continuous)
      uint16_t main_voltage;    //#41 LSB Main power voltage using 0.1V steps 100 == 10,0V
				//#42 MSB (Appears in GAM display right as alternate display.)
      uint16_t batt_cap;        //#43 LSB used battery capacity in 10mAh steps
				//#44 MSB 
      uint16_t speed;           //#45 LSB (air?) speed in km/h(?) we are using ground speed here per default
				//#46 MSB speed
      byte min_cell_volt;       //#47 minimum cell voltage in 2mV steps. 124 = 2,48V
      byte min_cell_volt_num;   //#48 number of the cell with the lowest voltage
      uint16_t rpm2;            //#49 LSB 2nd RPM in 10 RPM steps. 100 == 1000rpm
				//#50 MSB
      byte general_error_number;//#51 General Error Number (Voice Error == 12) TODO: more documentation
      byte pressure;            //#52 High pressure up to 16bar. 0,1bar scale. 20 == 2.0bar
      byte version;             //#53 version number (Bytes 35 .43 new but not yet in the record in the display!)
      uint8_t DummyJ[2];
      byte stop_byte;           //#56 stop byte 0x7D
      byte parity;              //#57 CHECKSUM CRC/Parity (calculated dynamicaly)
      } GamData;

			static float fHottGetGpsDegree(uint16_t high, uint16_t low)
			{
				float fResult;
				fResult = (float) (high % 100);
				fResult += ((float) low / 10000.0);
				fResult /= 60.0;
				fResult += (float) ((uint16_t)(high / 100));
				return fResult;
			}

			static uint32_t ui32HottGetGpsDegree(uint16_t high, uint16_t low)
			{
				uint32_t ui32Result;
				ui32Result = ((uint32_t)(high % 100)) * 10000000;
				ui32Result += (uint32_t) low * 1000;
				ui32Result /= 60;
				ui32Result += (uint32_t)(high / 100) * 10000000;
				return ui32Result;
			}

			void vHottInit(void)
			{
                                TELEMETRY_SERIAL.begin(57600); // DATA Port for Front upload
#ifdef HOTT_DEBUG
                                DEBUG_SERIAL.begin(19200);
				DEBUG_SERIAL.println("vHottInit");
#endif
			}

			void vHottTelemetrie(void)
			{
				static uint8_t ui8State = HOTT_IDLE;
				static uint8_t ui8GpsCnt = 0;
				static uint32_t ui32RequestTime = 0;
				static uint8_t lastState = 0;

				uint32_t ui32Timeout;

				ui32Timeout = millis() - ui32RequestTime;

#ifdef HOTT_DEBUG

				if (lastState != ui8State)
				{
					DEBUG_SERIAL.println(ui8State);
					lastState = ui8State;
				}

#endif

				switch (ui8State)
				{
				case HOTT_REQUEST_GPS:
                                        //delay(1000);
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendGpsRequest();
						ui8State = HOTT_WAIT_GPS;
					}
					break;
				case HOTT_WAIT_GPS:
					if ( TELEMETRY_SERIAL.available() >= sizeof(GPSData))
					{
						if (bHottReadGpsResponse())
						{
							ui16GpsOk++;
							vUpdateGlobalData();
							ui8State = HOTT_REQUEST_EAM;
						}
						else
						{
							ui16GpsFail++;
							ui8State = HOTT_REQUEST_EAM;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16GpsFail++;
						ui8State = HOTT_REQUEST_EAM;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.print("HOTT_GPS_TIMEOUT - ");
						DEBUG_SERIAL.print(TELEMETRY_SERIAL.available());
						DEBUG_SERIAL.print(" ");
						DEBUG_SERIAL.println(sizeof(GPSData));
#endif
					}

					break;
				case HOTT_REQUEST_EAM:
                                        //delay(1000);
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendEamRequest();
						ui8State = HOTT_WAIT_EAM;
					}
					break;
				case HOTT_WAIT_EAM:
					if ( TELEMETRY_SERIAL.available() >= sizeof(EamData))
					{
						if (bHottReadEamResponse())
						{
							ui16EamOk++;
							vUpdateGlobalData();
							ui8State = HOTT_REQUEST_GAM;
						}
						else
						{
							ui16EamFail++;
							ui8State = HOTT_REQUEST_GAM;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16EamFail++;
						ui8State = HOTT_REQUEST_GAM;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.print("HOTT_EAM_TIMEOUT - ");
						DEBUG_SERIAL.print(TELEMETRY_SERIAL.available());
						DEBUG_SERIAL.print(" ");
						DEBUG_SERIAL.println(sizeof(EamData));
#endif
					}

					break;
				case HOTT_REQUEST_GAM:
                                        //delay(1000);
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendGamRequest();
						ui8State = HOTT_WAIT_GAM;
					}
					break;
				case HOTT_WAIT_GAM:
					if ( TELEMETRY_SERIAL.available() >= sizeof(GamData))
					{
						if (bHottReadGamResponse())
						{
							ui16GamOk++;
							vUpdateGlobalData();
							ui8State = HOTT_IDLE; //HOTT_REQUEST_RX;
						}
						else
						{
							ui16GamFail++;
							ui8State = HOTT_IDLE; //HOTT_REQUEST_RX;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16GamFail++;
						ui8State = HOTT_IDLE; //HOTT_REQUEST_RX;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.print("HOTT_GAM_TIMEOUT - ");
						DEBUG_SERIAL.print(TELEMETRY_SERIAL.available());
						DEBUG_SERIAL.print(" ");
						DEBUG_SERIAL.println(sizeof(GamData));
#endif
					}

					break;
				case HOTT_REQUEST_RX:
					if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui32RequestTime = millis();
						vHottSendReceiverRequest();
						ui8State = HOTT_WAIT_RX;
					}
					break;
				case HOTT_WAIT_RX:
					if ( TELEMETRY_SERIAL.available() >= sizeof(ReceiverData))
					{
						if (bHottReadReceiverResponse())
						{
							ui16RxOk++;
							vUpdateGlobalData();
							ui8State = HOTT_IDLE;
						}
						else
						{
							ui16RxFail++;
							ui8State = HOTT_IDLE;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16RxFail++;
						ui8State = HOTT_IDLE;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.println("HOTT_RX_TIMEOUT");
#endif
					}

					break;
				case HOTT_IDLE:
				default:
                                       /* allow CLI to be started by hitting enter 3 times, if no
                                        heartbeat packets have been received */
                                        if (millis() < 20000 ) 
                                        {
                                          if ( TELEMETRY_SERIAL.available())
                                          {
                                            uint8_t c = char(TELEMETRY_SERIAL.read());
                                            if (c == '\n' || c == '\r') 
                                            {
                                              crlf_count++;
                                            } 
                                            else 
                                            {
                                              crlf_count = 0;
                                            }
                                            if (crlf_count == 3) 
                                            {
                                              uploadFont();
                                            }
                                          }
                                        }
                                        else
                                        {
                                          TELEMETRY_SERIAL.begin(19200); // DATA Port for Telemetrie 
                                          ui8State = HOTT_REQUEST_GPS;
                                        }
                                        /*
					if (bIsBtConnected())
					{
						ui8State = HOTT_REQUEST_GPS;
					}
					else
					{
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.println("HOTT no BT");
#endif
                                                ui8State = HOTT_REQUEST_GPS;
					} */
					break;
				}
			}

		        static void vHottSendGpsRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write(0x80);  //Byte 1: 0x80 = Receiver byte 0x180
                                delay(10);
				TELEMETRY_SERIAL.write(0x8a);  //Byte 2: 0x8A = GPS Sensor byte
			}

			static bool bHottReadGpsResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((char *) &GPSData,
						sizeof(GPSData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(GPSData));
			}

			static void vHottSendReceiverRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x34);
				TELEMETRY_SERIAL.write(0x13);
				TELEMETRY_SERIAL.write(0xba);
			}

			static bool bHottReadReceiverResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((char *) &ReceiverData,
						sizeof(ReceiverData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(ReceiverData));
			}

			static void vHottSendEamRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write(0x80);  //Byte 1: 80 = Receiver byte
                                delay(10);
				TELEMETRY_SERIAL.write(0x8e);  //Byte 2: 8E = Electric Sensor byte
			}

			static bool bHottReadEamResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((char *) &EamData,
						sizeof(EamData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(EamData));
			}

			static void vHottSendGamRequest(void)
			{
				vHottClean();
				TELEMETRY_SERIAL.write(0x80);  //Byte 1: 80 = Receiver byte
                                delay(10);
				TELEMETRY_SERIAL.write(0x8d);  //Byte 2: 8D = General Sensor byte
			}

			static bool bHottReadGamResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((char *) &GamData,
						sizeof(GamData));
#ifdef HOTT_DEBUG
				DEBUG_SERIAL.println(ui8Cnt);
#endif
				return (ui8Cnt == sizeof(GamData));
			}

			static void vHottClean(void)
			{
				while ( TELEMETRY_SERIAL.available() > 0)
					TELEMETRY_SERIAL.read();
			}

			static bool bIsBtConnected(void)
			{
				return digitalRead(PIN_BT_STATUS);
			}

			static void vUpdateGlobalData(void)
			{
                                  lastpacketreceived = millis();
                                  
                                  osd_rssi = GamData.Rec_RX_S_STR;
                                  				
                                  osd_vbat_A = GamData.Battery1;
                                  osd_vbat_A = osd_vbat_A / 10;
                                  osd_vbat_B = GamData.Battery2;
                                  osd_vbat_B = osd_vbat_B / 10;                                  
                                  osd_curr_A = GamData.current;
                                  osd_curr_A *= 10;
                                  mah_used = GamData.batt_cap;
                                  mah_used *= 10;
                                  
                                  osd_lat = ui32HottGetGpsDegree(GPSData.LatitudeMin,GPSData.LatitudeSec) / 10000000.0;
                                  osd_lon = ui32HottGetGpsDegree(GPSData.longitudeMin,GPSData.longitudeSec) / 10000000.0;
                                  osd_satellites_visible = GPSData.GPSNumSat;
                                  if(GPSData.GPSFixChar == 0x2d || GPSData.GPSFixChar == 0x32) osd_fix_type = 0; 
                                  else if(GPSData.GPSFixChar == 0x33 || GPSData.GPSFixChar == 0x44) osd_fix_type = 2;
                                  osd_heading = GPSData.flightDirection * 2;
                                  osd_home_direction = GPSData.HomeDirection / 12;
                                  if(osd_home_direction < 1) osd_home_direction = 1; 
                                  osd_home_distance = GPSData.distance;
                                  osd_alt = GPSData.altitude - 500;
                                  osd_airspeed = GPSData.GPSSpeed;
                                 /*
				 GPSData.ui16DistanceToHome
				 

				uav_lat = ui32HottGetGpsDegree(GPSData.ui16LatitudeHigh,GPSData.ui16LatitudeLow); 	// latitude (deg * 1e7)
				uav_lon = ui32HottGetGpsDegree(GPSData.ui16LongitudeHigh,
						GPSData.ui16LongitudeLow);	// longitude
				uav_satellites_visible = GPSData.ui8Sat;// number of satellites
				uav_fix_type = GPSData.ui8FixChar;// GPS lock 0-1=no fix, 2=2D, 3=3D
				uav_alt = (int32_t) GPSData.ui16Altitude * 10;  // altitude (cm)
				uav_groundspeed = GPSData.ui16Speed;     // ground speed in km/h
				uav_groundspeedms = GPSData.ui16Speed / 3.6f; // ground speed in m/s
				uav_pitch = GPSData.ui8AngleY;                 // attitude pitch
				uav_pitch = uav_pitch * 2;
				if( uav_pitch > 180 ) uav_pitch -= 360;
				uav_roll = GPSData.ui8AngleX;                   // attitude roll
				uav_roll = uav_roll * 2;
				if( uav_roll > 180 ) uav_roll -= 360;
				uav_heading = GPSData.ui8AngleZ;             // attitude heading
				uav_gpsheading = GPSData.ui16Direction;           // gps heading
				//uav_bat = ReceiverData.ui8Volt*100U;            // battery voltage (mv)
				uav_bat =
						(GamData.ui16Batt1 > GamData.ui16Batt2) ?
								GamData.ui16Batt1 : GamData.ui16Batt2;
				uav_bat *= 100;                                 //??
				uav_current = GamData.ui16PowerCurrent;			// actual curren
                                uav_current *= 100;
                                uav_current = 100;
                                uav_amp = GamData.ui16PowerCapacity;
                                uav_amp *= 10;
				uav_rssi = ReceiverData.ui8Strength;		// radio RSSI (%)
				uav_linkquality = ReceiverData.ui8Quality;// radio link quality
				uav_airspeed = 0;               		// Airspeed sensor (m/s)
				uav_arm = 0;                    		// 0: disarmed, 1: armed
				uav_failsafe = 0;               	// 0: normal,   1: failsafe
				uav_flightmode = 19; // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
									 // 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
									 // 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown
				protocol = "HoTT";
				telemetry_ok = true;
				lastpacketreceived = millis();
                                */
#ifdef HOTT_DEBUG
				static uint32_t ui32FirstPacketTime = millis();
				DEBUG_SERIAL.print("HoTT active for ");
				DEBUG_SERIAL.print(lastpacketreceived - ui32FirstPacketTime);
				DEBUG_SERIAL.println(" ms.");
				DEBUG_SERIAL.print("GPS (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16GpsOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16GpsFail);
				DEBUG_SERIAL.print("EAM (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16EamOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16EamFail);
				DEBUG_SERIAL.print("GAM (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16GamOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16GamFail);
				DEBUG_SERIAL.print("Rx (OK/FAIL): ");
				DEBUG_SERIAL.print(ui16RxOk);
				DEBUG_SERIAL.print(" / ");
				DEBUG_SERIAL.println(ui16RxFail);

                                //GPS
                                DEBUG_SERIAL.print("flightDirection:");
		                DEBUG_SERIAL.println(GPSData.flightDirection);
                                DEBUG_SERIAL.print("GPSSpeed:");
		                DEBUG_SERIAL.println(GPSData.GPSSpeed);
                                DEBUG_SERIAL.print("LatitudeNS:");
		                DEBUG_SERIAL.println(GPSData.LatitudeNS);
                                DEBUG_SERIAL.print("LatitudeMin:");
		                DEBUG_SERIAL.println(GPSData.LatitudeMin);
                                DEBUG_SERIAL.print("LatitudeSec:");
		                DEBUG_SERIAL.println(GPSData.LatitudeSec);
                                DEBUG_SERIAL.print("Lat:");
		                DEBUG_SERIAL.println(ui32HottGetGpsDegree(GPSData.LatitudeMin,GPSData.LatitudeSec));
                                DEBUG_SERIAL.print("Lat:");
		                DEBUG_SERIAL.println(ui32HottGetGpsDegree(GPSData.LatitudeMin,GPSData.LatitudeSec)/10000000.0);
                                DEBUG_SERIAL.print("longitudeEW:");                                
		                DEBUG_SERIAL.println(GPSData.longitudeEW);                                 
                                DEBUG_SERIAL.print("longitudeMin:");
		                DEBUG_SERIAL.println(GPSData.longitudeMin);
                                DEBUG_SERIAL.print("longitudeSec:");
		                DEBUG_SERIAL.println(GPSData.longitudeSec);
                                DEBUG_SERIAL.print("Lon:");
		                DEBUG_SERIAL.println(ui32HottGetGpsDegree(GPSData.longitudeMin,GPSData.longitudeSec));
                                DEBUG_SERIAL.print("Lon:");
		                DEBUG_SERIAL.println(ui32HottGetGpsDegree(GPSData.longitudeMin,GPSData.longitudeSec)/10000000.0);
                                DEBUG_SERIAL.print("distance:");
		                DEBUG_SERIAL.println(GPSData.distance);
                                DEBUG_SERIAL.print("altitude:");
		                DEBUG_SERIAL.println(GPSData.altitude);



                                //GAM 
                                /*DEBUG_SERIAL.print("cell1:");
		                DEBUG_SERIAL.println(GamData.cell[1]);
                                DEBUG_SERIAL.print("cell2:");
		                DEBUG_SERIAL.println(GamData.cell[2]);
                                DEBUG_SERIAL.print("cell3:");
		                DEBUG_SERIAL.println(GamData.cell[3]);
                                DEBUG_SERIAL.print("cell4:");
		                DEBUG_SERIAL.println(GamData.cell[4]);
                                DEBUG_SERIAL.print("cell5:");
		                DEBUG_SERIAL.println(GamData.cell[5]);
                                DEBUG_SERIAL.print("cell6:");
		                DEBUG_SERIAL.println(GamData.cell[6]);
                                DEBUG_SERIAL.print("Battery1:");
		                DEBUG_SERIAL.println(GamData.Battery1);
                                DEBUG_SERIAL.print("Battery2:");
		                DEBUG_SERIAL.println(GamData.Battery2);
                                DEBUG_SERIAL.print("temperature1:");
		                DEBUG_SERIAL.println(GamData.temperature1);
                                DEBUG_SERIAL.print("temperature2:");
		                DEBUG_SERIAL.println(GamData.temperature2);
                                DEBUG_SERIAL.print("rpm:");
		                DEBUG_SERIAL.println(GamData.rpm);
                                DEBUG_SERIAL.print("altitude:");
		                DEBUG_SERIAL.println(GamData.altitude);
                                DEBUG_SERIAL.print("current:");
		                DEBUG_SERIAL.println(GamData.current);
                                DEBUG_SERIAL.print("main_voltage:");
		                DEBUG_SERIAL.println(GamData.main_voltage);
                                DEBUG_SERIAL.print("batt_cap:");
		                DEBUG_SERIAL.println(GamData.batt_cap); */





#endif

#ifdef HOTT_SIMULATION_DEBUG
				static float w=0;
				static int16_t k=0;
				static int8_t d=1;
				if(w>5){
					uav_satellites_visible = 6;
					uav_fix_type = '3';
				}

				// The home pos:
				uav_lat = 1e7 * 51.3678;
				uav_lon = 1e7 * 6.78756;
				if( home_pos )
				{
					// fly circles (r=1km) around the home pos...
					uav_lat += 1e7 * 0.01113195 * sin(w);// north
					uav_lon += 1e7 * 0.01113195 * cos(w);// east

					// 0 .. 1000m height depend on pan angle...
					uav_alt = (int32_t)(50000.0 * ( 1 + sin(w) ) );
				}
				w+=0.1;

				uav_pitch = 0;
				uav_roll = k;
				k+=d;
				if( k >= 30 ) d=-1;
				if( k <= -30 ) d=1;
#endif

			}

#endif

