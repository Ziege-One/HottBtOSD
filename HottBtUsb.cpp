/* HoTT USB/BT Telemetry , 2015 by CopterFail */

#include <Arduino.h>

//#include "defines.h"
//#include "boards.h"
//#include "globals.h"
//#include "HottBtUsb.h"

#include <SoftwareSerial.h>

#ifdef PROTOCOL_HOTT_BT


#ifdef HOTT_DEBUG
  SoftwareSerial DEBUG_SERIAL(2, 3); // RX, TX
#endif
/* Configuration */
#define HOTT_WAIT_TIME 100U		// time [ms] to wait for response or next request

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
		// Telegram Header:
		uint8_t Start; // 0x00
		uint16_t Dummy;
		uint8_t Header1; // 0x18
		uint8_t Header2; // 0x00
		uint8_t Header3; // 0x04
		uint8_t Header4; // 0x01
		// GPS Data:
		uint16_t DummyH;
		uint16_t GPSSpeed; 
		uint16_t distance; //12+13
		uint16_t altitude; //14+15
		uint16_t LatitudeMin; //16+17
		uint16_t LatitudeSec; //18+19
		uint16_t longitudeMin; //20+21
		uint16_t longitudeSec; //22+23
		uint16_t flightDirection; //24+25
		uint8_t LatitudeNS; //
		uint8_t longitudeEW; //
		uint16_t climbrate1s;
		uint8_t climbrate3s;
		uint8_t Dbm;
		uint8_t GPSNumSat;
		uint8_t GPSFixChar;
		uint16_t HomeDirection;
		uint8_t angleXdirection;
		uint8_t angleYdirection;
		uint8_t angleZdirection;
		uint16_t GyroX;
		uint16_t GyroY;
		uint16_t GyroZ;
		uint8_t ui8Vibration;
		uint8_t Ascii4;
		uint8_t Ascii5;
		uint8_t GPS_fix;
		uint8_t version;
		uint16_t DummyD;
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
		// Telegram Header:
		uint8_t Start; // 0x00
		uint16_t Dummy;
		uint8_t Header1; // 0x30
		uint8_t Header2; // 0x00
		uint8_t Header3; // 0x04
		uint8_t Header4; // 0x01
		// EAM Data:
		uint16_t DummyH;
		uint16_t cell_L[7];
		uint16_t cell_H[7];;
		uint16_t Battery1;
		uint16_t Battery2;
        	uint16_t temperature1;
		uint16_t temperature2;
		uint16_t altitude;
		uint16_t current;
		uint16_t main_voltage;
		uint16_t batt_cap;
		uint16_t climbrate_L;
		uint8_t climbrate3s;
		uint8_t DBM;
		uint16_t rpm;
		uint8_t Minutes;
		uint8_t Seconds;
		uint8_t speed;
		uint8_t Version;
//		uint16_t DummyD;
		} EamData;

struct
	__attribute__((__packed__))
	{
		// Telegram Header:
		uint8_t Start; // 0x00
		uint16_t Dummy;
		uint8_t Header1; // 0x23
		uint8_t Header2; // 0x00
		uint8_t Header3; // 0x04
		uint8_t Header4; // 0x01
		// GAM Data:
		uint16_t DummyH;
		uint16_t cell[6];
		uint16_t Battery1;
		uint16_t Battery2;
		uint16_t temperature1;
		uint16_t temperature2;
		uint16_t rpm;
		uint16_t altitude;
		uint16_t current;
		uint16_t main_voltage;
		uint16_t batt_cap;
		uint16_t fuel_ml;
		uint8_t OilLevel;
		uint16_t climbrate_L;
		uint8_t climbrate3s;
		uint8_t DBM;
		uint8_t Free;
		uint16_t speed;
		uint16_t min_cell_volt;
		uint16_t rpm2;
		uint8_t min_cell_volt_num;
		uint8_t general_error_number;
		uint8_t pressure;
		uint8_t Version;
//		uint16_t ui16DummyD;
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
				pinMode(PIN_BT_PIN34, OUTPUT);
				pinMode(PIN_BT_STATUS, INPUT);
				digitalWrite(PIN_BT_PIN34, LOW);
				TELEMETRY_SERIAL.begin(57600); // DATA Port for Front upload
                                
#ifdef HOTT_DEBUG
                                DEBUG_SERIAL.begin(57600);
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
						DEBUG_SERIAL.println("HOTT_GPS_TIMEOUT");
#endif
					}
					/*
					 if( ui8GpsCnt > 0)
					 {
					 ui8GpsCnt--;
					 ui8State = HOTT_REQUEST_GPS;
					 }
					 else
					 {
					 ui8GpsCnt = 0;
					 ui8State = HOTT_REQUEST_RX;
					 }
					 */
					break;
				case HOTT_REQUEST_EAM:
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
							ui8State = HOTT_REQUEST_RX;
						}
						else
						{
							ui16GamFail++;
							ui8State = HOTT_REQUEST_RX;
						}
					}
					else if (ui32Timeout > HOTT_WAIT_TIME)
					{
						ui16GamFail++;
						ui8State = HOTT_REQUEST_RX;
#ifdef HOTT_DEBUG
						DEBUG_SERIAL.print("HOTT_GAM_TIMEOUT - ");
						DEBUG_SERIAL.print(TELEMETRY_SERIAL.available());
						DEBUG_SERIAL.print(" ");
						DEBUG_SERIAL.println(sizeof(EamData));
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
                                          TELEMETRY_SERIAL.begin(115200); // DATA Port for Telemetrie 
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
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x38);
				TELEMETRY_SERIAL.write(0x9f);
				TELEMETRY_SERIAL.write(0x7b);
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
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x36);
				TELEMETRY_SERIAL.write(0x51);
				TELEMETRY_SERIAL.write(0x9a);
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
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x03);
				TELEMETRY_SERIAL.write(0xfc);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write((byte)0x00);
				TELEMETRY_SERIAL.write(0x04);
				TELEMETRY_SERIAL.write(0x35);
				TELEMETRY_SERIAL.write(0x32);
				TELEMETRY_SERIAL.write(0xaa);
			}

			static bool bHottReadGamResponse(void)
			{
				uint8_t ui8Cnt;
				ui8Cnt = TELEMETRY_SERIAL.readBytes((char *) &GamData,
						sizeof(GamData));
#ifdef HOTT_DEBUG
				//DEBUG_SERIAL.println(ui8Cnt);
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
                                  				
                                  osd_vbat_A = GamData.Battery1;
                                  osd_vbat_A = osd_vbat_A / 10;
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

