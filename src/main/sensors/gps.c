/*
 * This file is part of Ninjaflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/light_led.h"

#include "io/serial.h"

#include "gps.h"

#define LOG_ERROR		'?'
#define LOG_IGNORED	  '!'
#define LOG_SKIPPED	  '>'
#define LOG_NMEA_GGA	 'g'
#define LOG_NMEA_RMC	 'r'
#define LOG_UBLOX_SOL	'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'

#define GPS_SV_MAXSATS   16

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in gpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)

//static serialPort_t *gpsPort;

typedef struct gpsInitData_s {
	uint8_t index;
	uint8_t baudrateIndex; // see baudRate_e
	const char *ubx;
	const char *mtk;
} gpsInitData_t;

// NMEA will cycle through these until valid data is received
static const gpsInitData_t gpsInitData[] = {
	{ GPS_BAUDRATE_115200,  BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
	{ GPS_BAUDRATE_57600,	BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
	{ GPS_BAUDRATE_38400,	BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
	{ GPS_BAUDRATE_19200,	BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
	// 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
	{ GPS_BAUDRATE_9600,	  BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(gpsInitData) / sizeof(gpsInitData[0]))

#define DEFAULT_BAUD_RATE_INDEX 0

static const uint8_t ubloxInit[] = {

	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,		   // CFG-NAV5 - Set engine settings
	0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,		   // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
	0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,		   // capturing the data from the U-Center binary console.
	0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,

	// DISABLE NMEA messages
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,		   // VGS: Course over ground and Ground speed
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,		   // GSV: GNSS Satellites in View
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,		   // GLL: Latitude and longitude, with time of position fix and status
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,		   // GGA: Global positioning system fix data
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,		   // GSA: GNSS DOP and Active Satellites
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,		   // RMC: Recommended Minimum data

	// Enable UBLOX messages
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,		   // set POSLLH MSG rate
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,		   // set STATUS MSG rate
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,		   // set SOL MSG rate
	//0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3,		   // set SVINFO MSG rate (every cycle - high bandwidth)
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x05, 0x40, 0xA7,		   // set SVINFO MSG rate (evey 5 cycles - low bandwidth)
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,		   // set VELNED MSG rate

	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,			 // set rate to 5Hz (measurement period: 200ms, navigation rate: 1 cycle)
};

// UBlox 6 Protocol documentation - GPS.G6-SW-10018-F
// SBAS Configuration Settings Desciption, Page 4/210
// 31.21 CFG-SBAS (0x06 0x16), Page 142/210
// A.10 SBAS Configuration (UBX-CFG-SBAS), Page 198/210 - GPS.G6-SW-10018-F

#define UBLOX_SBAS_MESSAGE_LENGTH 16
typedef struct ubloxSbas_s {
	sbasMode_e mode;
	uint8_t message[UBLOX_SBAS_MESSAGE_LENGTH];
} ubloxSbas_t;



// Note: these must be defined in the same order is sbasMode_e since no lookup table is used.
static const ubloxSbas_t ubloxSbas[] = {
	// NOTE this could be optimized to save a few bytes of flash space since the same prefix is used for each command.
	{ SBAS_AUTO,  { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xE5}},
	{ SBAS_EGNOS, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41}},
	{ SBAS_WAAS,  { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x04, 0xE0, 0x04, 0x00, 0x19, 0x9D}},
	{ SBAS_MSAS,  { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x00, 0x02, 0x02, 0x00, 0x35, 0xEF}},
	{ SBAS_GAGAN, { 0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x80, 0x01, 0x00, 0x00, 0xB2, 0xE8}}
};


typedef enum {
	GPS_UNKNOWN,
	GPS_INITIALIZING,
	GPS_CHANGE_BAUD,
	GPS_CONFIGURE,
	GPS_RECEIVING_DATA,
	GPS_LOST_COMMUNICATION,
} gpsState_e;

gpsData_t gpsData;


static void shiftPacketLog(struct gps *self)
{
	uint32_t i;

	for (i = ARRAYLEN(self->gpsPacketLog) - 1; i > 0 ; i--) {
		self->gpsPacketLog[i] = self->gpsPacketLog[i-1];
	}
}

static void gpsNewData(struct gps *self, uint16_t c);
static bool gpsNewFrameNMEA(struct gps *self, char c);
static bool gpsNewFrameUBLOX(struct gps *self, uint8_t data);

static void gpsSetState(struct gps *self, gpsState_e state){
	self->gpsData.state = state;
	self->gpsData.state_position = 0;
	self->gpsData.state_ts = sys_millis(self->system);
	self->gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}

int gps_init(struct gps *self, const struct system_calls *system, const struct config *config){
	memset(self, 0, sizeof(struct gps));

	self->config = config;
	self->gpsPacketLogChar = self->gpsPacketLog;
	self->system = system;
	self->gpsData.baudrateIndex = 0;
	self->gpsData.errors = 0;
	self->gpsData.timeouts = 0;

	// init gpsData structure. if we're not actually enabled, don't bother doing anything else
	gpsSetState(self, GPS_UNKNOWN);

	self->gpsData.lastMessage = sys_millis(self->system);

	const struct serial_port_config *gpsPortConfig = findSerialPortConfig(&self->config->serial, FUNCTION_GPS);
	if (!gpsPortConfig) {
		return -1;
	}

	while (gpsInitData[self->gpsData.baudrateIndex].baudrateIndex != gpsPortConfig->gps_baudrateIndex) {
		self->gpsData.baudrateIndex++;
		if (self->gpsData.baudrateIndex >= GPS_INIT_DATA_ENTRY_COUNT) {
			self->gpsData.baudrateIndex = DEFAULT_BAUD_RATE_INDEX;
			break;
		}
	}

	portMode_t mode = MODE_RXTX;
	// only RX is needed for NMEA-style GPS
	if (self->config->gps.provider == GPS_NMEA)
		mode &= ~MODE_TX;

	// no callback - buffer will be consumed in gpsThread()
	self->gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, gpsInitData[self->gpsData.baudrateIndex].baudrateIndex, mode, SERIAL_NOT_INVERTED);
	if (!self->gpsPort) {
		return -1;
	}

	// signal GPS "thread" to initialize when it gets to it
	gpsSetState(self, GPS_INITIALIZING);

	return 0;
}

static void gpsInitNmea(struct gps *self)
{
	switch(self->gpsData.state) {
		case GPS_INITIALIZING:
		case GPS_CHANGE_BAUD:
			serialSetBaudRate(self->gpsPort, baudRates[gpsInitData[self->gpsData.baudrateIndex].baudrateIndex]);
			gpsSetState(self, GPS_RECEIVING_DATA);
			break;
		default:
			break;
	}
}

static void gpsInitUblox(struct gps *self)
{
	sys_millis_t now;
	// UBX will run at the serial port's baudrate, it shouldn't be "autodetected". So here we force it to that rate

	// Wait until GPS transmit buffer is empty
	if (!isSerialTransmitBufferEmpty(self->gpsPort))
		return;


	switch (self->gpsData.state) {
		case GPS_INITIALIZING:
			now = sys_millis(self->system);
			if (now - self->gpsData.state_ts < GPS_BAUDRATE_CHANGE_DELAY)
				return;

			if (self->gpsData.state_position < GPS_INIT_ENTRIES) {
				// try different speed to INIT
				baudRate_e newBaudRateIndex = gpsInitData[self->gpsData.state_position].baudrateIndex;

				self->gpsData.state_ts = now;

				if (lookupBaudRateIndex(serialGetBaudRate(self->gpsPort)) != newBaudRateIndex) {
					// change the rate if needed and wait a little
					serialSetBaudRate(self->gpsPort, baudRates[newBaudRateIndex]);
					return;
				}

				// print our FIXED init string for the baudrate we want to be at
				serialPrint(self->gpsPort, gpsInitData[self->gpsData.baudrateIndex].ubx);

				self->gpsData.state_position++;
			} else {
				// we're now (hopefully) at the correct rate, next state will switch to it
				gpsSetState(self, GPS_CHANGE_BAUD);
			}
			break;
		case GPS_CHANGE_BAUD:
			serialSetBaudRate(self->gpsPort, baudRates[gpsInitData[self->gpsData.baudrateIndex].baudrateIndex]);
			gpsSetState(self, GPS_CONFIGURE);
			break;
		case GPS_CONFIGURE:

			// Either use specific config file for GPS or let dynamically upload config
			if( self->config->gps.autoConfig == GPS_AUTOCONFIG_OFF ) {
				gpsSetState(self, GPS_RECEIVING_DATA);
				break;
			}

			if (self->gpsData.messageState == GPS_MESSAGE_STATE_IDLE) {
				self->gpsData.messageState++;
			}

			if (self->gpsData.messageState == GPS_MESSAGE_STATE_INIT) {

				if (self->gpsData.state_position < sizeof(ubloxInit)) {
					serialWrite(self->gpsPort, ubloxInit[self->gpsData.state_position]);
					self->gpsData.state_position++;
				} else {
					self->gpsData.state_position = 0;
					self->gpsData.messageState++;
				}
			}

			if (self->gpsData.messageState == GPS_MESSAGE_STATE_SBAS) {
				if (self->gpsData.state_position < UBLOX_SBAS_MESSAGE_LENGTH) {
					serialWrite(self->gpsPort, ubloxSbas[self->config->gps.sbasMode].message[self->gpsData.state_position]);
					self->gpsData.state_position++;
				} else {
					self->gpsData.messageState++;
				}
			}

			if (self->gpsData.messageState >= GPS_MESSAGE_STATE_ENTRY_COUNT) {
				// ublox should be initialised, try receiving
				gpsSetState(self, GPS_RECEIVING_DATA);
			}
			break;
		default:
			break;
	}
}

static void gpsInitHardware(struct gps *self)
{
	switch (self->config->gps.provider) {
		case GPS_NMEA:
			gpsInitNmea(self);
			break;
		case GPS_UBLOX:
			gpsInitUblox(self);
			break;
		case GPS_PROVIDER_MAX:
		default:
			break;
	}
}

void gps_update(struct gps *self)
{
	// read out available GPS bytes
	if (self->gpsPort) {
		while (serialRxBytesWaiting(self->gpsPort))
			gpsNewData(self, serialRead(self->gpsPort));
	}

	switch (self->gpsData.state) {
		case GPS_UNKNOWN:
		default:
			break;

		case GPS_INITIALIZING:
		case GPS_CHANGE_BAUD:
		case GPS_CONFIGURE:
			gpsInitHardware(self);
			break;

		case GPS_LOST_COMMUNICATION:
			self->gpsData.timeouts++;
			if (self->config->gps.autoBaud) {
				// try another rate
				self->gpsData.baudrateIndex++;
				self->gpsData.baudrateIndex %= GPS_INIT_ENTRIES;
			}
			self->gpsData.lastMessage = sys_millis(self->system);
			// TODO - move some / all of these into gpsData
			self->GPS_numSat = 0;
			gpsSetState(self, GPS_INITIALIZING);
			break;

		case GPS_RECEIVING_DATA:
			// check for no data/gps timeout/cable disconnection etc
			if (sys_millis(self->system) - self->gpsData.lastMessage > GPS_TIMEOUT) {
				// remove GPS from capability
				gpsSetState(self, GPS_LOST_COMMUNICATION);
			}
			break;
	}
}

static bool gpsNewFrame(struct gps *self, uint8_t c){
	switch (self->config->gps.provider) {
		case GPS_NMEA:		  // NMEA
			return gpsNewFrameNMEA(self, c);
		case GPS_UBLOX:		 // UBX binary
			return gpsNewFrameUBLOX(self, c);
		default:
		case GPS_PROVIDER_MAX:
			break;
	}

	return false;
}
static void gpsNewData(struct gps *self, uint16_t c)
{
	if (!gpsNewFrame(self, c)) {
		return;
	}

	// new data received and parsed, we're in business
	self->gpsData.lastLastMessage = self->gpsData.lastMessage;
	self->gpsData.lastMessage = sys_millis(self->system);

	if (self->GPS_update == 1)
		self->GPS_update = 0;
	else
		self->GPS_update = 1;

#if 0
	debug[3] = GPS_update;
#endif

	// TODO: gps new data notification
	//onGpsNewData(self);
}



/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output 5 frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Now verifies checksum correctly before applying data

   Here we use only the following data :
	 - latitude
	 - longitude
	 - GPS fix is/is not ok
	 - GPS num sat (4 is enough to be +/- reliable)
	 // added by Mis
	 - GPS altitude (for OSD displaying)
	 - GPS speed (for OSD displaying)
*/

#define NO_FRAME   0
#define FRAME_GGA  1
#define FRAME_RMC  2
#define FRAME_GSV  3


// This code is used for parsing NMEA data

/* Alex optimization
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
	- d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
	- m is always 2 char long
	- f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
static uint32_t GPS_coord_to_degrees(char *coordinateString)
{
	char *p = s, *d = s;
	uint8_t min, deg = 0;
	uint16_t frac = 0, mult = 10000;

	while (*p) {				// parse the string until its end
		if (d != s) {
			frac += (*p - '0') * mult;  // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
			mult /= 10;
		}
		if (*p == '.')
			d = p;			  // locate '.' char in the string
		p++;
	}
	if (p == s)
		return 0;
	while (s < d - 2) {
		deg *= 10;			  // convert degrees : all chars before minutes ; for the first iteration, deg = 0
		deg += *(s++) - '0';
	}
	min = *(d - 1) - '0' + (*(d - 2) - '0') * 10;	   // convert minutes : 2 previous char before '.'
	return deg * 10000000UL + (min * 100000UL + frac) * 10UL / 6;
}
  */

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult)
{							   // convert string to uint32
	uint32_t i;
	uint32_t tmp = 0;
	for (i = 0; src[i] != 0; i++) {
		if (src[i] == '.') {
			i++;
			if (mult == 0)
				break;
			else
				src[i + mult] = 0;
		}
		tmp *= 10;
		if (src[i] >= '0' && src[i] <= '9')
			tmp += src[i] - '0';
		if (i >= 15)
			return 0; // out of bounds
	}
	return tmp;
}

typedef struct gpsDataNmea_s {
	int32_t latitude;
	int32_t longitude;
	uint8_t numSat;
	uint16_t altitude;
	uint16_t speed;
	uint16_t ground_course;
} gpsDataNmea_t;

static bool gpsNewFrameNMEA(struct gps *self, char c)
{
	static gpsDataNmea_t gps_Msg;

	uint8_t frameOK = 0;
	static uint8_t param = 0, offset = 0, parity = 0;
	static char string[15];
	static uint8_t checksum_param, gps_frame = NO_FRAME;
	static uint8_t svMessageNum = 0;
	uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;

	switch (c) {
		case '$':
			param = 0;
			offset = 0;
			parity = 0;
			break;
		case ',':
		case '*':
			string[offset] = 0;
			if (param == 0) {	   //frame identification
				gps_frame = NO_FRAME;
				if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A')
					gps_frame = FRAME_GGA;
				if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C')
					gps_frame = FRAME_RMC;
				if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'V')
					gps_frame = FRAME_GSV;
			}

			switch (gps_frame) {
				case FRAME_GGA:		//************* GPGGA FRAME parsing
					switch(param) {
			//		  case 1:			 // Time information
			//			  break;
						case 2:
							// TODO: gps coord to degrees
							//gps_Msg.latitude = GPS_coord_to_degrees(string);
							break;
						case 3:
							if (string[0] == 'S')
								gps_Msg.latitude *= -1;
							break;
						case 4:
							//gps_Msg.longitude = GPS_coord_to_degrees(string);
							break;
						case 5:
							if (string[0] == 'W')
								gps_Msg.longitude *= -1;
							break;
						case 6:
							if (string[0] > '0') {
								// TODO: set numsats and add gps_has_fix()
							} else {
							}
							break;
						case 7:
							gps_Msg.numSat = grab_fields(string, 0);
							break;
						case 9:
							gps_Msg.altitude = grab_fields(string, 0);	 // altitude in meters added by Mis
							break;
						default:
							break;
					}
					break;
				case FRAME_RMC:		//************* GPRMC FRAME parsing
					switch(param) {
						case 7:
							gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);	// speed in cm/s added by Mis
							break;
						case 8:
							gps_Msg.ground_course = (grab_fields(string, 1));	  // ground course deg * 10
							break;
						default:
							break;
					}
					break;
				case FRAME_GSV:
					switch(param) {
					  /*case 1:
							// Total number of messages of this type in this cycle
							break; */
						case 2:
							// Message number
							svMessageNum = grab_fields(string, 0);
							break;
						case 3:
							// Total number of SVs visible
							self->GPS_numCh = grab_fields(string, 0);
							break;
						default:
							break;
					}
					if(param < 4)
						break;

					svPacketIdx = (param - 4) / 4 + 1; // satellite number in packet, 1-4
					svSatNum	= svPacketIdx + (4 * (svMessageNum - 1)); // global satellite number
					svSatParam  = param - 3 - (4 * (svPacketIdx - 1)); // parameter number for satellite

					if(svSatNum > GPS_SV_MAXSATS)
						break;

					switch(svSatParam) {
						case 1:
							// SV PRN number
							self->GPS_svinfo_chn[svSatNum - 1]  = svSatNum;
							self->GPS_svinfo_svid[svSatNum - 1] = grab_fields(string, 0);
							break;
					  /*case 2:
							// Elevation, in degrees, 90 maximum
							break;
						case 3:
							// Azimuth, degrees from True North, 000 through 359
							break; */
						case 4:
							// SNR, 00 through 99 dB (null when not tracking)
							self->GPS_svinfo_cno[svSatNum - 1] = grab_fields(string, 0);
							self->GPS_svinfo_quality[svSatNum - 1] = 0; // only used by ublox
							break;
						default:
							break;
					}

					self->GPS_svInfoReceivedCount++;

					break;
				default:
					break;
			}

			param++;
			offset = 0;
			if (c == '*')
				checksum_param = 1;
			else
				parity ^= c;
			break;
		case '\r':
		case '\n':
			if (checksum_param) {   //parity checksum
				shiftPacketLog(self);
				uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
				if (checksum == parity) {
					*self->gpsPacketLogChar = LOG_IGNORED;
					self->GPS_packetCount++;
					switch (gps_frame) {
					case FRAME_GGA:
						*self->gpsPacketLogChar = LOG_NMEA_GGA;
						frameOK = 1;
						self->GPS_coord[LAT] = gps_Msg.latitude;
						self->GPS_coord[LON] = gps_Msg.longitude;
						self->GPS_numSat = gps_Msg.numSat;
						self->GPS_altitude = gps_Msg.altitude;
						break;
					case FRAME_RMC:
						*self->gpsPacketLogChar = LOG_NMEA_RMC;
						self->GPS_speed = gps_Msg.speed;
						self->GPS_ground_course = gps_Msg.ground_course;
						break;
					default:
						break;
					} // end switch
				} else {
					*self->gpsPacketLogChar = LOG_ERROR;
				}
			}
			checksum_param = 0;
			break;
		default:
			if (offset < 15)
				string[offset++] = c;
			if (!checksum_param)
				parity ^= c;
	}
	return frameOK;
}

// UBX support
typedef struct {
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
} ubx_header;

typedef struct {
	uint32_t time;			  // GPS msToW
	int32_t longitude;
	int32_t latitude;
	int32_t altitude_ellipsoid;
	int32_t altitude_msl;
	uint32_t horizontal_accuracy;
	uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
	uint32_t time;			  // GPS msToW
	uint8_t fix_type;
	uint8_t fix_status;
	uint8_t differential_status;
	uint8_t res;
	uint32_t time_to_first_fix;
	uint32_t uptime;			// milliseconds
} ubx_nav_status;

typedef struct {
	uint32_t time;
	int32_t time_nsec;
	int16_t week;
	uint8_t fix_type;
	uint8_t fix_status;
	int32_t ecef_x;
	int32_t ecef_y;
	int32_t ecef_z;
	uint32_t position_accuracy_3d;
	int32_t ecef_x_velocity;
	int32_t ecef_y_velocity;
	int32_t ecef_z_velocity;
	uint32_t speed_accuracy;
	uint16_t position_DOP;
	uint8_t res;
	uint8_t satellites;
	uint32_t res2;
} ubx_nav_solution;

typedef struct {
	uint32_t time;			  // GPS msToW
	int32_t ned_north;
	int32_t ned_east;
	int32_t ned_down;
	uint32_t speed_3d;
	uint32_t speed_2d;
	int32_t heading_2d;
	uint32_t speed_accuracy;
	uint32_t heading_accuracy;
} ubx_nav_velned;

typedef struct {
	uint8_t chn;				// Channel number, 255 for SVx not assigned to channel
	uint8_t svid;			   // Satellite ID
	uint8_t flags;			  // Bitmask
	uint8_t quality;			// Bitfield
	uint8_t cno;				// Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
	uint8_t elev;			   // Elevation in integer degrees
	int16_t azim;			   // Azimuth in integer degrees
	int32_t prRes;			  // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;

typedef struct {
	uint32_t time;			  // GPS Millisecond time of week
	uint8_t numCh;			  // Number of channels
	uint8_t globalFlags;		// Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
	uint16_t reserved2;		 // Reserved
	ubx_nav_svinfo_channel channel[16];		 // 16 satellites * 12 byte
} ubx_nav_svinfo;

enum {
	PREAMBLE1 = 0xb5,
	PREAMBLE2 = 0x62,
	CLASS_NAV = 0x01,
	CLASS_ACK = 0x05,
	CLASS_CFG = 0x06,
	MSG_ACK_NACK = 0x00,
	MSG_ACK_ACK = 0x01,
	MSG_POSLLH = 0x2,
	MSG_STATUS = 0x3,
	MSG_SOL = 0x6,
	MSG_VELNED = 0x12,
	MSG_SVINFO = 0x30,
	MSG_CFG_PRT = 0x00,
	MSG_CFG_RATE = 0x08,
	MSG_CFG_SET_RATE = 0x01,
	MSG_CFG_NAV_SETTINGS = 0x24
} ubx_protocol_bytes;

enum {
	FIX_NONE = 0,
	FIX_DEAD_RECKONING = 1,
	FIX_2D = 2,
	FIX_3D = 3,
	FIX_GPS_DEAD_RECKONING = 4,
	FIX_TIME = 5
} ubs_nav_fix_type;

enum {
	NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static bool _skip_packet;
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

static bool next_fix;
static uint8_t _class;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

// Example packet sizes from UBlox u-center from a Glonass capable GPS receiver.
//15:17:55  R -> UBX NAV-STATUS,  Size  24,  'Navigation Status'
//15:17:55  R -> UBX NAV-POSLLH,  Size  36,  'Geodetic Position'
//15:17:55  R -> UBX NAV-VELNED,  Size  44,  'Velocity in WGS 84'
//15:17:55  R -> UBX NAV-CLOCK,  Size  28,  'Clock Status'
//15:17:55  R -> UBX NAV-AOPSTATUS,  Size  24,  'AOP Status'
//15:17:55  R -> UBX 03-09,  Size 208,  'Unknown'
//15:17:55  R -> UBX 03-10,  Size 336,  'Unknown'
//15:17:55  R -> UBX NAV-SOL,  Size  60,  'Navigation Solution'
//15:17:55  R -> UBX NAV,  Size 100,  'Navigation'
//15:17:55  R -> UBX NAV-SVINFO,  Size 328,  'Satellite Status and Information'

// from the UBlox6 document, the largest payout we receive i the NAV-SVINFO and the payload size
// is calculated as 8 + 12*numCh.  numCh in the case of a Glonass receiver is 28.
#define MAX_UBLOX_PAYLOAD_SIZE 344
#define UBLOX_BUFFER_SIZE MAX_UBLOX_PAYLOAD_SIZE


// Receive buffer
static union {
	ubx_nav_posllh posllh;
	ubx_nav_status status;
	ubx_nav_solution solution;
	ubx_nav_velned velned;
	ubx_nav_svinfo svinfo;
	uint8_t bytes[UBLOX_BUFFER_SIZE];
} _buffer;

/*
static void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	while (len--) {
		*ck_a += *data;
		*ck_b += *ck_a;
		data++;
	}
}
*/

static bool UBLOX_parse_gps(struct gps *self)
{
	uint32_t i;

	*self->gpsPacketLogChar = LOG_IGNORED;

	switch (_msg_id) {
	case MSG_POSLLH:
		*self->gpsPacketLogChar = LOG_UBLOX_POSLLH;
		//i2c_dataset.time				= _buffer.posllh.time;
		self->GPS_coord[LON] = _buffer.posllh.longitude;
		self->GPS_coord[LAT] = _buffer.posllh.latitude;
		self->GPS_altitude = _buffer.posllh.altitude_msl / 10 / 100;  //alt in m
		if (next_fix) {
			// TODO: gps fix 
		} else {

		}
		_new_position = true;
		break;
	case MSG_STATUS:
		*self->gpsPacketLogChar = LOG_UBLOX_STATUS;
		next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
		// TODO: gps fix
		//if (!next_fix)
		//	DISABLE_STATE(GPS_FIX);
		break;
	case MSG_SOL:
		*self->gpsPacketLogChar = LOG_UBLOX_SOL;
		next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
		// TODO: gps fix
		//if (!next_fix)
	//		DISABLE_STATE(GPS_FIX);
		self->GPS_numSat = _buffer.solution.satellites;
		self->GPS_hdop = _buffer.solution.position_DOP;
		break;
	case MSG_VELNED:
		*self->gpsPacketLogChar = LOG_UBLOX_VELNED;
		// speed_3d						= _buffer.velned.speed_3d;  // cm/s
		self->GPS_speed = _buffer.velned.speed_2d;	// cm/s
		self->GPS_ground_course = (uint16_t) (_buffer.velned.heading_2d / 10000);	 // Heading 2D deg * 100000 rescaled to deg * 10
		_new_speed = true;
		break;
	case MSG_SVINFO:
		*self->gpsPacketLogChar = LOG_UBLOX_SVINFO;
		self->GPS_numCh = _buffer.svinfo.numCh;
		if (self->GPS_numCh > 16)
			self->GPS_numCh = 16;
		for (i = 0; i < self->GPS_numCh; i++){
			self->GPS_svinfo_chn[i]= _buffer.svinfo.channel[i].chn;
			self->GPS_svinfo_svid[i]= _buffer.svinfo.channel[i].svid;
			self->GPS_svinfo_quality[i]=_buffer.svinfo.channel[i].quality;
			self->GPS_svinfo_cno[i]= _buffer.svinfo.channel[i].cno;
		}
		self->GPS_svInfoReceivedCount++;
		break;
	default:
		return false;
	}

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = false;
		return true;
	}
	return false;
}

static bool gpsNewFrameUBLOX(struct gps *self, uint8_t data)
{
	bool parsed = false;

	switch (_step) {
		case 0: // Sync char 1 (0xB5)
			if (PREAMBLE1 == data) {
				_skip_packet = false;
				_step++;
			} else {
				self->GPS_garbageByteCount++;
			}
			break;
		case 1: // Sync char 2 (0x62)
			if (PREAMBLE2 != data) {
				_step = 0;
				break;
			}
			_step++;
			break;
		case 2: // Class
			_step++;
			_class = data;
			_ck_b = _ck_a = data;   // reset the checksum accumulators
			break;
		case 3: // Id
			_step++;
			_ck_b += (_ck_a += data);	   // checksum byte
			_msg_id = data;
			break;
		case 4: // Payload length (part 1)
			_step++;
			_ck_b += (_ck_a += data);	   // checksum byte
			_payload_length = data; // payload length low byte
			break;
		case 5: // Payload length (part 2)
			_step++;
			_ck_b += (_ck_a += data);	   // checksum byte
			_payload_length |= (uint16_t)(data << 8);

			if (_payload_length > MAX_UBLOX_PAYLOAD_SIZE ) {
				// we can't receive the whole packet, just log the error and start searching for the next packet.
				shiftPacketLog(self);
				*self->gpsPacketLogChar = LOG_SKIPPED;
				self->gpsData.errors++;
				_step = 0;
				break;
			}

			if (_payload_length > UBLOX_BUFFER_SIZE) {
				_skip_packet = true;
			}

			// prepare to receive payload
			_payload_counter = 0;

			if (_payload_length == 0) {
				_step = 7;
			}
			break;
		case 6:
			_ck_b += (_ck_a += data);	   // checksum byte
			if (_payload_counter < UBLOX_BUFFER_SIZE) {
				_buffer.bytes[_payload_counter] = data;
			}
			// NOTE: check counter BEFORE increasing so that a payload_size of 65535 is correctly handled.  This can happen if garbage data is received.
			if (_payload_counter ==  _payload_length - 1) {
				_step++;
			}
			_payload_counter++;
			break;
		case 7:
			_step++;
			if (_ck_a != data) {
				_skip_packet = true;		  // bad checksum
				self->gpsData.errors++;
			}
			break;
		case 8:
			_step = 0;

			shiftPacketLog(self);

			if (_ck_b != data) {
				*self->gpsPacketLogChar = LOG_ERROR;
				self->gpsData.errors++;
				break;			  // bad checksum
			}

			self->GPS_packetCount++;

			if (_skip_packet) {
				*self->gpsPacketLogChar = LOG_SKIPPED;
				break;
			}

			if (UBLOX_parse_gps(self)) {
				parsed = true;
			}
		default:
			break;
	}
	return parsed;
}

void gps_enable_passthrough(struct gps *self, serialPort_t *gpsPassthroughPort)
{
	waitForSerialPortToFinishTransmitting(self->gpsPort);
	waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

	if(!(self->gpsPort->mode & MODE_TX))
		serialSetMode(self->gpsPort, self->gpsPort->mode | MODE_TX);

	sys_led_off(self->system, 0);
	sys_led_off(self->system, 1);

#ifdef DISPLAY
	if (feature(FEATURE_DISPLAY)) {
		// TODO: move display stuff from gps
		//displayShowFixedPage(PAGE_GPS);
	}
#endif
	char c;
	while(1) {
		if (serialRxBytesWaiting(self->gpsPort)) {
			sys_led_on(self->system, 0);
			c = serialRead(self->gpsPort);
			gpsNewData(self, c);
			serialWrite(gpsPassthroughPort, c);
			sys_led_on(self->system, 0);
		}
		if (serialRxBytesWaiting(gpsPassthroughPort)) {
			sys_led_on(self->system, 1);
			c = serialRead(gpsPassthroughPort);
			serialWrite(self->gpsPort, c);
			sys_led_on(self->system, 1);
		}
	}
}

/*
// TODO: gps indicator
void updateGpsIndicator(uint32_t currentTime)
{
	static uint32_t GPSLEDTime;
	if ((int32_t)(currentTime - GPSLEDTime) >= 0 && (self->GPS_numSat >= 5)) {
		GPSLEDTime = currentTime + 150000;
		led_toggle(1);
	}
}
*/
