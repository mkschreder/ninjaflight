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

#include <stdint.h>
#include <string.h>
#include <math.h>

//#define DEBUG_MSP

extern "C" {
    #include <platform.h>
    #include "build_config.h"
    #include "version.h"
    #include "debug.h"

    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"
    #include "common/streambuf.h"
    #include "common/utils.h"

    #include "config/parameter_group.h"
    #include "config/config_eeprom.h"

    #include "drivers/system.h"
    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/serial.h"
    #include "drivers/serial_softserial.h"
    #include "drivers/buf_writer.h"

    #include "rx/rx.h"

    #include "io/rc_controls.h"
    #include "flight/rate_profile.h"
    #include "io/rc_adjustments.h"
    #include "io/gps.h"
    #include "io/ledstrip.h"
    #include "io/msp_protocol.h"
    #include "io/transponder_ir.h"
    #include "io/serial.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky.h"

    #include "sensors/boardalignment.h"
    #include "sensors/battery.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"

    #include "flight/mixer.h"
    #include "flight/anglerate.h"
    #include "flight/navigation.h"
    #include "flight/failsafe.h"

    #include "config/parameter_group_ids.h"
    #include "config/runtime_config.h"
    #include "config/config.h"

    #include "io/msp.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    uint8_t pgMatcherForMSPSet(const pgRegistry_t *candidate, const void *criteria);
    uint8_t pgMatcherForMSP(const pgRegistry_t *candidate, const void *criteria);
}


#define MSP_BUFFER_SIZE 256
#define BARRIER "Memory barrier!"  // 15 bytes + \0

class MspTest : public ::testing::Test {
protected:
    mspPacket_t cmd, reply;
    uint8_t rbuf[MSP_BUFFER_SIZE];
    char barrier_r[16];                    // check for buffer overflow
    uint8_t sbuf[MSP_BUFFER_SIZE];
    char barrier_s[16];
    void resetReply() {
        memset(rbuf, 0xde, sizeof(rbuf));
        reply.buf.ptr = rbuf;
        reply.buf.end = ARRAYEND(rbuf);       // whole buffer available
    }
    void resetCmd() {
        memset(sbuf, 0xad, sizeof(sbuf));
        cmd.buf.ptr = sbuf;
        cmd.buf.end = sbuf;                   // command buffer is empty by default
    }
    void resetPackets() {
        resetCmd();
        resetReply();
    }
    void copyReplyDataToCmd() {
        resetCmd();                           // cleanup first
        // copy previously received data to command buffer
        memcpy(sbuf, rbuf, reply.buf.ptr - rbuf);
        cmd.buf.ptr = sbuf;
        cmd.buf.end = sbuf + (reply.buf.ptr - rbuf);
    }
    virtual void SetUp() {
        strcpy(barrier_r, BARRIER);
        strcpy(barrier_s, BARRIER);
        resetPackets();
    }
    virtual void TearDown() {
        EXPECT_STREQ(barrier_r, BARRIER);
        EXPECT_STREQ(barrier_s, BARRIER);
    }
};

TEST_F(MspTest, TestMsp_API_VERSION)
{
    cmd.cmd = MSP_API_VERSION;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(3, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_API_VERSION, reply.cmd);
    EXPECT_EQ(MSP_PROTOCOL_VERSION, rbuf[0]);
    EXPECT_EQ(API_VERSION_MAJOR, rbuf[1]);
    EXPECT_EQ(API_VERSION_MINOR, rbuf[2]);
}

TEST_F(MspTest, TestMsp_FC_VARIANT)
{
    cmd.cmd = MSP_FC_VARIANT;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(FLIGHT_CONTROLLER_IDENTIFIER_LENGTH, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_FC_VARIANT, reply.cmd);
    EXPECT_EQ('C', rbuf[0]);
    EXPECT_EQ('L', rbuf[1]);
    EXPECT_EQ('F', rbuf[2]);
    EXPECT_EQ('L', rbuf[3]);
}

TEST_F(MspTest, TestMsp_FC_VERSION)
{
    cmd.cmd = MSP_FC_VERSION;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(FLIGHT_CONTROLLER_VERSION_LENGTH, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_FC_VERSION, reply.cmd);
    EXPECT_EQ(FC_VERSION_MAJOR, rbuf[0]);
    EXPECT_EQ(FC_VERSION_MINOR, rbuf[1]);
    EXPECT_EQ(FC_VERSION_PATCH_LEVEL, rbuf[2]);
}

TEST_F(MspTest, TestMsp_PID_CONTROLLER)
{
    pgActivateProfile(0);
    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;

    cmd.cmd = MSP_PID_CONTROLLER;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(1, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_PID_CONTROLLER, reply.cmd);
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, rbuf[0]);
}

TEST_F(MspTest, TestMsp_SET_PID_CONTROLLER)
{
    // set the pidController to a different value so we can check if it gets read back properly
    pidProfile()->pidController = PID_CONTROLLER_LUX_FLOAT;

    cmd.cmd = MSP_SET_PID_CONTROLLER;
    *cmd.buf.end++ = PID_CONTROLLER_MWREWRITE;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, pidProfile()->pidController);
}

TEST_F(MspTest, TestMsp_PID)
{
    // check the buffer is big enough for the data to read in
    EXPECT_LE(3 * PID_ITEM_COUNT, MSP_BUFFER_SIZE);
    // set up some test data
    const int P8_ROLL = 40;
    const int I8_ROLL = 30;
    const int D8_ROLL = 23;
    const int P8_PITCH = 41;
    const int I8_PITCH = 31;
    const int D8_PITCH = 24;
    const int P8_YAW = 85;
    const int I8_YAW = 45;
    const int D8_YAW = 1;
    const int P8_PIDALT = 50;
    const int I8_PIDALT = 2;
    const int D8_PIDALT = 3;
    const int P8_PIDPOS = 15; // POSHOLD_P * 100;
    const int I8_PIDPOS = 4; // POSHOLD_I * 100;
    const int D8_PIDPOS = 5;
    const int P8_PIDPOSR = 34; // POSHOLD_RATE_P * 10;
    const int I8_PIDPOSR = 14; // POSHOLD_RATE_I * 100;
    const int D8_PIDPOSR = 53; // POSHOLD_RATE_D * 1000;
    const int P8_PIDNAVR = 25; // NAV_P * 10;
    const int I8_PIDNAVR = 33; // NAV_I * 100;
    const int D8_PIDNAVR = 83; // NAV_D * 1000;
    const int P8_PIDLEVEL = 90;
    const int I8_PIDLEVEL = 10;
    const int D8_PIDLEVEL = 100;
    const int P8_PIDMAG = 40;
    const int P8_PIDVEL = 120;
    const int I8_PIDVEL = 45;
    const int D8_PIDVEL = 7;

    pgActivateProfile(0);

    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;
    pidProfile()->P8[PIDROLL] = P8_ROLL;
    pidProfile()->I8[PIDROLL] = I8_ROLL;
    pidProfile()->D8[PIDROLL] = D8_ROLL;
    pidProfile()->P8[PIDPITCH] = P8_PITCH;
    pidProfile()->I8[PIDPITCH] = I8_PITCH;
    pidProfile()->D8[PIDPITCH] = D8_PITCH;
    pidProfile()->P8[PIDYAW] = P8_YAW;
    pidProfile()->I8[PIDYAW] = I8_YAW;
    pidProfile()->D8[PIDYAW] = D8_YAW;
    pidProfile()->P8[PIDALT] = P8_PIDALT;
    pidProfile()->I8[PIDALT] = I8_PIDALT;
    pidProfile()->D8[PIDALT] = D8_PIDALT;
    pidProfile()->P8[PIDPOS] = P8_PIDPOS;
    pidProfile()->I8[PIDPOS] = I8_PIDPOS;
    pidProfile()->D8[PIDPOS] = D8_PIDPOS;
    pidProfile()->P8[PIDPOSR] = P8_PIDPOSR;
    pidProfile()->I8[PIDPOSR] = I8_PIDPOSR;
    pidProfile()->D8[PIDPOSR] = D8_PIDPOSR;
    pidProfile()->P8[PIDNAVR] = P8_PIDNAVR;
    pidProfile()->I8[PIDNAVR] = I8_PIDNAVR;
    pidProfile()->D8[PIDNAVR] = D8_PIDNAVR;
    pidProfile()->P8[PIDLEVEL] = P8_PIDLEVEL;
    pidProfile()->I8[PIDLEVEL] = I8_PIDLEVEL;
    pidProfile()->D8[PIDLEVEL] = D8_PIDLEVEL;
    pidProfile()->P8[PIDMAG] = P8_PIDMAG;
    pidProfile()->P8[PIDVEL] = P8_PIDVEL;
    pidProfile()->I8[PIDVEL] = I8_PIDVEL;
    pidProfile()->D8[PIDVEL] = D8_PIDVEL;

    // use the MSP to write out the PID values
    cmd.cmd = MSP_PID;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(3 * PID_ITEM_COUNT, reply.buf.ptr - rbuf) << "Reply size";
    // check few values, just to make sure they have been written correctly
    EXPECT_EQ(P8_YAW, rbuf[6]);
    EXPECT_EQ(I8_YAW, rbuf[7]);
    EXPECT_EQ(D8_YAW, rbuf[8]);

    // reset test values to zero, so we can check if they get read properly
    memset(pidProfile(), 0, sizeof(*pidProfile()));

    // now use the MSP to set the PID values and check they are the same as written
    cmd.cmd = MSP_SET_PID;
    copyReplyDataToCmd();
    resetReply();

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    // check the values are as expected
    EXPECT_EQ(P8_ROLL, pidProfile()->P8[PIDROLL]);
    EXPECT_EQ(I8_ROLL, pidProfile()->I8[PIDROLL]);
    EXPECT_EQ(D8_ROLL, pidProfile()->D8[PIDROLL]);
    EXPECT_EQ(P8_PITCH, pidProfile()->P8[PIDPITCH]);
    EXPECT_EQ(I8_PITCH, pidProfile()->I8[PIDPITCH]);
    EXPECT_EQ(D8_PITCH, pidProfile()->D8[PIDPITCH]);
    EXPECT_EQ(P8_YAW, pidProfile()->P8[PIDYAW]);
    EXPECT_EQ(I8_YAW, pidProfile()->I8[PIDYAW]);
    EXPECT_EQ(D8_YAW, pidProfile()->D8[PIDYAW]);
    EXPECT_EQ(P8_PIDALT, pidProfile()->P8[PIDALT]);
    EXPECT_EQ(I8_PIDALT, pidProfile()->I8[PIDALT]);
    EXPECT_EQ(D8_PIDALT, pidProfile()->D8[PIDALT]);
    EXPECT_EQ(P8_PIDPOS, pidProfile()->P8[PIDPOS]);
    EXPECT_EQ(I8_PIDPOS, pidProfile()->I8[PIDPOS]);
    EXPECT_EQ(D8_PIDPOS, pidProfile()->D8[PIDPOS]);
    EXPECT_EQ(P8_PIDPOSR, pidProfile()->P8[PIDPOSR]);
    EXPECT_EQ(I8_PIDPOSR, pidProfile()->I8[PIDPOSR]);
    EXPECT_EQ(D8_PIDPOSR, pidProfile()->D8[PIDPOSR]);
    EXPECT_EQ(P8_PIDNAVR, pidProfile()->P8[PIDNAVR]);
    EXPECT_EQ(I8_PIDNAVR, pidProfile()->I8[PIDNAVR]);
    EXPECT_EQ(D8_PIDNAVR, pidProfile()->D8[PIDNAVR]);
    EXPECT_EQ(P8_PIDLEVEL, pidProfile()->P8[PIDLEVEL]);
    EXPECT_EQ(I8_PIDLEVEL, pidProfile()->I8[PIDLEVEL]);
    EXPECT_EQ(D8_PIDLEVEL, pidProfile()->D8[PIDLEVEL]);
    EXPECT_EQ(P8_PIDMAG, pidProfile()->P8[PIDMAG]);
    EXPECT_EQ(P8_PIDVEL, pidProfile()->P8[PIDVEL]);
    EXPECT_EQ(I8_PIDVEL, pidProfile()->I8[PIDVEL]);
    EXPECT_EQ(D8_PIDVEL, pidProfile()->D8[PIDVEL]);
}


TEST_F(MspTest, TestParameterGroup_BOARD_ALIGNMENT)
{
    // check that pgRegistry mapping is setup correctly
    const pgRegistry_t *reg = pgMatcher(pgMatcherForMSP, (void*)(intptr_t)MSP_BOARD_ALIGNMENT);
    EXPECT_NE(static_cast<const pgRegistry_t*>(0), reg);
    EXPECT_EQ(reinterpret_cast<struct board_alignment_config*>(reg->address), boardAlignment());

    const struct board_alignment_config testBoardAlignment = {295, 147, -202};

    *boardAlignment() = testBoardAlignment;
    // use the MSP to write out the test values

    cmd.cmd = MSP_BOARD_ALIGNMENT;

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    EXPECT_EQ(sizeof(struct board_alignment_config), reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_BOARD_ALIGNMENT, reply.cmd);
    EXPECT_EQ(testBoardAlignment.rollDegrees & 0xff, rbuf[0]);
    EXPECT_EQ(testBoardAlignment.rollDegrees >> 8, rbuf[1]);

    // reset test values to zero, so we can check if they get read properly
    memset(boardAlignment(), 0, sizeof(*boardAlignment()));

    // check that pgRegistry mapping is setup correctly
    const pgRegistry_t *regSet = pgMatcher(pgMatcherForMSPSet, (void*)(intptr_t)MSP_SET_BOARD_ALIGNMENT);
    EXPECT_NE(static_cast<const pgRegistry_t*>(0), regSet);
    EXPECT_EQ(reinterpret_cast<struct board_alignment_config*>(regSet->address), boardAlignment());

    // now use the MSP to set the values and check they are the same
    cmd.cmd = MSP_SET_BOARD_ALIGNMENT;
    copyReplyDataToCmd();
    resetReply();

    EXPECT_GT(mspProcess(&cmd, &reply), 0);

    // check the values are as expected
    EXPECT_FLOAT_EQ(testBoardAlignment.rollDegrees, boardAlignment()->rollDegrees);
    EXPECT_FLOAT_EQ(testBoardAlignment.pitchDegrees, boardAlignment()->pitchDegrees);
    EXPECT_FLOAT_EQ(testBoardAlignment.yawDegrees, boardAlignment()->yawDegrees);
}

TEST_F(MspTest, TestMspCommands)
{

    static const uint8_t outMessages[] = {
        MSP_API_VERSION,                // 1    //out message
        MSP_FC_VARIANT,                 // 2    //out message
        MSP_FC_VERSION,                 // 3    //out message
        MSP_BOARD_INFO,                 // 4    //out message
        MSP_BUILD_INFO,                 // 5    //out message
        // MSP commands for Ninjaflight original features
        MSP_MODE_RANGES,                // 34    //out message         Returns all mode ranges
        MSP_FEATURE,                    // 36
        MSP_BOARD_ALIGNMENT,            // 38
        MSP_CURRENT_METER_CONFIG,       // 40
        MSP_MIXER,                      // 42
        MSP_RX_CONFIG,                  // 44
        MSP_LED_COLORS,                 // 46
        MSP_LED_STRIP_CONFIG,           // 48
        MSP_RSSI_CONFIG,                // 50
        MSP_ADJUSTMENT_RANGES,          // 52
        // private - only to be used by the configurator, the commands are likely to change
//!! not tested        MSP_CF_SERIAL_CONFIG,           // 54
        MSP_VOLTAGE_METER_CONFIG,       // 56
        MSP_SONAR_ALTITUDE,             // 58 //out message get sonar altitude [cm]
        MSP_PID_CONTROLLER,             // 59
        MSP_ARMING_CONFIG,              // 61 //out message         Returns auto_disarm_delay and disarm_kill_switch parameters
        MSP_DATAFLASH_SUMMARY,          // 70 //out message - get description of dataflash chip
//!! not tested       MSP_DATAFLASH_READ,             // 71 //out message - get content of dataflash chip
        MSP_LOOP_TIME,                  // 73 //out message         Returns FC cycle time i.e looptime parameter
        MSP_FAILSAFE_CONFIG,            // 75 //out message         Returns FC Fail-Safe settings
        MSP_RXFAIL_CONFIG,              // 77 //out message         Returns RXFAIL settings
        MSP_SDCARD_SUMMARY,             // 79 //out message         Get the state of the SD card
        MSP_BLACKBOX_CONFIG,            // 80 //out message         Get blackbox settings
        MSP_TRANSPONDER_CONFIG,         // 82 //out message         Get transponder settings
        MSP_LED_STRIP_MODECOLOR,        // 85 //out message         Get LED strip mode_color settings
        MSP_SET_LED_STRIP_MODECOLOR,    // 86 //out message         Set LED strip mode_color settings
        // Baseflight MSP commands (if enabled they exist in Ninjaflight)
        MSP_RX_MAP,                     // 64 //out message get channel map (also returns number of channels total)
        // DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Ninjaflight, isolated commands already exist and should be used instead.
        MSP_BF_CONFIG,                  // 66 //out message baseflight-specific settings that aren't covered elsewhere
        // DEPRECATED - Use MSP_BUILD_INFO instead
        MSP_BF_BUILD_INFO,              // 69 //out message build date as well as some space for future expansion
        // Multwii original MSP commands
        // DEPRECATED - See MSP_API_VERSION and MSP_MIXER
        MSP_IDENT,               // 100    //out message         mixerMode + multiwii version + protocol version + capability variable
        MSP_STATUS,              // 101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
        MSP_RAW_IMU,             // 102    //out message         9 DOF
        MSP_SERVO,               // 103    //out message         servos
        MSP_MOTOR,               // 104    //out message         motors
        MSP_RC,                  // 105    //out message         rc channels and more
        MSP_RAW_GPS,             // 106    //out message         fix, numsat, lat, lon, alt, speed, ground course
        MSP_COMP_GPS,            // 107    //out message         distance home, direction home
        MSP_ATTITUDE,            // 108    //out message         2 angles 1 heading
        MSP_ALTITUDE,            // 109    //out message         altitude, variometer
        MSP_ANALOG,              // 110    //out message         vbat, powermetersum, rssi if available on RX
        MSP_RC_TUNING,           // 111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
        MSP_PID,                 // 112    //out message         P I D coeff (9 are used currently)
//!! not implemented in serial_msp.c      MSP_BOX,                 // 113    //out message         BOX setup (number is dependant of your setup)
        MSP_MISC,                // 114    //out message         powermeter trig
        MSP_MOTOR_PINS,          // 115    //out message         which pins are in use for motors & servos, for GUI
        MSP_BOXNAMES,            // 116    //out message         the aux switch names
        MSP_PIDNAMES,            // 117    //out message         the PID names
        MSP_WP,                  // 118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
        MSP_BOXIDS,              // 119    //out message         get the permanent IDs associated to BOXes
        MSP_SERVO_CONFIGURATIONS,// 120    //out message         All servo configurations.
//!! not implemented in serial_msp.c       MSP_NAV_STATUS,          // 121    //out message         Returns navigation status
//!! not implemented in serial_msp.c       MSP_NAV_CONFIG,          // 122    //out message         Returns navigation parameters
        MSP_3D,                  // 124    //out message         Settings needed for reversible ESCs
        MSP_RC_DEADBAND,         // 125    //out message         deadbands for yaw alt pitch roll
        MSP_SENSOR_ALIGNMENT,    // 126    //out message         orientation of acc,gyro,mag
//!! not implemented in serial_msp.c       MSP_DEBUGMSG,            // 253    //out message         debug string buffer
        MSP_DEBUG,               // 254    //out message         debug1,debug2,debug3,debug4
        // Additional commands that are not compatible with MultiWii
        MSP_STATUS_EX,           // 150    //out message         cycletime, errors_count, CPU load, sensor present etc
        MSP_UID,                 // 160    //out message         Unique device ID
        MSP_GPSSVINFO,           // 164    //out message         get Signal Strength (only U-Blox)
        MSP_ACC_TRIM,            // 240    //out message         get acc angle trim values
        MSP_SERVO_MIX_RULES,     // 241    //out message         Returns servo mixer configuration
    };
    for (uint ii = 0; ii < ARRAYLEN(outMessages); ii++) {
        resetPackets();
        cmd.cmd = outMessages[ii];

        EXPECT_GT(mspProcess(&cmd, &reply), 0);

        EXPECT_EQ(outMessages[ii], reply.cmd) << "Command index " << ii;
        EXPECT_LT(0, reply.result) << "Command index " << ii;
    }
}

// STUBS
extern "C" {
#include "ninja.h"
struct ninja ninja;
// from acceleration.c
acc_t acc;                       // acc access functions
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired) {UNUSED(calibrationCyclesRequired);}
// from altitudehold.c
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s
int32_t altitudeHoldGetEstimatedAltitude(void) {return 0;}
// from battery.c
uint16_t vbat = 0;                   // battery voltage in 0.1V steps (filtered)
int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start
// from compass.c
int32_t magADC[XYZ_AXIS_COUNT];
// from config.c
struct rate_config controlRateProfile;
struct rate_config *currentControlRateProfile = &controlRateProfile;
void resetPidProfile(struct pid_config *pidProfile) {UNUSED(pidProfile);}
void handleOneshotFeatureChangeOnRestart(void) {}
void readEEPROM(void) {}
void resetEEPROM(void) {}
void writeEEPROM(void) {}
void changeProfile(uint8_t) {};
void setProfile(uint8_t) {};
uint8_t getCurrentProfile(void) { return 0; };
bool feature(uint32_t mask) {UNUSED(mask);return false;}
void featureSet(uint32_t mask) {UNUSED(mask);}
void featureClearAll() {}
uint32_t featureMask(void) {return 0;}
// from debug.c
int16_t debug[DEBUG16_VALUE_COUNT];
// from gps.c
#define GPS_SV_MAXSATS   16
int32_t GPS_coord[2];               // LAT/LON
uint8_t GPS_numSat;
uint8_t GPS_update = 0;             // it's a binary toggle to distinct a GPS position update
uint16_t GPS_altitude;              // altitude in 0.1m
uint16_t GPS_speed;                 // speed in 0.1m/s
uint16_t GPS_ground_course = 0;     // degrees * 10
uint8_t GPS_numCh;                          // Number of channels
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];    // Satellite ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];     // Carrier to Noise Ratio (Signal Strength)
// from gyro.c
int32_t gyroADC[XYZ_AXIS_COUNT];
// from ledstrip.c
void reevalulateLedConfig(void) {}
bool setModeColor(ledModeIndex_e , int , int ) { return true; }
// from mixer.c
int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
int16_t servo[MAX_SUPPORTED_SERVOS];
struct mixer default_mixer; 
uint16_t mixer_get_motor_value(struct mixer *self, uint8_t id){ (void)self; (void)id; return 1000; }
uint16_t mixer_get_servo_value(struct mixer *self, uint8_t id){ (void)self; (void)id; return 1000; }
int16_t mixer_get_output_disarmed_pwm(struct mixer *self, uint8_t id){ (void)self; (void)id; return 900; }
void mixer_input_command(struct mixer *self, mixer_input_t id, int16_t value){ (void)self; (void)id; (void)value; }
void stopMotors(struct mixer *self) {(void)self;}
void loadCustomServoMixer(void) {}
// from msp.c
void rxMspFrameReceive(uint16_t *, int ) {}
// from mw.c
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
// from navigation.c
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
navigationMode_e navi_mode = NAV_MODE_NONE;    // Navigation mode
void GPS_set_next_wp(int32_t *, int32_t *) {}
// from pid.c
struct anglerate default_controller; 
void anglerate_set_algo(struct anglerate *self, pid_controller_type_t algo){ UNUSED(self); UNUSED(algo); }
struct battery default_battery;
struct instruments default_ins;
// from rc_controls.c
uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e
bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
void useRcControlsConfig(modeActivationCondition_t *) {};
void ins_start_acc_calibration(struct instruments *self) { (void)self; }
// from runtime_config.c
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint16_t disableFlightMode(flightModeFlags_e) { return 0; }
bool sensors(uint32_t mask) {UNUSED(mask);return 0;}
// from rx.c
uint16_t rssi = 0;                  // range: [0;1023]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
// TODO: proper way to do this is to write a mock receiver
int16_t rc_get_channel_value(uint8_t id){ return rcData[id]; }
void rc_set_channel_value(uint8_t id, int16_t value){ rcData[id] = value; }
uint16_t rc_get_rssi(void){ return rssi; }
rxRuntimeConfig_t rxRuntimeConfig;
void ninja_calibrate_acc(){ }
void ninja_calibrate_mag() {}
// from system_stm32fN0x.c
void systemReset(void) {}
void systemResetToBootloader(void) {}
// from scheduler.c
uint16_t averageSystemLoadPercent = 0;
// from transponder_ir.c
void transponderUpdateData(uint8_t*) {}
// from serial port drivers
serialPort_t *usbVcpOpen(void) { return NULL; }
serialPort_t *uartOpen(USART_TypeDef *, serialReceiveCallbackPtr, uint32_t, portMode_t, portOptions_t) { return NULL; }
serialPort_t *openSoftSerial(softSerialPortIndex_e, serialReceiveCallbackPtr, uint32_t, portOptions_t) { return NULL; }
void serialSetMode(serialPort_t *, portMode_t) {}

void imu_get_raw_accel(struct imu *self, union imu_accel_reading *acc){
	acc->values.x = self->accSmooth[X]; 
	acc->values.y = self->accSmooth[Y]; 
	acc->values.z = self->accSmooth[Z]; 
}

int16_t imu_get_roll_dd(struct imu *self){
	return self->attitude.values.roll; 
}

int16_t imu_get_pitch_dd(struct imu *self){
	return self->attitude.values.pitch; 
}

int16_t imu_get_yaw_dd(struct imu *self){
	return self->attitude.values.yaw; 
}

uint16_t battery_get_voltage(struct battery *self){
	return self->vbat;
}

uint8_t battery_get_cell_count(struct battery *self){
	return self->batteryCellCount;
}

int32_t battery_get_current(struct battery *self){
	return self->amperage;
}

int32_t battery_get_spent_capacity(struct battery *self){
	return self->mAhDrawn;
}

uint16_t battery_get_cell_voltage(struct battery *self){
	return ((uint32_t)self->vbat * 100 + self->batteryCellCount) / (self->batteryCellCount * 2);
}


void mspSerialProcess() {}
bool isSerialTransmitBufferEmpty(serialPort_t *) { return true; }
}

