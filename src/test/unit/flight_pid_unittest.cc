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
#include <math.h>

// TODO: pid unit test
extern "C" {
    #include "build_config.h"
    #include "debug.h"
	#include "target.h"

    #include "common/axis.h"
    #include "common/maths.h"

	#include "config/config.h"
	#include "config/serial.h"
	#include "config/gps.h"
	#include "config/navigation.h"
	#include "config/rate_profile.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    #include "rx/rx.h"

    #include "flight/rate_profile.h"
    #include "flight/anglerate.h"
	#include "flight/mixer.h"

    struct pid_config testPidProfile;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    extern uint32_t rcModeActivationMask;
    float dt = 0.1; // dT for pidLuxFloat
    int32_t targetLooptime; // targetLooptime for pidMultiWiiRewrite
	int32_t gyro_sync_get_looptime(void){ return targetLooptime; }
    float unittest_pidLuxFloatCore_lastRateForDelta[3];
    int32_t unittest_pidLuxFloatCore_deltaState[3][DTERM_AVERAGE_COUNT];
    int32_t unittest_pidMultiWiiRewriteCore_lastRateForDelta[3];
    int32_t unittest_pidMultiWiiRewriteCore_deltaState[3][DTERM_AVERAGE_COUNT];

	struct mixer default_mixer; 
	uint8_t mixer_get_motor_count(struct mixer *self) { (void)self; return 4; }; 
	bool mixer_motor_limit_reached(struct mixer *self) { (void)self; return false; }; 

	struct instruments default_ins;
	struct anglerate default_controller;
}

static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale
static const int mwrGyroScaleNum = 4;
static const int mwrGyroScaleDenom = 1;
#define TARGET_LOOPTIME 2048

static const int DTermAverageCount = 4;

static struct config config;

void resetPidProfile(){
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;

    pid->pidController = PID_CONTROLLER_MWREWRITE;

    pid->P8[PIDROLL] = 40;
    pid->I8[PIDROLL] = 30;
    pid->D8[PIDROLL] = 10;
    pid->P8[PIDPITCH] = 40;
    pid->I8[PIDPITCH] = 30;
    pid->D8[PIDPITCH] = 10;
    pid->P8[PIDYAW] = 85;
    pid->I8[PIDYAW] = 45;
    pid->D8[PIDYAW] = 10;
    pid->P8[PIDALT] = 50;
    pid->I8[PIDALT] = 0;
    pid->D8[PIDALT] = 0;
    pid->P8[PIDPOS] = 15; // POSHOLD_P * 100;
    pid->I8[PIDPOS] = 0; // POSHOLD_I * 100;
    pid->D8[PIDPOS] = 0;
    pid->P8[PIDPOSR] = 34; // POSHOLD_RATE_P * 10;
    pid->I8[PIDPOSR] = 14; // POSHOLD_RATE_I * 100;
    pid->D8[PIDPOSR] = 53; // POSHOLD_RATE_D * 1000;
    pid->P8[PIDNAVR] = 25; // NAV_P * 10;
    pid->I8[PIDNAVR] = 33; // NAV_I * 100;
    pid->D8[PIDNAVR] = 83; // NAV_D * 1000;
    pid->P8[PIDLEVEL] = 90;
    pid->I8[PIDLEVEL] = 10;
    pid->D8[PIDLEVEL] = 100;
    pid->P8[PIDMAG] = 40;
    pid->P8[PIDVEL] = 120;
    pid->I8[PIDVEL] = 45;
    pid->D8[PIDVEL] = 1;

    pid->yaw_p_limit = YAW_P_LIMIT_MAX;
    pid->dterm_cut_hz = 0;
}

void resetPID(struct rate_config *rates){
	(void)rates;
	gyro.scale = 1.0f/16.4f;
	
	config_reset(&config);

	ins_init(&default_ins, &config);
	
	ins_set_gyro_scale(&default_ins, gyro.scale);
	ins_set_acc_scale(&default_ins, 512);

	anglerate_init(&default_controller, &default_ins, 0, &config);
}

void pidControllerInitLuxFloatCore(void)
{
    anglerate_set_algo(&default_controller, PID_CONTROLLER_LUX_FLOAT);
    resetPidProfile();
    anglerate_reset_angle_i(&default_controller);
    anglerate_reset_rate_i(&default_controller);
    targetLooptime = TARGET_LOOPTIME;
    dt = TARGET_LOOPTIME * 0.000001f;

    gyro.scale = 1.0 / 16.4; // value for 6050 family of gyros
    resetGyroADC();
    // set up the PIDWeights to 100%, so they are neutral in the tests
    default_controller.PIDweight[FD_ROLL] = 100;
    default_controller.PIDweight[FD_PITCH] = 100;
    default_controller.PIDweight[FD_YAW] = 100;
    // reset the pidLuxFloat static values
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        unittest_pidLuxFloatCore_lastRateForDelta[axis] = 0.0f;
        for (int ii = 0; ii < DTERM_AVERAGE_COUNT; ++ii) { \
            unittest_pidLuxFloatCore_deltaState[axis][ii] = 0.0f; \
        } \
    }
	
}

void pidControllerInitLuxFloat(struct rate_config *controlRate, uint16_t max_angle_inclination, rollAndPitchTrims_t *rollAndPitchTrims, rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(rxConfig);

	rollAndPitchTrims->values.roll = 0;
	rollAndPitchTrims->values.pitch = 0;
    // set up the control rates for calculation of rate error
    controlRate->rates[ROLL] = 173;
    controlRate->rates[PITCH] = 173;
    controlRate->rates[YAW] = 173;

	resetPID(controlRate, rollAndPitchTrims);
    pidControllerInitLuxFloatCore();

	testPidProfile.pidController = PID_CONTROLLER_LUX_FLOAT;
}

/*
 * calculate the value of rcCommand[ROLL] required to give a desired rateError
 */
int16_t calcLuxRcCommandRoll(float rateError, const struct rate_config *controlRate) {
    return 16.0f * rateError / ((float)controlRate->rates[ROLL] + 27.0f);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRateRoll(const struct rate_config *controlRate) {
    return ((controlRate->rates[ROLL] + 27.0f) * rcCommand[ROLL]) / 16.0f;
}

/*
 * calculate the value of rcCommand[PITCH] required to give a desired rateError
 */
int16_t calcLuxRcCommandPitch(float rateError, const struct rate_config *controlRate) {
    return 16.0f * rateError / (controlRate->rates[PITCH] + 27.0f);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRatePitch(const struct rate_config *controlRate) {
    return ((controlRate->rates[PITCH] + 27.0f) * rcCommand[PITCH]) / 16.0f;
}

/*
 * calculate the value of rcCommand[YAW] required to give a desired rateError
 */
int16_t calcLuxRcCommandYaw(float rateError, const struct rate_config *controlRate) {
    return 32.0f * rateError / (controlRate->rates[YAW] + 27.0f);
}

/*
 * calculate the angleRate as done in pidLuxFloat, used for cross checking
 */
float calcLuxAngleRateYaw(const struct rate_config *controlRate) {
    return ((controlRate->rates[YAW] + 27.0f) * rcCommand[YAW]) / 32.0f;
}

float calcLuxPTerm(struct pid_config *pidProfile, flight_dynamics_index_t axis, float rateError) {
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    return luxPTermScale * rateError * pid->P8[axis];
}

float calcLuxITermDelta(struct pid_config *pidProfile, flight_dynamics_index_t axis, float rateError) {
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    float ret = luxITermScale * rateError * dt * pid->I8[axis];
    ret = constrainf(ret, -PID_MAX_I, PID_MAX_I);
    return ret;
}

float calcLuxDTerm(struct pid_config *pidProfile, flight_dynamics_index_t axis, float rateError) {
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    float ret = luxDTermScale * rateError * pid->D8[axis] / dt;
    ret = constrainf(ret, -PID_MAX_D, PID_MAX_D);
    return ret;

}

void update_anglerate(void){
	ins_update(&default_ins, dt);

	anglerate_input_user(&default_controller, rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW]);
	anglerate_input_body_rates(&default_controller, gyr[0], gyr[1], gyr[2]);
	anglerate_input_body_angles(&default_controller, ins_get_roll_dd(&default_ins), ins_get_pitch_dd(&default_ins), ins_get_yaw_dd(&default_ins));
	anglerate_update(&default_controller, dt);
}

TEST(PIDUnittest, TestUserInput){

}

TEST(PIDUnittest, TestPidLuxFloat)
{
    struct rate_config controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    struct pid_config *pidProfile = &testPidProfile;
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of zero on all axes
    resetRcCommands();
    resetGyroADC();

	// TODO: unit testing should focus on testing each term by itself. Remove this kinf of crap.
    EXPECT_EQ(0, default_controller.output.axis_P[FD_ROLL]);
    EXPECT_EQ(0, default_controller.output.axis_I[FD_ROLL]);
    EXPECT_EQ(0, default_controller.output.axis_D[FD_ROLL]);
    EXPECT_EQ(0, default_controller.output.axis_P[FD_PITCH]);
    EXPECT_EQ(0, default_controller.output.axis_I[FD_PITCH]);
    EXPECT_EQ(0, default_controller.output.axis_D[FD_PITCH]);
    EXPECT_EQ(0, default_controller.output.axis_P[FD_YAW]);
    EXPECT_EQ(0, default_controller.output.axis_I[FD_YAW]);
    EXPECT_EQ(0, default_controller.output.axis_D[FD_YAW]);

    // set up a rateError of 100 on the roll axis
    const float rateErrorRoll = 100;

    // set up a rateError of 100 on the pitch axis
    const float rateErrorPitch = 100;

    // set up a rateError of 100 on the yaw axis
    const float rateErrorYaw = 100;

    // run the PID controller. Check expected PID values
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
    gyr[PITCH] = -rateErrorPitch / (luxGyroScale * gyro.scale);
    gyr[YAW] = -rateErrorYaw / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
    float ITermRoll = calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    float ITermPitch = calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
    float ITermYaw = calcLuxITermDelta(pidProfile, FD_YAW, rateErrorYaw);
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, default_controller.output.axis_I[FD_ROLL]);
    float expectedDTerm = calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), default_controller.output.axis_P[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, default_controller.output.axis_I[FD_PITCH]);
    expectedDTerm = calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_PITCH]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_YAW, rateErrorYaw), default_controller.output.axis_P[FD_YAW]);
    EXPECT_FLOAT_EQ(ITermYaw, default_controller.output.axis_I[FD_YAW]);
    expectedDTerm = calcLuxDTerm(pidProfile, FD_YAW, rateErrorYaw) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_YAW]);

    // run the PID controller a second time.
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over DTermAverageCount
    ITermRoll += calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, default_controller.output.axis_I[FD_ROLL]);
    expectedDTerm = DTermAverageCount < 2 ? 0 : calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), default_controller.output.axis_P[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, default_controller.output.axis_I[FD_PITCH]);
    expectedDTerm = DTermAverageCount < 2 ? 0 : calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_PITCH]);

    // run the PID controller a third time. Error rates unchanged, so expect P and D unchanged, I integrated
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over DTermAverageCount
    ITermRoll += calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, default_controller.output.axis_I[FD_ROLL]);
    expectedDTerm = DTermAverageCount < 3 ? 0 : calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), default_controller.output.axis_P[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, default_controller.output.axis_I[FD_PITCH]);
    expectedDTerm = DTermAverageCount < 3 ? 0 : calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_PITCH]);

    // run the PID controller a fourth time.
    // Error rates unchanged, so expect P unchanged, I integrated and D averaged over DTermAverageCount
    ITermRoll += calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
    ITermPitch += calcLuxITermDelta(pidProfile, FD_PITCH, rateErrorPitch);
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRoll), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, default_controller.output.axis_I[FD_ROLL]);
    expectedDTerm = DTermAverageCount < 4 ? 0 : calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_ROLL]);
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_PITCH, rateErrorPitch), default_controller.output.axis_P[FD_PITCH]);
    EXPECT_FLOAT_EQ(ITermPitch, default_controller.output.axis_I[FD_PITCH]);
    expectedDTerm = DTermAverageCount < 4 ? 0 : calcLuxDTerm(pidProfile, FD_PITCH, rateErrorPitch) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTerm, default_controller.output.axis_D[FD_PITCH]);
}

TEST(PIDUnittest, TestPidLuxFloatIntegrationForLinearFunction)
{
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    struct rate_config controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    struct pid_config *pidProfile = &testPidProfile;

    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dt = 0.1f; // set large dT so constraints on PID values are not invoked
    EXPECT_FLOAT_EQ(0.1f, dt);

    // Test PID integration for a linear function:
    //    rateError = k * t
    // Integral:
    //    IrateError = 0.5 * k * t ^ 2
    // dt = 0.1s, t ranges from 0.0 to 1.0 in steps of 0.1s

    const float k = 1000; // arbitrary value of k
    float t = 0.0f;
    // set rateError to k * t
    gyr[ROLL] = -k * t  / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
	update_anglerate();
    float pidITerm = default_controller.output.axis_I[FD_ROLL]; // integral as estimated by PID
    float actITerm = 0.5 * k * t * t * pid->I8[ROLL] * luxITermScale; // actual value of integral
    EXPECT_FLOAT_EQ(actITerm, pidITerm); // both are zero at this point
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    resetRcCommands();
    dt = 0.01f; // set large dT so constraints on PID values are not invoked

    for (int ii = 0; ii < 10; ++ii) {
        const float actITermPrev = actITerm;
        const float pidITermPrev = pidITerm;
        t += dt;
        // set rateError to k * t
        gyr[ROLL] = -k * t / (luxGyroScale * gyro.scale);
		ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
		update_anglerate();
        pidITerm = default_controller.output.axis_I[FD_ROLL];
        actITerm = 0.5 * k * t * t * pid->I8[ROLL] * luxITermScale;
        const float pidITermDelta = pidITerm - pidITermPrev;
        const float actITermDelta = actITerm - actITermPrev;
        const float error = fabs(actITermDelta - pidITermDelta);
        // error is limited by rectangle of height k * dt and width dt (then multiplied by pidProfile)
        const float errorLimit = k * dt * dt * pid->I8[ROLL] * luxITermScale;
        EXPECT_GE(errorLimit, error); // ie expect errorLimit >= error
    }
}

TEST(PIDUnittest, TestPidLuxFloatIntegrationForQuadraticFunction)
{
    struct rate_config controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    struct pid_config *pidProfile = &testPidProfile;

    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    resetRcCommands();

    // Test PID integration for a quadratic function:
    //    rateError = k * t * t
    // Integral:
    //    IrateError = (1/3) * k * t ^ 3
    // dt = 0.1s, t ranges from 0.0 to 0.6 in steps of 0.1s

    const float k = 800; // arbitrary value of k
    float t = 0.0f;
    // set rateError to k * t * t
    gyr[ROLL] = -k * t * t / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
	update_anglerate();

    float pidITerm = default_controller.output.axis_I[FD_ROLL]; // integral as estimated by PID
    float actITerm = (1.0f/3.0f) * k * t * t * t * pid->I8[ROLL] * luxITermScale; // actual value of integral
    EXPECT_FLOAT_EQ(actITerm, pidITerm); // both are zero at this point

    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    resetRcCommands();
    // limit to 6 iterations, since at 7th iteration rateError == 392 and so ITerm is constrained
    for (int ii = 0; ii < 6; ++ii) {
        const float actITermPrev = actITerm;
        const float pidITermPrev = pidITerm;
        t += dt;
        // set rateError to k * t * t
        gyr[ROLL] = -k * t * t / (luxGyroScale * gyro.scale);
		ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
		update_anglerate();
        pidITerm = default_controller.output.axis_I[FD_ROLL];
        actITerm = (1.0f/3.0f) * k * t * t * t * pid->I8[ROLL] * luxITermScale;
        const float pidITermDelta = pidITerm - pidITermPrev;
        const float actITermDelta = actITerm - actITermPrev;
        const float error = fabs(actITermDelta - pidITermDelta);
        // error is limited by rectangle of height k * dt and width dt (then multiplied by pidProfile)
        const float errorLimit = k * dt * dt * pid->I8[ROLL] * luxITermScale;
        EXPECT_GE(errorLimit, error); // ie expect errorLimit >= error
    }
}

TEST(PIDUnittest, TestPidLuxFloatITermConstrain)
{
    struct rate_config controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    struct pid_config *pidProfile = &testPidProfile;

    resetRcCommands();

    // set rateError to zero, ITerm should be zero
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(0, &controlRate);
    float rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
	update_anglerate();
    EXPECT_EQ(0, default_controller.output.axis_I[FD_ROLL]);

    // set rateError to 100, ITerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcLuxRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
    const float ITerm = calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRoll);
	printf("luxrc: %f, luxerr: %f\n", (double)rcCommand[ROLL], (double)rateErrorRoll);
	update_anglerate();
    EXPECT_FLOAT_EQ(ITerm, default_controller.output.axis_I[FD_ROLL]);

    // set up a very large rateError to force ITerm to be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    pid->I8[PIDROLL] = 255;
    rcCommand[ROLL] = calcLuxRcCommandRoll(10000, &controlRate);
    rateErrorRoll = calcLuxAngleRateRoll(&controlRate);
    EXPECT_EQ(10000, rateErrorRoll);// cross check
	update_anglerate();

//    EXPECT_FLOAT_EQ(PID_LUX_FLOAT_MAX_I, default_controller.output.axis_I[FD_ROLL]);
}

TEST(PIDUnittest, TestPidLuxFloatDTermConstrain)
{
    struct rate_config controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    struct pid_config *pidProfile = &testPidProfile;

    // set rateError to zero, DTerm should be zero
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    float rateErrorRoll = 0;
    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
	update_anglerate();
    EXPECT_EQ(0, default_controller.output.axis_D[FD_ROLL]);

    // set rateError to 100, DTerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rateErrorRoll = 100;
    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, default_controller.output.axis_D[FD_ROLL]);

    // set up a very large rateError to force DTerm to be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rateErrorRoll = 10000;
    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
	update_anglerate();
    //!!EXPECT_FLOAT_EQ(PID_LUX_FLOAT_MAX_D, default_controller.output.axis_D[FD_ROLL]);

    // now try a smaller value of dt
    // set rateError to 50, DTerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dt = 0.01;
    rateErrorRoll = 50;
    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
	update_anglerate();

    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, default_controller.output.axis_D[FD_ROLL]);

    // now try a test for dt = 0.001, which is typical for real world case
    // set rateError to 30, DTerm should not be constrained
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dt = 0.001;
    rateErrorRoll = 30;
    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, default_controller.output.axis_D[FD_ROLL]);

    // set rateError to 32
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    dt = 0.001;
    rateErrorRoll = 32;
    gyr[ROLL] = -rateErrorRoll / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();
	update_anglerate();
    // following test will fail, since DTerm will be constrained for when dt = 0.001
    //!!!!//EXPECT_FLOAT_EQ(calcLuxDTerm(&pidProfile, FD_ROLL, rateErrorRoll) / DTermAverageCount, default_controller.output.axis_D[FD_ROLL]);
}

void pidControllerInitMultiWiiRewriteCore(void)
{
    resetPidProfile();
    testPidProfile.pidController = PID_CONTROLLER_MWREWRITE;
    targetLooptime = TARGET_LOOPTIME; // normalised targetLooptime for pidMultiWiiRewrite
    dt = TARGET_LOOPTIME * 0.000001f;
    anglerate_reset_angle_i(&default_controller);
    anglerate_reset_rate_i(&default_controller);
    resetGyroADC();
    // set up the PIDWeights to 100%, so they are neutral in the tests
    default_controller.PIDweight[FD_ROLL] = 100;
    default_controller.PIDweight[FD_PITCH] = 100;
    default_controller.PIDweight[FD_YAW] = 100;
    // reset the pidMultiWiiRewrite static values
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        unittest_pidMultiWiiRewriteCore_lastRateForDelta[axis] = 0;
        for (int ii = 0; ii < DTERM_AVERAGE_COUNT; ++ii) { \
            unittest_pidMultiWiiRewriteCore_deltaState[axis][ii] = 0; \
        } \
    }
}

void pidControllerInitMultiWiiRewrite(struct rate_config *controlRate, uint16_t max_angle_inclination, rollAndPitchTrims_t *rollAndPitchTrims, rxConfig_t *rxConfig)
{
    UNUSED(max_angle_inclination);
    UNUSED(rxConfig);
    // set up the control rates for calculation of rate error
    controlRate->rates[ROLL] = 173;
    controlRate->rates[PITCH] = 173;
    controlRate->rates[YAW] = 173;
	rollAndPitchTrims->values.roll = 0;
	rollAndPitchTrims->values.pitch = 0;

	resetPID(controlRate, rollAndPitchTrims);

    pidControllerInitMultiWiiRewriteCore();
}

/*
 * calculate the value of rcCommand[ROLL] required to give a desired rateError
 */
int16_t calcMwrRcCommandRoll(int rateError, const struct rate_config *controlRate) {
    return (rateError << 4) / ((int32_t)(controlRate->rates[ROLL] + 27));
}

/*
 * calculate the angleRate as done in pidMultiWiiRewrite, used for cross checking
 */
int32_t calcMwrAngleRateRoll(const struct rate_config *controlRate) {
    return ((int32_t)(controlRate->rates[ROLL] + 27) * rcCommand[ROLL]) >> 4;
}

/*
 * calculate the value of rcCommand[PITCH] required to give a desired rateError
 */
int16_t calcMwrRcCommandPitch(int rateError, const struct rate_config *controlRate) {
    return (rateError << 4) / ((int32_t)(controlRate->rates[PITCH] + 27));
}

/*
 * calculate the angleRate as done in pidMultiWiiRewrite, used for cross checking
 */
int32_t calcMwrAngleRatePitch(const struct rate_config *controlRate) {
    return ((int32_t)(controlRate->rates[PITCH] + 27) * rcCommand[PITCH]) >> 4;
}

/*
 * calculate the value of rcCommand[YAW] required to give a desired rateError
 */
int16_t calcMwrRcCommandYaw(int rateError, const struct rate_config *controlRate) {
    return (rateError << 5) / ((int32_t)(controlRate->rates[YAW] + 27));
}

/*
 * calculate the angleRate as done in pidMultiWiiRewrite, used for cross checking
 */
int32_t calcMwrAngleRateYaw(const struct rate_config *controlRate) {
    return ((int32_t)(controlRate->rates[YAW] + 27) * rcCommand[YAW]) >> 5;
}

int32_t calcMwrPTerm(struct pid_config *pidProfile, pid_index_t axis, int rateError) {
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    return (pid->P8[axis] * rateError) >> 7;
}

int32_t calcMwrITermDelta(struct pid_config *pidProfile, pid_index_t axis, int rateError) {
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    int32_t ret = pid->I8[axis] * (rateError * targetLooptime >> 11) >> 13;
    ret = constrain(ret, -PID_MAX_I, PID_MAX_I);
    return ret;
}

int32_t calcMwrDTerm(struct pid_config *pidProfile, pid_index_t axis, int rateError) {
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;
    int32_t ret = (rateError * ((uint16_t)0xFFFF / ((uint16_t)targetLooptime >> 4))) >> 5;
    ret =  (ret * pid->D8[axis]) >> 8;
    ret /= DTermAverageCount;
    ret = constrain(ret, -PID_MAX_D, PID_MAX_D);
    return ret;
}

TEST(PIDUnittest, TestPidMultiWiiRewrite)
{
	struct pid_config *pid = &config_get_profile_rw(&config)->pid;

    struct rate_config controlRate;

    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of zero on all axes
    resetRcCommands();
    resetGyroADC();
	update_anglerate();

    EXPECT_EQ(0, default_controller.output.axis_P[FD_ROLL]);
    EXPECT_EQ(0, default_controller.output.axis_I[FD_ROLL]);
    EXPECT_EQ(0, default_controller.output.axis_D[FD_ROLL]);
    EXPECT_EQ(0, default_controller.output.axis_P[FD_PITCH]);
    EXPECT_EQ(0, default_controller.output.axis_I[FD_PITCH]);
    EXPECT_EQ(0, default_controller.output.axis_D[FD_PITCH]);
    EXPECT_EQ(0, default_controller.output.axis_P[FD_YAW]);
    EXPECT_EQ(0, default_controller.output.axis_I[FD_YAW]);
    EXPECT_EQ(0, default_controller.output.axis_D[FD_YAW]);

    // set up a rateError of 100 on the roll axis
    const int32_t rateErrorRoll = 100;
    // set up a rateError of 100 on the pitch axis
    const int32_t rateErrorPitch = 100;
    // set up a rateError of 100 on the yaw axis
    const int32_t rateErrorYaw = 100;

    // run the PID controller. Check expected PID values
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    gyr[ROLL] = -rateErrorRoll * mwrGyroScaleNum / mwrGyroScaleDenom;
    gyr[PITCH] = -rateErrorPitch * mwrGyroScaleNum / mwrGyroScaleDenom;
    gyr[YAW] = -rateErrorYaw * mwrGyroScaleNum / mwrGyroScaleDenom;
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();

	update_anglerate();

    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDROLL, rateErrorRoll), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, rateErrorRoll), default_controller.output.axis_I[FD_ROLL]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDROLL, rateErrorRoll), default_controller.output.axis_D[FD_ROLL]);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDPITCH, rateErrorPitch), default_controller.output.axis_P[FD_PITCH]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDPITCH, rateErrorPitch), default_controller.output.axis_I[FD_PITCH]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDPITCH, rateErrorPitch), default_controller.output.axis_D[FD_PITCH]);
    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDYAW, rateErrorYaw), default_controller.output.axis_P[FD_YAW]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDYAW, rateErrorYaw), default_controller.output.axis_I[FD_YAW]);
    EXPECT_EQ(calcMwrDTerm(pidProfile, PIDYAW, rateErrorYaw) , default_controller.output.axis_D[FD_YAW]);
}

TEST(PIDUnittest, TestPidMultiWiiRewriteITermConstrain)
{
    struct rate_config controlRate;
    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;

    struct pid_config *pidProfile = &testPidProfile;

    // set rateError to zero, ITerm should be zero
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcMwrRcCommandRoll(0, &controlRate);
    int16_t rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(0, rateErrorRoll);// cross check
	update_anglerate();
    EXPECT_EQ(0, default_controller.output.axis_I[FD_ROLL]);

    // set rateError to 100, ITerm should not be constrained
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    rcCommand[ROLL] = calcMwrRcCommandRoll(100, &controlRate);
    rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(100, rateErrorRoll);// cross check
	update_anglerate();
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, rateErrorRoll), default_controller.output.axis_I[FD_ROLL]);

    // set up a very large rateError and a large targetLooptime to force ITerm to be constrained
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    targetLooptime = 8192;
    rcCommand[ROLL] = calcMwrRcCommandRoll(32750, &controlRate); // can't use INT16_MAX, since get rounding error
    rateErrorRoll = calcMwrAngleRateRoll(&controlRate);
    EXPECT_EQ(32750, rateErrorRoll);// cross check
	for(int c = 0; c < 20; c++)
		update_anglerate();
    EXPECT_EQ(GYRO_I_MAX, default_controller.output.axis_I[FD_ROLL]);
}

/* 
Testing core explicitly is unnecessary because it is already tested by the function below. 
*/

TEST(PIDUnittest, TestPidMultiWiiRewritePidLuxFloatEquivalence)
{
    struct pid_config *pidProfile = &testPidProfile;

    struct rate_config controlRate;

    const uint16_t max_angle_inclination = 500; // 50 degrees
    rollAndPitchTrims_t rollAndPitchTrims;
    rxConfig_t rxConfig;


    resetPidProfile();
    pidControllerInitLuxFloat(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);
    EXPECT_FLOAT_EQ(1.0f / 16.4f, gyro.scale);

    // set up a rateError of 200 on the roll axis
    const float rateErrorRollf = 200;
    gyr[ROLL] = -rateErrorRollf / (luxGyroScale * gyro.scale);
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();

    float ITermRoll = calcLuxITermDelta(pidProfile, FD_ROLL, rateErrorRollf);

    // run the PID controller. Check expected PID values
	update_anglerate();
    EXPECT_FLOAT_EQ(calcLuxPTerm(pidProfile, FD_ROLL, rateErrorRollf), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_FLOAT_EQ(ITermRoll, default_controller.output.axis_I[FD_ROLL]);
    float expectedDTermf = calcLuxDTerm(pidProfile, FD_ROLL, rateErrorRollf) / DTermAverageCount;
    EXPECT_FLOAT_EQ(expectedDTermf, default_controller.output.axis_D[FD_ROLL]);

    resetPidProfile();
    pidControllerInitMultiWiiRewrite(&controlRate, max_angle_inclination, &rollAndPitchTrims, &rxConfig);

    // set up a rateError of 200 on the roll axis
    const int32_t rateErrorRoll = 200;
    gyr[ROLL] = -rateErrorRoll * mwrGyroScaleNum / mwrGyroScaleDenom;
	ins_process_gyro(&default_ins, gyr[ROLL], gyr[PITCH], gyr[YAW]);
    resetRcCommands();

    // run the PID controller. Check expected PID values
	update_anglerate();

    EXPECT_EQ(calcMwrPTerm(pidProfile, PIDROLL, rateErrorRoll), default_controller.output.axis_P[FD_ROLL]);
    EXPECT_EQ(calcMwrITermDelta(pidProfile, PIDROLL, rateErrorRoll), default_controller.output.axis_I[FD_ROLL]);
    int32_t expectedDTerm = calcMwrDTerm(pidProfile, PIDROLL, rateErrorRoll);
    EXPECT_EQ(expectedDTerm, default_controller.output.axis_D[FD_ROLL]);

    const float allowedPError = (float)default_controller.output.axis_P[FD_ROLL] / 100; // 1% error allowed
    EXPECT_NEAR(default_controller.output.axis_P[FD_ROLL], default_controller.output.axis_P[FD_ROLL], allowedPError);

    const float allowedIError = 1;
    EXPECT_NEAR(default_controller.output.axis_I[FD_ROLL], default_controller.output.axis_I[FD_ROLL], allowedIError);

    const float allowedDError = (float)default_controller.output.axis_D[FD_ROLL] / 100; // 1% error allowed
    EXPECT_NEAR(default_controller.output.axis_D[FD_ROLL], default_controller.output.axis_D[FD_ROLL], allowedDError);
}
