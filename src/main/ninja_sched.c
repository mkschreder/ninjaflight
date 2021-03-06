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

/**
 * @file ninja_sched.c
 * @author Cleanflight
 * @author Martin Schröder 2016
 */
/**
 * @addtogroup ninja
 * @{
 */
/**
 * @defgroup scheduler Scheduler
 * @{
 *
 * Primary responsibility of this module is to define core tasks and their
 * timing for the ninjaflight application. The scheduler is nothing more than a
 * small cooperative scheduler that helps organize different cooperative
 * processes. For example, you can define that the pid loop should run every
 * 1000 microseconds and that receiver loop should run at 20000. This module
 * helps decouple timing from the tasks themselves and uses current tick
 * counter for determining when tasks need to run. It does not (and should not)
 * do any preemptive task switching. It is a rather primitive scheduler actually.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "debug.h"
#include "build_config.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "config/feature.h"

#include "io/serial.h"
#include "io/beeper.h"
#include "io/statusindicator.h"
#include "io/transponder_ir.h"
#include "io/display.h"
#include "io/ledstrip.h"
#include "io/serial_msp.h"

#include "flight/altitudehold.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/imu.h"

#include "telemetry/telemetry.h"

#include "ninja.h"
#include "ninja_sched.h"
#include "rx/rc.h"

#define REALTIME_GUARD_INTERVAL_MIN	 10
#define REALTIME_GUARD_INTERVAL_MAX	 300
#define REALTIME_GUARD_INTERVAL_MARGIN  25

static cfTask_t cfTasks[TASK_COUNT];

static void _queue_clear(struct ninja_sched *self){
	memset(self->taskQueueArray, 0, sizeof(self->taskQueueArray));
	self->taskQueuePos = 0;
	self->taskQueueSize = 0;
}

static bool _queue_contains(struct ninja_sched *self, cfTask_t *task){
	for (int ii = 0; ii < self->taskQueueSize; ++ii) {
		if (self->taskQueueArray[ii] == task) {
			return true;
		}
	}
	return false;
}

static bool _queue_add(struct ninja_sched *self, cfTask_t *task){
	if ((self->taskQueueSize >= TASK_COUNT) || _queue_contains(self, task)) {
		return false;
	}
	for (int ii = 0; ii <= self->taskQueueSize; ++ii) {
		if (self->taskQueueArray[ii] == NULL || self->taskQueueArray[ii]->staticPriority < task->staticPriority) {
			memmove(&self->taskQueueArray[ii+1], &self->taskQueueArray[ii], sizeof(task) * (self->taskQueueSize - ii));
			self->taskQueueArray[ii] = task;
			++self->taskQueueSize;
			return true;
		}
	}
	return false;
}

static bool _queue_remove(struct ninja_sched *self, cfTask_t *task){
	for (int ii = 0; ii < self->taskQueueSize; ++ii) {
		if (self->taskQueueArray[ii] == task) {
			memmove(&self->taskQueueArray[ii], &self->taskQueueArray[ii+1], sizeof(task) * (self->taskQueueSize - ii));
			--self->taskQueueSize;
			return true;
		}
	}
	return false;
}

static cfTask_t *_queue_first(struct ninja_sched *self){
	self->taskQueuePos = 0;
	return self->taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

static cfTask_t *_queue_next(struct ninja_sched *self){
	return self->taskQueueArray[++self->taskQueuePos]; // guaranteed to be NULL at end of queue
}

#ifndef SKIP_TASK_STATISTICS
void ninja_sched_get_task_info(struct ninja_sched *self, cfTaskId_e taskId, cfTaskInfo_t * taskInfo){
	taskInfo->taskName = cfTasks[taskId].taskName;
	taskInfo->isEnabled = _queue_contains(self, &cfTasks[taskId]);
	taskInfo->desiredPeriod = cfTasks[taskId].desiredPeriod;
	taskInfo->staticPriority = cfTasks[taskId].staticPriority;
	taskInfo->maxExecutionTime = cfTasks[taskId].maxExecutionTime;
	taskInfo->totalExecutionTime = cfTasks[taskId].totalExecutionTime;
	taskInfo->averageExecutionTime = cfTasks[taskId].averageExecutionTime;
	taskInfo->latestDeltaTime = cfTasks[taskId].taskLatestDeltaTime;
}
#endif

void ninja_sched_set_task_period(struct ninja_sched *self, cfTaskId_e taskId, uint32_t newPeriodMicros){
	if (taskId == TASK_SELF || taskId < TASK_COUNT) {
		cfTask_t *task = taskId == TASK_SELF ? self->currentTask : &cfTasks[taskId];
		task->desiredPeriod = MAX(100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
	}
}

void ninja_sched_set_task_enabled(struct ninja_sched *self, cfTaskId_e taskId, bool enabled){
	if (taskId == TASK_SELF || taskId < TASK_COUNT) {
		cfTask_t *task = taskId == TASK_SELF ? self->currentTask : &cfTasks[taskId];
		if (enabled && task->taskFunc) {
			_queue_add(self, task);
		} else {
			_queue_remove(self, task);
		}
	}
}

uint32_t ninja_sched_get_task_dt(struct ninja_sched *self, cfTaskId_e taskId){
	if (taskId == TASK_SELF || taskId < TASK_COUNT) {
		cfTask_t *task = taskId == TASK_SELF ? self->currentTask : &cfTasks[taskId];
		return task->taskLatestDeltaTime;
	} else {
		return 0;
	}
}

void ninja_sched_init(struct ninja_sched *self, const struct system_calls_time *time, const struct config *config){
	memset(self, 0, sizeof(struct ninja_sched));
	self->time = time;
	self->config = config;
	self->realtimeGuardInterval = REALTIME_GUARD_INTERVAL_MAX;

	_queue_clear(self);
	_queue_add(self, &cfTasks[TASK_SYSTEM]);

	ninja_sched_set_task_enabled(self, TASK_GYROPID, true);
	// TODO: we need to make sure that gyro has correct value here. That predisposes that all hardware is initialized before ninja_init. However there are back and forth dependencies that need to be sorted first.
	//rescheduleTask(TASK_GYROPID, imuConfig()->gyroSync ? gyro_sync_get_looptime() - INTERRUPT_WAIT_TIME : gyro_sync_get_looptime());
	ninja_sched_set_task_enabled(self, TASK_ACCEL, true);
	ninja_sched_set_task_enabled(self, TASK_SERIAL, true);
	if(USE_BEEPER){
		ninja_sched_set_task_enabled(self, TASK_BEEPER, true);
	}
	ninja_sched_set_task_enabled(self, TASK_BATTERY, feature(self->config, FEATURE_VBAT) || feature(self->config, FEATURE_CURRENT_METER));
	ninja_sched_set_task_enabled(self, TASK_RX, true);
#ifdef GPS
	ninja_sched_set_task_enabled(self, TASK_GPS, feature(self->config, FEATURE_GPS));
#endif
#if USE_MAG == 1
	ninja_sched_set_task_enabled(self, TASK_COMPASS, true);
	#if defined(MPU6500_SPI_INSTANCE) && defined(USE_MAG_AK8963)
		// fixme temporary solution for AK6983 via slave I2C on MPU9250
		ninja_sched_set_task_period(self, TASK_COMPASS, 1000000 / 40);
	#endif
#endif
#ifdef BARO
	ninja_sched_set_task_enabled(self, TASK_BARO, true);
#endif
#ifdef SONAR
	ninja_sched_set_task_enabled(self, TASK_SONAR, true);
#endif
#if defined(BARO) || defined(SONAR)
	ninja_sched_set_task_enabled(self, TASK_ALTITUDE, true);
#endif
#ifdef DISPLAY
	ninja_sched_set_task_enabled(self, TASK_DISPLAY, feature(self->config, FEATURE_DISPLAY));
#endif
#ifdef TELEMETRY
	ninja_sched_set_task_enabled(self, TASK_TELEMETRY, feature(self->config, FEATURE_TELEMETRY));
#endif
#ifdef LED_STRIP
	ninja_sched_set_task_enabled(self, TASK_LEDSTRIP, feature(self->config, FEATURE_LED_STRIP));
#endif
#ifdef TRANSPONDER
	ninja_sched_set_task_enabled(self, TASK_TRANSPONDER, feature(self->config, FEATURE_TRANSPONDER));
#endif
}

void ninja_sched_run(struct ninja_sched *self){
	// Cache currentTime
	int32_t currentTime = self->time->micros(self->time);

	// Check for realtime tasks
	uint32_t timeToNextRealtimeTask = UINT32_MAX;
	for (const cfTask_t *task = _queue_first(self); task != NULL && task->staticPriority >= TASK_PRIORITY_REALTIME; task = _queue_next(self)) {
		const uint32_t nextExecuteAt = task->lastExecutedAt + task->desiredPeriod;
		if ((int32_t)(currentTime - nextExecuteAt) >= 0) {
			timeToNextRealtimeTask = 0;
		} else {
			const uint32_t newTimeInterval = nextExecuteAt - currentTime;
			timeToNextRealtimeTask = MIN(timeToNextRealtimeTask, newTimeInterval);
		}
	}
	const bool outsideRealtimeGuardInterval = (timeToNextRealtimeTask > self->realtimeGuardInterval);

	// The task to be invoked
	cfTask_t *selectedTask = NULL;
	uint16_t selectedTaskDynamicPriority = 0;

	// Update task dynamic priorities
	uint16_t waitingTasks = 0;
	for (cfTask_t *task = _queue_first(self); task != NULL; task = _queue_next(self)) {
		// Task has checkFunc - event driven
		if (task->checkFunc != NULL) {
			// Increase priority for event driven tasks
			if (task->dynamicPriority > 0) {
				task->taskAgeCycles = 1 + ((currentTime - task->lastSignaledAt) / task->desiredPeriod);
				task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
				waitingTasks++;
			} else if (task->checkFunc(self, currentTime - task->lastExecutedAt)) {
				task->lastSignaledAt = currentTime;
				task->taskAgeCycles = 1;
				task->dynamicPriority = 1 + task->staticPriority;
				waitingTasks++;
			} else {
				task->taskAgeCycles = 0;
			}
		} else {
			// Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
			// Task age is calculated from last execution
			task->taskAgeCycles = ((currentTime - task->lastExecutedAt) / task->desiredPeriod);
			if (task->taskAgeCycles > 0) {
				task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
				waitingTasks++;
			}
		}

		if (task->dynamicPriority > selectedTaskDynamicPriority) {
			const bool taskCanBeChosenForScheduling =
				(outsideRealtimeGuardInterval) ||
				(task->taskAgeCycles > 1) ||
				(task->staticPriority == TASK_PRIORITY_REALTIME);
			if (taskCanBeChosenForScheduling) {
				selectedTaskDynamicPriority = task->dynamicPriority;
				selectedTask = task;
			}
		}
	}

	self->totalWaitingTasksSamples++;
	self->totalWaitingTasks += waitingTasks;

	self->currentTask = selectedTask;

	if (selectedTask != NULL) {
		// Found a task that should be run
		selectedTask->taskLatestDeltaTime = currentTime - selectedTask->lastExecutedAt;
		selectedTask->lastExecutedAt = currentTime;
		selectedTask->dynamicPriority = 0;

		// Execute task
		const uint32_t currentTimeBeforeTaskCall = self->time->micros(self->time);
		selectedTask->taskFunc(self);
		const uint32_t taskExecutionTime = self->time->micros(self->time) - currentTimeBeforeTaskCall;

		selectedTask->averageExecutionTime = ((uint32_t)selectedTask->averageExecutionTime * 31 + taskExecutionTime) / 32;
		selectedTask->totalExecutionTime += taskExecutionTime;   // time consumed by scheduler + task
		selectedTask->maxExecutionTime = MAX(selectedTask->maxExecutionTime, taskExecutionTime);
	}
}

uint16_t ninja_sched_get_load(struct ninja_sched *self){
	return self->averageSystemLoadPercent;
}

static void _task_transponder(struct ninja_sched *sched){
	(void)sched;
	#ifdef TRANSPONDER
	if (feature(FEATURE_TRANSPONDER)) {
		updateTransponder();
	}
	#endif
}

static void _task_system(struct ninja_sched *self){
	/* Calculate system load */
	if (self->totalWaitingTasksSamples > 0) {
		self->averageSystemLoadPercent = 100 * self->totalWaitingTasks / self->totalWaitingTasksSamples;
		self->totalWaitingTasksSamples = 0;
		self->totalWaitingTasks = 0;
	}

	/* Calculate guard interval */
	uint32_t maxNonRealtimeTaskTime = 0;
	for (const cfTask_t *task = _queue_first(self); task != NULL; task = _queue_next(self)) {
		if (task->staticPriority != TASK_PRIORITY_REALTIME) {
			maxNonRealtimeTaskTime = MAX(maxNonRealtimeTaskTime, task->averageExecutionTime);
		}
	}

	self->realtimeGuardInterval = constrain(maxNonRealtimeTaskTime, REALTIME_GUARD_INTERVAL_MIN, REALTIME_GUARD_INTERVAL_MAX) + REALTIME_GUARD_INTERVAL_MARGIN;
}

#if 0
// TODO: create a pid task state with this kind of things
static filterStatePt1_t filteredCycleTimeState;
static void _task_gyro(struct ninja_sched *sched){
	struct ninja *self = container_of(sched, struct ninja, sched);
	self->cycleTime = ninja_sched_get_task_dt(sched, TASK_SELF);

	// Calculate average cycle time and average jitter
	self->filteredCycleTime = filterApplyPt1(self->cycleTime, &filteredCycleTimeState, 1, (self->cycleTime * 1e-6f));

	// TODO: should we use filtered cycle time for dt or the raw one? Filtered may be better.
	float dT = (float)self->cycleTime * 1e-6f;
	// prevent zero dt
	if(dT < 1e-6f) dT = 1e-6f;

	// getTaskDeltaTime() returns delta time freezed at the moment of entering the scheduler. currentTime is freezed at the very same point.
	// To make busy-waiting timeout work we need to account for time spent within busy-waiting loop

	// TODO: reenable gyro sync
	/*
	uint32_t currentDeltaTime = getTaskDeltaTime(TASK_SELF);
	static const int GYRO_WATCHDOG_DELAY=100; // Watchdog for boards without interrupt for gyro
	if (imuConfig()->gyroSync) {
		if(gyroSyncCheckUpdate() || ((currentDeltaTime + (micros() - currentTime)) >= (gyro_sync_get_looptime() + GYRO_WATCHDOG_DELAY))) {
			ninja_run_pid_loop(&ninja, dT);
		}
	} else {
		ninja_run_pid_loop(&ninja, dT);
	}
	*/
	ninja_run_pid_loop(self, self->cycleTime);
}
#endif

static void _task_beeper(struct ninja_sched *sched){
	if(USE_BEEPER){
		struct ninja *self = container_of(sched, struct ninja, sched);
		beeper_update(&self->beeper);		  //call periodic beeper handler
	}
}

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

static void _task_bat(struct ninja_sched *sched){
	static uint32_t vbatLastServiced = 0;
	static uint32_t ibatLastServiced = 0;

	int32_t currentTime = sched->time->micros(sched->time);
	if (feature(sched->config, FEATURE_VBAT)) {
		if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
			vbatLastServiced = currentTime;
			// TODO: fix this
			//battery_update(&default_battery);
		}
	}

	if (feature(sched->config, FEATURE_CURRENT_METER)) {
		int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

		if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
			ibatLastServiced = currentTime;

			// TODO: fix this
			//throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig(), rcControlsConfig()->deadband3d_throttle);

			//battery_update_current_meter(&default_battery, ibatTimeSinceLastServiced, throttleStatus);
		}
	}
}

static bool _task_rx_check(struct ninja_sched *sched, uint32_t currentDeltaTime){
	struct ninja *self = container_of(sched, struct ninja, sched);
	UNUSED(currentDeltaTime);

	rx_update(&self->rx);
	return rx_has_signal(&self->rx);
}

static void updateLEDs(struct ninja_sched *sched){
	struct ninja *self = container_of(sched, struct ninja, sched);
	if (self->is_armed) {
		//sys_led_on(self->system, 0);
	} else {
		/*
		// TODO: for now allow arming when not leveled but rethink this logic entirely
		if (!imu_is_leveled(&default_imu, armingConfig()->max_arm_angle)) {
			DISABLE_ARMING_FLAG(OK_TO_ARM);
		}
*/
		// TODO: this should probably be checked elsewhere
		/*if(calibrating && !isCalibrating()){
			// save config here since calibration may have changed it
			saveConfigAndNotify();
			calibrating = 0;
		}*/

		// TODO: do we need to check calibration here or can we just do it in the main handler?
		// also these things need to be organized as a state machine - not like this.
		//if (isCalibrating() || isSystemOverloaded()) {
		if (sched->averageSystemLoadPercent > 100) {
			warningLedFlash();
		} else {
			warningLedFlash();
		}

		warningLedUpdate(&self->system->leds, sched->time->micros(sched->time));
	}
}

static void _task_rx(struct ninja_sched *sched){
	struct ninja *self = container_of(sched, struct ninja, sched);
	updateLEDs(sched);

	// read rx and update rc state
	rx_update(&self->rx);
	rc_update(&self->rc);
}

#ifdef GPS
static void _task_gps(struct ninja_sched *sched){
	struct ninja *self = container_of(sched, struct ninja, sched);
	// if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
	// hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
	// change this based on available hardware
	if (feature(sched->config, FEATURE_GPS)) {
		gps_update(&self->gps);
	}

	// TODO: better way to detect if we have gps
	/*
	if (sensors(SENSOR_GPS)) {
		updateGpsIndicator(sched->time->micros(sched->time));
	}
	*/
}
#endif

static void _task_mag(struct ninja_sched *sched){
	(void)sched;
/*
	if(USE_MAG){
		int16_t raw[3];
		if (sensors(SENSOR_MAG) && mag.read(raw)){
			ins_process_mag(&ninja.ins, raw[0], raw[1], raw[2]);
			//updateCompass(&sensorTrims()->magZero);
		}
	}
	*/
}

#ifdef BARO
static void _task_baro(struct ninja_sched *sched){
	struct ninja *self = container_of(sched, struct ninja, sched);
	// TODO: baro read
	uint32_t pressure = 101325;
	if(sys_read_pressure(self->system, &pressure) == 0){
		//ins_process_pressure(&self->ins, pressure);
		//uint32_t alt = ins_get_altitude_cm(&self->ins);
	}
	/*
	if (sensors(SENSOR_BARO)) {
		uint32_t newDeadline = baroUpdate();
		ninja_sched_set_task_period(sched, TASK_SELF, newDeadline);
	}
	*/
}
#endif

#ifdef SONAR
static void _task_sonar(struct ninja_sched *sched)
{
	(void)sched;
	/*
	// TODO: sonar
	if (sensors(SENSOR_SONAR)) {
		sonar_update(&default_sonar);
	}
	*/
}
#endif

#if defined(BARO) || defined(SONAR)
static void _task_altitude(struct ninja_sched *sched)
{
	(void)sched;
	// TODO: calc altitude
	/*
	if (false
#if defined(BARO)
		|| (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(SONAR)
		|| sensors(SENSOR_SONAR)
#endif
		) {
		calculateEstimatedAltitude(sched->time->micros(sched->time));
	}
	*/
}
#endif

#ifdef DISPLAY
static void _task_display(struct ninja_sched *sched){
	(void)sched;
	if (feature(sched->config, FEATURE_DISPLAY)) {
		updateDisplay();
	}
}
#endif

#ifdef TELEMETRY
static void _task_telemetry(struct ninja_sched *sched){
	(void)sched;
/*
	telemetryCheckState();

	if (!cliMode && feature(FEATURE_TELEMETRY)) {
		telemetryProcess(rcControlsConfig()->deadband3d_throttle);
	}
	*/
}
#endif

#ifdef LED_STRIP
static void _task_ledstrip(struct ninja_sched *sched){
	struct ninja *self = container_of(sched, struct ninja, sched);
	if (feature(sched->config, FEATURE_LED_STRIP)) {
		ledstrip_update(&self->ledstrip);
	}
}
#endif

static cfTask_t cfTasks[TASK_COUNT] = {
	[TASK_SYSTEM] = {
		.taskName = "SYSTEM",
		.taskFunc = _task_system,
		.desiredPeriod = 1000000 / 10,		  // 10 Hz, every 100 ms
		.staticPriority = TASK_PRIORITY_HIGH,
	},
	// TODO: sort out the problem of system load being too high when testing
/*
	[TASK_GYROPID] = {
		.taskName = "GYRO/PID",
		.taskFunc = _task_gyro,
#ifdef SITL
		.desiredPeriod = 3500,				  // every 1 ms
#else
		.desiredPeriod = 10000,
#endif
		.staticPriority = TASK_PRIORITY_REALTIME,
	},

	[TASK_ACCEL] = {
		.taskName = "ACCEL",
		.taskFunc = _task_acc,
#ifdef SITL
		.desiredPeriod = 3500,				  // every 1 ms
#else
		.desiredPeriod = 10000,
#endif
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
*/
	[TASK_BEEPER] = {
		.taskName = "BEEPER",
		.taskFunc = _task_beeper,
		.desiredPeriod = 1000000 / 100,		 // 100 Hz, every 10 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},

	[TASK_BATTERY] = {
		.taskName = "BATTERY",
		.taskFunc = _task_bat,
		.desiredPeriod = 1000000 / 50,		  // 50 Hz, every 20 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},

	[TASK_RX] = {
		.taskName = "RX",
		.checkFunc = _task_rx_check,
		.taskFunc = _task_rx,
		.desiredPeriod = 1000000 / 50,		  // If event-based scheduling doesn't work, fallback to periodic scheduling
		.staticPriority = TASK_PRIORITY_HIGH,
	},

#ifdef GPS
	[TASK_GPS] = {
		.taskName = "GPS",
		.taskFunc = _task_gps,
		.desiredPeriod = 1000000 / 10,		  // GPS usually don't go faster than 10Hz
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
#endif

	[TASK_COMPASS] = {
		.taskName = "COMPASS",
		.taskFunc = _task_mag,
		.desiredPeriod = 1000000 / 10,		  // 10 Hz, every 100 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},

#ifdef BARO
	[TASK_BARO] = {
		.taskName = "BARO",
		.taskFunc = _task_baro,
		.desiredPeriod = 1000000 / 20,		  // 20 Hz, every 50 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
#endif

#ifdef SONAR
	[TASK_SONAR] = {
		.taskName = "SONAR",
		.taskFunc = _task_sonar,
		.desiredPeriod = 70000,				 // every 70 ms, approximately 14 Hz
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
#endif

#if defined(BARO) || defined(SONAR)
	[TASK_ALTITUDE] = {
		.taskName = "ALTITUDE",
		.taskFunc = _task_altitude,
		.desiredPeriod = 1000000 / 40,		  // 40 Hz, every 25 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,
	},
#endif

	[TASK_TRANSPONDER] = {
		.taskName = "TRANSPONDER",
		.taskFunc = _task_transponder,
		.desiredPeriod = 1000000 / 250,		 // 250 Hz, every 4 ms
		.staticPriority = TASK_PRIORITY_LOW,
	},

#ifdef DISPLAY
	[TASK_DISPLAY] = {
		.taskName = "DISPLAY",
		.taskFunc = _task_display,
		.desiredPeriod = 1000000 / 10,		  // 10 Hz, every 100 ms
		.staticPriority = TASK_PRIORITY_LOW,
	},
#endif

#ifdef TELEMETRY
	[TASK_TELEMETRY] = {
		.taskName = "TELEMETRY",
		.taskFunc = _task_telemetry,
		.desiredPeriod = 1000000 / 250,		 // 250 Hz, every 4 ms
		.staticPriority = TASK_PRIORITY_IDLE,
	},
#endif

#ifdef LED_STRIP
	[TASK_LEDSTRIP] = {
		.taskName = "LEDSTRIP",
		.taskFunc = _task_ledstrip,
		.desiredPeriod = 1000000 / 100,		 // 100 Hz, every 10 ms
		.staticPriority = TASK_PRIORITY_IDLE,
	},
#endif
};

/** @} */
/** @} */
