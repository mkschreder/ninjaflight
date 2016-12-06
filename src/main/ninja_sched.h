#pragma once

#include <stdbool.h>
#include "common/list.h"

typedef enum {
    TASK_PRIORITY_IDLE = 0,     // Disables dynamic scheduling, task is executed only if no other task is active this cycle
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_REALTIME = 6,
    TASK_PRIORITY_MAX = 255
} cfTaskPriority_e;

typedef struct {
    const char * taskName;
    bool         isEnabled;
    uint32_t     desiredPeriod;
    uint8_t      staticPriority;
    uint32_t     maxExecutionTime;
    uint32_t     totalExecutionTime;
    uint32_t     averageExecutionTime;
    uint32_t     latestDeltaTime;
} cfTaskInfo_t;

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_GYROPID,
    TASK_ACCEL,
    TASK_SERIAL,
#ifdef BEEPER
    TASK_BEEPER,
#endif
    TASK_BATTERY,
    TASK_RX,
#ifdef GPS
    TASK_GPS,
#endif
    TASK_COMPASS,
#ifdef BARO
    TASK_BARO,
#endif
#ifdef SONAR
    TASK_SONAR,
#endif
#if defined(BARO) || defined(SONAR)
    TASK_ALTITUDE,
#endif
#ifdef DISPLAY
    TASK_DISPLAY,
#endif
#ifdef TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef LED_STRIP
    TASK_LEDSTRIP,
#endif
    TASK_TRANSPONDER,

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} cfTaskId_e;

struct ninja_sched;
typedef struct cfTask_s {
    /* Configuration */
    const char * taskName;
    bool (*checkFunc)(struct ninja_sched *self, uint32_t currentDeltaTime);
    void (*taskFunc)(struct ninja_sched *self);
    uint32_t desiredPeriod;         // target period of execution
    const uint8_t staticPriority;   // dynamicPriority grows in steps of this size, shouldn't be zero

    /* Scheduling */
    uint16_t dynamicPriority;       // measurement of how old task was last executed, used to avoid task starvation
    uint16_t taskAgeCycles;
    uint32_t lastExecutedAt;        // last time of invocation
    uint32_t lastSignaledAt;        // time of invocation event for event-driven tasks

    /* Statistics */
    uint32_t averageExecutionTime;  // Moving average over 6 samples, used to calculate guard interval
    uint32_t taskLatestDeltaTime;   //
#ifndef SKIP_TASK_STATISTICS
    uint32_t maxExecutionTime;
    uint32_t totalExecutionTime;    // total time consumed by task since boot
#endif
} cfTask_t;

struct ninja_sched {
	// No need for a linked list for the queue, since items are only inserted at startup
	cfTask_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue
	cfTask_t *currentTask;

	uint32_t totalWaitingTasks;
	uint32_t totalWaitingTasksSamples;
	uint32_t realtimeGuardInterval;

	uint32_t currentTime;
	uint16_t averageSystemLoadPercent;

	int taskQueuePos;
	int taskQueueSize;

	const struct system_calls_time *time;
	const struct config *config;
};

/*
void getTaskInfo(cfTaskId_e taskId, cfTaskInfo_t * taskInfo);
void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodMicros);
void setTaskEnabled(cfTaskId_e taskId, bool newEnabledState);
uint32_t getTaskDeltaTime(cfTaskId_e taskId);
*/

#define LOAD_PERCENTAGE_ONE 100

struct ninja;
void ninja_sched_init(struct ninja_sched *self, const struct system_calls_time *time, const struct config *config);
void ninja_sched_run(struct ninja_sched *self);
void ninja_sched_get_task_info(struct ninja_sched *self, cfTaskId_e taskId, cfTaskInfo_t * taskInfo);
uint16_t ninja_sched_get_load(struct ninja_sched *self);
