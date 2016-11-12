#pragma once

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;

typedef struct accelerometerConfig_s {
    rollAndPitchTrims_t accelerometerTrims; // accelerometer trim

    // sensor-related stuff
    uint8_t acc_cut_hz;                     // Set the Low Pass Filter factor for ACC. Reducing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    float accz_lpf_cutoff;                  // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    accDeadband_t accDeadband;
    uint8_t acc_unarmedcal;                 // turn automatic acc compensation on/off
} accelerometerConfig_t;

PG_DECLARE_PROFILE(accelerometerConfig_t, accelerometerConfig);
