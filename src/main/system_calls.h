#pragma once

struct system_calls_pwm {
	void (*write_motor)(const struct system_calls_pwm *self, uint8_t id, uint16_t value);
	void (*write_servo)(const struct system_calls_pwm *self, uint8_t id, uint16_t value);
	uint16_t (*read_pwm)(const struct system_calls_pwm *self, uint8_t chan);
	uint16_t (*read_ppm)(const struct system_calls_pwm *self, uint8_t chan);
};

struct system_calls_imu {
	int (*read_gyro)(const struct system_calls_imu *self, int16_t out[3]);
	int (*read_acc)(const struct system_calls_imu *self, int16_t out[3]);
};

struct system_calls_leds {
	void (*on)(const struct system_calls_leds *self, uint8_t led, bool on);
	void (*toggle)(const struct system_calls_leds *self, uint8_t led);
};

struct system_calls_time {
	int32_t (*micros)(const struct system_calls_time *self);
};

struct system_calls {
	struct system_calls_pwm pwm;
	struct system_calls_imu imu;
	struct system_calls_leds leds;
	struct system_calls_time time;
};


