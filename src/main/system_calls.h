#pragma once

#include <stdint.h>
#include <stddef.h>

typedef int32_t sys_millis_t;
typedef int32_t sys_micros_t;

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

struct system_calls_beeper {
	void (*on)(const struct system_calls_beeper *self, bool on);
};

struct system_calls_time {
	sys_micros_t (*micros)(const struct system_calls_time *self);
};

struct system_eeprom_info {
	uint16_t page_size;
	uint16_t num_pages;
};

struct system_calls_eeprom {
	int (*read)(const struct system_calls_eeprom *self, void *dst, uint16_t addr, size_t size);
	int (*write)(const struct system_calls_eeprom *self, uint16_t addr, const void *src, size_t size);
	int (*erase_page)(const struct system_calls_eeprom *self, uint16_t addr);
	void (*get_info)(const struct system_calls_eeprom *self, struct system_eeprom_info *info);
};

struct system_calls {
	struct system_calls_pwm pwm;
	struct system_calls_imu imu;
	struct system_calls_leds leds;
	struct system_calls_beeper beeper;
	struct system_calls_time time;
	struct system_calls_eeprom eeprom;
};

// these are just for convenience
#define sys_led_on(sys, id) sys->leds.on(&sys->leds, id, true)
#define sys_led_off(sys, id) sys->leds.on(&sys->leds, id, false)
#define sys_led_toggle(sys, id) sys->leds.toggle(&sys->leds, id)

#define sys_beeper_on(sys) sys->beeper.on(&sys->beeper, true)
#define sys_beeper_off(sys) sys->beeper.on(&sys->beeper, false)

#define sys_millis(sys) (sys->time.micros(&sys->time) / 1000)
#define sys_micros(sys) (sys->time.micros(&sys->time))

#define sys_gyro_read(sys, data) (sys->imu.read_gyro(&sys->imu, data))
#define sys_acc_read(sys, data) (sys->imu.read_acc(&sys->imu, data))

#define sys_motor_write(sys, id, val) (sys->pwm.write_motor(&sys->pwm, id, val))
#define sys_servo_write(sys, id, val) (sys->pwm.write_servo(&sys->pwm, id, val))

#define sys_eeprom_write(sys, addr, src, size) (sys->eeprom.write(&sys->eeprom, addr, src, size))
#define sys_eeprom_read(sys, dst, addr, size) (sys->eeprom.read(&sys->eeprom, dst, addr, size))
#define sys_eeprom_get_info(sys, info) (sys->eeprom.get_info(&sys->eeprom, info))
#define sys_eeprom_erase_page(sys, addr) (sys->eeprom.erase_page(&sys->eeprom, addr))

