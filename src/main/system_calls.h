#pragma once

#include <stdint.h>
#include <stddef.h>

typedef int32_t sys_millis_t;
typedef int32_t sys_micros_t;

/**
 * @addtogroup ninja
 * @{
 */
/**
 * @addtogroup syscalls
 * @{
 */
/**
 * \file
 * Sysytem calls interface
 * \author
 * Martin Schröder <mkschreder.uk@gmail.com>
 *
 * Main interface between the flight controller and the system. Keep it to a
 * minimum.
 */

/**
 * standard maximum gyro rate in deg/s which the 2^15 (int16) gyro value range
 * represents. System should use this value to scale the gyro readings to 2^15
 * range according to this value.
 */
#define SYSTEM_GYRO_RANGE 2000
/**
 * standard value for accelerometer readings that represents 1G. Drivers should use this value to scale accelerometer readings into this range.
 */
#define SYSTEM_ACC_1G 1000
/**
 * Helper macro for multiplier to convert gyro reading into deg/s
 */
#define SYSTEM_GYRO_SCALE ((float)SYSTEM_GYRO_RANGE / (int16_t)0x7fff)

/**
 * @brief pwm related system calls
 *
 * System calls for reading and writing pwm/ppm values.
 */
struct system_calls_pwm {
	/**
	 * @param self instance of the system calls interface
	 * @param id motor id (typically in range 0 - 7)
	 * @param value pwm value (typically in range 1000-2000 but can be slightly more or less)
	 *
	 * Writes a motor value to a motor output with given id. If id is out of
	 * range then values should be ignored. Motor outputs are typically
	 * separate from servo outputs and run at 490hz instead of 50hz update
	 * rate.
	 */
	void (*write_motor)(const struct system_calls_pwm *self, uint8_t id, uint16_t value);
	/**
	 * @param self instance of the system calls interface
	 * @param id servo id (typically in range 0 - 7)
	 * @param value pwm value (typically in range 1000-2000 but can be slightly more or less)
	 *
	 * Writes a servo value to the given output. If output id is out of range
	 * then value should be ignored.
	 **/
	void (*write_servo)(const struct system_calls_pwm *self, uint8_t id, uint16_t value);
	/**
	 * @param chan pwm channel to read
	 * @return pwm value of the given channel. If channel is out of range then
	 * 0 should be returned.
	 */
	uint16_t (*read_pwm)(const struct system_calls_pwm *self, uint8_t chan);
	/**
	 * @param chan ppm channel to read
	 * @return ppm value of the given channel. If channel is out of range then
	 * 0 should be returned.
	 */
	uint16_t (*read_ppm)(const struct system_calls_pwm *self, uint8_t chan);
};

/**
 * System calls responsible for reading sensor information.
 */
struct system_calls_imu {
	/**
	 * Reads gyro rotational rate as a 16 bit integer. Full range (+-2^15) is
	 * expected to equal SYSTEM_GYRO_PRECISION degrees per second.
	 *
	 * @param out array of three
	 * components where gyro data will be written.
	 * @return negative errno number on error, 0 on success
	 */
	int (*read_gyro)(const struct system_calls_imu *self, int16_t out[3]);
	/**
	 * Reads acceleration currently excerting force on the object (in
	 * stationary position it should be -g: ie in the opposite direction from
	 * gravity). Units are such that SYSTEM_ACC_1G corresponds to 1G.
	 *
	 * @param out array of three components where gyro data will be
	 * written.
	 * @return negative errno number on error, 0 on success
	 */
	int (*read_acc)(const struct system_calls_imu *self, int16_t out[3]);
	int (*read_pressure)(const struct system_calls_imu *self, uint32_t *out);
	int (*read_temperature)(const struct system_calls_imu *self, int16_t *out);
};

/**
 * System calls for interacting with leds.
 */
struct system_calls_leds {
	/**
	 * Turns a led on or off. If led is out of range then this function should ignore the command.
	 */
	void (*on)(const struct system_calls_leds *self, uint8_t led, bool on);
	/**
	 * Toggles a led. If led id is out of range then this function should ignore the command.
	 */
	void (*toggle)(const struct system_calls_leds *self, uint8_t led);
};

/**
 * Interface to a single tone beeper.
 */
struct system_calls_beeper {
	/**
	 * Turns the beeper on or off (on means tone should be heard)
	 */
	void (*on)(const struct system_calls_beeper *self, bool on);
};

/**
 * Timing interface
 */
struct system_calls_time {
	/**
	 * Returns current time in microseconds since program start. This value
	 * should wrap around and sys_micros_t should always be a signed integer to
	 * allow easy wrap-agnostic comparison of timestamps.
	 */
	sys_micros_t (*micros)(const struct system_calls_time *self);
};

/**
 * Onboard eeprom information structure
 */
struct system_eeprom_info {
	uint16_t page_size;
	uint16_t num_pages;
};

/**
 * Interface to onboard config storage eeprom. Assumption is made that there is
 * only one config storage per aircraft instance. If a more advanced config
 * storage interface is desired, this system interface can be changed. An
 * eeprom is typically a paged device and for sake of being compatible with
 * wide variety of different paged devices (including flash) the interface
 * below is used to access the eeprom.
 */
struct system_calls_eeprom {
	/**
	 * Reads a block of memory up to a page boundary. Returns number of bytes
	 * read upon success and negative number upon error.
	 */
	int (*read)(const struct system_calls_eeprom *self, void *dst, uint16_t addr, size_t size);
	/**
	 * Writes data to the eeprom up to a page boundary. Returns number of bytes
	 * written or a negative number upon error.
	 */
	int (*write)(const struct system_calls_eeprom *self, uint16_t addr, const void *src, size_t size);
	/**
	 * Erases a page of the eeprom. Address should be first address of the page.
	 */
	int (*erase_page)(const struct system_calls_eeprom *self, uint16_t addr);
	/**
	 * Returns information about the onboard eeprom such as page size and number of pages.
	 */
	void (*get_info)(const struct system_calls_eeprom *self, struct system_eeprom_info *info);
};

/**
 * Interface to a lateral range sensor. If no range sensor is available then
 * this function should return -1. If all directions are not covered then
 * implementation can decide what to return for the directions that are not
 * covered - either an error can be returned or a mix of surrounding points.
 * For example if only four directions are available then read range may return
 * value of the front sensor for a range of angles.
 */
struct system_calls_range {
	/**
	 * Read range specified by an angle direction. 0 is forward, 90 is to the right etc.
	 */
	int (*read_range)(const struct system_calls_range *self, uint16_t deg, uint16_t *range);
};

/**
 * System calls interface for the flight controller for interacting with the board.
 */
struct system_calls {
	struct system_calls_pwm pwm;
	struct system_calls_imu imu;
	struct system_calls_leds leds;
	struct system_calls_beeper beeper;
	struct system_calls_time time;
	struct system_calls_eeprom eeprom;
	struct system_calls_range range;
};

#define sys_led_on(sys, id) sys->leds.on(&(sys)->leds, id, true)
#define sys_led_off(sys, id) sys->leds.on(&(sys)->leds, id, false)
#define sys_led_toggle(sys, id) sys->leds.toggle(&(sys)->leds, id)

#define sys_beeper_on(sys) sys->beeper.on(&(sys)->beeper, true)
#define sys_beeper_off(sys) sys->beeper.on(&(sys)->beeper, false)

#define sys_millis(sys) (sys->time.micros(&(sys)->time) / 1000)
#define sys_micros(sys) (sys->time.micros(&(sys)->time))

#define sys_gyro_read(sys, data) (sys->imu.read_gyro(&(sys)->imu, data))
#define sys_acc_read(sys, data) (sys->imu.read_acc(&(sys)->imu, data))
#define sys_read_pressure(sys, data) (sys->imu.read_pressure(&(sys)->imu, data))
#define sys_read_temperature(sys, data) (sys->imu.read_temperature(&(sys)->imu, data))

#define sys_motor_write(sys, id, val) (sys->pwm.write_motor(&(sys)->pwm, id, val))
#define sys_servo_write(sys, id, val) (sys->pwm.write_servo(&(sys)->pwm, id, val))

#define sys_eeprom_write(sys, addr, src, size) (sys->eeprom.write(&(sys)->eeprom, addr, src, size))
#define sys_eeprom_read(sys, dst, addr, size) (sys->eeprom.read(&(sys)->eeprom, dst, addr, size))
#define sys_eeprom_get_info(sys, info) (sys->eeprom.get_info(&(sys)->eeprom, info))
#define sys_eeprom_erase_page(sys, addr) (sys->eeprom.erase_page(&(sys)->eeprom, addr))

#define sys_range_read(sys, deg, dst) (sys->range.read_range(&(sys)->range, deg, dst))

/** @} */
/** @} */
