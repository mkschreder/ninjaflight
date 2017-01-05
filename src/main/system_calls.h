#pragma once

#include <stdint.h>
#include <stdbool.h>
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
 * Specifies how many g is full range of accelerometer
 */
#define SYSTEM_ACCEL_RANGE 2

#define SYSTEM_ACCEL_1G (0x7fff / SYSTEM_ACCEL_RANGE)

/**
 * Helper macro for multiplier to convert gyro reading into deg/s
 */
#define SYSTEM_GYRO_SCALE ((float)SYSTEM_GYRO_RANGE / (int16_t)0x7fff)

/**
 * Helper macro that converts raw accel reading into m/s2
 */
#define SYSTEM_ACCEL_SCALE (((float)SYSTEM_ACCEL_RANGE * 9.82f) / (int16_t)0x7fff)

#if 0
typedef enum {
	STIME,
	SPWMMO,			// motor out
	SPWMSO,			// servo out
	SPWMRI,
	SPPMRI,
	SSGYR,
	SSACC,
	SSPRE,
	SSTMP,
	SLEDO,
	SLEDT,
	SBPR,
	SEEPR,
	SLOG
} sys_reg_t;

struct sys_pwm_args {
	uint8_t id;
	uint16_t value;
};

struct system_calls {
	/**
	 * A generic read call that copies data from the system thread into the
	 * caller thread. Type of data is specified by the register to read.
	 */
	int (*read)(sys_reg_t reg, void *data, size_t data_size);
	/**
	 * A generic write call that writes data to a system device.
	 */
	int (*write)(sys_reg_t reg, const void *data, size_t data_size);
};

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
void sys_pwm_motor_out(const struct system_calls_pwm *self, uint8_t id, uint16_t value);
/**
 * @param self instance of the system calls interface
 * @param id servo id (typically in range 0 - 7)
 * @param value pwm value (typically in range 1000-2000 but can be slightly more or less)
 *
 * Writes a servo value to the given output. If output id is out of range
 * then value should be ignored.
 **/
void sys_pwm_servo_out(const struct system_calls_pwm *self, uint8_t id, uint16_t value);
/**
 * @param chan pwm channel to read
 * @return pwm value of the given channel. If channel is out of range then
 * 0 should be returned.
 */
uint16_t sys_pwm_rc_in(const struct system_calls_pwm *self, uint8_t chan);
/**
 * @param chan ppm channel to read
 * @return ppm value of the given channel. If channel is out of range then
 * 0 should be returned.
 */
uint16_t sys_ppm_rc_in(const struct system_calls_pwm *self, uint8_t chan);
/**
 * Reads gyro rotational rate as a 16 bit integer. Full range (+-2^15) is
 * expected to equal SYSTEM_GYRO_PRECISION degrees per second.
 *
 * @param out array of three
 * components where gyro data will be written.
 * @return negative errno number on error, 0 on success
 */
int sys_gyro_read(const struct system_calls_imu *self, int16_t out[3]);
/**
 * Reads acceleration currently excerting force on the object (in
 * stationary position it should be -g: ie in the opposite direction from
 * gravity). Units are such that SYSTEM_ACC_1G corresponds to 1G.
 *
 * @param out array of three components where gyro data will be
 * written.
 * @return negative errno number on error, 0 on success
 */
int sys_acc_read(const struct system_calls_imu *self, int16_t out[3]);
int sys_pressure_read(const struct system_calls_imu *self, uint32_t *out);
int sys_temp_read(const struct system_calls_imu *self, int16_t *out);

/**
 * Turns a led on or off. If led is out of range then this function should ignore the command.
 */
void sys_led_on(const struct system_calls_leds *self, uint8_t led, bool on);
/**
 * Toggles a led. If led id is out of range then this function should ignore the command.
 */
void sys_led_toggle(const struct system_calls_leds *self, uint8_t led);

/**
 * Turns the internal beeper on or off.
 */
void sys_beeper_on(const struct system_calls_beeper *self, bool on);

sys_micros_t sys_micros(const struct system_calls_time *self);
#endif
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
	 * This function should sleep on the gyro interrupt. If this functionality
	 * is not supported by the system the function should return -1.
	 */
	int (*gyro_sync)(const struct system_calls_imu *self);
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
struct system_bdev_info {
	uint16_t page_size;
	uint16_t num_pages;
};

/**
 * Generic interface to a paged/block memory device used to represent
 * eeprom/flash/file. We use this interface for both the onboard configuration
 * eeprom and for the dataflash that is available on some hardware (for saving
 * logs).
 */
struct system_calls_bdev {
	/**
	 * Reads a block of memory up to a page boundary. Returns number of bytes
	 * read upon success and negative number upon error.
	 */
	int (*read)(const struct system_calls_bdev *self, void *dst, uint16_t addr, size_t size);
	/**
	 * Writes data to the memory up to a page boundary. Returns number of bytes
	 * written or a negative number upon error.
	 */
	int (*write)(const struct system_calls_bdev *self, uint16_t addr, const void *src, size_t size);
	/**
	 * Erases a page of the memory. Address should be first address of the page.
	 */
	int (*erase_page)(const struct system_calls_bdev *self, uint16_t addr);
	/**
	 * Returns information about the memory device such as page size and number of pages.
	 */
	void (*get_info)(const struct system_calls_bdev *self, struct system_bdev_info *info);
};

/**
 * A logger interface that accepts delta frames
 */
struct system_calls_logger {
	/**
	 * Write a block of data to the logger device. If data can not be written
	 * in full then we return number of bytes written.
	 */
	int16_t (*write)(const struct system_calls_logger *self, const void *src, int16_t size);
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
	struct system_calls_bdev eeprom; //! configuration eeprom
	struct system_calls_logger logger;
	//struct system_calls_bdev dataflash; //! dataflash eeprom/flash/whatever
	struct system_calls_range range;
};

#define sys_led_on(sys, id) sys->leds.on(&(sys)->leds, id, true)
#define sys_led_off(sys, id) sys->leds.on(&(sys)->leds, id, false)
#define sys_led_toggle(sys, id) sys->leds.toggle(&(sys)->leds, id)

#define sys_beeper_on(sys) sys->beeper.on(&(sys)->beeper, true)
#define sys_beeper_off(sys) sys->beeper.on(&(sys)->beeper, false)

#define sys_millis(sys) ((sys)->time.micros(&(sys)->time) / 1000)
#define sys_micros(sys) ((sys)->time.micros(&(sys)->time))

#define sys_gyro_sync(sys) ((sys)->imu.gyro_sync(&(sys)->imu))
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

#define sys_logger_write(sys, data, size) (sys->logger.write(&(sys)->logger, data, size))

#define sys_dataflash_write(sys, addr, src, size) (sys->dataflash.write(&(sys)->dataflash, addr, src, size))
#define sys_dataflash_read(sys, dst, addr, size) (sys->dataflash.read(&(sys)->dataflash, dst, addr, size))
#define sys_dataflash_get_info(sys, info) (sys->dataflash.get_info(&(sys)->dataflash, info))
#define sys_dataflash_erase_page(sys, addr) (sys->dataflash.erase_page(&(sys)->dataflash, addr))

// block device functions
#define sys_bdev_write(bdev, addr, src, size) (bdev->write(bdev, addr, src, size))
#define sys_bdev_read(bdev, dst, addr, size) (bdev->read(bdev, dst, addr, size))
#define sys_bdev_get_info(bdev, info) (bdev->get_info(bdev, info))
#define sys_bdev_erase_page(bdev, addr) (bdev->erase_page(bdev, addr))

#define sys_range_read(sys, deg, dst) (sys->range.read_range(&(sys)->range, deg, dst))

/** @} */
/** @} */
