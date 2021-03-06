###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the ninjaflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#

###############################################################################
# Things that the user might override on the commandline
#

# The target to build, see VALID_TARGETS below
TARGET		?= NAZE

# Compile-time options
OPTIONS		?=
export OPTIONS

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME			 = ninjaflight

SITL_FLAGS = -fPIC -D_XOPEN_SOURCE=2016 -DSITL -Ifreertos/Source/portable/GCC/POSIX/

64K_TARGETS  = CJMCU
128K_TARGETS = ALIENFLIGHTF1 CC3D NAZE OLIMEXINO RMDO
256K_TARGETS = ALIENFLIGHTF3 CHEBUZZF3 COLIBRI_RACE EUSTM32F103RC IRCFUSIONF3 LUX_RACE MOTOLAB NAZE32PRO PORT103R SPARKY SPRACINGF3 SPRACINGF3EVO SPRACINGF3MINI STM32F3DISCOVERY

F3_TARGETS = ALIENFLIGHTF3 CHEBUZZF3 COLIBRI_RACE IRCFUSIONF3 LUX_RACE MOTOLAB NAZE32PRO RMDO SPARKY SPRACINGF3 SPRACINGF3EVO SPRACINGF3MINI STM32F3DISCOVERY

VALID_TARGETS = $(64K_TARGETS) $(128K_TARGETS) $(256K_TARGETS)
VALID_TARGETS += SITL

VCP_TARGETS = CC3D ALIENFLIGHTF3 CHEBUZZF3 COLIBRI_RACE LUX_RACE MOTOLAB NAZE32PRO SPARKY SPRACINGF3EVO SPRACINGF3MINI STM32F3DISCOVERY

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),$(64K_TARGETS)))
FLASH_SIZE = 64
else ifeq ($(TARGET),$(filter $(TARGET),$(128K_TARGETS)))
FLASH_SIZE = 128
else ifeq ($(TARGET),$(filter $(TARGET),$(256K_TARGETS)))
FLASH_SIZE = 256
else ifeq ($(TARGET),SITL)
#skip
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif

REVISION := $(shell git log -1 --format="%h")

# Working directories
ROOT		 := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR		 = $(ROOT)/src/main
OBJECT_DIR	 = $(ROOT)/obj/main
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	 = $(SRC_DIR) $(ROOT)/include/ $(ROOT)/freertos/Source/include
LINKER_DIR	 = $(ROOT)/src/main/target

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR	= $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))

# Compiler flags for coverage instrumentation
COVERAGE_FLAGS = --coverage -DDEBUG -g3 -O0 -fprofile-arcs -ftest-coverage

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
# F3 TARGETS

STDPERIPH_DIR	= $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f30x_crc.c \
		stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = \
		$(STDPERIPH_SRC)


VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \
		   $(ROOT)/src/main/freertos/Source/portable/GCC/ARM_CM4F/\

ifeq ($(TARGET),$(filter $(TARGET),$(VCP_TARGETS)))
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/drivers/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
		   $(USBPERIPH_SRC)

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mno-unaligned-access -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)

else ifeq ($(TARGET),$(filter $(TARGET),EUSTM32F103RC PORT103R))
# TARGETS: EUSTM32F103RC PORT103R


STDPERIPH_DIR	 = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f10x_crc.c \
		stm32f10x_cec.c \
		stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET) 
DEVICE_FLAGS = -DSTM32F10X_HD -DSTM32F10X

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

else ifeq ($(TARGET),SITL)
ARCH_FLAGS = $(SITL_FLAGS)
ifeq ($(PROFILE),1)
ARCH_FLAGS += $(COVERAGE_FLAGS)
endif
LD_SCRIPT = ./src/test/unit/parameter_group.ld
LDFLAGS += -lgcov
DEBUG=GDB
else
# F1 TARGETS

STDPERIPH_DIR	 = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f10x_crc.c \
		stm32f10x_cec.c \
		stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \
		   $(ROOT)/src/main/freertos/Source/portable/GCC/ARM_CM3/\

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

ifeq ($(TARGET),$(filter $(TARGET),$(VCP_TARGETS)))
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(ROOT)/src/main/drivers/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC) \
		   $(USBPERIPH_SRC)

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET)
DEVICE_FLAGS = -DSTM32F10X_MD -DSTM32F10X

endif #TARGETS

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

# VARIANTS
ifeq ($(TARGET),ALIENFLIGHTF1)
# ALIENFLIGHTF1 is a VARIANT of NAZE
TARGET_FLAGS := $(TARGET_FLAGS) -DNAZE -DALIENFLIGHT
TARGET_DIR = $(ROOT)/src/main/target/NAZE
endif
ifeq ($(TARGET),CHEBUZZF3)
# CHEBUZZ is a VARIANT of STM32F3DISCOVERY
TARGET_FLAGS := $(TARGET_FLAGS) -DSTM32F3DISCOVERY
endif
ifeq ($(TARGET),$(filter $(TARGET),RMDO IRCFUSIONF3))
# RMDO and IRCFUSIONF3 are a VARIANT of SPRACINGF3
TARGET_FLAGS := $(TARGET_FLAGS) -DSPRACINGF3
endif


INCLUDE_DIRS := $(INCLUDE_DIRS) \
		    $(TARGET_DIR)

VPATH		:= $(VPATH):$(TARGET_DIR)

COMMON_SRC = build_config.c \
			debug.c \
			version.c \
			$(TARGET_SRC) \
			drivers/config_flash.c \
			config/config.c \
			config/rx.c \
			config/ledstrip.c \
			config/feature.c \
			common/packer.c \
			common/maths.c \
			common/quaternion.c \
			common/buf_writer.c \
			common/printf.c \
			common/typeconversion.c \
			common/encoding.c \
			common/filter.c \
			common/streambuf.c \
			common/ulink.c \
			main.c \
			ninja.c \
			ninja_config.c \
			ninja_sched.c \
			ninjaflight.c \
			fastloop.c \
			cli.c \
			msp.c \
			flight/altitudehold.c \
			flight/failsafe.c \
			flight/anglerate.c \
			flight/mixer.c \
			flight/tilt.c \
			drivers/bus_i2c_soft.c \
			drivers/serial.c \
			drivers/sound_beeper.c \
			drivers/system.c \
			drivers/dma.c \
			io/beeper.c \
			io/rc_adjustments.c \
			io/serial.c \
			io/serial_msp.c \
			io/serial_4way.c \
			io/serial_4way_avrootloader.c \
			io/serial_4way_stk500v2.c \
			io/statusindicator.c \
			rx/msp.c \
			rx/rc.c \
			rx/rc_command.c \
			rx/rx.c \
			rx/pwm.c \
			rx/msp.c \
			rx/sbus.c \
			rx/sumd.c \
			rx/sumh.c \
			rx/spektrum.c \
			rx/xbus.c \
			rx/ibus.c \
			sensors/acceleration.c \
			sensors/battery.c \
			sensors/boardalignment.c \
			sensors/compass.c \
			sensors/gyro.c \
			sensors/initialisation.c \
			sensors/instruments.c \
			sensors/imu.c \
			libutype/src/cbuf.c\
			$(CMSIS_SRC) \
			$(DEVICE_STDPERIPH_SRC)\
			freertos/Source/croutine.c \
			freertos/Source/event_groups.c \
			freertos/Source/list.c \
			freertos/Source/tasks.c \
			freertos/Source/timers.c \
			freertos/Source/queue.c \
			freertos/Source/portable/MemMang/heap_2.c \

HIGHEND_SRC = \
		   flight/gtune.c \
		   flight/navigation.c \
		   flight/gps_conversion.c \
		   common/colorconversion.c \
		   io/ledstrip.c \
		   io/display.c \
		   telemetry/telemetry.c \
		   telemetry/frsky.c \
		   telemetry/hott.c \
		   telemetry/smartport.c \
		   telemetry/ltm.c \
		   telemetry/mavlink.c \
		   sensors/gps.c \
		   sensors/sonar.c \
		   sensors/barometer.c \
		   blackbox.c \
		   blackbox/blackbox_io.c

VCP_SRC = \
		   drivers/vcp/hw_config.c \
		   drivers/vcp/stm32_it.c \
		   drivers/vcp/usb_desc.c \
		   drivers/vcp/usb_endp.c \
		   drivers/vcp/usb_istr.c \
		   drivers/vcp/usb_prop.c \
		   drivers/vcp/usb_pwr.c \
		   drivers/serial_usb_vcp.c \
		   drivers/usb_io.c 

NAZE_SRC = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/barometer_bmp280.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   drivers/flashfs.c \
		   hardware_revision.c \
			freertos/Source/portable/GCC/ARM_CM3/port.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

ALIENFLIGHTF1_SRC = $(NAZE_SRC)

EUSTM32F103RC_SRC = startup_stm32f10x_hd_gcc.S \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/flash_m25p16.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   drivers/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

PORT103R_SRC = $(EUSTM32F103RC_SRC)

OLIMEXINO_SRC = startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/bus_spi.c \
		   drivers/compass_hmc5883l.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

CJMCU_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   hardware_revision.c \
		   flight/gtune.c \
		   blackbox.c \
		   blackbox/blackbox_io.c \
		   $(COMMON_SRC)

CC3D_SRC = \
		   startup_stm32f10x_md_gcc.S \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/adc.c \
		   drivers/adc_stm32f10x.c \
		   drivers/barometer_bmp085.c \
		   drivers/barometer_ms5611.c \
		   drivers/bus_spi.c \
		   drivers/bus_i2c_stm32f10x.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/flash_m25p16.c \
		   drivers/gpio_stm32f10x.c \
		   drivers/inverter.c \
		   drivers/light_led_stm32f10x.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f10x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_softserial.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f10x.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sound_beeper_stm32f10x.c \
		   drivers/system_stm32f10x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f10x.c \
		   drivers/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

STM32F30x_COMMON_SRC = \
		   startup_stm32f30x_md_gcc.S \
		   drivers/adc.c \
		   drivers/adc_stm32f30x.c \
		   drivers/bus_i2c_stm32f30x.c \
		   drivers/bus_spi.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/gpio_stm32f30x.c \
		   drivers/light_led_stm32f30x.c \
		   drivers/pwm_mapping.c \
		   drivers/pwm_output.c \
		   drivers/pwm_rx.c \
		   drivers/serial_uart.c \
		   drivers/serial_uart_stm32f30x.c \
		   drivers/sound_beeper_stm32f30x.c \
		   drivers/system_stm32f30x.c \
		   drivers/timer.c \
		   drivers/timer_stm32f30x.c \
			freertos/Source/portable/GCC/ARM_CM4F/port.c \

NAZE32PRO_SRC = \
		   $(COMMON_SRC) \
		   $(STM32F30x_COMMON_SRC) \
		   $(HIGHEND_SRC) \
		   $(VCP_SRC)

STM32F3DISCOVERY_COMMON_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_l3gd20.c \
		   drivers/accgyro_lsm303dlhc.c \
		   drivers/compass_hmc5883l.c \
		   $(VCP_SRC)

STM32F3DISCOVERY_SRC = \
		   $(STM32F3DISCOVERY_COMMON_SRC) \
		   drivers/accgyro_adxl345.c \
		   drivers/accgyro_bma280.c \
		   drivers/accgyro_mma845x.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu3050.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_l3g4200d.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/sdcard.c \
		   drivers/sdcard_standard.c \
		   drivers/asyncfatfs/asyncfatfs.c \
		   drivers/asyncfatfs/fat_standard.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

CHEBUZZF3_SRC = \
		   $(STM32F3DISCOVERY_SRC) \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

COLIBRI_RACE_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8963.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)
		   
LUX_RACE_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)		   

SPARKY_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sonar_hcsr04.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

ALIENFLIGHTF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/display_ug2864hsweg01.c \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/compass_ak8963.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sonar_hcsr04.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

RMDO_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_bmp280.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   drivers/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

SPRACINGF3_SRC = \
		   $(COMMON_SRC) \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_ak8975.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/sonar_hcsr04.c \
		   drivers/flashfs.c \
		   $(HIGHEND_SRC) \

SPRACINGF3EVO_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/accgyro_spi_mpu6500.c \
		   drivers/barometer_bmp280.c \
		   drivers/compass_ak8963.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sdcard.c \
		   drivers/sdcard_standard.c \
		   drivers/transponder_ir.c \
		   drivers/transponder_ir_stm32f30x.c \
		   drivers/asyncfatfs/asyncfatfs.c \
		   drivers/asyncfatfs/fat_standard.c \
		   io/transponder_ir.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

MOTOLAB_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_spi_mpu6000.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_ms5611.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_usb_vcp.c \
		   drivers/flash_m25p16.c \
		   drivers/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)

SPRACINGF3MINI_SRC	 = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6500.c \
		   drivers/barometer_bmp280.c \
		   drivers/compass_ak8963.c \
		   drivers/compass_hmc5883l.c \
		   drivers/display_ug2864hsweg01.h \
		   drivers/flash_m25p16.c \
		   drivers/light_ws2811strip.c \
		   drivers/light_ws2811strip_stm32f30x.c \
		   drivers/serial_softserial.c \
		   drivers/serial_usb_vcp.c \
		   drivers/sonar_hcsr04.c \
		   drivers/sdcard.c \
		   drivers/sdcard_standard.c \
		   drivers/transponder_ir.c \
		   drivers/transponder_ir_stm32f30x.c \
		   drivers/asyncfatfs/asyncfatfs.c \
		   drivers/asyncfatfs/fat_standard.c \
		   io/transponder_ir.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC) \
		   $(VCP_SRC)
		   
IRCFUSIONF3_SRC = \
		   $(STM32F30x_COMMON_SRC) \
		   drivers/accgyro_mpu.c \
		   drivers/accgyro_mpu6050.c \
		   drivers/barometer_bmp085.c \
		   drivers/flash_m25p16.c \
		   drivers/flashfs.c \
		   $(HIGHEND_SRC) \
		   $(COMMON_SRC)

SITL_SRC = \
		blackbox.c \
		blackbox/blackbox_io.c \
		build_config.c \
		cli.c \
		common/buf_writer.c \
		common/colorconversion.c \
		common/encoding.c \
		common/filter.c \
		common/packer.c \
		common/maths.c \
		common/printf.c \
		common/quaternion.c \
		common/streambuf.c \
		common/typeconversion.c \
		common/ulink.c \
		config/config.c \
		config/rx.c \
		config/ledstrip.c \
		config/feature.c \
		debug.c \
		flight/altitudehold.c \
		flight/anglerate.c \
		flight/failsafe.c \
		flight/gps_conversion.c \
		flight/gtune.c \
		flight/mixer.c \
		flight/navigation.c \
		flight/tilt.c \
		io/beeper.c \
		io/display.c \
		io/ledstrip.c \
		io/rc_adjustments.c \
		io/serial.c \
		io/serial_msp.c \
		io/statusindicator.c \
		io/transponder_ir.c \
		msp.c \
		ninja.c \
		fastloop.c \
		ninja_config.c \
		ninjaflight.c \
		ninja_sched.c \
		rx/ibus.c \
		rx/msp.c \
		rx/pwm.c \
		rx/rc.c \
		rx/rc_command.c \
		rx/rx.c \
		rx/sbus.c \
		rx/spektrum.c \
		rx/sumd.c \
		rx/sumh.c \
		rx/xbus.c \
		sensors/acceleration.c \
		sensors/barometer.c \
		sensors/battery.c \
		sensors/boardalignment.c \
		sensors/compass.c \
		sensors/gps.c \
		sensors/gyro.c \
		sensors/imu.c \
		sensors/instruments.c \
		sensors/sonar.c \
		sitl/main.c \
		sitl/sys_pqueue.c \
		telemetry/frsky.c \
		telemetry/hott.c \
		telemetry/ltm.c \
		telemetry/mavlink.c \
		telemetry/smartport.c \
		telemetry/telemetry.c \
		version.c \
		drivers/serial.c \
		freertos/Source/croutine.c \
		freertos/Source/event_groups.c \
		freertos/Source/list.c \
		freertos/Source/tasks.c \
		freertos/Source/timers.c \
		freertos/Source/queue.c \
		freertos/Source/portable/GCC/POSIX/port.c \
		freertos/Source/portable/MemMang/heap_2.c \
		libutype/src/cbuf.c\
		../../ninjasitl/src/fc_sitl.c 

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
ifeq ($(TARGET),SITL)
SIZE = size
#ARCH_FLAGS += -fPIC -D_XOPEN_SOURCE=2016 $(COVERAGE_FLAGS) 
#LD_SCRIPT = ./src/test/unit/parameter_group.ld
#LDFLAGS += -lgcov -Wl,-gc-sections,-Map,$(TARGET_MAP) 
else
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy
SIZE		 = arm-none-eabi-size
C_FLAGS += -ffunction-sections -fdata-sections 
LDFLAGS += -nostartfiles \
		   --specs=nano.specs \
		   -lc \
		   -static \
		   -Wl,-u,vTaskSwitchContext \
		   -Wl,-gc-sections \
		   -lnosys 
endif

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -O2
LTO_FLAGS	 =  -flto -fuse-linker-plugin $(OPTIMIZE)
endif

ifeq ($(STACK_USAGE),1)
WARN_FLAGS		+= -Wstack-usage=100 -fstack-usage
else
# I have not found a way to suppress -fstack-usage for asm functions so -Werror effectively ensures we can not compile when we use -fstack-usage
WARN_FLAGS      += -Werror
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -std=gnu99 \
           -Wall \
           -Wbad-function-cast \
           -Wdouble-promotion \
           -Wextra \
           -Wfloat-equal \
           -Wformat=2 \
           -Wlogical-op \
           -Wmissing-field-initializers \
           -Wmissing-include-dirs \
           -Wmissing-include-dirs \
           -Wunreachable-code \
           -Wno-error=nested-externs \
           -Wno-error=cast-align \
           -Wno-error=cast-qual \
           -Wno-error=conversion \
           -Wold-style-definition \
           -Wpointer-arith \
           -Wredundant-decls \
           -Wreturn-type \
           -Wshadow \
           -Wswitch-enum \
		   -Wno-error=strict-overflow \
		   -Wno-error=switch-enum \
           -Wno-error=switch-default \
           -Wno-error=redundant-decls \
           -Wundef \
           -Wuninitialized \
           -Wunsafe-loop-optimizations \
           -Wwrite-strings \
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   $(TARGET_FLAGS) \
		   -D'__FORKNAME__="$(FORKNAME)"' \
		   -D'__TARGET__="$(TARGET)"' \
		   -D'__REVISION__="$(REVISION)"' \
		   -fverbose-asm -ffat-lto-objects \
		   -save-temps=obj \
		   -MMD -MP

ASFLAGS		 = $(ARCH_FLAGS) \
		   $(WARN_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		  -MMD -MP

LDFLAGS	+= -lm \
			$(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -Wl,-L$(LINKER_DIR) \
		   $(if $(LD_SCRIPT),-T$(LD_SCRIPT),)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK         = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
		   --std=c99 --inline-suppr --quiet --force \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -I/usr/include -I/usr/include/linux

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_BIN	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).hex
TARGET_ELF	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_SITL	 = lib$(FORKNAME).so
TARGET_NATIVE = ninjaflight
TARGET_SITL_LIB	 = lib$(FORKNAME).a
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

## Default make goal:
## hex         : Make filetype hex only
#.DEFAULT_GOAL := hex

## Optional make goals:
## all         : Make all filetypes, binary and hex
ifeq ($(TARGET),SITL)
all: ninjasitl $(TARGET_SITL_LIB) $(TARGET_SITL) $(TARGET_NATIVE)
else
all: hex bin
endif 

## binary      : Make binary filetype
## bin         : Alias of 'binary'
## hex         : Make hex filetype
bin:    $(TARGET_BIN)
binary: $(TARGET_BIN)
hex:    $(TARGET_HEX)

# rule to reinvoke make with TARGET= parameter
# rules that should be handled in toplevel Makefile, not dependent on TARGET
GLOBAL_GOALS	= all_targets cppcheck test

start-sitl: 
	make TARGET=SITL
	cp $(TARGET_SITL) ninjasitl/fc_ninjaflight.so
	cd ninjasitl && ./start-quadsim.sh

.PHONY: $(VALID_TARGETS) docs ninjasitl lcov start-sitl
$(VALID_TARGETS):
	$(MAKE) TARGET=$@ $(filter-out $(VALID_TARGETS) $(GLOBAL_GOALS), $(MAKECMDGOALS))

## all_targets : Make all TARGETs
.PHONY: all_targets
all_targets : $(VALID_TARGETS)

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)
	cd src/test && $(MAKE) clean || true

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash       : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	st-flash --reset write $< 0x08000000

## st-flash    : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick     : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck    : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

## docs
docs:
	doxygen scripts/doxygen

## help        : print this help message and exit
help: Makefile
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [goal] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $<

## test        : run the ninjaflight test suite
## junittest   : run the ninjaflight test suite, producing Junit XML result files.
test junittest:
	cd src/test && $(MAKE) $@
	make lcov

lcov:
	# visualize coverage
	lcov --directory obj/main -b src/main --capture --output-file coverage.info
	lcov --remove coverage.info 'lib/test/*' 'src/test/*' '/usr/*' 'ninjasitl/*' --output-file coverage.info
	lcov --list coverage.info
	if [ "$$(which genhtml)" != "" ]; then genhtml coverage.info --output-directory coverage-html; fi

test-memory test-cache test-stack:
	cd src/test && $(MAKE) $@

# sitl source code needs to be checked out from my git repo
ninjasitl:
	if [ ! -d ninjasitl ]; then git clone https://github.com/mkschreder/ninjasitl.git ninjasitl; fi

$(ROOT)/include/utype:
	if [ ! -d libutype/src/ ]; then echo "Please checkout utype to libutype directory (https://github.com/mkschreder/libutype.git)"; exit 1; fi
	mkdir -p include
	if [ ! -d $(ROOT)/include/utype ]; then ln -s ../libutype/src $(ROOT)/include/utype || echo "Not creating link to utype. Already exists."; fi

# rebuild everything when makefile changes
$(TARGET_OBJS) : $(ROOT)/include/utype Makefile

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_SITL_LIB): $(TARGET_OBJS)
	$(AR) rcs lib$(FORKNAME).a $^
	
$(TARGET_SITL): $(TARGET_OBJS)
	$(CC) -shared -Wl,--no-undefined -o $@ $^ $(LDFLAGS) -ldl -lpthread
	cp $(TARGET_SITL) ninjasitl/fc_ninjaflight.so
	$(SIZE) $(TARGET_SITL)

bbdump: src/main/bb_dump.c
	make TARGET=SITL
	gcc -std=gnu99 -Isrc/main -Isrc/main/target/SITL -I./include  -D_XOPEN_SOURCE=2016 -DSITL -Isrc/main/freertos/Source/include -Isrc/main/freertos/Source/portable/GCC/POSIX/ -o $@ src/main/bb_dump.c -L. -lninjaflight

$(TARGET_NATIVE): sitl.o
	$(CC) -Wl,--no-undefined -o $@ $^ $(LDFLAGS) -ldl -lpthread
	
$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<



# include auto-generated dependencies
-include $(TARGET_DEPS)
