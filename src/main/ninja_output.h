#pragma once

struct ninja;

//! Retreives output pwm value (servos and motors are treated the same). Range [1000;2000]
uint16_t ninja_output_get_value(struct ninja *self, uint8_t idx);
