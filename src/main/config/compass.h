#pragma once

typedef struct compassConfig_s {
    int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
} compassConfig_t;

PG_DECLARE_PROFILE(compassConfig_t, compassConfig);
