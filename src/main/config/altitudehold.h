#pragma once

struct airplane_althold_config {
    int8_t fixedwing_althold_dir;           // +1 or -1 for pitch/althold gain. later check if need more than just sign
};

PG_DECLARE(struct airplane_althold_config, airplaneConfig);
