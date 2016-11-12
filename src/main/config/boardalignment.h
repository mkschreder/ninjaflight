#pragma once

struct board_alignment_config {
    int16_t rollDegrees;
    int16_t pitchDegrees;
    int16_t yawDegrees;
} __attribute__((packed));

PG_DECLARE(struct board_alignment_config, boardAlignment);
