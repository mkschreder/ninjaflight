#pragma once

typedef struct quaternion {
	float w, x, y, z;
} quat_t;

quat_t quat_scale(const quat_t *q, float scale);
quat_t quat_add(const quat_t *a, const quat_t *b);
quat_t quat_mul(const quat_t *a, const quat_t *b);
quat_t quat_normalize(const quat_t *q);
