#include "quaternion.h"
#include "maths.h"
#include <math.h>

quat_t quat_scale(const quat_t *q, float scale){
	return (quat_t){ q->w * scale, q->x * scale, q->y * scale, q->z * scale };
}

quat_t quat_add(const quat_t *a, const quat_t *b){
	return (quat_t){ a->w + b->w, a->x + b->x, a->y + b->y, a->z + b->z };
}

quat_t quat_normalize(const quat_t *q){
	float norm = sq(q->w) + sq(q->x) + sq(q->y) + sq(q->z);
    if(fabsf(1.0f - norm) < 2.107342e-08f){
        norm = 2.0f / (1.0f + norm);
    } else {
        norm = 1.0f / sqrtf(norm);
    }
	return (quat_t){ q->w * norm, q->x * norm, q->y * norm, q->z * norm };
}

quat_t quat_mul(const quat_t *a, const quat_t *b){
	return (quat_t){
		a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z,
		a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y,
		a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x,
		a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w
	};
}

