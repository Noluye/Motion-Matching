#pragma once
#include "mmpch.h"

#include "character.h"
#include "database.h"
#include "nnet.h"
#include "lmm.h"

static inline Vector3 to_Vector3(vec3 v) { return Vector3{ v.x, v.y, v.z }; }