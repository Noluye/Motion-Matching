#pragma once

extern "C"
{
#include "raylib.h"
#include "raymath.h"
#include "raygui.h"
}

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

#include <initializer_list>
#include <functional>
#include <assert.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

#include <vector>

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "spring.h"
#include "array.h"
