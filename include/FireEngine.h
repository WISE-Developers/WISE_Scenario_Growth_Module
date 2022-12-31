/**
 * WISE_Scenario_Growth_Module: FireEngine_ext.h
 * Copyright (C) 2023  WISE
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "math_cfg.h"

//only include this part inside of FireEngine
#ifdef FIRE_EXPORTS
#if defined(SUPPORT_BIGFLOATS) && defined(USE_BIGFLOATS)
#include "quad_inlines.h"

typedef __float128 fireengine_float_type;
#define DELAUNAY_REMOVE_FLOAT80_TEMPLATE
#define DELAUNAY_REMOVE_DOUBLE_TEMPLATE
#else
typedef double fireengine_float_type;
#define DELAUNAY_REMOVE_FLOAT80_TEMPLATE
#define DELAUNAY_REMOVE_FLOAT128_TEMPLATE
#endif //SUPPORT_BIGFLOATS
#endif //FIRE_EXPORTS

#if defined(_MSC_VER) || defined(__CYGWIN__)
#  ifdef FIRE_EXPORTS
#    ifdef __GNUC__
#      define FIRECOM_API __attribute__((dllexport))
#      define NO_THROW __attribute__((nothrow))
#    else
#      define FIRECOM_API __declspec(dllexport)
#      define NO_THROW __declspec(nothrow)
#    endif
#  else
#    ifdef __GNUC__
#      define FIRECOM_API __attribute__((dllimport))
#      define NO_THROW __attribute__((nothrow))
#    else
#      define FIRECOM_API __declspec(dllimport)
#      define NO_THROW __declspec(nothrow)
#    endif
#  endif
#else
#  define NO_THROW __attribute__((nothrow))
#  if __GNUC__ >= 4
#    define FIRECOM_API __attribute__((visibility("default")))
#  else
#    define FIRECOM_API
#  endif
#endif
