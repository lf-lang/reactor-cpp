/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

// A custom definition of assert that mitigates unused variable warnings
// when assertions are disabled
#ifdef NDEBUG
#define ASSERT(x) do { (void)sizeof(x);} while (0)
#else
#include <cassert>
#define ASSERT(x) assert(x)
#endif
