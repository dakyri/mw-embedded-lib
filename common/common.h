#pragma once

#include <stdint.h>

namespace mw {
	template <typename T> inline T pin(T n, T min, T max) {
		return n > max? max : n < min ? min : n;
	}
}
