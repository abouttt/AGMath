#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>

#include "utilities.h"

namespace agm
{
	namespace detail
	{
		class Xoshiro128PlusPlus
		{
		public:
			explicit Xoshiro128PlusPlus(uint64_t seed = 0)
			{
				if (seed == 0)
				{
					seed = static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
				}

				SetSeed(seed);
			}

			void SetSeed(uint64_t seed)
			{
				uint64_t z = seed + 0x9E3779B97F4A7C15ULL;
				for (int i = 0; i < 4; ++i)
				{
					z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
					z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
					mState[i] = static_cast<uint32_t>((z ^ (z >> 31)) >> (i % 2 * 32));
				}

				if (mState[0] == 0 && mState[1] == 0 && mState[2] == 0 && mState[3] == 0)
				{
					mState[0] = 0xBAD5EED1;
					mState[1] = 0xBAD5EED2;
					mState[2] = 0xBAD5EED3;
					mState[3] = 0xBAD5EED4;
				}
			}

			uint32_t Next()
			{
				const uint32_t result = std::rotl(mState[0] + mState[3], 7) + mState[0];
				const uint32_t t = mState[1] << 9;

				mState[2] ^= mState[0];
				mState[3] ^= mState[1];
				mState[1] ^= mState[2];
				mState[0] ^= mState[3];
				mState[2] ^= t;
				mState[3] = std::rotl(mState[3], 11);

				return result;
			}

		private:
			std::array<uint32_t, 4> mState{};
		};

		struct NormalCache
		{
			bool hasValue = false;
			float cachedValue = 0.f;
		};

		inline Xoshiro128PlusPlus& GetRNG()
		{
			thread_local Xoshiro128PlusPlus rng;
			return rng;
		}

		inline NormalCache& GetNormalCache()
		{
			thread_local NormalCache cache;
			return cache;
		}
	}

	inline void SetSeed(uint64_t seed)
	{
		detail::GetRNG().SetSeed(seed);
	}

	inline uint32_t Rand()
	{
		return detail::GetRNG().Next();
	}

	inline float Rand01()
	{
		return static_cast<float>(Rand()) / static_cast<float>(std::numeric_limits<uint32_t>::max());
	}

	inline int32_t RandRange(int32_t minInclusive, int32_t maxExclusive)
	{
		if (minInclusive == maxExclusive)
		{
			return minInclusive;
		}

		if (minInclusive > maxExclusive)
		{
			std::swap(minInclusive, maxExclusive);
		}

		uint32_t range = static_cast<uint32_t>(maxExclusive) - static_cast<uint32_t>(minInclusive);
		if (range == 0)
		{
			return minInclusive;
		}

		uint32_t limit = (std::numeric_limits<uint32_t>::max() / range) * range;
		uint32_t value;
		do
		{
			value = Rand();
		} while (value >= limit && limit != 0);

		return static_cast<int32_t>(minInclusive + (value % range));
	}

	inline float RandRange(float minInclusive, float maxInclusive)
	{
		if (minInclusive > maxInclusive)
		{
			std::swap(minInclusive, maxInclusive);
		}

		return minInclusive + (maxInclusive - minInclusive) * Rand01();
	}

	inline bool RandBool(float probability = 0.5f)
	{
		probability = Clamp01(probability);

		if (probability == 0.f)
		{
			return false;
		}
		else if (probability == 1.f)
		{
			return true;
		}

		return Rand01() < probability;
	}

	inline int32_t RandSign()
	{
		return RandBool(0.5f) ? 1 : -1;
	}

	inline float RandNormal(float mean = 0.f, float stddev = 1.f)
	{
		auto& cache = detail::GetNormalCache();
		if (cache.hasValue)
		{
			cache.hasValue = false;
			return cache.cachedValue * stddev + mean;
		}

		float u1;
		do
		{
			u1 = Rand01();
		} while (u1 <= EPSILON);

		float u2 = Rand01();
		float R = std::sqrt(-2.f * std::log(u1));
		float angle = TWO_PI * u2;
		float z0 = R * std::cos(angle);
		float z1 = R * std::sin(angle);

		cache.cachedValue = z1;
		cache.hasValue = true;

		return z0 * stddev + mean;
	}
}

