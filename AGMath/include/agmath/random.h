#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <random>

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
				std::seed_seq seq
				{
					static_cast<uint32_t>(seed),
					static_cast<uint32_t>(seed >> 32),
					0x9E3779B9U,
					0x85EBCA6BU,
					0xC2B2AE35U
				};

				seq.generate(mState.begin(), mState.end());

				if (std::all_of(mState.begin(), mState.end(), [](uint32_t x) { return x == 0; }))
				{
					mState = { 0xBAD5EED1, 0xBAD5EED2, 0xBAD5EED3, 0xBAD5EED4 };
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

			std::array<uint32_t, 4> GetState() const
			{
				return mState;
			}

			void SetState(const std::array<uint32_t, 4>& state)
			{
				mState = state;
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
		detail::GetNormalCache().hasValue = false;
	}

	inline auto GetSeedState()
	{
		return detail::GetRNG().GetState();
	}

	inline void RestoreSeedState(const std::array<uint32_t, 4>& state)
	{
		detail::GetRNG().SetState(state);
		detail::GetNormalCache().hasValue = false;
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

		uint64_t diff = static_cast<uint64_t>(maxExclusive) - static_cast<uint64_t>(minInclusive);
		uint32_t range = static_cast<uint32_t>(diff);
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
		if (IsNearlyEqual(minInclusive, maxInclusive))
		{
			return minInclusive;
		}

		if (minInclusive > maxInclusive)
		{
			std::swap(minInclusive, maxInclusive);
		}

		return minInclusive + (maxInclusive - minInclusive) * Rand01();
	}

	inline bool RandBool(float probability = 0.5f)
	{
		probability = Clamp01(probability);

		if (IsNearlyEqual(probability, 1.f))
		{
			return true;
		}

		if (IsNearlyEqual(probability, 0.f))
		{
			return false;
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

		float u1, u2;
		do
		{
			u1 = Rand01();
		} while (u1 <= EPSILON);
		u2 = Rand01();

		float R = std::sqrt(-2.f * std::log(u1));
		float angle = TWO_PI * u2;

		float z0 = R * std::cos(angle);
		float z1 = R * std::sin(angle);

		cache.cachedValue = z1;
		cache.hasValue = true;

		return z0 * stddev + mean;
	}
}

