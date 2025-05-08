#pragma once

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <utility>

#include "utilities.h"

namespace agm
{
	namespace detail
	{
		class Xoshiro128PlusPlus
		{
		public:
			explicit Xoshiro128PlusPlus(uint32_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count())
			{
				SetSeed(seed);
			}

			void SetSeed(uint32_t seed)
			{
				uint32_t z = seed + 0x9E3779B9UL;
				for (auto& s : mState)
				{
					z = (z ^ (z >> 15)) * 0x85EBCA77UL;
					z = (z ^ (z >> 13)) * 0xC2B2AE3DUL;
					s = z ^ (z >> 16);
				}
			}

			uint32_t Next()
			{
				uint32_t result = std::rotl(mState[0] + mState[3], 7) + mState[0];
				uint32_t t = mState[1] << 9;

				mState[2] ^= mState[0];
				mState[3] ^= mState[1];
				mState[1] ^= mState[2];
				mState[0] ^= mState[3];
				mState[2] ^= t;
				mState[3] = std::rotl(mState[3], 11);

				return result;
			}

		private:
			std::array<uint32_t, 4> mState;
		};

		inline Xoshiro128PlusPlus& GetRNG()
		{
			thread_local Xoshiro128PlusPlus rng;
			return rng;
		}
	}

	inline void SetSeed(uint32_t seed)
	{
		detail::GetRNG().SetSeed(seed);
	}

	inline uint32_t Rand()
	{
		return detail::GetRNG().Next();
	}

	inline float Rand01()
	{
		return std::ldexp(static_cast<float>(Rand()), -32);
	}

	inline int32_t RandRange(int32_t minInclusive, int32_t maxExclusive)
	{
		if (minInclusive == maxExclusive)
		{
			return minInclusive;
		}
		else if (minInclusive > maxExclusive)
		{
			std::swap(minInclusive, maxExclusive);
		}

		uint32_t range = static_cast<uint32_t>(maxExclusive) - static_cast<uint32_t>(minInclusive);
		uint32_t threshold = (~range + 1) % range;
		uint32_t value;

		do
		{
			value = Rand();
		} while (value < threshold);

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
		return Rand01() < probability;
	}
}

