#pragma once

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <utility>

#include "utilities.hpp"

namespace agm
{
	namespace detail
	{
		class Xoshiro128PlusPlus
		{
		public:
			explicit Xoshiro128PlusPlus(uint32_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count())
			{
				set_seed(seed);
			}

			void set_seed(uint32_t seed)
			{
				uint32_t z = seed + 0x9E3779B9UL;
				for (auto& s : m_state)
				{
					z = (z ^ (z >> 15)) * 0x85EBCA77UL;
					z = (z ^ (z >> 13)) * 0xC2B2AE3DUL;
					s = z ^ (z >> 16);
				}
			}

			uint32_t next()
			{
				uint32_t result = std::rotl(m_state[0] + m_state[3], 7) + m_state[0];
				uint32_t t = m_state[1] << 9;

				m_state[2] ^= m_state[0];
				m_state[3] ^= m_state[1];
				m_state[1] ^= m_state[2];
				m_state[0] ^= m_state[3];
				m_state[2] ^= t;
				m_state[3] = std::rotl(m_state[3], 11);

				return result;
			}

		private:
			std::array<uint32_t, 4> m_state;
		};

		inline Xoshiro128PlusPlus& get_rng()
		{
			thread_local Xoshiro128PlusPlus rng;
			return rng;
		}
	}

	inline void set_seed(uint32_t seed)
	{
		detail::get_rng().set_seed(seed);
	}

	inline uint32_t rand()
	{
		return detail::get_rng().next();
	}

	inline float rand01()
	{
		static constexpr float inv_max = 1.f / static_cast<float>(UINT32_MAX);
		return static_cast<float>(rand()) * inv_max;
	}

	inline int32_t rand_range(int32_t min_inclusive, int32_t max_exclusive)
	{
		if (min_inclusive == max_exclusive)
		{
			return min_inclusive;
		}
		else if (min_inclusive > max_exclusive)
		{
			std::swap(min_inclusive, max_exclusive);
		}

		uint32_t range = static_cast<uint32_t>(max_exclusive) - static_cast<uint32_t>(min_inclusive);
		uint32_t threshold = (~range + 1) % range;
		uint32_t value;

		do
		{
			value = rand();
		} while (value < threshold);

		return static_cast<int32_t>(min_inclusive + (value % range));
	}

	inline float rand_range(float min_inclusive, float max_inclusive)
	{
		if (min_inclusive > max_inclusive)
		{
			std::swap(min_inclusive, max_inclusive);
		}

		return min_inclusive + (max_inclusive - min_inclusive) * rand01();
	}

	inline bool rand_bool(float probability = 0.5f)
	{
		probability = clamp01(probability);
		return rand01() < probability;
	}
}

