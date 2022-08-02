#pragma once

#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace alcc {
	class Rand {
	public:
		static std::vector<float> UniformFloat(float min, float max, int num);

		// limit in degrees
		// [-rot_lim, rot_lim] , [-trans_lim, trans_lim] 
		static Eigen::Isometry3f SingleUniformIsometry3f(float rot_lim, float trans_lim);
		static std::vector<Eigen::Isometry3f> MultiUniformIsometry3f(float rot_lim, float trans_lim, int num);
	private:
		static void Init();

		class Impl;
		static std::unique_ptr<Impl> impl_;
	};
}