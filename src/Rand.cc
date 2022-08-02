#include <random>

#include "Rand.hpp"
#include "Constants.hpp"

using alcc::Rand;
using alcc::Constants;

std::unique_ptr<Rand::Impl> Rand::impl_ = nullptr;

class Rand::Impl {
public:
	Impl() : mt_(std::random_device()()) {}

	std::mt19937 mt_;
};

void Rand::Init() {
	if (!impl_) {
		impl_.reset(new Impl);
	}
}

std::vector<float> Rand::UniformFloat(float min, float max, int num) {
	Init();

	std::uniform_real_distribution<float> udist(min, max);
	std::mt19937& mt = impl_->mt_;

	std::vector<float> result(num);
	for (int i = 0; i < num; ++i) {
		result[i] = udist(mt);
	}

	return result;
}

Eigen::Isometry3f Rand::SingleUniformIsometry3f(float rot_lim, float trans_lim) {
	Init();

	return MultiUniformIsometry3f(rot_lim, trans_lim, 1).front();
}

std::vector<Eigen::Isometry3f> Rand::MultiUniformIsometry3f(float rot_lim, float trans_lim, int num) {
	using Eigen::Quaternionf;
	using Eigen::AngleAxisf;
	using Eigen::Vector3f;
	using Eigen::Isometry3f;

	Init();

	std::vector<float> trans_rands = UniformFloat(-trans_lim, trans_lim, 3 * num);
	std::vector<float> rot_rands = UniformFloat(-rot_lim, rot_lim, 3 * num);

	std::vector<float>::iterator rot_it = rot_rands.begin();
	std::vector<float>::iterator trans_it = trans_rands.begin();

	float deg2rad = Constants::Deg2Rad();

	std::vector<Isometry3f> results(num);
	for (int i = 0; i < num; ++i) {
		Quaternionf q_ptb = AngleAxisf(deg2rad * (*rot_it++), Vector3f::UnitX())
			* AngleAxisf(deg2rad * (*rot_it++), Vector3f::UnitY())
			* AngleAxisf(deg2rad * (*rot_it++), Vector3f::UnitZ());

		Isometry3f iso(q_ptb);
		iso.pretranslate(Vector3f((*trans_it++), (*trans_it++), (*trans_it++)));

		results[i] = iso;
	}

	return results;
}