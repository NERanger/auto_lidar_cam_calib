#include "Types.hpp"

using alcc::CamIntrisic;

Eigen::Matrix3f CamIntrisic::AsMat() const {
	Eigen::Matrix3f mat;
	mat <<
		fx, 0.0f, cx,
		0.0f, fy, cy,
		0.0f, 0.0f, 1.0f;

	return mat;
}