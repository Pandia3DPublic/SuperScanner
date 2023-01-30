#pragma once
#define MATUTIL_PI 3.14159265358979323846264

Eigen::Matrix4d getRotMatrix(const Eigen::Vector3d &vec_in, double &a);
Eigen::Matrix4d getflip();
Eigen::Vector6d MattoDof(const Eigen::Matrix4d& R);

//gets half the transformation matrix by quaternion slerp and linear translation interpolation;
Eigen::Matrix4d getHalf(const Eigen::Matrix4d& a,const Eigen::Matrix4d& b);
Eigen::Matrix4d getQuaternionCross(const Eigen::Quaterniond& a);
Eigen::Quaterniond getQuaternionSlerp( const Eigen::Matrix4d& a,const Eigen::Matrix4d& b);
Eigen::Quaterniond getQuaternionAverage(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &quats);

double toDegrees(const double& radians);
double toRadians(const double& degrees);

//matrix utility functions
Eigen::Matrix4d getRotMatrix(Eigen::Vector3d& u, double& a);
Eigen::Matrix4d getflip();
template <typename T>
Eigen::Matrix4d gettrans(T t) {
	Eigen::Matrix4d trans;
	trans << 1, 0, 0, t[0],
		0, 1, 0, t[1],
		0, 0, 1, t[2],
		0, 0, 0, 1;
	return trans;
}

template <typename T>
Eigen::Matrix4d getRx(T a) {
	Eigen::Matrix4d rx;
	rx << 1, 0, 0, 0,
		0, std::cos(a), -std::sin(a), 0,
		0, std::sin(a), std::cos(a), 0,
		0, 0, 0, 1;
	return rx;
}

template <typename T>
Eigen::Matrix4d getRy(T a) {
	Eigen::Matrix4d ry;
	ry << std::cos(a), 0, std::sin(a), 0,
		0, 1, 0, 0,
		-std::sin(a), 0, std::cos(a), 0,
		0, 0, 0, 1;
	return ry;
}

template <typename T>
Eigen::Matrix4d getRz(T a) {
	Eigen::Matrix4d rz;
	rz << std::cos(a), -std::sin(a), 0, 0,
		std::sin(a), std::cos(a), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return rz;
}


template <typename T> Eigen::Matrix4d getT(T x) {
	return gettrans(x + 3) * getRz(x[2]) * getRy(x[1]) * getRx(x[0]);
}



// //function templates for ceres


template <typename T>
Eigen::Matrix<T, 4, 4> gettrans2(T const* const t) {
	Eigen::Matrix<T, 4, 4> trans;
	trans << T(1), T(0), T(0), t[0],
		T(0), T(1), T(0), t[1],
		T(0), T(0), T(1), t[2],
		T(0), T(0), T(0), T(1);

	return trans;
}

template <typename T>
Eigen::Matrix<T, 4, 4> getRx2(T a) {
	Eigen::Matrix<T, 4, 4> rx;
	rx << T(1), T(0), T(0), T(0),
		T(0), cos(a), -sin(a), T(0),
		T(0), sin(a), cos(a), T(0),
		T(0), T(0), T(0), T(1);
	return rx;
}

template <typename T>
Eigen::Matrix<T, 4, 4> getRy2(T a) {
	Eigen::Matrix<T, 4, 4> ry;
	ry << cos(a), T(0), sin(a), T(0),
		T(0), T(1), T(0), T(0),
		-sin(a), T(0), cos(a), T(0),
		T(0), T(0), T(0), T(1);

	return ry;
}

template <typename T>
Eigen::Matrix<T, 4, 4> getRz2(T a) {
	Eigen::Matrix<T, 4, 4> rz;
	rz << cos(a), -sin(a), T(0), T(0),
		sin(a), cos(a), T(0), T(0),
			T(0), T(0), T(1), T(0),
			T(0), T(0), T(0), T(1);

	return rz;
}
template <typename T> Eigen::Matrix<T, 4, 4> getT2(T const* const x) {
	Eigen::Matrix<T, 4, 4> tran;
	tran = gettrans2(x + 3) * getRz2(x[2]) * getRy2(x[1]) * getRx2(x[0]);
	return tran;
}

//templated and ready for max efficiency
template <typename T> Eigen::Matrix<T, 4, 4> getT22(T const* const x) {
	Eigen::Matrix<T, 4, 4> tran;
	tran << cos(x[2]) * cos(x[1]), cos(x[2])* sin(x[1])* sin(x[0]) - sin(x[2]) * cos(x[0]), cos(x[2])* sin(x[1])* cos(x[0]) + sin(x[2]) * sin(x[0]), x[3],
		sin(x[2])* cos(x[1]), sin(x[2])* sin(x[1]) * sin(x[0]) + cos(x[2]) * cos(x[0]), sin(x[2])* sin(x[1])* cos(x[0]) - cos(x[2]) * sin(x[0]), x[4],
		-sin(x[1]), sin(x[0])* cos(x[1]), cos(x[0])* cos(x[1]), x[5],
		T(0.0), T(0.0), T(0.0), T(1.0);
	return tran;
}

template <typename T>
Eigen::Matrix<T, 3, 4, Eigen::RowMajor> getTOpt(const T* const x)
{
	Eigen::Matrix<T, 3, 4, Eigen::RowMajor> tran;
	tran << cos(x[2]) * cos(x[1]), cos(x[2]) * sin(x[1]) * sin(x[0]) - sin(x[2]) * cos(x[0]), cos(x[2]) * sin(x[1]) * cos(x[0]) + sin(x[2]) * sin(x[0]), x[3],
		sin(x[2]) * cos(x[1]), sin(x[2]) * sin(x[1]) * sin(x[0]) + cos(x[2]) * cos(x[0]), sin(x[2]) * sin(x[1]) * cos(x[0]) - cos(x[2]) * sin(x[0]), x[4],
		-sin(x[1]), sin(x[0]) * cos(x[1]), cos(x[0]) * cos(x[1]), x[5];
	return tran;
}

template <typename T>
Eigen::Matrix<T, 3, 4, Eigen::RowMajor> getIdentityOpt(const T* const x)
{
	Eigen::Matrix<double, 3, 4, Eigen::RowMajor> tmp;
	tmp << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0;
	return tmp.cast<T>();
}

Eigen::Matrix4d getIdentity();
