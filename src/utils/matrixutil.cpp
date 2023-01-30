#include "matrixutil.h"


Eigen::Matrix4d getIdentity() {
	Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
	return eye;
}
// get the rotation matrix around an arbitrary vector u for an angle a.
Eigen::Matrix4d getRotMatrix(const Eigen::Vector3d& vec_in, double& a)
{
	Eigen::Vector3d vec = vec_in;
	vec.normalize();
	double u = vec(0);
	double v = vec(1);
	double w = vec(2);
	double tmp = 1 - std::cos(a);
	double cosa = std::cos(a);
	double sina = std::sin(a);
	Eigen::Matrix4d out;
	out << u * u + (1.0 - u * u) * cosa, u * v * tmp - w * sina, u * w * tmp + v * sina, 0,
		u * v * tmp + w * sina, v * v + (1.0 - v * v) * cosa, v * w * tmp - u * sina, 0,
		u * w * tmp - v * sina, v * w * tmp + u * sina, w * w + (1 - w * w) * cosa, 0,
		0, 0, 0, 1;
	return out;
}

Eigen::Matrix4d getflip()
{
	Eigen::Matrix4d flip;
	flip << 1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;
	return flip;
}

//tested and working for our getT (x,y,z rot than trans)
Eigen::Vector6d MattoDof(const Eigen::Matrix4d& R) {
	// Decompose Rotation Matrix
	
	Eigen::Vector6d v; // check this if initialized ok
	v(0) = atan2(R(2, 1), R(2, 2));
	v(1) = -asin(R(2, 0));
	v(2) = atan2(R(1, 0), R(0, 0));
	v(3) = R(0,3);
	v(4) = R(1,3);
	v(5) = R(2,3);

	// double* a = new double[6];
	// for (int i = 0; i < 6; i++) {
	// 	a[i] = v(i);
	// }
	// // cout << "original kabsch matrix: \n";
	// // cout << R << endl;
	// // cout << "reconstructed matrix from dof: \n";
	// // cout << getT(a) << endl;

	// if ((R - getT(a)).norm() > 1e-5) {
	// 	cout << "bad \n ";
	// }
	// delete[]a;

	return v;
}

//seems to be working
Eigen::Matrix4d getQuaternionCross(const Eigen::Quaterniond& a) {
	Eigen::Matrix4d out = a.coeffs() * a.coeffs().transpose();
	return out;
}

Eigen::Quaterniond getQuaternionSlerp( const Eigen::Matrix4d& a,const Eigen::Matrix4d& b) {
	Eigen::Quaterniond aq(a.block<3,3>(0,0));
	Eigen::Quaterniond bq(b.block<3,3>(0,0));
	Eigen::Quaterniond midq = aq.slerp(0.5,bq);
	return midq;
}

Eigen::Quaterniond getQuaternionAverage(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &quats)
{
	//create matrix 
	Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
	for (const Eigen::Quaterniond &q : quats)
		{
			M += getQuaternionCross(q);
		}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);
	if (eigensolver.info() != Eigen::Success) std::cout << "No eigenvector found in quaternion averaging! \n";

	Eigen::Quaterniond a;
	a.coeffs() = eigensolver.eigenvectors().col(3); 
	//if (quats.size() == 1) {
	//	std::cout << "input quaternion was \n" << quats[0].coeffs() << std::endl;
	//	std::cout << "output is \n" << a.coeffs() <<std::endl;
	//}
	//std::cout << "eigen values are " << eigensolver.eigenvalues() << std::endl;
	//std::cout << "eigen vectors are " << eigensolver.eigenvectors() << std::endl;
	//std::cout << "we use  " << a.coeffs() << std::endl;
	return a;
}

Eigen::Matrix4d getHalf(const Eigen::Matrix4d& a,const Eigen::Matrix4d& b) {

	Eigen::Quaterniond aq(a.block<3,3>(0,0));
	Eigen::Quaterniond bq(b.block<3,3>(0,0));
	Eigen::Quaterniond midq = aq.slerp(0.5,bq);
	Eigen::Matrix3d midsmall(midq);
	Eigen::Matrix4d out =  Eigen::Matrix4d::Identity(); //rotation only
	out.block<3,3>(0,0) = midsmall;
	out(3,3) =1;
	out.block<3,1>(0,3) = (a.block<3,1>(0,3)+ b.block<3,1>(0,3)) /2; 


	return out;
}

double toDegrees(const double& radians) {
	return radians * 180.0 / MATUTIL_PI;
}
double toRadians(const double& degrees) {
	return degrees * MATUTIL_PI / 180.0;
}
