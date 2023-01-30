#include "kabsch.h"



// The input 3D points are stored as columns.
Eigen::Matrix3d kabsch(const Eigen::Matrix3Xd & in, const Eigen::Matrix3Xd & out) {

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0) //why?
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();


  return R;
}



//deprecated
//void TestDOF(Eigen::Matrix3d R) {
//	// Decompose Rotation Matrix
//	double x = atan2(R(2, 1), R(2, 2));
//	double y = -asin(R(2, 0)); // problem if excactly 90 deg?
//	double z = atan2(R(1, 0), R(0, 0));
//
//	// Compose Rotation Matrix with Euler Angles
//	Eigen::Matrix3d C;
//	C << cos(z) * cos(y), cos(z)* sin(y)* sin(x) - sin(z) * cos(x), cos(z)* sin(y)* cos(x) + sin(z) * sin(x),
//		sin(z)* cos(y), sin(z)* sin(y)* sin(x) + cos(z) * cos(x), sin(z)* sin(y)* cos(x) - cos(z) * sin(x),
//		-sin(y), cos(y)* sin(x), cos(y)* cos(x);
//
//	std::cout << "Rotation matrix: " << std::endl;
//	std::cout << R << std::endl;
//	std::cout << "Composed matrix: " << std::endl;
//	std::cout << C << std::endl;
//
//	// check if matrices match, frobenius norm
//	double err = 0;
//	for (int i = 0; i < 3; i++) {
//		for (int j = 0; j < 3; j++) {
//			err += (C(i, j) - R(i, j)) * (C(i, j) - R(i, j));
//		}
//	}
//	err = sqrt(err);
//	if (err < (double)0.00001)
//		std::cout << "Match!" << std::endl;
//	else
//		std::cout << "No Match." << std::endl;
//	std::cout << "\n\n";
//
//}

// A function to test Find3DAffineTransform()

//void TestFind3DAffineTransform(){
//
//  // Create datasets with known transform
//  Eigen::Matrix3Xd in(3, 100), out(3, 100);
//  Eigen::Quaternion<double> Q(1, 3, 5, 2);
//  Q.normalize();
//  Eigen::Matrix3d R = Q.toRotationMatrix();
//  double scale = 2.0;
//  for (int row = 0; row < in.rows(); row++) {
//    for (int col = 0; col < in.cols(); col++) {
//      in(row, col) = log(2*row + 10.0)/sqrt(1.0*col + 4.0) + sqrt(col*1.0)/(row + 1.0);
//    }
//  }
//  Eigen::Vector3d S;
//  S << -5, 6, -27;
//  for (int col = 0; col < in.cols(); col++)
//    out.col(col) = scale*R*in.col(col) + S;
//
//  Eigen::Affine3d A = Find3DAffineTransform(in, out);
//
//  // See if we got the transform we expected
//  if ( (scale*R-A.linear()).cwiseAbs().maxCoeff() > 1e-13 ||
//       (S-A.translation()).cwiseAbs().maxCoeff() > 1e-13)
//    throw "Could not determine the affine transform accurately enough";
//}
// First find the scale, by finding the ratio of sums of some distances,
// then bring the datasets to the same scale.
//double dist_in = 0, dist_out = 0;
//for (int col = 0; col < in.cols()-1; col++) {
//  dist_in  += (in.col(col+1) - in.col(col)).norm();
//  dist_out += (out.col(col+1) - out.col(col)).norm();
//}
//if (dist_in <= 0 || dist_out <= 0)
//  return A;
//double scale = dist_out/dist_in;
//out /= scale;


  // Default output
  //Eigen::Affine3d A;
  //A.linear() = Eigen::Matrix3d::Identity(3, 3);
  //A.translation() = Eigen::Vector3d::Zero();


  // The final transform
  //A.linear() = scale * R;
  //A.translation() = scale*(out_ctr - R*in_ctr);