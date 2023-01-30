#include "kabschfilter.h"
#include "utils/matrixutil.h"
#include "utils/coreutil.h"
#include "core/threadCom.h"
#include "kabsch.h"
using namespace std;

void writemax(double& towrite, double comp) {
	if (comp > towrite)
		towrite = comp;
}
void writemin(double& towrite, double comp) {
	if (comp < towrite)
		towrite = comp;
}



//todo check for rotational symmetrie in valid keypoint pcds
//(bottom up kabschfilter)
//Note: this gives the transform from lower id frame  to the higher id frame
//todo this might work slightly worse than with mincors = 20. Try to make it work just as well -> might be done
//Aggregates cors until a stable distribution with at least 6 cors is found
void kabschfilter(pairTransform& pt, int debugmist) {
	// if (debugmist == 36)
	// {
	// 	cout << "kabsch incoming match size (from genCors) " << matches.size() << "\n";
	// }
	auto& matches = pt.filteredmatches;
	std::vector<match> fm; // filtered matches according to kabsch
	int maxcors = 12;
	int mincors = 6;
	int initialcors = 3;
	int removeindex = -1;
	bool firstiteration = true;
	float maxerror = std::numeric_limits<float>::max();
	int mInd = 0;
	Eigen::Vector3d a_ctr = Eigen::Vector3d::Zero();
	Eigen::Vector3d b_ctr = Eigen::Vector3d::Zero();
	Eigen::Matrix4d Rh = Eigen::Matrix4d::Zero();
	Eigen::Matrix3d covA = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d covB = Eigen::Matrix3d::Zero();
	Rh(3, 3) = 1;

	//initialize with 3 cors 
	if (matches.size() >= initialcors){
		for (int i = 0; i < initialcors; i++) { //start first iteration with cor number 4.
			fm.push_back(matches[mInd]);
			mInd++;
		}
	} else {
		mInd+= initialcors;
	}

	while (mInd < matches.size()) { 
		fm.push_back(matches[mInd]);
		mInd++;
		//################################################################################################
		Eigen::Matrix3Xd pointsa(3, fm.size());
		Eigen::Matrix3Xd pointsb(3, fm.size());

		//fill filtered correspondences in eigen matrix. points are stored in columns
		for (int i = 0; i < fm.size(); i++) {
			pointsa.block<3, 1>(0, i) = Eigen::Vector3d(fm[i].p1(0), fm[i].p1(1), fm[i].p1(2));
			pointsb.block<3, 1>(0, i) = Eigen::Vector3d(fm[i].p2(0), fm[i].p2(1), fm[i].p2(2));
		}

		// Find the centroids then shift to the origin
		a_ctr = Eigen::Vector3d::Zero();
		b_ctr = Eigen::Vector3d::Zero();
		for (int col = 0; col < pointsa.cols(); col++) {
			a_ctr += pointsa.col(col);
			b_ctr += pointsb.col(col);
		}
		a_ctr /= pointsa.cols();
		b_ctr /= pointsb.cols();
		for (int col = 0; col < pointsa.cols(); col++) {
			pointsa.col(col) -= a_ctr;
			pointsb.col(col) -= b_ctr;
		}
		// pass zero shifted pcds to kabsch algorithms
		Eigen::Matrix3d R = kabsch(pointsa, pointsb);
		Rh.block<3, 3>(0, 0) = R;

		// write the transformed pcd in pointsa
		for (int i = 0; i < pointsa.cols(); i++) {
			Eigen::Vector4d new_point = Rh * Eigen::Vector4d(pointsa(0, i), pointsa(1, i), pointsa(2, i), 1.0);
			pointsa.block<3, 1>(0, i) = new_point.block<3, 1>(0, 0);
		}
		//pointsa now cotains zero shifted and rotated coordinates. pointsb contains zero shift coordinates. Time to substract.
		//drawEigen({ pointsa,pointsb });
		//Eigen::Array<double,3,Eigen::Dynamic> err = (pointsa - pointsb).array();
		//err = err.pow(2);

		Eigen::Matrix3Xd err = pointsa - pointsb;
		Eigen::VectorXd euclideanError(pointsa.cols()); // vector for euclidean distances
		for (int i = 0; i < pointsa.cols(); i++) {
			euclideanError(i) = err.block<3, 1>(0, i).norm(); // fill the error vector
		}

		maxerror = euclideanError.maxCoeff(&removeindex);
		//################################################################################################
		double conditionA = 0;
		double conditionB = 0;
		double surfA = 0;
		double surfB = 0;
		//todo doesnt kabsch also do an svd? checkt this out for performance gain
		Eigen::JacobiSVD<Eigen::MatrixXd> svd;
		if (pointsa.cols() > 4) {
			//condition
			covA = pointsa * pointsa.transpose();
			covA = covA / (pointsa.cols());
			svd.compute(covA, Eigen::ComputeThinU);
			conditionA = svd.singularValues()(0) / svd.singularValues()(1);
			//surface area
			Eigen::Matrix<double, 2, 3> Ur;
			vector<double> extrema(4); //minx, maxx, miny, maxy
			Ur = svd.matrixU().block<3, 2>(0, 0).transpose();
			//project the points in the pca plane and calculate approx area
			for (int col = 0; col < pointsa.cols(); col++) {
				Eigen::Vector2d z = Ur * pointsa.block<3, 1>(0, col);
				writemin(extrema[0], z(0));
				writemax(extrema[1], z(0));
				writemin(extrema[2], z(1));
				writemax(extrema[3], z(1));
			}
			surfA = (extrema[1] - extrema[0]) * (extrema[3] - extrema[2]) / 2.0;


			//condition
			covB = pointsb * pointsb.transpose();
			covB = covB / (pointsb.cols());
			svd.compute(covB, Eigen::ComputeThinU);
			conditionB = svd.singularValues()(0) / svd.singularValues()(1);
			//surface
			for (int col = 0; col < pointsb.cols(); col++) {
				Eigen::Vector2d z = Ur * pointsb.block<3, 1>(0, col);
				writemin(extrema[0], z(0));
				writemax(extrema[1], z(0));
				writemin(extrema[2], z(1));
				writemax(extrema[3], z(1));
			}
			surfB = (extrema[1] - extrema[0]) * (extrema[3] - extrema[2]) / 2.0;

			//vis
			//if (surfB < g_minArea) {
			//	cout << " SurfB " << surfB << "\n";
			//	drawEigen({ pointsb });
			//}
			//if (conditionB > 10000*g_conditionThres) {
			//	cout << " B " << conditionB << "\n";
			//	cout << "singular values \n";
			//	cout << svd.singularValues() << endl;
			//	cout << pointsb << endl;
			//	drawEigen({ pointsb });
			//}
		}
		//note that coniditionA and B will often be in the e17 range or even inf, if mulitple points in pointsa or b are the same
		if (maxerror > 0.02 || conditionA > g_conditionThres || conditionB > g_conditionThres) { 
			fm.erase(fm.begin() + removeindex); 
			//if (maxerror < 0.02) {
			//	cout << conditionA << " B " << conditionB << endl;
			//	cout << svd.singularValues() << endl;
			//}
		}
		else {
			//todo only compute surfaces if question is necessary
			if ((surfA < g_minArea || surfB < g_minArea) && fm.size() >=mincors) {
				mincors++;
				//todo the filter rate seems to be extremly dependend on maxcors. Try to make kabsch more robust to that and try to get fewer cors
				//maxcors 11 gives 16 chunks on custom data, maxors 12 gives 25
				if (mincors > maxcors) {
					break; //failure
				}
			}
		}

		if (fm.size() >= mincors) {
			break; //success
		}

	}
	
	if (fm.size() >= mincors) {
		matches = fm;
		//transform from a to b
		Eigen::Matrix4d kabsch = gettrans(b_ctr) * Rh * gettrans(a_ctr).inverse();
		pt.setkabschtrans(kabsch);
		pt.setinvkabschtrans(kabsch.inverse());
		pt.set = true;

	}
	else {
		std::vector<match> empty;
		matches = empty; 
		pt.set =false;
		open3d::utility::LogDebug("All matches removed by kabsch filter! \n");
	}

}