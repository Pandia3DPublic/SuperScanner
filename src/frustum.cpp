#include "frustum.h"

Plane::Plane(const Eigen::Vector3d& n, const Eigen::Vector3d& point) {

	normal = n.normalized();
	d = point.transpose() * normal;
}

void Plane::transform(const Eigen::Matrix4d& trans) {
	Eigen::Vector3d pt(1.0,1.0,(d-normal(0)-normal(1)) / normal(2));
	pt = trans.block<3,3>(0,0)* pt + trans.block<3,1>(0,3);
	normal = trans.block<3,3>(0,0)* normal;
	d = pt.transpose() * normal;

}



Frustum::Frustum(const Eigen::Matrix4d& camerapos, const open3d::camera::PinholeCameraIntrinsic& intrinsic, const double& dmin, const double& dmax) {
	const Eigen::Matrix3d& intr = intrinsic.intrinsic_matrix_;
	double factor = 0.8;
	double invfactor = 1-factor;
	pos = camerapos;
	corners.reserve(8);
	//counter clockwise, starting top left from the cams perspective. origin is in bottom right for pinhole camera

	corners.push_back(cordtoPoint(factor * intrinsic.width_,factor * intrinsic.height_,dmin,intr));
	corners.push_back(cordtoPoint(factor * intrinsic.width_, invfactor * intrinsic.height_,dmin,intr));
	corners.push_back(cordtoPoint(invfactor * intrinsic.width_,invfactor * intrinsic.height_,dmin,intr));
	corners.push_back(cordtoPoint(invfactor * intrinsic.width_,factor * intrinsic.height_,dmin,intr));
	//same with dmax
	corners.push_back(cordtoPoint(factor * intrinsic.width_,factor * intrinsic.height_,dmax,intr));
	corners.push_back(cordtoPoint(factor * intrinsic.width_,invfactor * intrinsic.height_,dmax,intr));
	corners.push_back(cordtoPoint(invfactor * intrinsic.width_,invfactor * intrinsic.height_,dmax,intr));
	corners.push_back(cordtoPoint(invfactor * intrinsic.width_,factor * intrinsic.height_,dmax,intr));

	//corners.push_back(cordtoPoint(intrinsic.width_,intrinsic.height_,dmin,intr));
	//corners.push_back(cordtoPoint(intrinsic.width_,0,dmin,intr));
	//corners.push_back(cordtoPoint(0,0,dmin,intr));
	//corners.push_back(cordtoPoint(0,intrinsic.height_,dmin,intr));
	////same with dmax
	//corners.push_back(cordtoPoint(intrinsic.width_,intrinsic.height_,dmax,intr));
	//corners.push_back(cordtoPoint(intrinsic.width_,0,dmax,intr));
	//corners.push_back(cordtoPoint(0,0,dmax,intr));
	//corners.push_back(cordtoPoint(0,intrinsic.height_,dmax,intr));


	//transform
	for (Eigen::Vector3d& c : corners) {
		c = camerapos.block<3, 3>(0, 0) * c + camerapos.block<3, 1>(0, 3);
	}

	//planes
	Eigen::Vector3d n; //use right hand rule to get outside pointing normals
	//near plane
	n = -(corners[3] -corners[0]).cross(corners[1] - corners[0]);
	planes.emplace_back(Plane(n,corners[0]));
	//far plane
	n = (corners[7] -corners[4]).cross(corners[5] - corners[4]);
	planes.emplace_back(Plane(n,corners[4]));
	//left plane 
	n = (corners[4] -corners[0]).cross(corners[1] - corners[0]);
	planes.emplace_back(Plane(n,corners[0]));
	//bottom plane
	n = (corners[5] -corners[1]).cross(corners[2] - corners[1]);
	planes.emplace_back(Plane(n,corners[1]));
	//right plane
	n = -(corners[3] -corners[2]).cross(corners[6] - corners[2]);
	planes.emplace_back(Plane(n,corners[2]));
	//top plane
	n = -(corners[4] -corners[0]).cross(corners[3] - corners[0]);
	planes.emplace_back(Plane(n,corners[0]));

}


Eigen::Vector3d Frustum::cordtoPoint(const int& x, const int& y,const double& d,const Eigen::Matrix3d& intr) {
	Eigen::Vector3d out((x - intr(0, 2)) * d / intr(0, 0), (y - intr(1, 2)) * d / intr(1, 1),d);
	return out;
}

//transform the frustum to the new position, instead of building a new one.
void Frustum::reposition(const Eigen::Matrix4d& newpos) {
	Eigen::Matrix4d transform = newpos * pos.inverse();
	for (Eigen::Vector3d& c : corners) {
		c= transform.block<3,3>(0,0) * c + transform.block<3,1>(0,3);
	}
	//transform all planes
	for (auto& p : planes) {
		p.transform(transform);
	}

	pos = newpos;
}



//pyramid pyramid much cheaper and doing the same?
//Check if each corner of one frustum lies on the normal pointing side of 
//our normals point outside the frustum!
//takes 0.7us as frustum-frustum single core. A bit too slow
bool Frustum::intersect(const Frustum& other) const {
	using namespace std;
	bool intersects = true;

	//local planes vs other vertices
	for (auto& pl : planes) {
		bool isAnyVertexInNegativeSide = false;
		for (const Eigen::Vector3d& c : other.corners) {
			double res = pl.normal.transpose() * c - pl.d; //is equivalent to checkingif the vertex lies on the side of the plane the normal points (if res is greater than 0)
			isAnyVertexInNegativeSide |= (res < 0); //negative side
		}
		intersects &= isAnyVertexInNegativeSide; //if for any plane all verts lie on one side we don't intersect
		if (!intersects) //early out
			return false;
	}
	//still have lot of false positives at this point so do reverse check to rule most out.

	//other planes vs local vertices
	for (auto& pl : other.planes) {
		bool isAnyVertexInNegativeSide = false;
		for (const Eigen::Vector3d &c : corners) {
			double res = pl.normal.transpose() * c - pl.d;
			isAnyVertexInNegativeSide |= (res < 0); //our normals point outside
		}
		intersects &= isAnyVertexInNegativeSide; //if for any plane all verts lie on one side we don't intersect
		if (!intersects) //early out
			return false;
	}

	return intersects;
}




bool Frustum::inLocalGroup(const Frustum& f2){
	Eigen::Vector3d& n1 = planes[1].normal; //far plane, same normal as camera
	const Eigen::Vector3d& n2 = f2.planes[1].normal; //far plane, same normal as camera
	if (n1.transpose() * n2 > 0) { //cameras look in the same direction
		return intersect(f2);
	} else {
		return false;
	}
}