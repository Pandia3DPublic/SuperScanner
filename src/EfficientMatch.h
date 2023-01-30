#pragma once

//for valid keypoints
struct rmatch
{
	rmatch(int a, int b, int c, int d) : fi1(a), fi2(b), i1(c), i2(d){};
	int fi1; //frameindex 1
	int fi2; //frameindex 2
	// std::shared_ptr<Frame> f1;
	// std::shared_ptr<Frame> f2;
	int i1;	 //keypoint index 1
	int i2;	 //keypoint index 2

	friend std::ostream &operator<<(std::ostream &os, const rmatch &r);
};
