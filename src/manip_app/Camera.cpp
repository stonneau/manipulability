#include "Camera.h"
#include "Simulation.h"

using namespace matrices;

CameraFollow::CameraFollow(const matrices::Vector3& xyz, const matrices::Vector3& hpr)
		: Camera_ABC(xyz, hpr)
		, moved_(false)
		, unitz(0,0,1)
		, screenArrowPosition_(0,0,1)
	{
		arrowOffset_[0] =0;
		arrowOffset_[1] =2;
		arrowOffset_[2] =2;
		for(int i =0; i< 3; ++i)
		{
			arrow_[i] = xyz_[i] + arrowOffset_[i];
		}
		matrices::Matrix3 transformCube;
		matrices::GetRotationMatrix(unitz, unitz, transformCube);
		matrices::matrix3ToArray(idMatrix, transformCube);
	}

void CameraFollow::DrawArrow()
{
	for(int i =0; i<3; ++i)
	{
		screenArrowPosition_(i) = xyz_[i] + arrowOffset_[i];
	}
	oldDir_ = Simulation::GetInstance()->motionHandler_.GetDirection();
	Matrix3 transformCube;
	GetRotationMatrix(unitz, oldDir_, transformCube);
	matrix3ToArray(idMatrix, transformCube);
	int sign = 1; 
	oldDir_.normalize();
	if(oldDir_(0) == 0 && oldDir_(1) == 0 && oldDir_(2) -0.1 <= -1)
	{
		sign =-1;
		matrices::vect3ToArray(arrowExtr_, screenArrowPosition_ +  oldDir_ * 1.15);
	}
	else
	{
		matrices::vect3ToArray(arrowExtr_, screenArrowPosition_ +  oldDir_  / 2.);
	}
	matrices::vect3ToArray(arrow_, screenArrowPosition_ + oldDir_ / 2.);
	Vector3 z(0,0,1./3.);
	matrices::vect3ToArray(v1_, z);
	v1_[1] += sign* 0.2;
	matrices::vect3ToArray(v2_, z);
	v2_[1] += sign * -0.2;
	matrices::vect3ToArray(v3_, z);
	v3_[2] += sign * 0.3;

	dsSetColor(0,0,0);
	dsDrawCapsule(arrow_, idMatrix, 1/2., 0.03);
	dsDrawTriangle (arrowExtr_, idMatrix, v3_, v2_, v1_, 1);
	
	matrices::vect3ToArray(v1_, z);
	v1_[0] += sign* 0.2;
	matrices::vect3ToArray(v2_, z);
	v2_[0] +=sign* -0.2;
	matrices::vect3ToArray(v3_, z);
	v3_[2] +=sign* 0.3; 
	dsDrawTriangle (arrowExtr_, idMatrix, v3_, v2_, v1_, 1);
	//glutSolidCone(/*GLdouble base*/0.05, /*GLdouble height*/0.05, /*GLint slices*/10, /*GLint stacks*/10);
}
