
#include "FilterDistanceObstacle.h"

#include "world/Obstacle.h"
#include "kinematic/Robot.h"
#include "kinematic/Tree.h"
#include "sampling/Sample.h"
#include "world/Obstacle.h"
#include "kinematic/Enums.h"
#include "Pi.h"
#include <math.h>

using namespace matrices;
using namespace Eigen;

struct FilterDPImpl
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	FilterDPImpl(NUMBER treshold, const Tree& tree, const Obstacle& obstacle, const Robot& robot, const Vector3& direction)
	: treshold_(treshold)
	, treePos_(tree.GetPosition())
	, direction_(direction)
	{
		//toObstacleCoordinates_ = robot.ToWorldCoordinates() * obstacle.BasisInv();
		IsSpider_ = robot.RobotType() == manip_core::enums::robot::Spider;
		if(IsSpider_)
		{
			NUMBER angle = (float)((tree.GetTemplateId() % 4 + 1 ) * RADIAN(36)) * ((tree.GetId() > 3 ) ? 1 : -1);
			isSpiderRot_ = matrices::Rotz3(angle);
		}
		treeEffPos_ = tree.GetEffectorPosition(tree.GetNumEffector()-1);
		toObstacleCoordinates_ = obstacle.BasisInv() * robot.ToWorldCoordinates();
		obsW_ = obstacle.GetW(); obsH_ = obstacle.GetH();
		currentTarget_ = matrices::matrix4TimesVect3(robot.ToRobotCoordinates(), tree.GetTarget()) - tree.GetPosition();
		direction_.normalize();
		robotRot_ = robot.ToRobotCoordinates().block<3,3>(0,0);
	}

	~FilterDPImpl()
	{
		// NOTHING
	}
	
	NUMBER treshold_;
	Vector3 direction_;
	Vector3 currentTarget_;
	Matrix4 toObstacleCoordinates_;
	Matrix3 robotRot_;
	Vector3 treePos_;
	Vector3 treeEffPos_;
	NUMBER obsW_, obsH_;
	bool IsSpider_;
	matrices::Matrix3 isSpiderRot_;

};

FilterDistanceObstacle::FilterDistanceObstacle(NUMBER treshold, const Tree& tree, const Obstacle& obstacle, const Robot& robot, const Vector3& direction)
	: Filter_ABC()
	, pImpl_(new FilterDPImpl(treshold, tree, obstacle, robot, direction))
{
	// NOTHING
}

FilterDistanceObstacle::~FilterDistanceObstacle()
{
	// NOTHING
}

bool FilterDistanceObstacle::ApplyFilter(const Sample& sample) const
{
	// if spider, well horrible stuff
	//if(false)
	if(pImpl_->IsSpider_)
	{
		/*return sample.GetPosition().x() < pImpl_->treeEffPos_.x();
		Vector3 rPos = pImpl_->isSpiderRot_ * sample.GetPosition();
		return sample.GetPosition()(0)  >=0;*/
		return true;
	}
	else
	{
		// check that sample position is inside obstacle radius and not too far from its plan
		// we want an angle inferior to 90 degrees
		//Vector3 dir = sample.GetPosition() - pImpl_->currentTarget_;
		//NUMBER test = dir.norm();
		//dir = pImpl_->robotRot_ * dir;
		////dir = pImpl_->direction_ * dir.transpose();
		//if (dir(0) * pImpl_->direction_(0) >= 0. && dir(1) * pImpl_->direction_(1) >= 0. && dir(2) * pImpl_->direction_(2) >=0.)
		//{
		//	return true;
		//}
		//return false;
		//if(sample.GetPosition()(0) * pImpl_->direction_(0) >=0 )// && sample.GetPosition()(1) * pImpl_->direction_(1) >=0 ) // && sample.GetPosition()(2) * pImpl_->direction_(2) >=0) // we want to move forward
		//{
			return true;
		//}
	//	//return false;
	//	Vector3 center = matrix4TimesVect3(pImpl_->toObstacleCoordinates_, sample.GetPosition() + pImpl_->treePos_);
	//	NUMBER xc, yc;
	//	xc = center(0); yc = center(1);
	//	return (abs(center(2)) <= pImpl_->treshold_) && (xc >= 0 && xc <= pImpl_->obsW_) && (yc >= 0 && yc <= pImpl_->obsH_);
	}
}

