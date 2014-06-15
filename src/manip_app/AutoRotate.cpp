#include "AutoRotate.h"
#include "Simulation.h"

using namespace matrices;

namespace
{
	
    typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3> > T_Vector3;
	typedef T_Vector3::const_iterator CIT_Vector3;
	//points in robot coordinates. return value -1, 1, 0 
	Vector2 XYRotation(const T_Vector3& points)
	{
		Vector2 res(0,0);
		if(points.size() < 2)
		{
			return res;
		}
		if(points.size() == 2) // normal
		{

		}
		else // plan parallel to first three points
		{

		}
	}
}

AutoRotate::AutoRotate(const matrices::Vector3& angleSpeeds)
	: angleSpeeds_(angleSpeeds)
{
	// NOTHING
}

AutoRotate::~AutoRotate()
{
	// NOTHING
}

void AutoRotate::Update(const Timer::t_time t, const Timer::t_time dt)
{
	Vector3 deltas = angleSpeeds_ * dt;
	// for the time being only z
	deltas(0) = 0;
	deltas(1) = 0;
	// insert actions
	Simulation* sg = Simulation::GetInstance();
	manip_core::RobotI* robot = sg->pRobot;
	/*v0*/
	robot->ComTarget(comDir_);
	if(comDir_[1] != 0)
	{
		if (matrices::Project(Vector3(1,0,0), Vector3(comDir_[0], comDir_[1], 0)) < 0.5)
		{
			deltas(2) *= comDir_[1] / abs(comDir_[1]);
			sg->motionHandler_.Rotate( matrices::Rotz3(deltas(2)));
		}
	}

}

AutoRotateLeg::AutoRotateLeg(const matrices::Vector3& angleSpeeds)
	: angleSpeeds_(angleSpeeds)
{
	// NOTHING
}

AutoRotateLeg::~AutoRotateLeg()
{
	// NOTHING
}

void AutoRotateLeg::Update(const Timer::t_time t, const Timer::t_time dt)
{
	Vector3 deltas = angleSpeeds_ * dt;
	// for the time being only z
	deltas(0) = 0;
	deltas(1) = 0;
	// insert actions
	Simulation* sg = Simulation::GetInstance();
	manip_core::RobotI* robot = sg->pRobot;
	/*v0*/
	assert(robot->GetType() == manip_core::enums::robot::Human);
	Vector3 leftLeg, rightLeg, centre;
	bool center(false);
	if(robot->GetTreeI(0)->IsAnchored())
	{
		center = true;
		robot->GetTreeI(0)->EndEffectorPosition(legBuffer_);
		matrices::arrayToVect3(legBuffer_, rightLeg);
		centre = rightLeg;
		if(robot->GetTreeI(1)->IsAnchored())
		{
			robot->GetTreeI(1)->EndEffectorPosition(legBuffer_);
			matrices::arrayToVect3(legBuffer_, leftLeg);
			centre += leftLeg;
			centre /= 2;
		}
	}
	else if(robot->GetTreeI(1)->IsAnchored())
	{
		robot->GetTreeI(1)->EndEffectorPosition(legBuffer_);
		matrices::arrayToVect3(legBuffer_, leftLeg);
		center = true;
		centre = leftLeg;
	}

	if(center)
	{
		matrices::Matrix4 transform;
		double transf[16];
		robot->ToRobotCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		centre = matrices::matrix4TimesVect3(transform, centre);
		if(centre(1) != 0)
		{
			if (matrices::Project(Vector3(1,0,0), Vector3(centre(0), centre(1), 0)) < 0.9)
			{
				deltas(2) *= centre(1) / abs(centre(1));
				if(abs(deltas(2)) > 0)
				sg->motionHandler_.Rotate( matrices::Rotz3(deltas(2)));
			}
		}
	}
}
