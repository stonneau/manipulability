#include "RootTrajectory.h"
#include "Simulation.h"
#include "Draw/DrawSpline.h"

using namespace matrices;

namespace
{
	
    typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3> > T_Vector3;
	typedef T_Vector3::const_iterator CIT_Vector3;
}

RootTrajectory::RootTrajectory(curve_abc_t* spline, const matrices::Vector3& initDir)
	: spline_(spline)
	, initDir_(initDir)
	, previous_(initDir)
{
	// NOTHING
}

RootTrajectory::~RootTrajectory()
{
	delete spline_;
	// NOTHING
}

void RootTrajectory::Reset()
{
	previous_ = initDir_;
}

void RootTrajectory::Update(const Timer::t_time t, const Timer::t_time dt)
{
	Simulation* sg = Simulation::GetInstance();
	if(sg->simpParams_.rootTrajectory_)
	{
		manip_core::RobotI* robot = sg->pRobot;
		int nbPoses(0);
		for(unsigned int i = 0; i < robot->GetNumTrees(); ++i)
		{
			if(robot->GetTreeI(i)->IsAnchored())
			{
				++nbPoses;
			}
		}
		Vector3 currentPos((*spline_)(std::min(t * sg->simpParams_.speed_, spline_->max())));
		double transf[16];
		robot->ToWorldCoordinates(transf);
		//std::cout << "position  " << transf[3] << "  " << transf[7]  << "  " <<  transf[11] << std::endl;
		//std::cout << "current position  " << currentPos << std::endl;
		currentPos = currentPos - Vector3(transf[3], transf[7], transf[11]);
		sg->motionHandler_.MoveBy(currentPos);
		// ok now compute rotation
		if(currentPos.norm() > 0.0001 && sg->simpParams_.rotatewithspline_ )
		{
			currentPos.normalize();
			matrices::Matrix3 rotation;
			//matrices::GetRotationMatrix(currentPos, previous_, rotation);
			matrices::GetRotationMatrix(previous_, currentPos , rotation);
			//std::cout << "Rotation matrix  " << std::endl << rotation << std::endl;
			previous_ = currentPos;
			sg->motionHandler_.Rotate(rotation);
		}
		else if(currentPos.norm() > 0.0001 && sg->simpParams_.rotatewithsplinedir_)
		{
			currentPos.normalize();
			matrices::Matrix3 rotation;
			matrices::GetRotationMatrix(currentPos, previous_, rotation);
			//matrices::GetRotationMatrix(previous_, currentPos , rotation);
			//std::cout << "Rotation matrix  " << std::endl << rotation << std::endl;
			previous_ = currentPos;
			sg->motionHandler_.Rotate(rotation);
		}
		// let's try com rotation
		
		if(sg->simpParams_.humanrotate_)
		{
			double comDir[3];
			robot->ComputeCom(comDir);
			matrices::arrayToVect3(comDir, currentPos);
			if(currentPos.norm() > 0.01 && nbPoses > 0)
			{
				currentPos.normalize();
				matrices::Matrix3 rotation;
				//matrices::GetRotationMatrix(currentPos, previous_, rotation);
				matrices::GetRotationMatrix(previous_, currentPos , rotation);
				previous_ = currentPos;
				sg->motionHandler_.Rotate(rotation);
			}
		}

		if(sg->simpParams_.drawSplines_)
		{
			DrawSpline ds(*spline_);
			ds.Draw();
		}
	}
}
