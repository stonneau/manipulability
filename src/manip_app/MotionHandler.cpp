#include "MotionHandler.h"
#include "Simulation.h"

#include "IKSolver/IKSolver.h"

#include "Draw/DrawSpline.h"

#include "API/TreeI.h"
#include "API/RobotI.h"

#include "Pi.h"

class MotionActionTranslation : public MotionAction_ABC
{
public:
	MotionActionTranslation(NUMBER speed, const matrices::Vector3& direction) 
		: MotionAction_ABC()
		, direction_(direction)
		, speed_(speed)
	{
		if(direction_.norm() > 0)
		{
			direction_.normalize();
		}
	}

	~MotionActionTranslation()
	{
		// NOTHING
	}

	virtual void operator() (manip_core::RobotI& robot, const Timer::t_time dt)
	{
		double tab [3];
		matrices::vect3ToArray(tab, direction_* dt * speed_);
		robot.Translate(tab);
		Simulation::GetInstance()->simpParams_.GetCamera()->OnCharacterMotion(direction_* dt * speed_);
	}

	matrices::Vector3 direction_;
	NUMBER speed_;
};


class MotionActionMotion : public MotionAction_ABC
{
public:
	MotionActionMotion(const matrices::Vector3& direction) 
		: MotionAction_ABC()
		, direction_(direction)
	{
		// NOTHING
	}

	~MotionActionMotion()
	{
		// NOTHING
	}

	virtual void operator() (manip_core::RobotI& robot, const Timer::t_time dt)
	{
		double tab [3];
		matrices::vect3ToArray(tab, direction_);
		robot.Translate(tab);
		Simulation::GetInstance()->simpParams_.GetCamera()->OnCharacterMotion(direction_);
	}

	matrices::Vector3 direction_;
};

class MotionActionRotation : public MotionAction_ABC
{
public:
	MotionActionRotation(NUMBER speed, const matrices::Matrix3& rotation) 
		: MotionAction_ABC()
		, rotation_(rotation)
		, speed_(speed)
	{
		// NOTHING
	}

	~MotionActionRotation()
	{
		// NOTHING
	}

	virtual void operator() (manip_core::RobotI& robot, const Timer::t_time dt)
	{
		matrices::Matrix4 transform;
		double transf[16];
		robot.ToWorldCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		transform.block<3,3>(0,0) = transform.block<3,3>(0,0) * rotation_;
		matrices::matrixTo16Array(transf, transform);
		robot.SetTransform(transf);
	}

	matrices::Matrix3 rotation_;
	NUMBER speed_;
};

MotionHandler::MotionHandler(manip_core::ManipManager& manager, const float rotationSpeed, const float translationSpeed)
	: rotationSpeed_(rotationSpeed)
	, translationSpeed_(translationSpeed)
	, solver_(manager.GetIkSolver())
	, postureManager_(manager.GetPostureManager())
	, previousDirection_(1,0,0)
{
	// NOTHING
}

MotionHandler::~MotionHandler()
{
	// NOTHING
}


void MotionHandler::Update(const Timer::t_time t, const Timer::t_time dt)
{
	// insert actions
	Simulation* sg = Simulation::GetInstance();
	manip_core::RobotI* robot = sg->pRobot;
	
	matrices::Matrix4 transform;
	double transf[16];
	robot->ToRobotCoordinates(transf);
	matrices::array16ToMatrix4(transf, transform);

	bool findPosture =  !(actions_.empty)(); // don't call NexPosture if I did not move
	while(!actions_.empty())
	{
		(actions_.front())->operator()(*robot, dt);
		delete actions_.front();
		actions_.pop();
	}

	//now let's get more serious
	if(findPosture)
	{
		manip_core::T_CubicTrajectory cubics;
		/*if(sg->simpParams_.rotatewithsplinedir_)
		{
			matrices::Vector3 tg(previousDirection_(1),previousDirection_(0),previousDirection_(2));
			cubics = postureManager_->NextPosture(robot, tg);
		}
		else
		{*/
			cubics = postureManager_->NextPosture(robot, previousDirection_);
		//}
		// replace previous splines
		for(manip_core::IT_CubicTrajectory it = cubics.begin(); it!= cubics.end(); ++it)
		{
			IT_SplineManager it2 = splineManagers_.find(it->first);
			if( it2 != splineManagers_.end())
			{
				delete(it2->second);
				splineManagers_.erase(it2);
			}
			splineManagers_.insert(std::make_pair((it->first), new spline::SplineTimeManager(t, it->second)));
		}
	}
	double targ[3];
	if(! sg->simpParams_.jumpToTarget_)
	{
		for(int i=0; i< robot->GetNumTrees(); ++i)
		{
		//for(IT_SplineManager itSpline = splineManagers_.begin(); itSpline!= splineManagers_.end(); ++itSpline)
		//{
			//perform ik
			IT_SplineManager itSpline = splineManagers_.find(i);
			if(itSpline != splineManagers_.end())
			{
				itSpline->second->Update(t,dt);
				matrices::Vector3 target = matrices::matrix4TimesVect3(transform, itSpline->second->GetTarget());
				manip_core::TreeI* t = robot->GetTreeI(itSpline->first);
				matrices::vect3ToArray(targ, itSpline->second->GetTarget());
				t->SetTarget(targ);
				manip_core::TreeI * tree = robot->GetTreeI(itSpline->first);
				if(sg->simpParams_.jumpToTarget_)
				{
					double tar[3]; tree->GetTarget(tar);
					matrices::arrayToVect3(tar, target);
					target = matrices::matrix4TimesVect3(transform, target);
				}
				if (tree->IsAnchored())
				{
					solver_.StepClamping(robot, robot->GetTreeI(itSpline->first), target, previousDirection_);
				}
				/*else
				{
					double tar[3]; tree->GetReferenceTarget(tar);
					matrices::arrayToVect3(tar, target);
					solver_.StepClamping(robot, robot->GetTreeI(i), target, previousDirection_);
				}*/
				if(sg->simpParams_.drawSplines_)
				{
					DrawSpline ds(itSpline->second->GetCubic());
					ds.Draw();
				}
			}
			else
			{
				manip_core::TreeI* tree = robot->GetTreeI(i);
				double tar[3]; tree->GetTarget(tar);
				matrices::Vector3 target; matrices::arrayToVect3(tar, target);
				target = matrices::matrix4TimesVect3(transform, target);
				if (tree->IsAnchored())
				{
					solver_.StepClamping(robot, robot->GetTreeI(i), target, previousDirection_);
				}
				else
				{
					/*double tar[3]; robot->GetTreeI(i)->GetReferenceTarget(tar);
					matrices::Vector3 target;
					matrices::arrayToVect3(tar, target);
					solver_.StepClamping(robot, robot->GetTreeI(i), target, previousDirection_);*/
				}
			}
		}
	}
	else
	{
		matrices::Vector3 target;
		for(int i =0; i< robot->GetNumTrees(); ++i)
		{
			manip_core::TreeI* tree = robot->GetTreeI(i);
			if(tree->IsAnchored())
			{
				double tar[3]; tree->GetTarget(tar);
				matrices::arrayToVect3(tar, target);
				target = matrices::matrix4TimesVect3(transform, target);
				solver_.StepClamping(robot, tree, target, previousDirection_);
			}
			else
			{
				//tree->ToRest();
			}
		}
	}
}

void MotionHandler::Reset()
{
	previousDirection_ = Simulation::GetInstance()->simpParams_.initDir_;
	while(!actions_.empty())
	{
		delete actions_.front();
		actions_.pop();
	}
	splineManagers_.clear();
}

void MotionHandler::PushAction(MotionAction_ABC* action)
{
	actions_.push(action);
}

void MotionHandler::Translate(const matrices::Vector3& direction)
{
	Simulation* sg = Simulation::GetInstance();
	previousDirection_ = direction;
	manip_core::RobotI* robot = sg->pRobot;
	/*update with robot data*/
	if(sg->simpParams_.reachCom_)
	{
		double comDir[3];

		robot->ComTarget(comDir);
		if(previousDirection_.x() == 0)
		{
			previousDirection_(0) = comDir[0];
		}
		if(direction.y() == 0)
		{
			previousDirection_(1) = comDir[1];
		}
		if(direction.z() == 0)
		{
			previousDirection_(2) = 0 ; //comDir[2] + 1.6;
		}
		previousDirection_.normalize();
		/*Experimental shit*/
		NUMBER x =comDir[0];
		if(sg->simpParams_.reachComRotate_)
		{
			if (robot->ComputeCom(comDir))
			{
				/*double currentTransform[16];
				robot->ToWorldCoordinates(currentTransform);
				matrices::Matrix4 transform;
				matrices::array16ToMatrix4(currentTransform, transform);
				matrices::Vector3 vCom(comDir[0], comDir[1], comDir[2]);
				vCom = matrices::matrix4TimesVect3(transform, vCom);
				if(x > 0.1 || x < -0.1)
				{
					float angle = DegreesToRadians * 5 * x;
					Rotate(matrices::Roty3(angle));
				}*/
			}
			else
			{
				double currentTransform[16];
				robot->ToRobotCoordinates(currentTransform);
				matrices::Matrix4 transform;
				matrices::array16ToMatrix4(currentTransform, transform);
				matrices::Vector3 vCom(0, 0, 1);
				vCom = matrices::matrix4TimesVect3(transform, vCom);
				x = vCom(0);
				if(x > 0.1 || x < -0.1)
				{
					float angle = -DegreesToRadians * 5 * x;
					Rotate(matrices::Roty3(angle));
				}
			}
		}
	}
	else if (sg->simpParams_.reachComY_)
	{
		double comDir[2];
		manip_core::RobotI* robot = sg->pRobot;

		robot->ComTarget(comDir);
		if(direction.y() == 0)
		{
			previousDirection_(1) = comDir[1];
		}
		previousDirection_.normalize();
	}
	PushAction(new MotionActionTranslation(translationSpeed_ * 0.75, previousDirection_));
	
}

void MotionHandler::MoveBy(const matrices::Vector3& direction)
{
	Simulation* sg = Simulation::GetInstance();
	previousDirection_ = direction;
	/*update with robot data*/

	if(sg->simpParams_.reachCom_)
	{
		double comDir[2];
		manip_core::RobotI* robot = sg->pRobot;

		robot->ComTarget(comDir);
		if(previousDirection_.x() == 0)
		{
			previousDirection_(0) = comDir[0];
		}
		if(direction.y() == 0)
		{
			previousDirection_(1) = comDir[1];
		}
		previousDirection_.normalize();
	}
	else if (sg->simpParams_.reachComY_)
	{
		double comDir[2];
		manip_core::RobotI* robot = sg->pRobot;

		robot->ComTarget(comDir);
		if(direction.y() == 0)
		{
			previousDirection_(1) = comDir[1];
		}
		previousDirection_.normalize();
	}
	PushAction(new MotionActionMotion(previousDirection_));
	/*Experimental shit*/
	/*matrices::Vector3 dir(comDir[0], comDir[1], 0);
	if(dir.norm() != 0)
	{
		dir.normalize();
		matrices::Vector3 x(1,0,0);
		matrices::Matrix3 result;
		matrices::GetRotationMatrix(x, dir, result);
		Rotate(result);
	}*/
}

void MotionHandler::Rotate(const matrices::Matrix3& rotation)
{
	PushAction(new MotionActionRotation(rotationSpeed_, rotation));
}
