#include "PostureManagerImpl.h"

#include "posture/PostureCriteriaToeOffBoundary.h"
#include "posture/PostureCriteriaToeOffJointLimit.h"
#include "posture/PostureCriteriaToeOffGait.h"
#include "posture/PostureCriteriaToeOnGait.h"
#include "posture/PostureCriteriaToeOnCOM.h"
#include "sampling/SampleGenerator.h"

#include "API/RobotI.h"
#include "API/TreeI.h"
#include "kinematic/Robot.h"

#include "MatrixDefs.h"

using namespace manip_core;
using namespace manip_core::enums;
using namespace matrices;


struct SampleCollector : public SampleGeneratorVisitor_ABC
{
	SampleCollector(SampleVisitorI * visitor, const manip_core::RobotI* robot, const World&(world), bool collide)
		: current_(0)
		, visitor_(visitor)
		, robot_(robot)
		, world_(world)
		, collide_(collide)
	{
		// NOTHING
	}

	~SampleCollector()
	{
		// NOTHING
	}
	
	virtual void Visit(const Robot& robot, /*const*/ Tree& tree, Sample& sample)
	{
		Tree* newTree = tree.Clone();
		sample.LoadIntoTree(*newTree);
		if(!collide_ || !world_.IsColliding(robot, *newTree))
		{
			visitor_->Visit(robot_, newTree);
		}
	}

	virtual void Visit(const Robot& robot, /*const*/ Tree& tree, Sample& sample, const Obstacle& obstacle)
	{
		Tree* newTree = tree.Clone();
		sample.LoadIntoTree(*newTree);
		newTree->LockTarget(newTree->attach_, &obstacle);
		visitor_->Visit(robot_, newTree);
	}
	Tree current_;
	const manip_core::RobotI* robot_;
	const World& world_;
	const bool collide_;
	SampleVisitorI * visitor_;
};

PostureManagerImpl::PostureManagerImpl(const World& world)
	: pSolver_(world)
	, initialized_(false)
	, spiderGait_(0)
	, world_(world)
{
	// NOTHING
}

PostureManagerImpl::~PostureManagerImpl()
{
	if (spiderGait_)
	{
		delete spiderGait_;
	}
}

void PostureManagerImpl::Release()
{
	delete this;
}

void PostureManagerImpl::AddTrajectoryPoint(const float time,const  double* transform)
{
	matrices::Vector3 transf;
	matrices::arrayToVect3(transform, transf);
	trajectory_.AddCheckPoint(time, transf);
}

void PostureManagerImpl::ResetTrajectory()
{
	trajectory_.Reset();
}



void PostureManagerImpl::AddPostureCriteria(const enums::postureCriteria::ePostureCriteria criteria)
{
	switch(criteria)
	{
		case postureCriteria::toeOffBoundary:
			{
				pSolver_.AddToeOffCriteria(new PostureCriteriaToeOffBoundary());
				break;
			}
		case postureCriteria::toeOnCOM:
			{
				pSolver_.AddToeOnCriteria(new PostureCriteriaToeOnCOM());
				break;
			}
		case postureCriteria::toeOffJointLimit:
			{
				pSolver_.AddToeOffCriteria(new PostureCriteriaToeOffJointLimit());
				break;
			}
		case postureCriteria::toeOffSpiderGait:
			{
				if (spiderGait_ == 0)
				{
					spiderGait_ = new SpiderGait();
				}
				pSolver_.AddToeOffCriteria(new PostureCriteriaToeOffGait(spiderGait_));
				break;
			}
		case postureCriteria::toeOnSpiderGait:
			{
				if (spiderGait_ == 0)
				{
					spiderGait_ = new SpiderGait();
				}
				pSolver_.AddToeOnCriteria(new PostureCriteriaToeOnGait(spiderGait_));
				break;
			}
		default:
			break;
	}
}

void PostureManagerImpl::SetJumpToTarget(const bool jump)
{
	pSolver_.SetJumpToTarget(jump);
}


void PostureManagerImpl::RegisterPostureCreatedListenerI(PostureCreatedListenerI* listener)	
{
	pSolver_.RegisterPostureListener(*listener);
}

void PostureManagerImpl::UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* listener)
{
	pSolver_.UnregisterPostureListener(*listener);
}

void PostureManagerImpl::InitSamples(const RobotI* robot, int nbSamples)
{
	assert(robot);
	Robot * rob = (static_cast<const Robot*>(robot))->Clone();
	SampleGenerator* sg = SampleGenerator::GetInstance();
	sg->GenerateSamples(*rob, nbSamples);
	initialized_ = true;
	delete rob;
}

#include "world/ObstacleVisitor_ABC.h"
#include "sampling/filters/Filter_ABC.h"

struct ReachableObstaclesContainer : public ObstacleVisitor_ABC
{
	ReachableObstaclesContainer(const World& world, const Tree& tree, const Robot& robot)
		: ObstacleVisitor_ABC()
		, world_(world)
		, tree_ (tree)
		, robot_(robot)
	{
		// NOTHING
	}

	~ReachableObstaclesContainer()
	{
		// NOTHING
	}

	virtual void Visit(const Obstacle& obstacle)
	{
		if(!obstacle.donttouch_ && world_.IsReachable(robot_, tree_, obstacle))
			obstacles_.push_back(&obstacle);
	}

	typedef std::vector<const Obstacle*> T_Obstacles;
	typedef T_Obstacles::const_iterator T_ObstaclesCIT;
	typedef T_Obstacles::iterator		T_ObstaclesIT;

	T_Obstacles obstacles_;
	const World& world_;
	const Tree&  tree_ ;
	const Robot& robot_;
};

namespace
{
struct DummyFilter : public Filter_ABC
{
	DummyFilter(){}
	~DummyFilter(){}
	virtual bool ApplyFilter(const Sample& sample) const {return true;}

};
}

void PostureManagerImpl::AcceptSampleVisitor(const RobotI* robot, const TreeI* tree, SampleVisitorI * visitor, bool collide)
{
	SampleCollector collector(visitor, robot, world_, collide);
	Robot * rob = (static_cast<const Robot*>(robot))->Clone();
	Tree * tre = (static_cast<const Tree*>(tree))->Clone();
	SampleGenerator* sg = SampleGenerator::GetInstance();
	if(collide)
	{
		DummyFilter filter;
		ReachableObstaclesContainer obstacles(world_, *tre, *rob);
		world_.Accept(obstacles);
		for(ReachableObstaclesContainer::T_ObstaclesCIT it = obstacles.obstacles_.begin(); it!= obstacles.obstacles_.end(); ++it)
		{
			sg->Request(*rob, *tre, &collector, filter, **it);
		}
	}
	else
	{
		sg->Request(*rob, *tre, &collector);
	}
	delete rob;
	delete tre;
}


void PostureManagerImpl::ComputeOnline(const RobotI* robot, int nbSamples)
{
	assert(robot);
	Robot * rob = (static_cast<const Robot*>(robot))->Clone();
	if(!initialized_)
	{
		InitSamples(rob, nbSamples);
	}// TODO : this is just horrible. Remove sample generation
	pSolver_.CreatePostures(*rob, trajectory_);
}

T_CubicTrajectory PostureManagerImpl::NextPosture(RobotI* robot, double* direction, bool closestDistance)
{
	Vector3 dir;
	matrices::arrayToVect3(direction, dir);
	assert(initialized_);
	Robot * rob = (static_cast<Robot*>(robot));
	return pSolver_.NextTrajectory(*rob, dir, false, closestDistance);
}

void PostureManagerImpl::Update(const unsigned long time)
{
	if (spiderGait_)
	{
		spiderGait_->Update(time);
	}
}

#ifdef PROFILE
void PostureManagerImpl::Log() const
{
	pSolver_.Log();
}
#endif

