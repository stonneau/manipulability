#include "PostureSolver.h"

#include "world/World.h"
#include "world/ObstacleVisitor_ABC.h"
#include "world/Obstacle.h"
#include "PostureCriteria_ABC.h"
#include "world/Intersection.h"
#include "sampling/filters/FilterDistanceObstacle.h"
#include "sampling/filters/FilterDistance.h"
#include "sampling/SampleGenerator.h"
#include "sampling/Sample.h"
#include "kinematic/Tree.h"
#include "kinematic/Robot.h"
#include "kinematic/SupportPolygon.h"

#include "Trajectory/TrajectoryHandler.h"

//#include "kinematic/IKSolver.h"

#include <math.h>
#include <vector>
#include <list>

#ifdef PROFILE
#include "TimerPerf.h"
#endif

using namespace matrices;
using namespace std;

struct PosturePImpl
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PosturePImpl(const World& world)
		: world_(world)
		, currentDir_(1, 0, 0)
		, oldDir_(1, 0, 0)
		//, ikSolver_()
		, currentDirWeight_(0)
		, lastLifted_(-1)
		, jumpToTarget_(false)
		, trajectoryHandler_(world)
	{
		//NOTHING
		#ifdef PROFILE
		totaltime_ = 0;
		timerperf_.restart();
		#endif
	}
	~PosturePImpl()
	{
		Clear();
		for(T_CriteriaIT it = toeOffs_.begin(); it != toeOffs_.end(); ++it)
		{
			delete(*it);
		}
		for(T_CriteriaIT it = toeOns_.begin() ; it != toeOns_.end() ; ++it)
		{
			delete(*it);
		}
	}
	
	typedef std::list<manip_core::PostureCreatedListenerI*>	T_Listener;
	typedef T_Listener::iterator					T_ListenerIT;
	typedef T_Listener::const_iterator				T_ListenerCIT;

	typedef std::list<PostureCriteria_ABC*> T_Criteria;
	typedef T_Criteria::iterator			T_CriteriaIT;
	typedef T_Criteria::const_iterator		T_CriteriaCIT;

	void WarnListeners(NUMBER time, Robot* robot)
	{
		for(T_ListenerIT it = listeners_.begin(); it != listeners_.end(); ++it)
		{
			(*it)->OnPostureCreated(time, robot);
		}
	}

	void Clear()
	{
		for(PostureSolver::T_RobotsIT it = postures_.begin(); it != postures_.end(); ++it)
		{
			delete((*it).second);
		}
		postures_.clear();
		currentDir_ = Vector3(1,0,0);
		oldDir_ = Vector3(1,0,0);
		lastLifted_ = -1;
	}

	//IKSolver ikSolver_;
	const World& world_;
	Vector3 currentDir_;
	Vector3 oldDir_;
	NUMBER currentDirWeight_;
	PostureSolver::T_Robots postures_;
	T_Listener listeners_;
	T_Criteria toeOffs_;
	T_Criteria toeOns_;
	Tree::TREE_ID lastLifted_;
	bool jumpToTarget_;
	TrajectoryHandler trajectoryHandler_;
	#ifdef PROFILE
	TimerPerf timerperf_;
	std::vector<float> times_;
	float totaltime_;
	std::vector<int> hits_;
	#endif
};

struct HandleLockedVisitor : SampleGeneratorVisitor_ABC
{
	HandleLockedVisitor(const Vector3& currentDir)
		: currentBest_(0)
		, currentDir_(currentDir)
		, currentBestManip_(-100000)
	{
		// NOTHING
	}

	~HandleLockedVisitor()
	{
		// NOTHING
	}

	virtual void Visit(const Robot& robot, /*const*/ Tree& tree, Sample& sample)
	{
		//TODO Manipulability and other constraints here
		NUMBER manip = sample.forceManipulabiliy(currentDir_);
		if(manip > currentBestManip_)
		{
			currentBest_ = &sample;
			currentBestManip_ = manip;
		}
	}

	const Vector3 currentDir_;
	NUMBER currentBestManip_;
	Sample* currentBest_;
};

struct LockVisitor : public SampleGeneratorVisitor_ABC
{
	LockVisitor(const Vector3& currentDir, const World& world)
		: currentBest_(0)
		, currentDir_(currentDir)
		, currentBestManip_(-100000)
		, hits_(0)
		, world_(world)
	{
		// NOTHING
	}

	~LockVisitor()
	{
		// NOTHING
	}

	bool KeepsBalance(const Robot& robot, Tree& tree, const Vector3& target) const
	{
		// TODO this is ugly 
		bool wasLocked = tree.IsLocked();
		Vector3 oldtarget = tree.GetTarget();

		tree.LockTarget(target);
		Vector3 com = robot.ComputeCom();
		SupportPolygon support(robot);
		
		if(wasLocked)
		{
			tree.LockTarget(oldtarget);
		}
		else
		{
			tree.UnLockTarget();		
		}
		return support.Contains(com);
	}

	virtual void Visit(const Robot& robot, /*const*/ Tree& tree, Sample& sample, const Obstacle& obstacle)
	{
		// remove posture that is not enriching sustentation polygon
		//if(KeepsBalance(robot, tree, matrix4TimesVect3(robot.ToWorldCoordinates(), sample.GetPosition() + tree.GetPosition())))
		if(true)
		{
			hits_++;
			//TODO Manipulability and other constraints here
			NUMBER manip = sample.forceManipulabiliy(currentDir_);
			// colinear product btw surface and wanted dir. 
			Vector3 norm = obstacle.n_;
			Vector3 nDir = currentDir_;
			nDir.normalize();
			norm.normalize();
			if(robot.GetType() != manip_core::enums::robot::HumanEscalade && robot.GetType() != manip_core::enums::robot::HumanEllipse)
			{
				manip = manip * norm.dot(nDir);
			}
			if(manip >= currentBestManip_ )
			{
				Tree* testtree = tree.Clone() ;
				sample.LoadIntoTree(*testtree);
				if(!world_.IsColliding(robot, *testtree))
				{
					currentBest_ = &sample;
					currentBestManip_ = manip;
					obs_ = &obstacle;
				}
			}
			/*if(manip > currentBestManip_)
			{
				currentBest_ = &sample;
				currentBestManip_ = manip;
				obs_ = &obstacle;
			}*/
		}
	}
	int hits_;
	const Obstacle* obs_;
	const Vector3 currentDir_;
	NUMBER currentBestManip_;
	Sample* currentBest_;
	const World& world_;
};

struct LockVisitorClosestPoint : public LockVisitor
{
	LockVisitorClosestPoint(const Vector3& currentDir, const World& world)
		: LockVisitor(currentDir, world)
	{
		currentBestManip_ = 10000;
	}

	~LockVisitorClosestPoint()
	{
		// NOTHING
	}

	virtual void Visit(const Robot& robot, /*const*/ Tree& tree, Sample& sample, const Obstacle& obstacle)
	{
		// remove posture that is not enriching sustentation polygon
		//if(KeepsBalance(robot, tree, matrix4TimesVect3(robot.ToWorldCoordinates(), sample.GetPosition() + tree.GetPosition())))
		if(true)
		{
			hits_++;
			//TODO Manipulability and other constraints here
			Sample oldS(tree);
			Vector3 oldPos (tree.GetEffectorPosition(tree.GetNumEffector() -1));
			sample.LoadIntoTree(tree);
			tree.Compute();
			Vector3 newPos (tree.GetEffectorPosition(tree.GetNumEffector() -1));
			oldS.LoadIntoTree(tree);
			tree.Compute();
			NUMBER distance = (newPos- oldPos).norm();
			//if(tree.GetTreeType() == manip_core::enums::LeftArmCanap || tree.GetTreeType() == manip_core::enums::RightArmCanap)
			//{
			//	Vector3 norm = obstacle.n_;
			//	Vector3 nDir = currentDir_;
			//	nDir.normalize();
			//	norm.normalize();
			//	//distance = distance * norm.dot(nDir);
			//	distance += (newPos(2) - oldPos(2)) * 0.5;
			//	if(tree.GetTreeType() == manip_core::enums::LeftArmCanap)
			//	{
			//		distance -= (newPos(2) - oldPos(2));
			//		distance += (newPos(1) - oldPos(1)) * 0.5;
			//	}
			//}
			// colinear product btw surface and wanted dir. 
			if(distance < currentBestManip_)
			{
				Vector3 norm = obstacle.n_;
				Vector3 nDir = currentDir_;
				nDir.normalize();
				norm.normalize();
				//distance =  (norm.dot(nDir) > 0.7 ? distance : 10000);
				Tree* testtree = tree.Clone() ;
				sample.LoadIntoTree(*testtree);
				//if(currentBest_ == 0 || currentBest_->GetPosition().x() < sample.GetPosition().x() && obstacle.IsAbove(currentBest_->GetPosition()))
				if(!world_.IsColliding(robot, *testtree))
				{
					currentBest_ = &sample;
					currentBestManip_ = distance;
					obs_ = &obstacle;
				}
			}
			/*if(manip > currentBestManip_)
			{
				currentBest_ = &sample;
				currentBestManip_ = manip;
				obs_ = &obstacle;
			}*/
		}
	}
};


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


PostureSolver::PostureSolver(const World& world)
	: pImpl_(new PosturePImpl(world))
{
	//NOTHING
}

PostureSolver::~PostureSolver()
{
	// NOTHING
}

void PostureSolver::SetJumpToTarget(const bool jump)
{
	pImpl_->jumpToTarget_ = jump;
}


bool PostureSolver::MustLift(const Robot& robot, const Tree& tree) const
{
	bool off = pImpl_->toeOffs_.empty();
	for(PosturePImpl::T_CriteriaCIT it2 = pImpl_->toeOffs_.begin(); (!off) && it2 != pImpl_->toeOffs_.end(); ++it2)
	{
		off = (*it2)->Evaluate(pImpl_->world_, robot, tree);
	}
	return off;
}

bool PostureSolver::MustLock(const Robot& robot, const Tree& tree) const
{
	bool off = pImpl_->toeOns_.empty();
	for(PosturePImpl::T_CriteriaCIT it2 = pImpl_->toeOns_.begin(); (!off) && it2 != pImpl_->toeOns_.end(); ++it2)
	{
		off = (*it2)->Evaluate(pImpl_->world_, robot, tree);
	}
	return off;
}

bool PostureSolver::LockTree(Robot& robot, Tree& tree, bool closestDistance) const
{
	SampleGenerator* sg = SampleGenerator::GetInstance();
	// Collecting reachable obstacles
	ReachableObstaclesContainer obstacles(pImpl_->world_, tree, robot);
	pImpl_->world_.Accept(obstacles);
	Vector3 dirRobot;
	if(robot.GetType() == manip_core::enums::robot::HumanEscalade)
	//if(tree.GetTreeType() == manip_core::enums::LeftLegEscalade || tree.GetTreeType() == manip_core::enums::RightLegEscalade
	//	|| tree.GetTreeType() == manip_core::enums::LeftArmEscalade || tree.GetTreeType() == manip_core::enums::RightArmEscalade)
	{
		Vector3 go(pImpl_->currentDir_);
		go.normalize(); // grosse hacke pour manip jambes
	if(abs(go(0)) > abs(go(1)) || abs(go(2)) > abs(go(1)))	
	{if(abs(go(0)) > abs(go(2)) + 0.3)
		{
			go(2) = -go(0) / 2;
		}
		else if(abs(go(2)) > abs(go(0)) + 0.3)
		{
			go(0) = go(2) / 2;
		}
		go.normalize();
	}
		dirRobot = robot.ToRobotCoordinates().block<3,3>(0,0) * go;
	}
	if(tree.GetTreeType() == manip_core::enums::LeftLeg || tree.GetTreeType() == manip_core::enums::RightLeg)
	{
		Vector3 go(pImpl_->currentDir_);
		go.normalize(); // grosse hacke pour manip jambes
	if(abs(go(0)) > abs(go(1)) || abs(go(2)) > abs(go(1)))	
	{if(abs(go(0)) > abs(go(2)) + 0.3)
		{
			go(2) = +go(0) / 2;
		}
		else if(abs(go(2)) > abs(go(0)) + 0.3)
		{
			go(0) =- go(2) / 2;
		}
		go.normalize();
	}
		dirRobot = robot.ToRobotCoordinates().block<3,3>(0,0) * go;
	}
	else
	{
		//dirRobot = robot.ToRobotCoordinates().block<3,3>(0,0) * pImpl_->currentDir_;
		dirRobot = pImpl_->currentDir_;
	}
	dirRobot.normalize();
	LockVisitor* visitor;
	if(!closestDistance)
	{
		visitor = new LockVisitor(dirRobot, pImpl_->world_);
	}
	else
	{
		visitor = new LockVisitorClosestPoint(dirRobot, pImpl_->world_);
	}
	Intersection intersect;
	bool ret = false;
	#ifdef PROFILE
	float bef = pImpl_->timerperf_.elapsedTime();
	int hits = 0;
	#endif
	Robot* futureRob = robot.Clone();
	futureRob->Translate(pImpl_->currentDir_ * 0.2);
	for(ReachableObstaclesContainer::T_ObstaclesCIT it = obstacles.obstacles_.begin(); it!= obstacles.obstacles_.end(); ++it)
	{
		Vector3 intersectionPoint;
		Vector3 treePositionWorld = matrix4TimesVect3(futureRob->ToWorldCoordinates(), tree.GetPosition());
		if(intersect.IntersectClosest(*futureRob, tree, treePositionWorld, (*(*it)), intersectionPoint))
		{
			if((treePositionWorld - intersectionPoint).norm() < tree.GetBoundaryRadius())
			{
				FilterDistanceObstacle filter(0.1, tree, (*(*it)), *futureRob, dirRobot);
				//sg->Request(robot, tree, visitor, filter);
				sg->Request(robot, tree, visitor, filter, *(*it));
			}
		}
	}
	if(visitor->currentBest_)
	{
		Sample old(tree);
		visitor->currentBest_->LoadIntoTree(tree);
		tree.Compute();
		tree.direction_ = pImpl_->currentDir_;
		tree.direction_.normalize();
// TMP
		pImpl_->world_.IsColliding(robot, tree);
		// sample needs to be replaced regarding tree root position,
		// compute exact position on obstacle( clostest)
		Vector3 samplePosition = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition() + visitor->currentBest_->GetPosition());
		//const Vector3& exactPosition = visitor.obs_->Center();
	//	pImpl_->world_.GetTarget(robot, tree, samplePosition, exactPosition);
		//Vector3 exactRobotPosition = matrices::matrix4TimesVect3(robot.ToRobotCoordinates(), exactPosition);
		// now let's go there with ik
		//pImpl_->IkToTarget(tree, exactRobotPosition);
		if(pImpl_->jumpToTarget_)
		{
			//tree.LockTarget(visitor->obs_->ProjectUp(samplePosition));
			tree.LockTarget(samplePosition, visitor->obs_);
		}
		else
		{
			tree.LockTarget(samplePosition, visitor->obs_);
			tree.targetSample_ = visitor->currentBest_;
			//tree.LockTarget(visitor->obs_->ProjectUp(samplePosition));
		}
		if(!pImpl_->jumpToTarget_)
		{
			old.LoadIntoTree(tree);
		}
		//tree.LockTarget(matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition() + pImpl_->currentBest_->GetPosition()));
		ret = true;
	}
	else
	{
		//tree.LockTarget(matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetEffectorPosition(tree.GetNumEffector()-1)));
		ret = false;
	}
	#ifdef PROFILE
	float end = pImpl_->timerperf_.elapsedTime();
	pImpl_->times_.push_back(end-bef);
	pImpl_->hits_.push_back(visitor->hits_);
	#endif
	delete visitor;
	return ret;
}


bool PostureSolver::LockTree(Robot& robot, Tree& tree, Sample& sample, bool closestDistance) const
{
	SampleGenerator* sg = SampleGenerator::GetInstance();
	// Collecting reachable obstacles
	ReachableObstaclesContainer obstacles(pImpl_->world_, tree, robot);
	pImpl_->world_.Accept(obstacles);
	Vector3 dirRobot;
	if(robot.GetType() == manip_core::enums::robot::HumanEscalade)
	//if(tree.GetTreeType() == manip_core::enums::LeftLegEscalade || tree.GetTreeType() == manip_core::enums::RightLegEscalade
	//	|| tree.GetTreeType() == manip_core::enums::LeftArmEscalade || tree.GetTreeType() == manip_core::enums::RightArmEscalade)
	{
		Vector3 go(pImpl_->currentDir_);
		go.normalize(); // grosse hacke pour manip jambes
	if(abs(go(0)) > abs(go(1)) || abs(go(2)) > abs(go(1)))	
	{if(abs(go(0)) > abs(go(2)) + 0.3)
		{
			go(2) = -go(0) / 2;
		}
		else if(abs(go(2)) > abs(go(0)) + 0.3)
		{
			go(0) = go(2) / 2;
		}
		go.normalize();
	}
		dirRobot = robot.ToRobotCoordinates().block<3,3>(0,0) * go;
	}
	else
	{
		dirRobot = robot.ToRobotCoordinates().block<3,3>(0,0) * pImpl_->currentDir_;
	}
	dirRobot.normalize();
	LockVisitor* visitor;
	if(!closestDistance)
	{
		visitor = new LockVisitor(dirRobot, pImpl_->world_);
	}
	else
	{
		visitor = new LockVisitorClosestPoint(dirRobot, pImpl_->world_);
	}
	Intersection intersect;
	bool ret = false;
	#ifdef PROFILE
	float bef = pImpl_->timerperf_.elapsedTime();
	int hits = 0;
	#endif
	for(ReachableObstaclesContainer::T_ObstaclesCIT it = obstacles.obstacles_.begin(); it!= obstacles.obstacles_.end(); ++it)
	{
		Vector3 intersectionPoint;
		Vector3 treePositionWorld = matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition());
		if(intersect.IntersectClosest(robot, tree, treePositionWorld, (*(*it)), intersectionPoint))
		{
			if((treePositionWorld - intersectionPoint).norm() < tree.GetBoundaryRadius())
			{
				FilterDistanceObstacle filter(0.1, tree, (*(*it)), robot, dirRobot);
				//sg->Request(robot, tree, visitor, filter);
				sg->Request(robot, tree, visitor, filter, *(*it));
			}
		}
	}
	if(visitor->currentBest_)
	{
		Sample old(tree);
		visitor->currentBest_->LoadIntoTree(tree);
		sample = *(visitor->currentBest_);
		tree.Compute();
		tree.direction_ = pImpl_->currentDir_;
		tree.direction_.normalize();
		// sample needs to be replaced regarding tree root position,
		// compute exact position on obstacle( clostest)
		Vector3 samplePosition = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition() + visitor->currentBest_->GetPosition());
		const Vector3& exactPosition(samplePosition);
		//const Vector3& exactPosition = visitor.obs_->Center();
	//	pImpl_->world_.GetTarget(robot, tree, samplePosition, exactPosition);
		//Vector3 exactRobotPosition = matrices::matrix4TimesVect3(robot.ToRobotCoordinates(), exactPosition);
		// now let's go there with ik
		//pImpl_->IkToTarget(tree, exactRobotPosition);
		tree.LockTarget(exactPosition, visitor->obs_);
		tree.targetSample_ = visitor->currentBest_;
		if(!pImpl_->jumpToTarget_)
		{
			old.LoadIntoTree(tree);
		}
		//tree.LockTarget(matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition() + pImpl_->currentBest_->GetPosition()));
		ret = true;
	}
	else
	{
		//tree.LockTarget(matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetEffectorPosition(tree.GetNumEffector()-1)));
		ret = false;
	}
	#ifdef PROFILE
	float end = pImpl_->timerperf_.elapsedTime();
	pImpl_->times_.push_back(end-bef);
	pImpl_->hits_.push_back(visitor->hits_);
	#endif
	delete visitor;
	return ret;
}

#ifdef PROFILE
#include "FileHandler.h"

void PostureSolver::Log() const
{
	// computing min max and average values
	float minTime = 0; float  maxTime= 0; float  avgTime = 0;
	float timeCumul = 0.f;
	int minHits= 0; int maxHits= 0; int avgHits = 0;
	int hitsCumul = 0;
	int nbReq = std::min(pImpl_->hits_.size(), pImpl_->times_.size());
	for(int i = 0; i< nbReq; ++i)
	{
		float cHit = pImpl_->hits_[i]; float cTime = pImpl_->times_[i];
		timeCumul += cTime;
		hitsCumul += cHit;
		if( cTime < minTime ) minTime = cTime;
		if( cTime > maxTime ) maxTime = cTime;
		if( cHit < minHits ) minHits = cHit;
		if( cHit > maxHits ) maxHits = cHit;
	}
	avgTime = timeCumul / nbReq;
	avgHits = hitsCumul / nbReq;
	FileHandler f("log.txt");
	f << "nbReq " << nbReq << "\n";
	f << "minTime " << minTime << "\n"; f << "maxTime " << maxTime << "\n"; f << "avgTime " << avgTime << "\n";
	f << "minHits " << minHits << "\n"; f << "maxHits " << maxHits << "\n"; f << "avgHits " << avgHits << "\n";
	f << "totaltime " << pImpl_->totaltime_ << "\n";
	f.Save();
}
#endif

bool PostureSolver::HandleLockedTree(Robot& robot, Tree& tree)
{
	FilterDistance filter(0.05, tree, matrices::matrix4TimesVect3(robot.ToRobotCoordinates(), tree.GetTarget()));
	SampleGenerator* sg = SampleGenerator::GetInstance();
	HandleLockedVisitor visitor(pImpl_->currentDir_);
	sg->Request(robot, tree, &visitor, filter);
	if(visitor.currentBest_)
	{
		visitor.currentBest_->LoadIntoTree(tree);
		tree.Compute();
		return true;
	}
	else
	{
		return false;
	}
	return true;
	//// now let's go there with ik
	//pImpl_->IkToTarget(tree, exactRobotPosition);
	// Collecting reachable obstacles
}

bool TargetReached(const Robot& robot, const Tree& tree)
{
	// converting real world target robot coordinates
	Vector3 roboTarget = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetTarget());
	return (tree.GetEffectorPosition(tree.GetNumEffector()-1) - roboTarget).norm() < 1.;
}

int PostureSolver::NextPosture(Robot& robot, const Vector3& direction, bool handleLock)
{
	int changes = 0;
	pImpl_->currentDir_ = direction;
	Robot::T_Tree& trees = robot.GetTrees();
	for(Robot::T_TreeIT it = trees.begin(); it != trees.end(); ++it)
	{
		Tree* tree = (*it);
		if(tree->IsLocked())
		{
			if(MustLift(robot, *tree))
			{
				tree->UnLockTarget();
				++changes;
			}
			else if(handleLock)
			{
				if(!HandleLockedTree(robot, *tree))
				{
					tree->UnLockTarget();
					++changes;
				}
			}
		}
		else if(!tree->IsLocked())
		{
			if(MustLock(robot, *tree))
			{
				LockTree(robot, *tree);
				changes++;
			}
		}
	}
	return changes;
}

manip_core::T_CubicTrajectory PostureSolver::NextTrajectory(Robot& robot, const Vector3& direction, bool handleLock, bool closestDistance)
{
	manip_core::T_CubicTrajectory cubics;
	int changes = 0;
	pImpl_->currentDir_ = direction;
	Vector3 normDir = direction;
	if (normDir.norm() != 0)
	{
		normDir.normalize();
	}
	Robot::T_Tree& trees = robot.GetTrees();
	for(Robot::T_TreeIT it = trees.begin(); it != trees.end(); ++it)
	{
		Tree* tree = (*it);
		if(tree->IsLocked())
		{
			NUMBER r = (pImpl_->currentDir_.transpose()*(tree->GetJacobian()->GetJacobianProduct())*pImpl_->currentDir_);
			r = 1 / sqrt(r);
			if( /*abs(tree->direction_.dot(normDir)) < 0.3 ||  r < 0.2 ||*/ MustLift(robot, *tree))
			{
				if (tree->targetReached_)
					{
						tree->targetReached_ = false;
					}
				tree->UnLockTarget();
				++changes;
			}
			else if(handleLock)
			{
				if(!HandleLockedTree(robot, *tree))
				{
					tree->UnLockTarget();
					++changes;
				}
			}
		}
		else if(!tree->IsLocked())
		{
			spline::curve_abc<>* cubic(0);
			if(MustLock(robot, *tree))
			{
				Tree* tree2 = tree->Clone();
				if(pImpl_->jumpToTarget_)
				{
					if(LockTree(robot, *tree, closestDistance))
					{
						cubic = pImpl_->trajectoryHandler_.ComputeTrajectory(robot, *tree2, *tree, tree->GetTarget());
					}
				}
				else
				{
					Sample sample(*tree);
					if(LockTree(robot, *tree, sample, closestDistance))
					{
						sample.LoadIntoTree(*tree2);
						tree2->Compute();
						cubic = pImpl_->trajectoryHandler_.ComputeTrajectory(robot, *tree, *tree2, tree->GetTarget());
					}
				}
				changes++;
			}
			if(cubic)
			{
				cubics.push_back(std::make_pair((int)(tree->GetId()), cubic));
			}
		}
	}
	return cubics;
}


const PostureSolver::T_Robots& PostureSolver::CreatePostures(const Robot& previousTransform, Trajectory& trajectory, bool stopAtFirst)
{
	#ifdef PROFILE
	float beg = pImpl_->timerperf_.elapsedTime();
	#endif
	pImpl_->Clear();
	Robot* previousPosture(0);
	Trajectory::T_TimePositions& timepos = trajectory.GetEditableTimePositions();
	Matrix3 rotation;
	Matrix4 oldTransformation;
	Matrix4 tranformation = previousTransform.ToWorldCoordinates();
	Vector3 oldPosition = tranformation.block(0,3,3,1);
	float currentTimeStep = 0.f;
	previousPosture = previousTransform.Clone();
	
	//current pos as first index
	//trajectory.AddCheckPoint(0.f, oldPosition);
	Trajectory::T_TimePositionsIT it = timepos.begin();
	++it;
	for(; it != timepos.end() && pImpl_->postures_.size()<80; ++it)
	{
		pImpl_->currentDir_ = ((*it).second - oldPosition);		
		if(pImpl_->currentDir_.norm() != 0.)
		{
			// only for vlimbing)
			if(pImpl_->currentDir_(0) <= 0)
			{
				pImpl_->currentDir_(0) = 0.2;
			}
			pImpl_->currentDir_.normalize();
		}
		else
		{
			pImpl_->currentDir_ = pImpl_->oldDir_;
			++it;
			if( it == timepos.end())
				break;
		}
		oldTransformation = tranformation;

		//compute transformation
		matrices::GetRotationMatrix(pImpl_->oldDir_, pImpl_->currentDir_, rotation);
		tranformation.block(0,0,3,3) = tranformation.block(0,0,3,3) * rotation; 
		tranformation.block(0,3,3,1) = (*it).second; 
		//End compute transformation


		Robot* newPosture = previousPosture->Clone();
		newPosture->SetPosOri(tranformation);
		int changes = NextPosture(*newPosture, pImpl_->currentDir_, true);
		
		if(changes > 1 && trajectory.AddWayPoint(--it)) // it placed on the new waypoint
		{
			it--; // because it ll be increased in loop
			tranformation = oldTransformation;
			pImpl_->currentDir_ = pImpl_->oldDir_;
		}
		else
		{
			if((changes == 0) &! (pImpl_->postures_.empty()))
			{
				pImpl_->postures_.pop_back();
			}
			/*else if(changes > 1)
			{
				delete newPosture;
				newPosture = CreateStillPosture2(*previousPosture);
				tranformation = oldTransformation;
				pImpl_->currentDir_ = pImpl_->oldDir_;
			}*/
			currentTimeStep = (*it).first;
			pImpl_->postures_.push_back(std::make_pair(currentTimeStep, newPosture));
			previousPosture = newPosture;
			// robot posture can be changed by optimization
			tranformation = newPosture->ToWorldCoordinates();
			oldPosition = tranformation.block(0,3,3,1);
			pImpl_->oldDir_ = pImpl_->currentDir_;
			if(stopAtFirst)
			{
				return pImpl_->postures_;
			}
		}
	}
	for(PostureSolver::T_RobotsIT it = pImpl_->postures_.begin(); it != pImpl_->postures_.end(); ++it)
	{
		pImpl_->WarnListeners((*it).first, (*it).second);
	}
	#ifdef PROFILE
	float end = pImpl_->timerperf_.elapsedTime();
	pImpl_->totaltime_ = end - beg;
	#endif
	return pImpl_->postures_;
}


void PostureSolver::RegisterPostureListener(manip_core::PostureCreatedListenerI& listener)
{
	pImpl_->listeners_.push_back(&listener);
	for(PostureSolver::T_RobotsIT it = pImpl_->postures_.begin(); it != pImpl_->postures_.end(); ++it)
	{
		listener.OnPostureCreated((*it).first, (*it).second);
	}
}

bool PostureSolver::UnregisterPostureListener(const manip_core::PostureCreatedListenerI& listener)
{
	//TODO
	return false;
}

void PostureSolver::AddToeOffCriteria(PostureCriteria_ABC* criteria)
{
	assert(criteria);
	pImpl_->toeOffs_.push_back(criteria);
}

void PostureSolver::AddToeOnCriteria(PostureCriteria_ABC* criteria)
{
	assert(criteria);
	pImpl_->toeOns_.push_back(criteria);
}
