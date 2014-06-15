
#include "Robot.h"
#include "RobotVisitor_ABC.h"
#include "kinematic/Tree.h"

#include "world/World.h"
#include "kinematic/Com.h"
#include "kinematic/ComVisitor_ABC.h"
#include "kinematic/SupportPolygon.h"

#include "API/TreeI.h"

//#include "IKSolver.h"
//#include "ForceManipulabilityConstraint.h"

using namespace matrices;
using namespace Eigen;

using namespace std;
using namespace manip_core::enums;

struct RobotPImpl
{
	typedef std::vector<matrices::Vector3>	T_Attach;
	typedef T_Attach::iterator IT_Attach;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RobotPImpl(const Matrix4& transform, Tree* torsoAndHead)
		: torso_(0)
	{
		SetTransform(transform);
		assert(torsoAndHead);
		torsoAndHead->Init();
		torsoAndHead->Compute();
		torso_ = torsoAndHead;
		InitColumnHierarchy();
	}

	~RobotPImpl()
	{
		for(Robot::T_TreeIT it = trees_.begin(); it!= trees_.end(); ++it)
		{
			delete(*it);
		}
		if(torso_)
		{
			delete(torso_);
		}
	}

	void InitColumnHierarchy()
	{
		Joint* j = torso_->GetRoot();
		while(j)
		{
			Robot::T_Tree trees;
			hierarchy_.push_back(trees);
			j = j->pChild_;
		}
	}

	void SetTransform(const Matrix4& transform)
	{
		toWorldCoo_ = transform;
		toRobotCoo_ = transform.inverse();
	}

	void AddTree(Tree* tree, const matrices::Vector3& attach, const unsigned int columnJoint)
	{
		assert(columnJoint < hierarchy_.size());
		hierarchy_.at(columnJoint).push_back(tree);
		trees_.push_back(tree);
		attaches_.push_back(attach);
	}
	
	T_Attach attaches_;
	//IKSolver ikSolver_;
	Matrix4 toRobotCoo_;
	Matrix4 toWorldCoo_;

	Robot::T_Tree trees_;
	Tree* torso_;
	Robot::T_Hierarchy hierarchy_;
};

struct ComVisitor : public ComVisitor_ABC
{
	 ComVisitor()
		: ComVisitor_ABC()
		, sigmaAi_(0.f)
		, positionSum_(0, 0, 0)
	 {
		// NOTHING
	 }
	~ComVisitor()
	 {
		// NOTHING
	 }

	// source http://fr.wikipedia.org/wiki/Fonctions_de_Leibniz#Fonction_vectorielle_de_Leibniz
void VisitCom(const matrices::Vector3& position, const float& weight)
	{
		positionSum_ += weight * position;
		sigmaAi_ += weight;
	}

	Vector3 GetCom() const
	{
		return ( 1.f / sigmaAi_ ) * positionSum_;
	}

	float sigmaAi_;
	Vector3 positionSum_;;
};

Robot::Robot(const Matrix4& transform, Tree* torsoAndHead, manip_core::enums::robot::eRobots robotType)
	: pImpl_(new RobotPImpl(transform, torsoAndHead))
	, numTrees_(0)
	, robotType_(robotType)
{
	// which constraints are we going to use ?
//	pImpl_->ikSolver_.Register(new ForceManipulabilityConstraint);
}

void Robot::SetPosOri(const matrices::Matrix4& transform)
{
	pImpl_->SetTransform(transform);
}

void Robot::SetTransform(const double* worldTransform)
{
	matrices::array16ToMatrix4(worldTransform, pImpl_->toWorldCoo_);
	pImpl_->toRobotCoo_ = pImpl_->toWorldCoo_.inverse();
}

void Robot::Translate(const double* directionVector)
{
	Vector3 tr;
	matrices::arrayToVect3(directionVector, tr);
	Translate(tr);
}

Robot::~Robot()
{
	// NOTHING
}

const Tree* Robot::GetTorso() const
{
	return pImpl_->torso_;
}

void Robot::LockOnCurrent(int treeId)
{
	Tree* t = pImpl_->trees_[treeId];
	t->Compute();
	t->LockTarget(matrices::matrix4TimesVect3(pImpl_->toWorldCoo_, t->GetEffectorPosition(t->GetNumEffector()-1)));
}

void Robot::Accept(RobotVisitor_ABC& visitor) const
{
	/*for(T_TreeCIT it = pImpl_->trees_.begin(); it!= pImpl_->trees_.end(); ++it)
	{
		visitor.Visit(*(*it));
	}*/
	
	Joint* anchor = pImpl_->torso_->GetRoot();
	for(T_Hierarchy::const_iterator it0 = pImpl_->hierarchy_.begin(); it0!= pImpl_->hierarchy_.end() && (anchor != 0); ++it0)
	{
		for(T_TreeCIT it = it0->begin(); it!= it0->end(); ++it)
		{
			visitor.Visit(*(*it), anchor);
		}
		anchor = anchor->pChild_;
	}
}

void Robot::AddTree(Tree* tree,  const matrices::Vector3& attach, const unsigned int columnJoint)
{
	assert(tree);
	tree->Init();
	tree->Compute();
	tree->directionForce_ = Vector3(0,0,1);
	tree->directionVel_ = Vector3(1,0,0);
	++ numTrees_;
	pImpl_->AddTree(tree, attach, columnJoint);
}

const unsigned int Robot::GetNumTrees() const
{
	return numTrees_;
}


Tree* Robot::GetTree(Tree::TREE_ID id) const
{
	return id >= numTrees_ ? 0 : pImpl_->trees_[id];
}

matrices::Vector3 Robot::ComputeCom() const
{
	ComVisitor visitor;
	for(T_TreeCIT it = pImpl_->trees_.begin(); it!= pImpl_->trees_.end(); ++it)
	{
		(*it)->AcceptComVisitor(&visitor);
	}
	if(pImpl_->torso_)
	{
		pImpl_->torso_->AcceptComVisitor(&visitor);
	}
	return visitor.GetCom();
}



bool Robot::ComputeCom(double* directionVector) const
{
	Vector3 com(ComputeCom());
	matrices::vect3ToArray(directionVector, com);
	SupportPolygon sp(*this);
	return sp.Contains(com);
}


void Robot::ComTarget(double* directionVector) const
{
	SupportPolygon sp(*this);
	Vector3 res = sp.Centroid();
	directionVector[0] = res(0);
	directionVector[1] = res(1);
	directionVector[2] = res(2);
}



//void Robot::Move(const matrices::Vector3& direction, const World& world)
//{
//	//Vector3 direction = (matrix4TimesVect3(pImpl_->toRobotCoo_, dir));
//	//direction.normalize();
//	Vector3 target;
//	for(T_TreeIT it = pImpl_->trees_.begin(); it!= pImpl_->trees_.end(); ++it)
//	{
//		if((*it)->IsLocked())
//		{
//			target = matrix4TimesVect3(pImpl_->toRobotCoo_, (*it)->GetTarget());
//			//if(world.Intersect((**it), target))
//			if(world.IsReachable(*this, (**it), (*it)->GetTarget()))
//			{
//				//pImpl_->ikSolver_.StepForceManipulability(**it, target, direction);
//				pImpl_->ikSolver_.StepClamping(**it, target, direction);
//			}
//			else 
//			{
//				(*it)->UnLockTarget();
//			}
//		}
//		else if(world.GetTarget(*this, **it, direction, target))
//		{
//			// translate target coordinates to robo coordinates2
//			Vector3 robotTarget = matrix4TimesVect3(pImpl_->toRobotCoo_, target);
//			//if (pImpl_->ikSolver_.StepForceManipulability(**it, robotTarget, direction))
//			if (pImpl_->ikSolver_.StepClamping(**it, robotTarget, direction))
//			{
//				(*it)->LockTarget(target);
//			}
//			//pImpl_->ikSolver_.Step(**it, target);
//		}
//		else
//		{
//			//pImpl_->ikSolver_.StepForceManipulability(**it, (*it)->referenceTarget_, direction);
//			pImpl_->ikSolver_.StepClamping(**it, (*it)->referenceTarget_, direction);
//		}
//	}
//}

void Robot::Reset()
{
	pImpl_->toWorldCoo_.block(0,3,3,1) = pImpl_->toWorldCoo_.block(0,3,3,1) - pImpl_->toWorldCoo_.block(0,3,3,1);
	Rest();
}



const matrices::Matrix4& Robot::ToWorldCoordinates() const
{
	return pImpl_->toWorldCoo_;
}

const matrices::Matrix4& Robot::ToRobotCoordinates() const
{
	return pImpl_->toRobotCoo_;
}

void Robot::Translate(const matrices::Vector3& direction)
{
	pImpl_->toWorldCoo_.block(0,3,3,1) = pImpl_->toWorldCoo_.block(0,3,3,1) + direction;
	pImpl_->toRobotCoo_.block(0,3,3,1) = pImpl_->toRobotCoo_.block(0,3,3,1) - direction;
}

Robot::T_Tree& Robot::GetTrees() const
{
	return pImpl_->trees_;
}

const Robot::T_Hierarchy& Robot::GetHierarchy() const
{
	return pImpl_->hierarchy_;
}

const manip_core::enums::robot::eRobots& Robot::RobotType() const
{
	return robotType_;
}

manip_core::enums::robot::eRobots Robot::GetType() const
{
	return RobotType();
}

Robot* Robot::Clone() const
{
	Robot* res = new Robot(pImpl_->toWorldCoo_, pImpl_->torso_->Clone(), robotType_);
	unsigned int anchor = 0;
	for(T_Hierarchy::const_iterator it0 = pImpl_->hierarchy_.begin(); it0!= pImpl_->hierarchy_.end(); ++it0)
	{
		for(T_TreeCIT it = it0->begin(); it!= it0->end(); ++it)
		{
			res->AddTree((*it)->Clone(), (pImpl_->attaches_[(*it)->GetId()]), anchor);
		}
		++anchor;
	}
	return res;
}

void Robot::Rest()
{
	for(T_TreeIT it = pImpl_->trees_.begin(); it!= pImpl_->trees_.end(); ++it)
	{
		(*it)->UnLockTarget();
		(*it)->ToRest();
	}
}


void Robot::Release()
{
	delete this;
}

void Robot::ToRobotCoordinates(double* worldTransform) const
{
	matrices::matrixTo16Array(worldTransform, pImpl_->toRobotCoo_);
}

void Robot::ToWorldCoordinates(double* worldTransform) const
{
	matrices::matrixTo16Array(worldTransform, pImpl_->toWorldCoo_);
}

manip_core::TreeI* Robot::GetTreeI(int id) const
{
	return GetTree(id);
}

const manip_core::TreeI* Robot::GetTorsoI() const
{
	return GetTorso();
}


manip_core::RobotI* Robot::Copy() const
{
	return Clone();
}

manip_core::RobotI* Robot::Copy(const double* robotTransform) const
{
	Robot * r = Clone();
	matrices::Matrix4 mat;
	matrices::array16ToMatrix4(robotTransform, mat);
	r->SetPosOri(mat);
	return r;
}

void Robot::GetTreeAttach(int id, double* attach) const
{
	const matrices::Vector3& vAttach(pImpl_->attaches_[id]);
	matrices::vect3ToArray(attach, vAttach);
}

