
#include "IKSolver.h"
#include "kinematic/Tree.h"
#include "kinematic/Robot.h"
#include "sampling/Sample.h"
#include "kinematic/Jacobian.h"
#include "PartialDerivativeConstraint.h"
#include "IkConstraintHandler.h"

#include "API/TreeI.h"

#include <vector>
#include <iostream>

using namespace matrices;
using namespace Eigen;
using namespace std;
using namespace manip_core;

struct IKPImpl
{
	IKPImpl()
	{
		//TODO
	}

	~IKPImpl()
	{
		// NOTHING
	}
};

IKSolver::IKSolver(const float espilon, const float treshold)
: epsilon_(espilon)
, treshold_(treshold)
, pImpl_(new IKPImpl)
{
	// NOTHING
}

IKSolver::~IKSolver()
{
	// NOTHING
}

void IKSolver::Release()
{
	delete this;
}

#include <iostream>

//REF: Boulic : An inverse kinematics architecture enforcing an arbitrary number of strict priority levels
bool IKSolver::StepClamping(const Robot& robot, Tree& tree, const matrices::Vector3& target, const Vector3& direction, const IkConstraintHandler* constraints) const
{
	assert(constraints);
	/*if(!intersection_.Intersect(tree, target))
	{
		tree.UnLockTarget();
		return false;
	}*/
	Jacobian jacobian(tree); 
	VectorX postureVariation(VectorX::Zero(jacobian.GetJacobian().cols()));
	PartialDerivatives(robot, tree, direction, postureVariation, constraints);

	Vector3 force = target - tree.GetEffectorPosition(tree.GetNumEffector()-1) ; //TODO we only have one effector  so weird huh ?
	
	if(force.norm() < treshold_) // reached treshold
	{
		tree.targetReached_ = true;
		return true;
	}
	tree.targetReached_ = false;
	VectorX velocities;
	MatrixX J = jacobian.GetJacobianCopy(); int colsJ = J.cols(); int rowsJ = J.rows();
	
	//Vector3 dX = (force / force.norm()); // * treshold_ * 2; // TODO real trajectory please ?
	Vector3 dX = force; // * treshold_ * 2; // TODO real trajectory please ?
	
	//null space projection
	MatrixX p0 = MatrixX::Identity(colsJ, colsJ);
	MatrixX nullSpace = p0;

	// init all joints to free.
	bool freeJoint[20];// no more than 20 joint ok :)
	for(int i = 0; i < colsJ; ++i)
	{
		freeJoint[i]=true;
	}
		
	bool clamp = false;
	//entering clamping loop
	do
	{
		jacobian.GetNullspace(p0, nullSpace); // Pn(j) = P0(j) - Jtr * J

		if(tree.targetReached_)
		{
			velocities = jacobian.GetJacobianInverse() * dX;
		}
		else
		{
			velocities = jacobian.GetJacobianInverse() * dX + nullSpace * postureVariation;

		}
		// now to the "fun" part
		clamp = false;
		for(int i =0; i < colsJ; ++ i)
		{
			if(freeJoint[i])
			{
				NUMBER overload = tree.GetJoint(i+1)->AddToTheta(velocities(i));
				if(overload != 0.f) // clamping happened
				{
					freeJoint[i] = false;
					clamp = true;
					dX -= J.col(i) * overload;
					J.col(i) = VectorX::Zero(rowsJ);
					p0(i,i) = 0;
				}
			}
		}
		if(clamp)
			jacobian.SetJacobian(J);
	} while(clamp);
	return false;
}

bool IKSolver::StepClamping(const RobotI* pRobot, TreeI* pTree, const double* target, const double* direction, const manip_core::IkConstraintHandlerI* constraints) const
{
	Tree* tree = (static_cast<Tree*>(pTree));
	const Robot* robot = (static_cast<const Robot*>(pRobot));
	matrices::Vector3 targ, tdir;
	matrices::arrayToVect3(target, targ);
	matrices::arrayToVect3(direction, tdir);
	const IkConstraintHandler* cons = (static_cast<const IkConstraintHandler*>(constraints));
	return StepClamping(*robot, *tree, targ, tdir, cons);
	//return StepClamping(*tree, targ, unitX);
}


void IKSolver::PartialDerivative(const Robot& robot, Tree& tree, const Vector3& direction, VectorX& velocities, const IkConstraintHandler* constraints, const int joint) const
{
	Sample save(tree); // saving previous tree
	tree.GetJoint(joint)->AddToTheta(-epsilon_);
	tree.Compute();
	Jacobian jacobMinus(tree);
	save.LoadIntoTree(tree); // loading it

	tree.GetJoint(joint)->AddToTheta(epsilon_);
	tree.Compute();
	Jacobian jacobPlus(tree);
	save.LoadIntoTree(tree); // loading it
	
	int i =0;
	const IkConstraintHandler::T_Constraint& cons = constraints->GetConstraints();
	for(IkConstraintHandler::T_ConstraintCIT it = cons.begin(); it!= cons.end(); ++it)
	{
		++ i;
		velocities(joint-1) += ((*it).second)->Evaluate(robot, tree, joint, jacobMinus, jacobPlus, epsilon_, direction);
	}
	if (i!= 0)
	{
		velocities(joint-1) = velocities(joint-1) / i;
	}
	else
	{
		velocities(joint-1) = 0;
	}
}

void IKSolver::PartialDerivatives(const Robot& robot, Tree& tree, const Vector3& direction, VectorX& velocities, const IkConstraintHandler* constraints) const
{
	for(int i =1; i<= velocities.rows() ;++i)
	{
		PartialDerivative(robot, tree, direction, velocities, constraints, i);
	}
}
