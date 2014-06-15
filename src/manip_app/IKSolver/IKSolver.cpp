#include "IKSolver.h"


#include "API/IKSolverI.h"
#include "API/IkConstraintHandlerI.h"
#include "API/TreeI.h"
#include "API/RobotI.h"

using namespace manip_core;


IKSolverApp::IKSolverApp(IkConstraintHandlerI* constraints)
	: solverI_(GetIKSolver())
	, constraints_(constraints)
{
	// NOTHING
	//constraints_->AddConstraint(enums::IKConstraints::ForceManip);
	//constraints_->AddConstraint(enums::IKConstraints::ObstacleManip);
	//constraints_->AddConstraint(enums::IKConstraints::Posture);
	//constraints_->AddConstraint(enums::IKConstraints::Joint);
}

IKSolverApp::~IKSolverApp()
{
	solverI_->Release();
	constraints_->Release();
}
bool IKSolverApp::StepClamping(const RobotI* robot, TreeI* pTree, const matrices::Vector3& target, const matrices::Vector3& dir) const
{
	double targ[3];
	double tdir[3];
	matrices::vect3ToArray(targ, target);
	matrices::vect3ToArray(tdir, dir);
	return solverI_->StepClamping(robot, pTree, targ, tdir, constraints_);
}

bool IKSolverApp::StepClampingToTargets(RobotI* robot, const matrices::Vector3& dir) const
{
	matrices::Matrix4 transform;
	double transf[16];
	robot->ToRobotCoordinates(transf);
	matrices::array16ToMatrix4(transf, transform);

	double tmp[3];
	matrices::Vector3 roboTarget;

	bool done = true;
	for(unsigned int i=0; i< robot->GetNumTrees(); ++i)
	{
		TreeI* tree = robot->GetTreeI(i);
		if(tree->IsAnchored())
		{
			tree->GetTarget(tmp);
			matrices::arrayToVect3(tmp, roboTarget);
			roboTarget = matrices::matrix4TimesVect3(transform, roboTarget);
		}
		else
		{
			tree->GetReferenceTarget(tmp);
			matrices::arrayToVect3(tmp, roboTarget);
		}
		if(tree->IsAnchored())
		{
			done = StepClamping(robot, tree, roboTarget, dir) && done;
		}
	}
	return done;
}

