
#include "ObstacleConstraint.h"
#include "kinematic/Tree.h"
#include "kinematic/Robot.h"
#include "world/World.h"
#include "world/Obstacle.h"
#include "world/ObstacleVisitor_ABC.h"

#include "kinematic/Jacobian.h"

using namespace matrices;
using namespace Eigen;

struct FindObstacle : public ObstacleVisitor_ABC
{
	FindObstacle(const matrices::Vector3& target)
		: target_(target)
		, bestDistance_(100000)
	{
		// NOTHING		
	};

	~FindObstacle()
	{
		// NOTHING
	}

	virtual void Visit(const Obstacle& obstacle)
	{
		NUMBER currentDistance_ = obstacle.Distance(target_, whoCares_);
		if(currentDistance_ < bestDistance_)
		{
			bestDistance_ = currentDistance_;
		}
	}

	matrices::Vector3 whoCares_;
	const matrices::Vector3& target_;
	NUMBER bestDistance_;

};

ObstacleConstraint::ObstacleConstraint(const World& world)
	: world_(world)
{
	// NOTHING
}

ObstacleConstraint::~ObstacleConstraint()
{
	// NOTHING
}

NUMBER ObstacleConstraint::Evaluate(const Robot& robot, const Tree& tree, const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3& direction)
{
	Vector3 minusVector, maxVector;
	Tree * myTree = tree.Clone();
	if(myTree->GetJoint(joint)->IsEffector())
	{
		return 0;
	}
 	myTree->GetJoint(joint)->AddToTheta(-epsilon);
	myTree->Compute();
	minusVector = myTree->GetJoint(joint)->ComputeS();
	//minusVector = myTree->GetEffectorPosition(myTree->GetNumEffector()-1);
	myTree->GetJoint(joint)->AddToTheta(2*epsilon);
	myTree->Compute();
	//maxVector = myTree->GetEffectorPosition(myTree->GetNumEffector()-1);
	maxVector = myTree->GetJoint(joint)->ComputeS();

	minusVector = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), minusVector);
	maxVector = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), maxVector);

	FindObstacle fObsMin(minusVector);
	world_.Accept(fObsMin);
	
	FindObstacle fObsMax(maxVector);
	world_.Accept(fObsMax);

	double res = (fObsMax.bestDistance_ - fObsMin.bestDistance_) / (epsilon * 2) * 0.2;
	res = res > 1 ? 1 : res;
	res = res < -1 ? -1 : res;
	return res;

	//return NUMBER ((ForceManipulability(jacobianPlus, direction) - ForceManipulability(jacobianMinus, direction)) / (epsilon * 2) * 0.3) ;
}


