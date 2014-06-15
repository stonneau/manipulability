
#include "ForceManipulabilityConstraint.h"

#include "kinematic/Robot.h"
#include "kinematic/Tree.h"

#include "kinematic/Jacobian.h"

using namespace matrices;
using namespace Eigen;

ForceManipulabilityConstraint::ForceManipulabilityConstraint()
{
	// NOTHING
}

ForceManipulabilityConstraint::~ForceManipulabilityConstraint()
{
	// NOTHING
}

NUMBER ForceManipulabilityConstraint::Evaluate(const Robot& robot, const Tree& tree, const int joint, Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3& direction)
{
 	NUMBER res = NUMBER ((ForceManipulability(jacobianPlus, direction) - ForceManipulability(jacobianMinus, direction)) / (epsilon * 2) * 0.2) ;
	res = res > 1 ? 1 : res;
	res = res < -1 ? -1 : res;
	return -res;
}
NUMBER ForceManipulabilityConstraint::ForceManipulability(Jacobian& jacobian, const matrices::Vector3& direction)
{ 
	NUMBER r = ((direction).transpose()*jacobian.GetJacobianProduct()*(direction));
	return r;
	//return 1/sqrt(r); 
}


