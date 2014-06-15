
#include "sampling/Sample.h"
#include "kinematic/Tree.h"
#include "kinematic/Joint.h"

using namespace matrices;

Sample::Sample(Tree& tree)
{
	Joint * j = tree.GetRoot();
	while(j)
	{
		angles_.push_back(j->GetTheta());
		position_ = j->GetS();
		j = j->pChild_;
	}
	position_ -= tree.GetPosition();
	jacobianProd_ = tree.GetJacobian()->GetJacobianProduct();
	jacobianProdInverse_ = tree.GetJacobian()->GetJacobianProductInverse();
}

	
//Sample::Sample(Jacobian& jacobian, const Vector3& position)
//: position_(position)
//{
//	jacobianProd_ = jacobian.GetJacobianProduct();
//	jacobianProdInverse_ = jacobian.GetJacobianProductInverse();
//}

Sample::~Sample()
{
	// NOTHING
}

const Vector3& Sample::GetPosition() const
{
	return position_;
}

void Sample::LoadIntoTree(Tree& tree) const
{
	Joint * j = tree.GetRoot();
	assert(angles_.size() == tree.GetNumJoint());
	{
		for(Sample::LAngles::const_iterator it = angles_.begin(); it < angles_.end() && (j != 0); ++it)
		{
			j->SetTheta(*it);
			j = j->pChild_;
		}
		tree.Compute();
	}
}

NUMBER Sample::velocityManipulabiliy(const Vector3& direction) const
{
	NUMBER r = (direction.transpose()*jacobianProdInverse_*direction);
	return 1/sqrt(r);
}

NUMBER Sample::forceManipulabiliy   (const Vector3& direction) const
{
	NUMBER r = (direction.transpose()*jacobianProd_*direction);
	return 1/sqrt(r);
}
