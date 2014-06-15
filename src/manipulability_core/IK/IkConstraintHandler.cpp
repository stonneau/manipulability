
#include "IkConstraintHandler.h"
#include "PartialDerivativeConstraint.h"
#include "PostureConstraint.h"
#include "ForceManipulabilityConstraint.h"
#include "ObstacleConstraint.h"
#include "JointConstraint.h"

#include <map>

using namespace matrices;
using namespace Eigen;
using namespace manip_core::enums;

struct ConstraintHandlerPimpl
{
	ConstraintHandlerPimpl(const World& world)
		:world_(world)
	{
		//TODO
	}

	~ConstraintHandlerPimpl()
	{
		for(IkConstraintHandler::T_ConstraintIT it = constraints_.begin(); it!= constraints_.end(); ++it)
		{
			delete it->second;
		}
	}
	const World& world_;
	IkConstraintHandler::T_Constraint constraints_;
};

IkConstraintHandler::IkConstraintHandler(const World& world)
	:pImpl_(new ConstraintHandlerPimpl(world))
{
	// NOTHING
}

IkConstraintHandler::~IkConstraintHandler()
{
	// NOTHING
}

void IkConstraintHandler::Release()
{
	delete this;
}

bool IkConstraintHandler::AddConstraint   (const manip_core::enums::IKConstraints::eIKConstraints constraint)
{
	T_ConstraintIT it = pImpl_->constraints_.find(constraint);
	if(it != pImpl_->constraints_.end())
	{
		return false;
	}
	else
	{
		PartialDerivativeConstraint* cons;
		switch(constraint)
		{
			case IKConstraints::ForceManip:
			{
				cons= new ForceManipulabilityConstraint;
				break;
			}
			case IKConstraints::ObstacleManip:
			{
				cons= new ObstacleConstraint(pImpl_->world_);
				break;
			}
			case IKConstraints::Joint:
			{
				cons= new JointConstraint();
				break;
			}
			case IKConstraints::Posture:
			{
				cons= new PostureConstraint();
				break;
			}
			default:
			{
				return false;
			}
			break;
		}
		pImpl_->constraints_.insert(std::make_pair(constraint, cons));
		return true;
	}
}

bool IkConstraintHandler::RemoveConstraint(const manip_core::enums::IKConstraints::eIKConstraints constraint)
{
	T_ConstraintIT it = pImpl_->constraints_.find(constraint);
	if(it == pImpl_->constraints_.end())
	{
		return false;
	}
	else
	{
		pImpl_->constraints_.erase(it);
		return true;
	}
}

const IkConstraintHandler::T_Constraint& IkConstraintHandler::GetConstraints() const
{
	return pImpl_->constraints_;
}

