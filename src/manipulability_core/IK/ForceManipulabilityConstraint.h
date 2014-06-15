
#ifndef _CLASS_FMCONSTRAINT
#define _CLASS_FMCONSTRAINT

#include "PartialDerivativeConstraint.h"

#include "MatrixDefs.h"

class Jacobian;

class ForceManipulabilityConstraint : public PartialDerivativeConstraint
{

public:
	 ForceManipulabilityConstraint();
	~ForceManipulabilityConstraint();

public:
	virtual NUMBER Evaluate(const Robot& /*robot*/, const Tree& /*tree*/, const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/);

private:
	NUMBER ForceManipulability(Jacobian& /*jacobian*/, const matrices::Vector3& /*direction*/);
};

#endif //_CLASS_FMCONSTRAINT