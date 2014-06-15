
#ifndef _CLASS_IKSOLVERIMP
#define _CLASS_IKSOLVERIMP

#include "Exports.h"
#include "MatrixDefs.h"

namespace manip_core
{
	struct TreeI;
	struct RobotI;
	struct IKSolverI;
	struct IkConstraintHandlerI;
} // namespace manip_core

class IKSolverApp {

public:
     IKSolverApp(manip_core::IkConstraintHandlerI* /*constraints*/);
    ~IKSolverApp();

public:
	bool StepClamping(const manip_core::RobotI* /*robot*/, manip_core::TreeI* /*pTree*/, const matrices::Vector3& target, const matrices::Vector3& /*dir*/) const; //true if target reached // target in robot coordinates
	bool StepClampingToTargets(manip_core::RobotI* /*robot*/, const matrices::Vector3& /*dir*/) const; //true if target reached // target in robot coordinates

private:
	manip_core::IKSolverI* solverI_;
public:
	manip_core::IkConstraintHandlerI* constraints_;
};

#endif //_CLASS_IKSOLVERI
