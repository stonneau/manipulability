
#ifndef _CLASS_IKSOLVERI
#define _CLASS_IKSOLVERI

#include "Exports.h"

namespace manip_core
{
struct IkConstraintHandlerI;
namespace enums
{
namespace IKConstraints
{
	enum eIKConstraints
	{ 
		ForceManip = 0,
		ObstacleManip,
		Posture,
		Joint
	};
} // namespace enums
} // namespace robot
struct TreeI;
struct RobotI;

struct MANIPCORE_API IKSolverI {

public:
	virtual bool StepClamping(const RobotI* /*robot*/, TreeI* /*pTree*/, const double* /*target*/, const double* /*direction*/, const IkConstraintHandlerI* /*constraints*/) const = 0; //true if target reached // target in robot coordinates
	virtual void Release() = 0;
};

extern "C" MANIPCORE_API IKSolverI* GetIKSolver();
} // namespace manip_core
#endif //_CLASS_IKSOLVERI