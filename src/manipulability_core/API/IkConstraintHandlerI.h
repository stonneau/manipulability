
#ifndef _CLASS_IKCONSTRAINTHANDLERI
#define _CLASS_IKCONSTRAINTHANDLERI

#include "IKSolverI.h"

#include "Exports.h"

namespace manip_core
{
struct MANIPCORE_API IkConstraintHandlerI {
		
public:
	virtual bool AddConstraint   (const manip_core::enums::IKConstraints::eIKConstraints /*constraint*/) = 0;
	virtual bool RemoveConstraint(const manip_core::enums::IKConstraints::eIKConstraints /*constraint*/) = 0;
	virtual void Release() = 0;
};

} // namespace manip_core
#endif //_CLASS_IKCONSTRAINTHANDLER
