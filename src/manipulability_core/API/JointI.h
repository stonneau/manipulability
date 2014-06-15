
#ifndef _CLASS_JOINTI
#define _CLASS_JOINTI

#include "Exports.h"

namespace manip_core
{
namespace enums
{
	namespace rotation
	{
		enum eRotation {X = 0, Y, Z, None};
	}
}

struct MANIPCORE_API JointI
{
	/** Deletes the current JointI.
	*/
	virtual void Release() = 0;

	/** Returns current joint angle value.
	*/
	virtual double GetAngle() const = 0;
	/** Joint translation vector from its parent (or Tree if joint is root).
	*/
	virtual void Offset(double * /*offset*/) const = 0;

	/** Get joint rotation axis. Can be either X Y or Z.
	*/
	virtual const manip_core::enums::rotation::eRotation GetRotation() const = 0;

	virtual const JointI* GetSon() const = 0;
	virtual const bool IsLocked() const = 0;
	virtual const JointI* GetParent() const = 0;

	virtual bool IsEffector() const = 0;
	virtual bool IsJoint() const = 0;

};
}
#endif //_CLASS_JOINTI