
#ifndef _CLASS_TREEI
#define _CLASS_TREEI

#include "Exports.h"

namespace manip_core
{

struct JointI;

struct MANIPCORE_API TreeI
{
	/** Deletes the current TreeI.
	*/
	virtual void Release() = 0;
	/**	Returns a 3 dimensional vector indicating the tree end effector position in robot coordinates
	 */
	virtual void EndEffectorPosition(double* /*position*/) const = 0;
	/**	Returns a 3 dimensional vector indicating the tree position in robot coordinates
	 */
	virtual void Position(double* /*position*/) const = 0;
	/**	Returns root JointI of the TreeI.
	 */
	virtual const int GetNumJoint() const = 0;
	/**	Returns root JointI of the TreeI.
	 */
	virtual const JointI* GetRootJointI() const = 0;

	virtual bool IsAnchored() const = 0;

	virtual void ToRest() = 0 ;
	virtual void SetTarget(double* /*target*/) = 0 ;
	virtual void GetTarget(double* /*target*/) const = 0 ;
	virtual void GetReferenceTarget(double* /*target*/) const = 0 ;
	virtual bool GetObstacleNormal(double* /*target*/) const = 0;
	virtual double GetManipulability(const double& x, const double& y, const double& z) const = 0; // this is expensive
	virtual void GetEllipsoidAxes(double* /*u1*/, double* /*u2*/, double* /*u3*/) const = 0 ;
	virtual void GetEllipsoidAxes(double* /*u1*/, double* /*u2*/, double* /*u3*/, double& /*sig1*/, double& /*sig2*/, double& /*sig3*/) const = 0 ;
};
}
#endif //_CLASS_TREEI