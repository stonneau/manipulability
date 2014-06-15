
#ifndef _CLASS_ROBOTI
#define _CLASS_ROBOTI

#include "Exports.h"
#include "WorldManagerI.h"

namespace manip_core
{

struct TreeI;

struct MANIPCORE_API RobotI
{
	/** Deletes the current RobotI.
	*/
	virtual void Release() = 0;
	/**	Returns a column-major 4 dimensional square matrix corresponding to the robot transformation
	 */
	virtual void ToWorldCoordinates(double* /*worldTransform*/) const = 0;
	/**	Returns a column-major 4 dimensional square matrix corresponding a transformation into the
	 *  robot coordinates.
	 */
	virtual void ToRobotCoordinates(double* /*robotTransform*/) const = 0;
	/**	Returns the number of trees (legs) of the Robot.
	 */
	virtual const unsigned int GetNumTrees() const = 0;
	/**	Returns the Robot tree present at index id.
	 */
	virtual TreeI* GetTreeI(int /*id*/) const = 0;
	virtual void GetTreeAttach(int /*id*/, double* /*attach*/) const = 0;

	/**	Returns Tree corresponding to the robot torso.
	 */
	virtual const TreeI* GetTorsoI() const = 0;
	virtual void Rest() = 0;

	virtual void SetTransform(const double* /*worldTransform*/) = 0;
	virtual void Translate(const double* /*directionVector*/) = 0;
	virtual void ComTarget(double* /*directionVector*/) const = 0;
	virtual bool ComputeCom(double* /*directionVector*/) const = 0;

	virtual void LockOnCurrent(int treeId) = 0 ;

	virtual enums::robot::eRobots GetType() const = 0;
	virtual RobotI* Copy() const = 0;
	virtual RobotI* Copy(const double* /*robotTransform*/) const = 0;
};
}
#endif //_CLASS_ROBOTI