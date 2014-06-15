
#ifndef _CLASS_WORLDMANAGERI
#define _CLASS_WORLDMANAGERI

#include "Exports.h"
//#include "kinematics/joint.h"
#include <vector>

namespace manip_core
{
struct RobotI;
struct PostureManagerI;
struct IkConstraintHandlerI;

namespace enums
{
namespace robot
{
	enum eRobots
	{ 
		Human = 0,
		HumanWalk,
		HumanEllipse,
		Quadruped,
		QuadrupedDown,
		Spider,
		SpiderRace,
		HumanCrouch,
		HumanEscalade,
		HumanCanap,
		SpiderSix,
		HumanCrouch180,
		UnknownRobot
	};
} // namespace enums
} // namespace robot

//typedef kinematics::joint<float, float, 3, 5, false> joint_def_t;

struct MANIPCORE_API WorldManagerI
{
	typedef std::vector<double> T_JointValues;
	typedef std::vector<T_JointValues> T_TreeValues;

	/** Deletes the current WorldManagerI.
	*/
	virtual void Release() = 0;
	/**	Creates a planar obstacle. Points must be indicated clockwise from upLeft and be in a plan.
	 */
	virtual void AddObstacle(double* /*upLeft*/, double* /*upRight*/, double* /*downRight*/, double* /*downLeft*/, bool donttouch=false)= 0;
	/**	Creates a pre-existing robot.
	 */
	virtual RobotI* CreateRobot(enums::robot::eRobots /*robotType*/, double* /*transform*/) = 0;
	virtual RobotI* CreateRobot(enums::robot::eRobots /*robotType*/, double* /*transform*/, const T_TreeValues /*values*/ ) = 0;
    //virtual RobotI* CreateRobot(const joint_def_t& /*jointDef*/, double* /*transform*/) = 0;
    //virtual RobotI* CreateRobot(const joint_def_t& /*jointDef*/, double* /*transform*/, const T_TreeValues /*values*/ ) = 0;

	/** Once all obstacles have been created, instantiate world.
	*/
	virtual void Initialize(bool /*activateCollisions*/) = 0;
	/**
	*/
	virtual PostureManagerI* GetPostureManager() = 0;
	virtual IkConstraintHandlerI* GetIkConstraintHandlerI() = 0;
};

extern "C" MANIPCORE_API WorldManagerI* GetWorldManager();

} // namespace manip_core
#endif //_CLASS_WORLDMANAGERI
