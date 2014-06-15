#include "PostureCriteriaToeOnCOM.h"
#include "world/World.h"
#include "kinematic/Tree.h"
#include "kinematic/SupportPolygon.h"
#include "kinematic/Robot.h"
#include "MatrixDefs.h"

using namespace matrices;

PostureCriteriaToeOnCOM::PostureCriteriaToeOnCOM()
{
	// NOTHING
}

PostureCriteriaToeOnCOM::~PostureCriteriaToeOnCOM()
{
	// NOTHING
}

bool PostureCriteriaToeOnCOM::Evaluate(const World& world, const Robot& robot, const Tree& tree) const
{
	// first let's compute current's com
	Vector3 com = robot.ComputeCom();
	SupportPolygon support(robot);
	if(!support.Contains(com))// && support.WouldContain(com, robot, tree))
	{
		return true;
	}
	return false; // TODO
}
