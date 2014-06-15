#include "PostureCriteriaToeOffJointLimit.h"
#include "world/World.h"
#include "kinematic/Tree.h"
#include "kinematic/Robot.h"

PostureCriteriaToeOffJointLimit::PostureCriteriaToeOffJointLimit()
{
	// NOTHING
}

PostureCriteriaToeOffJointLimit::~PostureCriteriaToeOffJointLimit()
{
	// NOTHING
}

bool PostureCriteriaToeOffJointLimit::Evaluate(const World& world, const Robot& robot, const Tree& tree) const
{
	return tree.targetReached_ && tree.JointLimitBroken();
}
