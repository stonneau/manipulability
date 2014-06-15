#include "WorldManagerI.h"
#include "posture/PostureManagerImpl.h"

#include "world/World.h"
#include "world/Obstacle.h"
#include "kinematic/RobotFactory.h"

#include "API/IkConstraintHandlerI.h"
#include "IK/IkConstraintHandler.h"

#include "MatrixDefs.h"

using namespace manip_core;
using namespace matrices;

class WorldManagerImpl : public WorldManagerI
{
public :

	WorldManagerImpl()
	{
		// NOTHING
	}
	~WorldManagerImpl()
	{
		// NOTHING
	}

public:

	virtual void Release()
	{
		delete this;
	}

	virtual void AddObstacle(double* upLeft, double* upRight, double* downRight, double* downLeft, bool donttouch)
	{
		Vector3 p1, p2, p3, p4;
		arrayToVect3(upLeft, p1);arrayToVect3(upRight, p2);arrayToVect3(downRight, p3);arrayToVect3(downLeft, p4);
		world_.AddObstacle(new Obstacle(p1, p2, p3, p4, donttouch));
	}

	virtual RobotI* CreateRobot(enums::robot::eRobots robotType, double* transform)
	{
		Matrix4 robotCoord;
		array16ToMatrix4(transform, robotCoord);
		return factory_.CreateRobot(robotType, robotCoord);
	}


	virtual RobotI* CreateRobot(enums::robot::eRobots robotType, double* transform, const WorldManagerI::T_TreeValues values)
	{
		Matrix4 robotCoord;
		array16ToMatrix4(transform, robotCoord);
		Robot* res = factory_.CreateRobot(robotType, robotCoord);
		int i=0;
		for(WorldManagerI::T_TreeValues::const_iterator it = values.begin(); it != values.end(); ++it, ++i)
		{
			Joint* j = res->GetTree(i)->GetRoot();;
			for(WorldManagerI::T_JointValues::const_iterator it2 = (*it).begin(); j && it2 != (*it).end(); ++it2)
			{
				j->SetTheta(*it2);
				j = j->pChild_;
			}
			res->GetTree(i)->Compute();
		}
		return res;
	}

    /*virtual RobotI* CreateRobot(const joint_def_t& joint, double* transform)
	{
		Matrix4 robotCoord;
		array16ToMatrix4(transform, robotCoord);
		return factory_.CreateRobot(joint, robotCoord);
	}


	virtual RobotI* CreateRobot(const joint_def_t& joint, double* transform, const WorldManagerI::T_TreeValues values)
	{
		Matrix4 robotCoord;
		array16ToMatrix4(transform, robotCoord);
		Robot* res = factory_.CreateRobot(joint, robotCoord);
		int i=0;
		for(WorldManagerI::T_TreeValues::const_iterator it = values.begin(); it != values.end(); ++it, ++i)
		{
			Joint* j = res->GetTree(i)->GetRoot();;
			for(WorldManagerI::T_JointValues::const_iterator it2 = (*it).begin(); j && it2 != (*it).end(); ++it2)
			{
				j->SetTheta(*it2);
				j = j->pChild_;
			}
			res->GetTree(i)->Compute();
		}
		return res;
    }*/
	
	/** Once all obstacles have been created, instantiate world.
	*/
	virtual void Initialize(bool activateCollisions)
	{
		world_.Instantiate(activateCollisions);		
	}

	virtual PostureManagerI* GetPostureManager()
	{
		return new PostureManagerImpl(world_);
	}

	virtual IkConstraintHandlerI* GetIkConstraintHandlerI()
	{
		IkConstraintHandlerI* constraintHandler = new IkConstraintHandler(world_);
		return constraintHandler;
	}

private:
	World world_;
	factories::RobotFactory factory_;
};

extern "C" MANIPCORE_API WorldManagerI* GetWorldManager()
{
	return new WorldManagerImpl;
}

