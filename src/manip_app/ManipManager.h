
#ifndef _CLASS_MANIPMANAGER
#define _CLASS_MANIPMANAGER


#include "MatrixDefs.h"
#include <vector>

#include "API/WorldManagerI.h"
#include "IKSolver/IKSolver.h"

namespace manip_core
{
struct RobotI;

class PostureManager;

class ObstacleGenerator;

struct ObstacleVisitor_ABC
{
public:
	/** Called whenever a new posture has been created.
	*/
	virtual void OnObstacleCreated(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, const float transparency, const int texture) = 0; // TODO posture destroyed ?
	virtual void OnWallCreated(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, const float transparency, const int texture) = 0; // TODO posture destroyed ?
	virtual void OnGroundCreated(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, const float transparenc, const int texture) = 0;	
};

class ManipManager
{
public:
	 ManipManager();
	~ManipManager();

public:
	/**	Creates a planar obstacle. Points must be indicated clockwise from upLeft and be in a plan.
	 */
	void AddObstacle(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/);
	
	void SetNextColor(const float r, const float g, const float b);
	void SetNextTransparency(const float t){transparency_ = t;}
	void SetNextTexture(const int t);
	void GenerateUnevenChess   (const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/, NUMBER /*height*/, const unsigned int /*depth*/);
	void GenerateChess		   (const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/, NUMBER /*height*/, const unsigned int /*depth*/);
	void GenerateVerticalChess (const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/, const unsigned int /*depth*/);
	void GenerateStair		   (const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/, NUMBER /*heightInit*/, NUMBER /*heightFinal*/, const unsigned int /*depth*/);
	void GenerateStairChess	   (const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/, NUMBER /*heightInit*/, NUMBER /*heightFinal*/, const unsigned int /*depth*/, const unsigned int /*depthChess*/);
	void GenerateYInclinedPlank(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/);
	void GenerateXInclinedPlank(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*bottomRight*/);

	void AddWall(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/);
	void AddGround(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/);
	
	/**
	Register a PostureCreatedListenerI that will be called whenever a new posture is created
	*/
	void RegisterObstacleCreatedListenerI(ObstacleVisitor_ABC* /*listener*/);	
	/**
	Unregister a previously created lister
	*/
	void UnRegisterObstacleCreatedListenerI(ObstacleVisitor_ABC* /*listener*/);

	/**	Creates a pre-existing robot.
	 */
	RobotI* CreateRobot(enums::robot::eRobots /*robotType*/, const matrices::Matrix4& /*transform*/);
	RobotI* CreateRobot(enums::robot::eRobots /*robotType*/, const matrices::Matrix4& /*transform*/, const manip_core::WorldManagerI::T_TreeValues /*values*/);
//RobotI* CreateRobot(const manip_core::joint_def_t* /*root*/, const matrices::Matrix4& /*transform*/);
//	RobotI* CreateRobot(const manip_core::joint_def_t* /*root*/, const matrices::Matrix4& /*transform*/, const manip_core::WorldManagerI::T_TreeValues /*values*/);
	/** Once all obstacles have been created, instantiate world.
	*/
	void Initialize(bool collision = true);
	/**
		Returns a PostureManagerI instance.	
	*/
	PostureManager* GetPostureManager();
    const IKSolverApp& GetIkSolver() const;

private:
	WorldManagerI* pWorldManager_;
	PostureManager* pPostureManager_;
    IKSolverApp ikSolver_;
	std::vector<ObstacleVisitor_ABC*> listeners_;
	float color_[3];
	float transparency_;
	int texture_;
};

} // namespace manip_core
#endif //_CLASS_MANIPMANAGER
