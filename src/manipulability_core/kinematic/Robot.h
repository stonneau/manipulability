
#ifndef _CLASS_ROBOT
#define _CLASS_ROBOT

#include "API/RobotI.h"

#include "MatrixDefs.h"
#include "kinematic/ComVisitor_ABC.h"
#include "kinematic/Tree.h"
#include "kinematic/Enums.h"

#include <memory>
#include <vector>

struct RobotPImpl;

class RobotVisitor_ABC;
class World;

namespace manip_core
{
	struct TreeI;
}

class Robot : public manip_core::RobotI
{

public:
	typedef std::vector<Tree*>		T_Tree;
	typedef T_Tree::iterator		T_TreeIT;
	typedef T_Tree::const_iterator	T_TreeCIT;
	typedef std::vector<T_Tree>		T_Hierarchy;

public:
	explicit Robot(const matrices::Matrix4& /*transform*/, Tree* /*torsoAndHead*/, manip_core::enums::robot::eRobots robotType = manip_core::enums::robot::UnknownRobot ); //basis switch
	~Robot();

//configure
public:
	void AddTree (Tree* /*tree*/, const matrices::Vector3& /*attach*/, const unsigned int columnJoint = 0);

//helpers
public:
	void  Accept(RobotVisitor_ABC& /*visitor*/) const;
	const matrices::Matrix4& ToWorldCoordinates() const;
	const matrices::Matrix4& ToRobotCoordinates() const;
	Tree* GetTree(Tree::TREE_ID /*id*/) const;
	const Tree* GetTorso() const;
	T_Tree& GetTrees() const;
	const T_Hierarchy& GetHierarchy() const;
	matrices::Vector3 ComputeCom() const;
	matrices::Vector3 ComputeTargetCom() const;
	const manip_core::enums::robot::eRobots& RobotType() const;

// inherited
	virtual void Release();
	virtual void ToWorldCoordinates(double* /*worldTransform*/) const;
	virtual void ToRobotCoordinates(double* /*robotTransform*/) const;
	virtual const unsigned int GetNumTrees() const;
	virtual manip_core::TreeI* GetTreeI(int /*id*/) const;
	virtual const manip_core::TreeI* GetTorsoI() const;
	virtual manip_core::enums::robot::eRobots GetType() const;
	virtual RobotI* Copy() const;
	virtual RobotI* Copy(const double* /*robotTransform*/) const;
	virtual void SetTransform(const double* /*worldTransform*/);
	virtual void Translate(const double* /*directionVector*/);
	virtual void ComTarget (double* /*directionVector*/) const;
	virtual bool ComputeCom(double* /*directionVector*/) const;
	virtual void GetTreeAttach(int /*id*/, double* /*attach*/) const;
	virtual void LockOnCurrent(int treeId);
	virtual void Rest();

//action
public:
	//void Move(const matrices::Vector3& /*direction*/, const World& /*world*/); 
	void Reset(); 
	void Translate(const matrices::Vector3& /*direction*/); 
	void SetPosOri(const matrices::Matrix4& /*transform*/); 

	Robot* Clone() const;

private:
	std::auto_ptr<RobotPImpl> pImpl_;
	int numTrees_;
	const manip_core::enums::robot::eRobots robotType_;
};

#endif //_CLASS_ROBOT