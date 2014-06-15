
#ifndef _CLASS_TREE
#define _CLASS_TREE

#include "Exports.h"
#include "kinematic/Joint.h"
#include "MatrixDefs.h"
#include "Enums.h"
#include "API/TreeI.h"
#include "world/Obstacle.h"

class Jacobian;
class ComVisitor_ABC;
class Sample;

namespace manip_core
{
	struct JointI;
}

class Tree : public manip_core::TreeI {

public:
	typedef int TREE_ID;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	explicit Tree(TREE_ID /*id*/, manip_core::enums::eMembers treeType = manip_core::enums::UnknownTree);
	explicit Tree(TREE_ID /*id*/, TREE_ID /*templateId*/, manip_core::enums::eMembers treeType = manip_core::enums::UnknownTree);
	~Tree();


private:
    Tree& operator =(const Tree&);
	Tree(const Tree&);

// inheritedLockOnCurrent
public:
	virtual void Release();
	virtual void EndEffectorPosition(double* /*position*/) const;
	virtual void Position(double* /*position*/) const;
	virtual void SetTarget(double* /*target*/);
	virtual void GetTarget(double* /*target*/) const ;
	virtual void GetReferenceTarget(double* /*target*/) const;
	virtual bool GetObstacleNormal(double* /*target*/) const;
	virtual const manip_core::JointI* GetRootJointI() const;
	virtual bool IsAnchored() const;
	virtual const int GetNumJoint() const { return nJoint; }
	virtual double GetManipulability(const double& x, const double& y, const double& z) const; // this is expensive
	virtual void GetEllipsoidAxes(double* /*u1*/, double* /*u2*/, double* /*u3*/) const; // this is expensive
	virtual void GetEllipsoidAxes(double* /*u1*/, double* /*u2*/, double* /*u3*/, double& /*sig1*/, double& /*sig2*/, double& /*sig3*/) const;

public:	
	const manip_core::enums::eMembers& GetTreeType() const;

	const int GetNumEffector() const { return nEffector; }
	void InsertRoot(Joint*);
	void InsertChild(Joint* parent, Joint* child);

	//compute jacobian
	void ComputeJacobian();

	void AcceptComVisitor(ComVisitor_ABC* /*visitor*/) const;

	// Accessors based on node numbers
	Joint* GetJoint(int) const;
	Joint* GetEffector(int) const;
	const matrices::Vector3& GetPosition() const;
	void SetBoundaryRadius(NUMBER radius) { sphereRadius_ = radius; };
	const NUMBER GetBoundaryRadius() const { return sphereRadius_; };
	const matrices::Vector3& GetEffectorPosition(int) const;

	// Accessors for tree traversal
	Joint* GetRoot() const { return root; }
	Joint* GetSuccessor (const Joint*) const;
	Joint* GetParent(const Joint* node) const { return node->pRealparent_; }

	const bool JointLimitBroken() const;

	Jacobian* GetJacobian() {return jacobian_; };

	// in contact
	// world coordinates
	void LockTarget(const matrices::Vector3& target){ target_ = target; lock_ = true; };
	void LockTarget(const matrices::Vector3& target, const Obstacle* obsTarget);//{ target_ = target; lock_ = true; obsTarget_ = obsTarget; onObstacle_ = true; };
	void UnLockTarget(){ lock_ = false; targetReached_ = false; obsTarget_ = 0; onObstacle_ = false; targetSample_ = 0; };
	bool IsLocked() const{ return lock_; };

	const matrices::Vector3& GetTarget() const {return target_;};
	const Obstacle* GetObstacleTarget() const {return obsTarget_;};

	void Compute();
	void Init();
	virtual void ToRest();
	
	const TREE_ID& GetId()const { return id_; }
	const TREE_ID& GetTemplateId()const { return templateId_; } // share sampling

	Tree* Clone() const;

public:
	matrices::Vector3 direction_;
	matrices::Vector3 directionForce_;
	matrices::Vector3 directionVel_;
	matrices::Vector3 target_;
	matrices::Vector3 referenceTarget_;
	matrices::Vector3 attach_;
	bool targetReached_;
	bool onObstacle_;
	Sample* targetSample_;

private:

	Jacobian* jacobian_;
	const Obstacle* obsTarget_;

	Joint* root;
	int nJoint;			// nJoint = nEffector + nJoint
	int nEffector;
	void SetSeqNum(Joint*);
	Joint* SearchJoint(Joint*, int) const;
	Joint* SearchEffector(Joint*, int) const;
	void ComputeTree(Joint*);
	void InitTree(Joint*);
	bool lock_;
	const TREE_ID id_;
	const TREE_ID templateId_;

	NUMBER sphereRadius_;

	const manip_core::enums::eMembers treeType_;
};

inline Joint* Tree::GetSuccessor (const Joint* node) const
{
	while (true) {
		if (node->pChild_) {
			return node->pChild_;
		}
		node = node->pRealparent_;
		if (!node) {
			return 0;		// Back to root, finished traversal
		} 
	}
}

#endif //_CLASS_TREE
