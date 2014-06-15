
#include "TreeFactory.h"
#include "RobotFactory.h"
#include "Robot.h"
#include "kinematic/Tree.h"
#include "kinematic/Enums.h"

#include <vector>
#include <list>

using namespace matrices;
using namespace manip_core::enums;
using namespace manip_core::enums::robot;


namespace factories
{
const Vector3 unitx(1, 0, 0);
const Vector3 unity(0, 1, 0);
const Vector3 unitz(0, 0, 1);
const Vector3 unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
const Vector3 zero(0,0,0);

struct RobotFactoryPimpl{
	RobotFactoryPimpl()
	{
		// NOTHING
	}
	
	~RobotFactoryPimpl()
	{
		// NOTHING
	}

	// TODO Joint that do not move

	Robot* CreateHuman(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorso, Vector3(0.0, 0., 0), 4), manip_core::enums::robot::Human);
		Tree* rightLeg = treeFact_.CreateTree( RightLeg, Vector3(0.0, -0.2, 0.1), 0);
		Tree* leftLeg  = treeFact_.CreateTree( LeftLeg, Vector3(0.0, 0.2 , 0.1), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArm, Vector3(0.0, -0.4, 1.3), 2);
		Tree* leftArm  = treeFact_.CreateTree( LeftArm, Vector3(0.0, 0.4, 1.3), 3);
		//rightArm->SetBoundaryRadius(rightArm->GetBoundaryRadius() * 8 / 10 );
		//leftArm->SetBoundaryRadius(leftArm->GetBoundaryRadius() * 8 / 10 );
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(rightArm, matrices::Vector3(0,0,1.2), 1);
		res->AddTree(leftArm, matrices::Vector3(0,0,1.2), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

	//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}

	Robot* CreateHumanEscalade(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorso, Vector3(0.0, 0., 0), 4), manip_core::enums::robot::HumanEscalade);
		Tree* rightLeg = treeFact_.CreateTree( RightLegEscalade, Vector3(0.0, -0.2, 0.1), 0);
		Tree* leftLeg  = treeFact_.CreateTree( LeftLegEscalade, Vector3(0.0, 0.2 , 0.1), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArmEscalade, Vector3(0.0, -0.4, 1.3), 2);
		Tree* leftArm  = treeFact_.CreateTree( LeftArmEscalade, Vector3(0.0, 0.4, 1.3), 3);
		//rightArm->SetBoundaryRadius(rightArm->GetBoundaryRadius() * 8 / 10 );
		//leftArm->SetBoundaryRadius(leftArm->GetBoundaryRadius() * 8 / 10 );
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(rightArm, matrices::Vector3(0,0,1.2), 1);
		res->AddTree(leftArm, matrices::Vector3(0,0,1.2), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

	//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}

	Robot* CreateHumanCanap(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorso, Vector3(0.0, 0., 0), 4), manip_core::enums::robot::HumanCanap);
		Tree* rightLeg = treeFact_.CreateTree( RightLegCanap, Vector3(0.0, -0.2, 0.1), 0);
		Tree* leftLeg  = treeFact_.CreateTree( LeftLegCanap, Vector3(0.0, 0.2 , 0.1), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArmCanap, Vector3(0.0, -0.4, 1.3), 2);
		Tree* leftArm  = treeFact_.CreateTree( LeftArmCanap, Vector3(0.0, 0.4, 1.3), 3);
		//rightArm->SetBoundaryRadius(rightArm->GetBoundaryRadius() * 8 / 10 );
		//leftArm->SetBoundaryRadius(leftArm->GetBoundaryRadius() * 8 / 10 );
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(rightArm, matrices::Vector3(0,0,1.2), 1);
		res->AddTree(leftArm, matrices::Vector3(0,0,1.2), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

	//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}

	Robot* CreateHumanCrouch(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorsoCrouch, Vector3(0.0, 0., 0), 4), manip_core::enums::robot::Human);
		Tree* rightLeg = treeFact_.CreateTree( RightLegCrouch, Vector3(0.1, -0.2, 0.), 0);
		Tree* leftLeg  = treeFact_.CreateTree( LeftLegCrouch, Vector3(0.1, 0.2 , 0.), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArmCrouch, Vector3(1.3, -0.4, 0.), 2);
		Tree* leftArm  = treeFact_.CreateTree( LeftArmCrouch, Vector3(1.3, 0.4, 0.), 3);
		//rightArm->SetBoundaryRadius(rightArm->GetBoundaryRadius() * 8 / 10 );
		//leftArm->SetBoundaryRadius(leftArm->GetBoundaryRadius() * 8 / 10 );
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(rightArm, matrices::Vector3(1.2,0,0), 1);
		res->AddTree(leftArm, matrices::Vector3(1.2,0,0), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

	//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}

	Robot* CreateHumanCrouch180(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorsoCrouch180, Vector3(0, 0., 0), 4), manip_core::enums::robot::Human);
		Tree* rightLeg = treeFact_.CreateTree( RightLegCrouch180, Vector3(0.1, -0.2, 0.), 0);
		Tree* leftLeg  = treeFact_.CreateTree( LeftLegCrouch180, Vector3(0.1, 0.2 , 0.), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArmCrouch180, Vector3(1.3, -0.4, 0.), 2);
		Tree* leftArm  = treeFact_.CreateTree( LeftArmCrouch180, Vector3(1.3, 0.4, 0.), 3);
		//rightArm->SetBoundaryRadius(rightArm->GetBoundaryRadius() * 8 / 10 );
		//leftArm->SetBoundaryRadius(leftArm->GetBoundaryRadius() * 8 / 10 );
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(rightArm, matrices::Vector3(1.2,0,0), 1);
		res->AddTree(leftArm, matrices::Vector3(1.2,0,0), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

	//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}

	Robot* CreateHumanWalk(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorso, Vector3(0.0, 0., 0), 4), manip_core::enums::robot::HumanWalk);
		Tree* rightLeg = treeFact_.CreateTree( RightLegWalk, Vector3(0.0, -0.2, 0.1), 0);
		Tree* leftLeg  = treeFact_.CreateTree( LeftLegWalk, Vector3(0.0, 0.2 , 0.1), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArm, Vector3(0.0, -0.4, 1.3), 2);
		Tree* leftArm  = treeFact_.CreateTree( LeftArm, Vector3(0.0, 0.4, 1.3), 3);
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(rightArm, matrices::Vector3(0,0,1.2), 1);
		res->AddTree(leftArm, matrices::Vector3(0,0,1.2), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

		//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}

	Robot* CreateHumanEllipse(const Matrix4& robotBasis) const
	{
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( HumanTorso, Vector3(0.0, 0., 0), 1), manip_core::enums::robot::HumanEllipse);
		//Tree* rightLeg = treeFact_.CreateTree( RightLegWalk, Vector3(0.0, -0.2, 0.1), 0);
		//Tree* leftLeg  = treeFact_.CreateTree( LeftLegWalk, Vector3(0.0, 0.2 , 0.1), 1);
		Tree* rightArm = treeFact_.CreateTree( RightArmEllipse, Vector3(0.0, -0.4, 1.3), 0);
		//Tree* leftArm  = treeFact_.CreateTree( LeftArm, Vector3(0.0, 0.4, 1.3), 3);
		/*res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);*/
		res->AddTree(rightArm, matrices::Vector3(0,0,1.2), 0);
		//res->AddTree(leftArm, matrices::Vector3(0,0,1.2), 1);
		/*rightLeg->LockTarget(rightLeg->GetTarget());
		leftLeg->LockTarget(leftLeg->GetTarget());*/

		//rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.6)));
		//leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));

		return res;
	}
	
	Robot* CreateQuadruped(const Matrix4& robotBasis) const
	{
		// TODO Remove order neccessity to create ids ...
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( QuadrupedTorso, Vector3(0.0, 0., 0.0), 4), manip_core::enums::robot::Quadruped);
		factories::TreeFactory factory;
		Tree* rightLeg = treeFact_.CreateTree( QuadrupedLegRight, Vector3(-1.2, -0.25, -0.1), 0);
		Tree* leftLeg  = treeFact_.CreateTree( QuadrupedLegLeft, Vector3(-1.2, 0.25 , -0.1), 1);
		Tree* leftArm  = treeFact_.CreateTree( QuadrupedLegLeft, Vector3(0.3, 0.25 , -0.1), 2);
		Tree* rightArm = treeFact_.CreateTree( QuadrupedLegRight, Vector3(0.3, -0.25, -0.1), 3);
		res->AddTree(rightLeg, matrices::Vector3(-1.2,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(-1.2,0,0), 0);
		res->AddTree(leftArm, matrices::Vector3(0.3,0,0), 1);
		res->AddTree(rightArm, matrices::Vector3(0.3,0,0), 1);
		/*rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3(-0.3, 0, -1.3)));
		leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.5, 0, -1.3)));
		rightArm->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightArm->GetPosition() + Vector3( 0.3, 0, -1.3)));*/

		rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.4)));
		leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));
		rightArm->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightArm->GetPosition() + Vector3( 0.4, 0, -1.4)));
		leftArm->LockTarget (matrix4TimesVect3(res->ToWorldCoordinates(), leftArm->GetPosition()  + Vector3( -0.4, 0, -1.4)));

		return res;
	}

	Robot* CreateQuadrupedDown(const Matrix4& robotBasis) const
	{
		// TODO Remove order neccessity to create ids ...
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( QuadrupedTorso, Vector3(0.0, 0., 0.0), 4), manip_core::enums::robot::Quadruped);
		factories::TreeFactory factory;
		Tree* rightLeg = treeFact_.CreateTree( QuadrupedLegDownRight, Vector3(0.0, -0.25, -0.1), 0);
		Tree* leftLeg  = treeFact_.CreateTree( QuadrupedLegDownLeft, Vector3(0.0, 0.25 , -0.1), 1);
		Tree* leftArm  = treeFact_.CreateTree( QuadrupedLegDownLeft, Vector3(1.5, 0.25 , -0.1), 2);
		Tree* rightArm = treeFact_.CreateTree( QuadrupedLegDownRight, Vector3(1.5, -0.25, -0.1), 3);
		res->AddTree(rightLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftLeg, matrices::Vector3(0,0,0), 0);
		res->AddTree(leftArm, matrices::Vector3(1.5,0,0), 1);
		res->AddTree(rightArm, matrices::Vector3(1.5,0,0), 1);
		/*rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3(-0.3, 0, -1.3)));
		leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.5, 0, -1.3)));
		rightArm->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightArm->GetPosition() + Vector3( 0.3, 0, -1.3)));*/

		rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.4)));
		leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));
		rightArm->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightArm->GetPosition() + Vector3( 0.4, 0, -1.4)));
		leftArm->LockTarget (matrix4TimesVect3(res->ToWorldCoordinates(), leftArm->GetPosition()  + Vector3( -0.4, 0, -1.4)));

		return res;
	}

	
	Robot* CreateSpider(const Matrix4& robotBasis) const
	{
		// TODO Remove order neccessity to create ids ...
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( SpiderTorso, Vector3(0.0, 0., 0.0), 8), manip_core::enums::robot::Spider);
		factories::TreeFactory factory;
		Vector3 zeroAngle(0.25, 0, 0);
		Tree* t1 = treeFact_.CreateTree( SpiderLeg, zeroAngle, 0);
		Tree* t2  = treeFact_.CreateTree( SpiderLeg, zeroAngle, 1);
		Tree* t3  = treeFact_.CreateTree( SpiderLeg, zeroAngle, 2);
		Tree* t4 = treeFact_.CreateTree( SpiderLeg, zeroAngle, 3);
		
		Tree* t5 = treeFact_.CreateTree( SpiderLeg, zeroAngle, 4);
		Tree* t6  = treeFact_.CreateTree( SpiderLeg, zeroAngle, 5);
		Tree* t7  = treeFact_.CreateTree( SpiderLeg, zeroAngle, 6);
		Tree* t8 = treeFact_.CreateTree( SpiderLeg, zeroAngle, 7);

		matrices::Vector3 position(0,0,0);
		res->AddTree(t1,position, 1);
		res->AddTree(t2,position, 1);
		res->AddTree(t3,position, 1);
		res->AddTree(t4,position, 1);
		res->AddTree(t5,position, 1);
		res->AddTree(t6,position, 1);
		res->AddTree(t7,position, 1);
		res->AddTree(t8,position, 1);
		/*rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3(-0.3, 0, -1.3)));
		leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.5, 0, -1.3)));
		rightArm->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightArm->GetPosition() + Vector3( 0.3, 0, -1.3)));*/

		/*rightLeg->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightLeg->GetPosition() + Vector3( -0.3, 0, -1.4)));
		leftLeg ->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), leftLeg->GetPosition()  + Vector3( 0.3, 0, -1.4)));
		rightArm->LockTarget(matrix4TimesVect3(res->ToWorldCoordinates(), rightArm->GetPosition() + Vector3( 0.4, 0, -1.4)));
		leftArm->LockTarget (matrix4TimesVect3(res->ToWorldCoordinates(), leftArm->GetPosition()  + Vector3( -0.4, 0, -1.4)));*/

		return res;
	}

	Robot* CreateSpiderRace(const Matrix4& robotBasis) const
	{
		// TODO Remove order neccessity to create ids ...
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( SpiderTorsoRace, Vector3(0.0, 0., 0.0), 4), manip_core::enums::robot::SpiderRace);
		factories::TreeFactory factory;
		Vector3 zeroAngle(0.25, 0, 0);
		Tree* t1 = treeFact_.CreateTree( SpiderLegRace, zeroAngle, 0);
		Tree* t2  = treeFact_.CreateTree( SpiderLegRace, zeroAngle, 1);
		Tree* t3  = treeFact_.CreateTree( SpiderLegRace, zeroAngle, 2);
		Tree* t4 = treeFact_.CreateTree( SpiderLegRace, zeroAngle, 3);
		
		matrices::Vector3 position(0,0,0);
		res->AddTree(t1,position, 1);
		res->AddTree(t2,position, 1);
		res->AddTree(t3,position, 1);
		res->AddTree(t4,position, 1);
		return res;
	}
	
	Robot* CreateSpiderSix(const Matrix4& robotBasis) const
	{
		// TODO Remove order neccessity to create ids ...
		Robot* res = new Robot(robotBasis, treeFact_.CreateTree( SpiderTorsoRace, Vector3(0.0, 0., 0.0), 4), manip_core::enums::robot::SpiderSix);
		factories::TreeFactory factory;
		Vector3 zeroAngle(0.25, 0, 0);
		Tree* t1 = treeFact_.CreateTree( SpiderLegSix, zeroAngle, 0);
		Tree* t2  = treeFact_.CreateTree( SpiderLegSix, zeroAngle, 1);
		Tree* t3  = treeFact_.CreateTree( SpiderLegSix, zeroAngle, 2);
		Tree* t4 = treeFact_.CreateTree( SpiderLegSix, zeroAngle, 3);
		Tree* t5 = treeFact_.CreateTree( SpiderLegSix, zeroAngle, 4);
		Tree* t6 = treeFact_.CreateTree( SpiderLegSix, zeroAngle, 5);
		
		matrices::Vector3 position(0,0,0);
		res->AddTree(t1,position, 1);
		res->AddTree(t2,position, 1);
		res->AddTree(t3,position, 1);
		res->AddTree(t4,position, 1);
		res->AddTree(t5,position, 1);
		res->AddTree(t6,position, 1);
		return res;
	}

	TreeFactory treeFact_;
};
}

using namespace factories;

RobotFactory::RobotFactory()
	: pImpl_(new RobotFactoryPimpl())
{
	// NOTHING
}


RobotFactory::~RobotFactory()
{
	// NOTHING
}

Robot* RobotFactory::CreateRobot(const eRobots robots, const Matrix4& robotBasis) const
{
	switch (robots)
		{
			case Human:
				{
					return pImpl_->CreateHuman(robotBasis);
				}
			case HumanWalk:
				{
					return pImpl_->CreateHumanWalk(robotBasis);
				}
			case HumanEscalade:
				{
					return pImpl_->CreateHumanEscalade(robotBasis);
				}
			case HumanCanap:
				{
					return pImpl_->CreateHumanCanap(robotBasis);
				}
			case HumanEllipse:
				{
					return pImpl_->CreateHumanEllipse(robotBasis);
				}
			case Quadruped:
				{
					return pImpl_->CreateQuadruped(robotBasis);
				}
			case QuadrupedDown:
				{
					return pImpl_->CreateQuadrupedDown(robotBasis);
				}
			case Spider:
				{
					return pImpl_->CreateSpider(robotBasis);
				}
			case SpiderSix:
				{
					return pImpl_->CreateSpiderSix(robotBasis);
				}
			case SpiderRace:
				{
					return pImpl_->CreateSpiderRace(robotBasis);
				}
			case HumanCrouch:
				{
					return pImpl_->CreateHumanCrouch(robotBasis);
				}
			case HumanCrouch180:
				{
					return pImpl_->CreateHumanCrouch180(robotBasis);
				}
			default:
                throw(std::exception());
		}
}
/*
bool IsSpine(const joint_def_t* root)
{
	const std::string headTag("head");
	std::string taf(root->tag);
	return (root->nbChildren_ == 0) ? (taf == headTag) : IsSpine(root->children[0]);
}

const joint_def_t* RetrieveSpine(std::list<const joint_def_t*>& trees)
{
	for(std::list<const joint_def_t*>::const_iterator it = trees.begin(); it!= trees.end(); ++it)
	{
		if(IsSpine(*it))
		{
			const joint_def_t* res = (*it);
			trees.remove(*it);
			return res;
		}
	}
	return 0;
}


void GetTrees(const joint_def_t* root, std::list<const joint_def_t*>& trees)
{
	if(root->is_simple() && !root->is_locked())
	{
			trees.push_back(root);
	}
	else
	{
		for(unsigned int i = 0; i < root->nbChildren_; ++i)
		{
			GetTrees(root->children[i], trees);
		}
	}
}

#include "Pi.h"
#include "kinematic/Com.h"
#include "kinematic/Joint.h"
#include "kinematic/Tree.h"

Joint* GetLast(Tree* tree)
{
	Joint* j = tree->GetRoot();
	while(j)
	{
		if(j->pChild_) j = j->pChild_;
		else return j;
	}
	return 0;
}

void ReadOneJointDef(matrices::Vector3 root, const joint_def_t* jointDef, Tree* tree, const ComFactory& comFact_)
{
	const Vector3 unitx(1, 0, 0);
	const Vector3 unity(0, 1, 0);
	const Vector3 unitz(0, 0, 1);
	const Vector3 vectors []= {unitx, unity, unitz};
	Joint* previous = GetLast(tree);
	matrices::Vector3 attach(jointDef->offset[0], jointDef->offset[1], jointDef->offset[2]);
	attach += root;
	bool lastIsEffector = jointDef->nbChildren_ == 0;
	for(int i = 0; i < 3; ++ i)
	{
		Joint* j = new Joint(attach, vectors[i], (lastIsEffector && i == 2) ? EFFECTOR : JOINT, comFact_.CreateCom(None)
							, RADIAN(jointDef->minAngleValues[i]), RADIAN(jointDef->maxAngleValues[i]), RADIAN(jointDef->defaultAngleValues[i]),
							rotation::eRotation(i));
		if(previous)
		{
			tree->InsertChild(previous, j);
		}
		else
		{
			tree->InsertRoot(j);
		}
		previous = j;
	}
	if(!lastIsEffector)
		ReadOneJointDef(attach, jointDef->children[0], tree, comFact_);
}

Tree* MakeTree(const joint_def_t* root, unsigned int& id)
{
	assert(root->is_simple());
	const ComFactory comFact;
	Tree* tree = new Tree(id++);
	const joint_def_t* current_joint = root;
	matrices::Vector3 attach(0,0,0);
	ReadOneJointDef(attach, root, tree, comFact);
	return tree;
	
}

Robot* RobotFactory::CreateRobot(const joint_def_t& joint, const Matrix4& robotBasis) const
{
	std::list<const joint_def_t*> trees;
	GetTrees(&joint, trees);
	const joint_def_t * spine = RetrieveSpine(trees);
	unsigned int id = 0; unsigned int torsoIndex = trees.size();
	Robot* res = new Robot(robotBasis, MakeTree(spine, torsoIndex));
	for(std::list<const joint_def_t*>::const_iterator it = trees.begin(); it!= trees.end(); ++it)
	{
		matrices::Vector3 offset((*it)->offset[0], (*it)->offset[1], (*it)->offset[2]);
		res->AddTree( MakeTree((*it), id), offset, 0); // TODO find good column joint for rendering
	}
	return res;
}*/
