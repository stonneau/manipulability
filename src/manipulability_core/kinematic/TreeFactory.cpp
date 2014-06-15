
#include "TreeFactory.h"
#include "kinematic/Tree.h"
#include "kinematic/Joint.h"
#include "Pi.h"
#include "kinematic/Com.h"

#include <vector>

using namespace matrices;
using namespace manip_core::enums;
using namespace manip_core::enums::rotation;

namespace factories
{
const Vector3 unitx(1, 0, 0);
const Vector3 unity(0, 1, 0);
const Vector3 unitz(0, 0, 1);
const Vector3 unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
const Vector3 zero(0,0,0);

struct TreeFactoryPimpl{
	TreeFactoryPimpl()
	{
		// NOTHING
	}
	
	~TreeFactoryPimpl()
	{
		// NOTHING
	}
	
	Tree* CreateHumanTorso(const Vector3& root, const Tree::TREE_ID id)
	{
		Tree* torso = new Tree(id, 1, manip_core::enums::HumanTorso );
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0), RADIAN(0), Y);
		torso->InsertRoot(j0);

		Joint* j1 = new Joint(Vector3(root(0), root(1), root(2) + 1.5), unity, JOINT, comFact_.CreateCom(Trunk), RADIAN(0)  , RADIAN(0)  , RADIAN(0), Y);
		torso->InsertChild(j0, j1);

		Joint* j2 = new Joint(Vector3(root(0), root(1), root(2) + 1.7), unitx, EFFECTOR, comFact_.CreateCom(Head), RADIAN(0), RADIAN(0) , RADIAN(0),  X);
		torso->InsertChild(j1, j2);

		return torso;
	}

	Tree* CreateHumanTorsoCrouch(const Vector3& root, const Tree::TREE_ID id)
	{
		Tree* torso = new Tree(id, 1, manip_core::enums::HumanTorso );
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0), RADIAN(0), Y);
		torso->InsertRoot(j0);

		Joint* j1 = new Joint(Vector3(root(0)+ 1.5, root(1), root(2) ), unity, JOINT, comFact_.CreateCom(Trunk), RADIAN(0)  , RADIAN(0)  , RADIAN(0), Y);
		torso->InsertChild(j0, j1);

		Joint* j2 = new Joint(Vector3(root(0) + 1.7, root(1), root(2)), unitx, EFFECTOR, comFact_.CreateCom(Head), RADIAN(0), RADIAN(0) , RADIAN(0),  X);
		torso->InsertChild(j1, j2);

		return torso;
	}

	Tree* CreateHumanTorsoCrouch180(const Vector3& root, const Tree::TREE_ID id)
	{
		Tree* torso = new Tree(id, 1, manip_core::enums::HumanTorso );
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0), RADIAN(0), Y);
		torso->InsertRoot(j0);

		Joint* j1 = new Joint(Vector3(root(0)+ 1.5, root(1), root(2) ), unity, JOINT, comFact_.CreateCom(Trunk), RADIAN(0)  , RADIAN(0)  , RADIAN(0), Y);
		torso->InsertChild(j0, j1);

		Joint* j2 = new Joint(Vector3(root(0) + 1.7, root(1), root(2)), unitx, EFFECTOR, comFact_.CreateCom(Head), RADIAN(0), RADIAN(0) , RADIAN(0),  X);
		torso->InsertChild(j1, j2);

		return torso;
	}

	Tree* CreateQuadrupedTorso(const Vector3& root, const Tree::TREE_ID id) // TODO
	{
		Tree* torso = new Tree(id, 1, manip_core::enums::QuadrupedTorso);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0), RADIAN(0), Y);
		torso->InsertRoot(j0);

		Joint* j1 = new Joint(Vector3(root(0) - 1.2, root(1), root(2)), unity, JOINT, comFact_.CreateCom(Trunk), RADIAN(0)  , RADIAN(0)  , RADIAN(0), Y);
		torso->InsertChild(j0, j1);

		Joint* j2 = new Joint(Vector3(root(0) + 1.3, root(1), root(2)), unitx, EFFECTOR, comFact_.CreateCom(Head), RADIAN(0), RADIAN(0) , RADIAN(0),  X);
		torso->InsertChild(j1, j2);

		return torso;
	}
	
	Tree* CreateSpiderTorso(const Vector3& root, const Tree::TREE_ID id) // TODO
	{
		Tree* torso = new Tree(id, 4, manip_core::enums::SpiderTorso);
		Joint* j0 = new Joint(Vector3(root(0) - 0.3, root(1), root(2)), unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0), RADIAN(0), Y);
		torso->InsertRoot(j0);

		Joint* j1 = new Joint(Vector3(root(0), root(1), root(2)), unity, JOINT, comFact_.CreateCom(Trunk), RADIAN(0)  , RADIAN(0)  , RADIAN(0), Y); 
		torso->InsertChild(j0, j1);

		Joint* j2 = new Joint(Vector3(root(0) + 0.1 , root(1), root(2)), unitx, EFFECTOR, comFact_.CreateCom(Head), RADIAN(0), RADIAN(0) , RADIAN(0),  X);
		torso->InsertChild(j1, j2);

		return torso;
	}

	Tree* CreateSpiderTorsoRace(const Vector3& root, const Tree::TREE_ID id) // TODO
	{
		Tree* torso = new Tree(id, 4, manip_core::enums::SpiderTorsoRace);
		Joint* j0 = new Joint(Vector3(root(0) - 0.3, root(1), root(2)), unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0), RADIAN(0), Y);
		torso->InsertRoot(j0);

		Joint* j1 = new Joint(Vector3(root(0), root(1), root(2)), unity, JOINT, comFact_.CreateCom(Trunk), RADIAN(0)  , RADIAN(0)  , RADIAN(0), Y);
		torso->InsertChild(j0, j1);

		Joint* j2 = new Joint(Vector3(root(0) + 0.1 , root(1), root(2)), unitx, EFFECTOR, comFact_.CreateCom(Head), RADIAN(0), RADIAN(0) , RADIAN(0),  X);
		torso->InsertChild(j1, j2);

		return torso;
	}
	

	Tree* CreateLeftArmCrouch(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 3, manip_core::enums::LeftArm);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(30), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-90)  , RADIAN(15)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-90)  , RADIAN(90)  , RADIAN(0.), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(0), RADIAN(-30),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateLeftArmCrouch180(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 3, manip_core::enums::LeftArm);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(30), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-90)  , RADIAN(15)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) + 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-90)  , RADIAN(90)  , RADIAN(0.), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) + 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(0), RADIAN(-30),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) + 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}
	
	Tree* CreateRightArmCrouch(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 2, manip_core::enums::RightArm);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(90), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-45)  , RADIAN(45)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm),  RADIAN(-90), RADIAN(90), RADIAN(0.), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None),RADIAN(-120)  , RADIAN(0)  , RADIAN(-30),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightArmCrouch180(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 2, manip_core::enums::RightArm);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(90), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-45)  , RADIAN(45)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		//Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) + 0.8), unity, JOINT, comFact_.CreateCom(UpperAm),  RADIAN(-90), RADIAN(90), RADIAN(0.), Y);
		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-90)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) + 0.8), unitx, JOINT, comFact_.CreateCom(None),RADIAN(-120)  , RADIAN(0)  , RADIAN(-30),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) + 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightArm(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 2, manip_core::enums::RightArm);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(45), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-45)  , RADIAN(45)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-90)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		//Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-180), RADIAN(180), RADIAN(0.),  X);
		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(120), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	
	Tree* CreateLeftArm(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 3, manip_core::enums::LeftArm);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180), RADIAN(70), RADIAN(0),   Y); // normal
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(45), RADIAN(0),   Y); //chair
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-45)  , RADIAN(45)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-90)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(0), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightArmEllipse(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 0, manip_core::enums::RightArm);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-45)  , RADIAN(45)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-160)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		/*Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);*/
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j3, j5);
		return arm;
	}

	Tree* CreateRightLeg(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::RightLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(-10), RADIAN(-10),    Y); // normal
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-70), RADIAN(70), RADIAN(0),    Y); // chair
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-170), RADIAN(100), RADIAN(0),    Y); // armoire
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(30) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-50)  , RADIAN(30)  , RADIAN(0),    Z); normal
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(30)  , RADIAN(0),    Z); // chair
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-15), RADIAN(15) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	
	
	Tree* CreateLeftLeg(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 1, manip_core::enums::LeftLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-100), RADIAN(-10), RADIAN(-10),    Y); // normal
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-70), RADIAN(70), RADIAN(0),    Y); // chair
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(110), RADIAN(0),    Y); // armoire
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(30) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(50)  , RADIAN(0),    Z);
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(30)  , RADIAN(0),    Z); // chair
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-15), RADIAN(15) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateLeftArmEscalade(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 3, manip_core::enums::LeftArmEscalade);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180), RADIAN(70), RADIAN(0),   Y); // normal
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-110), RADIAN(70), RADIAN(0),   Y); //chair
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-90)  , RADIAN(45)  , RADIAN(20),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-120)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightArmEscalade(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 2, manip_core::enums::RightArmEscalade);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-110), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-45)  , RADIAN(45)  , RADIAN(-20),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-120)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightLegEscalade(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::RightLegEscalade);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-170), RADIAN(70), RADIAN(0),    Y); // normal
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-70), RADIAN(70), RADIAN(0),    Y); // chair
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),    Y); // armoire
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(45) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-50)  , RADIAN(30)  , RADIAN(0),    Z); normal
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-90)  , RADIAN(10)  , RADIAN(-50),    Z); // chair
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-30)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-15), RADIAN(15) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}


	Tree* CreateLeftLegEscalade(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 1, manip_core::enums::LeftLegEscalade);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-170), RADIAN(70), RADIAN(0),    Y); // normal
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-70), RADIAN(70), RADIAN(0),    Y); // chair
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),    Y); // armoire
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-45) , RADIAN(90) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(50)  , RADIAN(0),    Z);
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(90)  , RADIAN(5),    Z); // chair
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-15), RADIAN(15) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateLeftArmCanap(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 3, manip_core::enums::LeftArmCanap);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180), RADIAN(70), RADIAN(0),   Y); // normal
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-70), RADIAN(70), RADIAN(0),   Y); //chair
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(90) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-0)  , RADIAN(70)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-70)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightArmCanap(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* arm = new Tree(id, 2, manip_core::enums::RightArmCanap);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-70), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);

		/*Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(70), RADIAN(0),   Y);
		arm->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-90) , RADIAN(30) , RADIAN(0),   X);
		arm->InsertChild(j0, j1);*/

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-0)  , RADIAN(0)  , RADIAN(0),   Z);
		arm->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(UpperAm), RADIAN(-70)  , RADIAN(0)  , RADIAN(-30), Y);
		arm->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0.),  X);
		arm->InsertChild(j3, j4);
	 
		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(ForeArm));
		arm->InsertChild(j4, j5);
		return arm;
	}

	Tree* CreateRightLegCanap(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::RightLegCanap);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120.), RADIAN(0), RADIAN(0),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(0) , RADIAN(0) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateLeftLegCanap(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 1, manip_core::enums::LeftLegCanap);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-150), RADIAN(0), RADIAN(0),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(0) , RADIAN(0) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateRightLegCrouch(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::RightLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-50) , RADIAN(-10) , RADIAN(0), Y);
		leg->InsertRoot(j0);

		//Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None) , RADIAN(-170), RADIAN(70), RADIAN(0),   X);
		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-0), RADIAN(0), RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-50)  , RADIAN(30)  , RADIAN(0),    Z);
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-15), RADIAN(110) , RADIAN(0.), Y);
		leg->InsertChild(j2, j3);

		//Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(110)  , RADIAN(30),  X);
		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),   X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateRightLegCrouch180(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::RightLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-50) , RADIAN(-10) , RADIAN(0), Y);
		leg->InsertRoot(j0);

		//Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None) , RADIAN(-170), RADIAN(70), RADIAN(0),   X);
		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-0), RADIAN(0), RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-50)  , RADIAN(30)  , RADIAN(0),    Z);
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) + 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-15), RADIAN(110) , RADIAN(0.), Y);
		leg->InsertChild(j2, j3);

		//Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(110)  , RADIAN(30),  X);
		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) + 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),   X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) + 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateLeftLegCrouch(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 1, manip_core::enums::LeftLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-50) , RADIAN(30) , RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-0) , RADIAN(0) , RADIAN(0),    Y);
		leg->InsertRoot(j0);

		//Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-170), RADIAN(70), RADIAN(0),    X);
		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-0), RADIAN(0), RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(50)  , RADIAN(0),    Z);
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-15), RADIAN(110) , RADIAN(0.), Y);
		leg->InsertChild(j2, j3);

		//Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(110)  , RADIAN(30),   X);
		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),   X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateLeftLegCrouch180(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 1, manip_core::enums::LeftLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-50) , RADIAN(30) , RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-0) , RADIAN(0) , RADIAN(0),    Y);
		leg->InsertRoot(j0);

		//Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-170), RADIAN(70), RADIAN(0),    X);
		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-0), RADIAN(0), RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		//Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-30)  , RADIAN(50)  , RADIAN(0),    Z);
		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) + 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-15), RADIAN(110) , RADIAN(0.), Y);
		leg->InsertChild(j2, j3);

		//Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(110)  , RADIAN(30),   X);
		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) + 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),   X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) + 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateRightLegWalk(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::RightLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120.), RADIAN(120.), RADIAN(0),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(0) , RADIAN(0) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateLeftLegWalk(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 1, manip_core::enums::LeftLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-150), RADIAN(120.), RADIAN(0),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(0) , RADIAN(0) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-0)  , RADIAN(0)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(110)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 1.), unitx, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(0) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 2.2), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateQuadrupedLeg(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0, manip_core::enums::QuadrupedLeg);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(30) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-15)  , RADIAN(15)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(120)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}
	
	
	Tree* CreateSpiderLeg(const Vector3& root, const Tree::TREE_ID id) const
	{
		int templateId = id;//(id > 4 ) ? id -1 : id;
		//float angle = (float)((templateId % 4) * RADIAN(30)) * ((id > 3 ) ? -1 : 1);
		int sign = (id > 3 ) ? -1 : 1;
		NUMBER angle;
		Vector3 length, length2;
		float y1, x1, z1, z12, y21, y2, x2, x21;
		switch (templateId)
		{
			case 0:
			case 7:
			{
				length = Vector3(0.7, 0., 0.3);
				length2 = Vector3(0.7, 0, -0.5);
				angle = RADIAN(25) * sign;
				y1 = 60; x1 = 30; z1 = 20; z12 = -20; y2 = 30; x2 = 15; x21 = 30; y21 = -30;
				break;
			}
			case 3:
			case 4:
			{
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = RADIAN(150) * sign;
				y1 = 60; x1 = 30; z1 = 10; z12 = -10; y2 = 30; x2 = 15; x21 = 30; y21 = -30;
				//y1 = 60; x1 = 30; z1 = 20;z12 = -z1; y2 = 120; x2 = 30; x21 = x2; y21 = -30;
				break;
			}
			case 1:
			case 6:
			{
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = (float)(RADIAN(72)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -z1; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				break;
			}
			default:
			{
				/*length = Vector3(0.6, 0., 0.6);
				length2 = Vector3(0.2, 0, -1.4);*/
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = (float)(RADIAN(110)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -z1; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				//y1 = 10; x1 = 30; z1 = 70;z12 = -z1; y2 = 10; x2 = 10; x21 = 10;  y21 = -10;
			}
		}

		matrices::Matrix3 rot = matrices::Rotz3(angle);
		Vector3 rotRoot = rot * root; // + Vector3(1, 0, 0);
		Tree* leg = new Tree(id, templateId, manip_core::enums::SpiderLegRace);
		Joint* j0 = new Joint(rotRoot, unity, JOINT, comFact_.CreateCom(None), RADIAN(-y1), RADIAN(y1), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(rotRoot, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-x1) , RADIAN(x1) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(rotRoot, unitz, JOINT, comFact_.CreateCom(None), RADIAN(z12)  , RADIAN(z1)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Vector3 rotatedVector = rot * length;
		Joint* j3 = new Joint(rotRoot + rotatedVector, unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(y21)  , RADIAN(y2)  , RADIAN(0), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(rotRoot + rotatedVector, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-x21), RADIAN(x2) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j4z = new Joint(rotRoot + rotatedVector, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-z1)  , RADIAN(z1)  , RADIAN(0),    Z);
		leg->InsertChild(j4, j4z);

		Joint* j5 = new Joint(rotRoot + rotatedVector + rot * length2, zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4z, j5);
		//leg->SetBoundaryRadius(1.4);
		return leg;
	}


	Tree* CreateSpiderLegRace(const Vector3& root, const Tree::TREE_ID id) const
	{
		int templateId = id;//(id > 4 ) ? id -1 : id;
		//float angle = (float)((templateId % 4) * RADIAN(30)) * ((id > 3 ) ? -1 : 1);
		int sign = (id > 1 ) ? -1 : 1;
		NUMBER angle;
		Vector3 length, length2;
		float y1, x1, z1, z12, y21, y2, x2, x21;
		switch (templateId)
		{
			case 0:
			case 3:
			{
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = RADIAN(45) * sign;
				y1 = 60; x1 = 30; z1 = 20; z12 = -20; y2 = 45; x2 = 15; x21 = 30; y21 = -30;
				//y1 = 60; x1 = 30; z1 = 20;z12 = -z1; y2 = 120; x2 = 30; x21 = x2; y21 = -30;
				break;
			}
			case 1:
			case 2:
			{
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = (float)(RADIAN(130)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -10; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				break;
			}
			default:
			{
				/*length = Vector3(0.6, 0., 0.6);
				length2 = Vector3(0.2, 0, -1.4);*/
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = (float)(RADIAN(110)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -z1; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				//y1 = 10; x1 = 30; z1 = 70;z12 = -z1; y2 = 10; x2 = 10; x21 = 10;  y21 = -10;
			}
		}

		matrices::Matrix3 rot = matrices::Rotz3(angle);
		Vector3 rotRoot = rot * root; // + Vector3(1, 0, 0);
		Tree* leg = new Tree(id, templateId, manip_core::enums::SpiderLeg);
		Joint* j0 = new Joint(rotRoot, unity, JOINT, comFact_.CreateCom(None), RADIAN(-y1), RADIAN(y1), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(rotRoot, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-x1) , RADIAN(x1) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(rotRoot, unitz, JOINT, comFact_.CreateCom(None), RADIAN(z12)  , RADIAN(z1)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Vector3 rotatedVector = rot * length;
		Joint* j3 = new Joint(rotRoot + rotatedVector, unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(y21)  , RADIAN(y2)  , RADIAN(0), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(rotRoot + rotatedVector, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-x21), RADIAN(x2) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j4z = new Joint(rotRoot + rotatedVector, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-z1)  , RADIAN(z1)  , RADIAN(0),    Z);
		leg->InsertChild(j4, j4z);

		Joint* j5 = new Joint(rotRoot + rotatedVector + rot * length2, zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4z, j5);
		return leg;
	}

	Tree* CreateSpiderLegSix(const Vector3& root, const Tree::TREE_ID id) const
	{
		int templateId = id;//(id > 4 ) ? id -1 : id;
		//float angle = (float)((templateId % 4) * RADIAN(30)) * ((id > 3 ) ? -1 : 1);
		int sign = (id > 2 ) ? -1 : 1;
		NUMBER angle;
		Vector3 length, length2;
		float y1, x1, z1, z12, y21, y2, x2, x21;
		switch (templateId)
		{
			case 0:
			case 5:
			{
				length = Vector3(0.6, 0., 0.2);
				length2 = Vector3(0.6, 0, -0.4);
				angle = RADIAN(45) * sign;
				y1 = 60; x1 = 30; z1 = 20; z12 = -20; y2 = 45; x2 = 15; x21 = 30; y21 = -30;
				//y1 = 60; x1 = 30; z1 = 20;z12 = -z1; y2 = 120; x2 = 30; x21 = x2; y21 = -30;
				break;
			}
			case 1:
			case 4:
			{
				length = Vector3(0.6, 0., 0.2);
				length2 = Vector3(0.6, 0, -0.4);
				angle = (float)(RADIAN(130)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -10; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				break;
			}
			case 2:
			case 3:
			{
				length = Vector3(0.6, 0., 0.2);
				length2 = Vector3(0.6, 0, -0.4);
				angle = (float)(RADIAN(87)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -10; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				break;
			}
			default:
			{
				/*length = Vector3(0.6, 0., 0.6);
				length2 = Vector3(0.2, 0, -1.4);*/
				length = Vector3(0.5, 0., 0.2);
				length2 = Vector3(0.5, 0, -0.4);
				angle = (float)(RADIAN(110)) * sign;
				y1 = 30; x1 = 30; z1 = 20;z12 = -z1; y2 = 30; x2 = 30; x21 = 30;  y21 = -30;
				//y1 = 10; x1 = 30; z1 = 70;z12 = -z1; y2 = 10; x2 = 10; x21 = 10;  y21 = -10;
			}
		}

		matrices::Matrix3 rot = matrices::Rotz3(angle);
		Vector3 rotRoot = rot * root; // + Vector3(1, 0, 0);
		Tree* leg = new Tree(id, templateId, manip_core::enums::SpiderLeg);
		Joint* j0 = new Joint(rotRoot, unity, JOINT, comFact_.CreateCom(None), RADIAN(0), RADIAN(y1), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(rotRoot, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-x1) , RADIAN(x1) , RADIAN(60 * sign),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(rotRoot, unitz, JOINT, comFact_.CreateCom(None), RADIAN(z12)  , RADIAN(z1)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Vector3 rotatedVector = rot * length;
		Joint* j3 = new Joint(rotRoot + rotatedVector, unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(y21)  , RADIAN(y2)  , RADIAN(0), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(rotRoot + rotatedVector, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-x21), RADIAN(x2) , RADIAN(-60 * sign),  X);
		leg->InsertChild(j3, j4);

		Joint* j4z = new Joint(rotRoot + rotatedVector, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-z1)  , RADIAN(z1)  , RADIAN(0),    Z);
		leg->InsertChild(j4, j4z);

		Joint* j5 = new Joint(rotRoot + rotatedVector + rot * length2, zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4z, j5);
		return leg;
	}

	Tree* CreateQuadrupedLegLeft(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(0) , RADIAN(30) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-15)  , RADIAN(15)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(120)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateQuadrupedLegRight(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(120), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(0) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-15)  , RADIAN(15)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(180)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}
	Tree* CreateQuadrupedDownLegLeft(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-120), RADIAN(120), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(0) , RADIAN(30) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-15)  , RADIAN(15)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(0)  , RADIAN(180)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}

	Tree* CreateQuadrupedLegDownRight(const Vector3& root, const Tree::TREE_ID id) const
	{
		Tree* leg = new Tree(id, 0);
		Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-90), RADIAN(90), RADIAN(0),    Y);
		//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
		leg->InsertRoot(j0);

		Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(0) , RADIAN(0),    X);
		leg->InsertChild(j0, j1);

		Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-15)  , RADIAN(15)  , RADIAN(0),    Z);
		leg->InsertChild(j1, j2);

		Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(5)  , RADIAN(120)  , RADIAN(30), Y);
		leg->InsertChild(j2, j3);

		Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
		leg->InsertChild(j3, j4);

		Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
		leg->InsertChild(j4, j5);
		return leg;
	}
	//Tree* CreateQuadrupedLegLeft(const Vector3& root, const Tree::TREE_ID id) const
	//{
	//	Tree* leg = new Tree(id, 0, manip_core::enums::QuadrupedLegLeft);
	//	Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0),    Y);
	//	//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
	//	leg->InsertRoot(j0);

	//	Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(30) , RADIAN(0),    X);
	//	leg->InsertChild(j0, j1);

	//	Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-150)  , RADIAN(150)  , RADIAN(0),    Z);
	//	leg->InsertChild(j1, j2);

	//	Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-5)  , RADIAN(120)  , RADIAN(30), Y);
	//	leg->InsertChild(j2, j3);

	//	Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
	//	leg->InsertChild(j3, j4);

	//	Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
	//	leg->InsertChild(j4, j5);
	//	return leg;
	//}

	//Tree* CreateQuadrupedLegRight(const Vector3& root, const Tree::TREE_ID id) const
	//{
	//	Tree* leg = new Tree(id, 1, manip_core::enums::QuadrupedLegRight);
	//	Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(0),    Y);
	//	//Joint* j0 = new Joint(root, unity, JOINT, comFact_.CreateCom(None), RADIAN(-180.), RADIAN(180.), RADIAN(180),    Y);
	//	leg->InsertRoot(j0);

	//	Joint* j1 = new Joint(root, unitx, JOINT, comFact_.CreateCom(None), RADIAN(-30) , RADIAN(30) , RADIAN(0),    X);
	//	leg->InsertChild(j0, j1);

	//	Joint* j2 = new Joint(root, unitz, JOINT, comFact_.CreateCom(None), RADIAN(-150)  , RADIAN(150)  , RADIAN(0),    Z);
	//	leg->InsertChild(j1, j2);

	//	Joint* j3 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unity, JOINT, comFact_.CreateCom(Thigh), RADIAN(-5)  , RADIAN(120)  , RADIAN(30), Y);
	//	leg->InsertChild(j2, j3);

	//	Joint* j4 = new Joint(Vector3(root(0), root(1), root(2) - 0.8), unitx, JOINT, comFact_.CreateCom(None), RADIAN(-10.), RADIAN(10.) , RADIAN(0.),  X);
	//	leg->InsertChild(j3, j4);

	//	Joint* j5 = new Joint(Vector3(root(0), root(1), root(2) - 1.7), zero, EFFECTOR, comFact_.CreateCom(Calf));
	//	leg->InsertChild(j4, j5);
	//	return leg;
	//}

	ComFactory comFact_;
};
}

using namespace factories;

TreeFactory::TreeFactory()
	: pImpl_(new TreeFactoryPimpl())
{
	// NOTHING
}


TreeFactory::~TreeFactory()
{
	// NOTHING
}

Tree* TreeFactory::CreateTree(const eMembers member, const Vector3& rootPosition, const Tree::TREE_ID id) const
{
	switch (member)
		{
			case RightArm:
				{
					return pImpl_->CreateRightArm(rootPosition, id);
				}
			case RightArmEscalade:
				{
					return pImpl_->CreateRightArmEscalade(rootPosition, id);
				}
			case RightArmCanap:
				{
					return pImpl_->CreateRightArmCanap(rootPosition, id);
				}
			case RightArmCrouch:
				{
					return pImpl_->CreateRightArmCrouch(rootPosition, id);
				}
			case RightArmCrouch180:
				{
					return pImpl_->CreateRightArmCrouch180(rootPosition, id);
				}
			case RightArmEllipse:
				{
					return pImpl_->CreateRightArmEllipse(rootPosition, id);
				}
			case LeftArm:
				{
					return pImpl_->CreateLeftArm(rootPosition, id);
				}
			case LeftArmEscalade:
				{
					return pImpl_->CreateLeftArmEscalade(rootPosition, id);
				}
			case LeftArmCanap:
				{
					return pImpl_->CreateLeftArmCanap(rootPosition, id);
				}
			case LeftArmCrouch:
				{
					return pImpl_->CreateLeftArmCrouch(rootPosition, id);
				}
			case LeftArmCrouch180:
				{
					return pImpl_->CreateLeftArmCrouch180(rootPosition, id);
				}
			case LeftLeg:
				{
					return pImpl_->CreateLeftLeg(rootPosition, id);
				}
			case LeftLegEscalade:
				{
					return pImpl_->CreateLeftLegEscalade(rootPosition, id);
				}
			case LeftLegCanap:
				{
					return pImpl_->CreateLeftLegCanap(rootPosition, id);
				}
			case RightLeg:
				{
					return pImpl_->CreateRightLeg(rootPosition, id);
				}
			case RightLegEscalade:
				{
					return pImpl_->CreateRightLegEscalade(rootPosition, id);
				}
			case RightLegCanap:
				{
					return pImpl_->CreateRightLegCanap(rootPosition, id);
				}
			case LeftLegCrouch:
				{
					return pImpl_->CreateLeftLegCrouch(rootPosition, id);
				}
			case RightLegCrouch:
				{
					return pImpl_->CreateRightLegCrouch(rootPosition, id);
				}
			case LeftLegCrouch180:
				{
					return pImpl_->CreateLeftLegCrouch180(rootPosition, id);
				}
			case RightLegCrouch180:
				{
					return pImpl_->CreateRightLegCrouch180(rootPosition, id);
				}
			case LeftLegWalk:
				{
					return pImpl_->CreateLeftLegWalk(rootPosition, id);
				}
			case RightLegWalk:
				{
					return pImpl_->CreateRightLegWalk(rootPosition, id);
				}
			case HumanTorso:
				{
					return pImpl_->CreateHumanTorso(rootPosition, id);
				}
			case HumanTorsoCrouch:
				{
					return pImpl_->CreateHumanTorsoCrouch(rootPosition, id);
				}
			case HumanTorsoCrouch180:
				{
					return pImpl_->CreateHumanTorsoCrouch180(rootPosition, id);
				}
			case QuadrupedTorso:
				{
					return pImpl_->CreateQuadrupedTorso(rootPosition, id);
				}
			case QuadrupedLeg:
				{
					return pImpl_->CreateQuadrupedLeg(rootPosition, id);
				}
			case QuadrupedLegLeft:
				{
					return pImpl_->CreateQuadrupedLegLeft(rootPosition, id);
				}
			case QuadrupedLegRight:
				{
					return pImpl_->CreateQuadrupedLegRight(rootPosition, id);
				}
			case QuadrupedLegDownLeft:
				{
					return pImpl_->CreateQuadrupedDownLegLeft(rootPosition, id);
				}
			case QuadrupedLegDownRight:
				{
					return pImpl_->CreateQuadrupedLegDownRight(rootPosition, id);
				}
			case SpiderTorso:
				{
					return pImpl_->CreateSpiderTorso(rootPosition, id);
				}
			case SpiderTorsoRace:
				{
					return pImpl_->CreateSpiderTorsoRace(rootPosition, id);
				}
			case SpiderLeg:
				{
					return pImpl_->CreateSpiderLeg(rootPosition, id);
				}
			case SpiderLegSix:
				{
					return pImpl_->CreateSpiderLegSix(rootPosition, id);
				}
			case SpiderLegRace:
				{
					return pImpl_->CreateSpiderLegRace(rootPosition, id);
				}
			default:
                throw(std::exception());
		}
}


