

#ifdef WIN32
#include <windows.h>
#endif

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include "MatrixDefs.h"
#include "Pi.h"

#include "Simulation.h"

namespace matrices
{
	const Vector3 unitx = Vector3(1, 0, 0);
	const Vector3 unity = Vector3(0, 1, 0);
	const Vector3 unitz = Vector3(0, 0, 1);
	const Vector3 zero  = Vector3(0, 0, 0);
}

using namespace matrices;
using namespace Eigen;
using namespace std;
using namespace manip_core;

bool takeInput = true;

Simulation* sim;

void PosePrise(const matrices::Vector3& p)
{
	Vector3 p1(p(0), p(1)+0.1, p(2)+0.1);
	Vector3 p2(p(0), p(1)-0.2, p(2)-0.2);
	sim->manager_.GenerateVerticalChess(p1, p2, 0);
}

void command(int cmd)   /**  key control function; */
{
	const Vector3 trX(0.03, 0, 0);
	const Vector3 trY(0,0.01,  0);
	const Vector3 trZ(0,0.0,  0.03);
	const Vector3 myZ(0.3, 0.0,  0.07);
	switch (cmd)
	{	
		case 'p' :
			sim->simpParams_.pause_ = ! sim->simpParams_.pause_;
		break;
		case 'u' :
			sim->simpParams_.drawArms_ = ! sim->simpParams_.drawArms_;
		break;
		case 's' :
			sim->motionHandler_.Translate(-matrices::unitz);
		break;
		case 'd' :
			sim->motionHandler_.Translate(-matrices::unitx);
		break;
		case 'z' :
//			sim->motionHandler_.Translate(myZ);
			sim->motionHandler_.Translate(matrices::unitz);
		break;
		case 'q' :
			sim->motionHandler_.Translate(matrices::unitx);
		break;
		case 'w' :
			sim->motionHandler_.Translate(-matrices::unity);
		break;
		case 'x' :
			sim->motionHandler_.Translate(matrices::unity);
		break;
		case 'g' :
			sim->simpParams_.drawSplines_ = ! sim->simpParams_.drawSplines_;
		break;
		case 'e' :
			sim->simpParams_.drawNormals_ = ! sim->simpParams_.drawNormals_;
		break;
		case 'r' :
			sim->pRobot->Rest();
		break;
		case 'a' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->Reset();
		break;
		case 'b' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->simpParams_.speed_ -= 0.1;
		break;
		case 'n' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->simpParams_.speed_ += 0.1;
		break;
		case 'h' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->simpParams_.jumpToTarget_ = !sim->simpParams_.jumpToTarget_;
			sim->postureManager_->SetJumpToTarget(sim->simpParams_.jumpToTarget_);
		break;
		case 'c' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->simpParams_.closestDistance_ = !sim->simpParams_.closestDistance_;
		break;
		case 'f' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->simpParams_.GetCamera()->ToggleFree();
		break;
		case '-' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->drawManager_.PreviousPosture();
		break;
		case '+' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			sim->drawManager_.NextPosture();
		break;
		case 't' :
			sim->simpParams_.drawSamples_ = !sim->simpParams_.drawSamples_;
		break;
		case 'y' :
			sim->simpParams_.drawSamplesByOne_ = !sim->simpParams_.drawSamplesByOne_;
		break;
		case '1' :
			sim->simpParams_.drawManip_ = !sim->simpParams_.drawManip_;
		break;
		case 'j' :
			sim->simpParams_.currentSample_ ++;
		break;
	}
}

int main(int argc, char *argv[])
{
	sim = Simulation::GetInstance();
	sim->simpParams_.command = &command;
	sim->simpParams_.jumpToTarget_ = false;
	sim->Start(argc, argv);
	
    return 0;
}
	