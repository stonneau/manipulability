#include "XboxMotion.h"
#include "MotionHandler.h"
#include "Simulation.h"
#include "MatrixDefs.h"
#include "Pi.h"

using namespace matrices;
#ifdef _WIN32
using namespace xbox;
XboxMotion::XboxMotion()
    : controller_(1)
{
    // NOTHING
}

#else
XboxMotion::XboxMotion()
{
    // NOTHING
}
#endif


XboxMotion::~XboxMotion()
{
	// NOTHING
}


void XboxMotion::Update(const Timer::t_time t, const Timer::t_time dt)
{
#ifdef _WIN32
	MotionHandler& handler = Simulation::GetInstance()->motionHandler_;
	controller_.Update();	
	float xL, yL;
	float xR, yR;
	bool transformed = false;
	const float slowDownFactor = 0.0042f;
	controller_.GetLeftStickMovingVector(xL, yL);
	controller_.GetRightStickMovingVector(xR, yR);
	if( xL != 0 || yL != 0)
	{
		handler.Translate(Vector3(0.,-((double)xL * slowDownFactor*3), ((double)yL * slowDownFactor * 2)));
		transformed = true;
	}
	if( xR != 0 || yR != 0)
	{
		handler.Translate(Vector3(((double)xR * slowDownFactor*3),0., ((double)yR * slowDownFactor * 2)));
		transformed = true;
	}
	if(controller_.AllPressed((int)xbox::Buttons::A))
	{
		handler.Translate(Vector3(slowDownFactor*2, 0., 0.));
		transformed = true;
	}
	if(controller_.AllPressed((int)xbox::Buttons::B))
	{
		handler.Translate(Vector3(-slowDownFactor*2, 0., 0.));
		transformed = true;
	}
	if(controller_.AllPressed((int)xbox::Buttons::RightShoulder))
	{
		Simulation::GetInstance()->pRobot->Rest();
		transformed = true;
	}
	if(controller_.AllPressed((int)xbox::Buttons::Y))
	{
		Simulation::GetInstance()->Reset();
    }
#endif
}

void XboxMotion::Reset()
{
}
