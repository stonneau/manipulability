
#ifndef _CLASS_SIMULATION
#define _CLASS_SIMULATION

#include "MatrixDefs.h"
#include "Pi.h"

#include "spline/exact_cubic.h"
#include "Draw/DrawSpline.h"

#include "TimerHandler.h"
#include "Camera.h"
#include "InputHandler.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "MotionHandler.h"

#include "drawstuff/drawstuff.h"
#include "Draw/DrawRobot.h"
#include "Draw/DrawManager.h"
#include "Draw/DrawTrajectory.h"

#include "API/TreeI.h"
#include "API/RobotI.h"
#include "API/IkConstraintHandlerI.h"

#include <string>

struct SimParams
{
	typedef spline::curve_abc<> curve_abc_t;

	SimParams()
		: nbSamples_(10000)
		, jumpToTarget_(false)
		, autorotate_(false)
		, humanrotate_(false)
		, autorotateleg_(false)
		, drawSplines_(false)
		, drawSky_(false)
		, drawGround_(false)
		, drawNormals_(false)
		, reachCom_(false)
		, reachComY_(false)
		, reachComRotate_(false)
		, closestDistance_(false)
		, obstacleOffset_(false)
		, drawEllipseAxes_(false)
		, drawEllipsoid_(false)
		, planif_(false)
		, drawShadows_(false)
		, robotBasis_(matrices::MatrixX::Identity(4,4))
		, robotType_(manip_core::enums::robot::Human)
		, inputHandler_(0)
		, initDir_(0,0,0)
		, pause_(false)
		, background_(1,1,1)
		, lightX(-1)
		, lightY(0.4)
		, joint_(false)
		, initTimer_(0)
        , ikposture_(false)
		, drawSamples_(false)
		, drawSamplesByOne_(false)
		, drawArms_(true)
		, currentSample_(0)
		, drawManip_(false)
	{
		fn_.path_to_textures = "../textures";
		command = 0;
		camera_ = new Camera_ABC(matrices::Vector3(-13,0,5), matrices::Vector3(0,0,0));
	}

	~SimParams()
	{
		delete camera_;
		if(inputHandler_)
		{
			delete inputHandler_;
		}
	}
	
	typedef std::vector<double> T_JointValues;
	typedef std::vector<T_JointValues> T_TreeValues;

	T_TreeValues angleValues_;
	std::vector<int> locked_;
	int nbSamples_;
	float speed_;
	float lightX;
	float lightY;
	Timer::t_time initTimer_;
	bool gait_;
	bool joint_;
	bool ikposture_;
	bool rotatewithspline_;
	bool rotatewithsplinedir_;
	bool humanrotate_;
	bool jumpToTarget_;
	bool autorotate_;
	bool drawSky_;
	bool drawGround_;
	bool autorotateleg_;
	bool drawSplines_;
	bool rootTrajectory_;
	bool planif_;
	bool closestDistance_;
	bool obstacleOffset_;
	bool splineRoot_;
	bool drawEllipseAxes_;
	bool drawShadows_;
	bool drawEllipsoid_;
	curve_abc_t* rootSpline;
	bool drawNormals_;
	bool reachCom_;
	bool reachComY_;
	bool pause_;
	bool drawSamples_;
	bool drawSamplesByOne_;
	int currentSample_;
	bool reachComRotate_;
	bool drawArms_;
	bool drawManip_;
	matrices::Matrix4 robotBasis_;
	matrices::Vector3 initDir_;
    matrices::Vector3 background_;
	void (*command)(int cmd);
	dsFunctions fn_;
	manip_core::enums::robot::eRobots robotType_;
	InputHandlerABC* inputHandler_;

	void SetCamera(Camera_ABC* camera)
	{
		delete camera_;
		camera_ = camera;
	}

	Camera_ABC* GetCamera()
	{
		return camera_;
	}

	private:
		Camera_ABC* camera_;
};


class Simulation {
	
private:
	  Simulation();
	 ~Simulation();

public:
	void Start(int argc, char *argv[]);
	void Update();
	void Draw();
	void Reset();
	void RegisterHandlers();
	
public:
	TimerHandler timerHandler_;
	manip_core::ManipManager manager_;
	DrawManager drawManager_;
	manip_core::PostureManager* postureManager_;

public:
	manip_core::RobotI* pRobot;
	DrawRobot* dRobot;

public:
	SimParams simpParams_;
	MotionHandler motionHandler_;
	
private:
	static Simulation* instance;
	std::vector<TimerHandled_ABC*> managedTimeListeners_;
	

public:
	static Simulation* GetInstance()
	{
		if(!instance)
		{
			instance = new Simulation();
		}
		return instance;
	}
}; // Timer

#endif //_CLASS_SIMULATION
