#include "ManipManager.h"
#include "PostureManager.h"

#include "API/PostureManagerI.h"

using namespace manip_core;
using namespace matrices;

ManipManager::ManipManager()
	: pWorldManager_(GetWorldManager())
	, ikSolver_(pWorldManager_->GetIkConstraintHandlerI())
	, pPostureManager_(0)
	, transparency_(1.f)
	, texture_(0)
{
	color_[0] = 0.5f;
	color_[1] = 0.5;
	color_[2] = 0.5;
}

ManipManager::~ManipManager()
{
	pWorldManager_->Release();
	if(pPostureManager_ != 0)
	{
		delete (pPostureManager_);
	}
}

void ManipManager::SetNextColor(const float r, const float g, const float b)
{
	color_[0] = r;
	color_[1] = g;
	color_[2] = b;
}

void ManipManager::SetNextTexture(const int t)
{
	texture_ = t;
}

void ManipManager::AddObstacle(const matrices::Vector3& upLeft, const matrices::Vector3& upRight, const matrices::Vector3& downRight, const matrices::Vector3& downLeft)
{
	double p1[3];double p2[3];double p3[3];double p4[3];
	matrices::vect3ToArray(p1, upLeft);matrices::vect3ToArray(p2, upRight);matrices::vect3ToArray(p3, downRight);matrices::vect3ToArray(p4, downLeft);
	pWorldManager_->AddObstacle(p1, p2, p3, p4);
	for(std::vector<ObstacleVisitor_ABC*>::iterator it = listeners_.begin(); it != listeners_.end(); ++it)
	{
		(*it)->OnObstacleCreated(upLeft, upRight, downRight, downLeft, color_, transparency_, texture_);
	}
}

void ManipManager::AddWall(const matrices::Vector3& upLeft, const matrices::Vector3& upRight, const matrices::Vector3& downRight, const matrices::Vector3& downLeft)
{
	for(std::vector<ObstacleVisitor_ABC*>::iterator it = listeners_.begin(); it != listeners_.end(); ++it)
	{
		(*it)->OnWallCreated(upLeft, upRight, downRight, downLeft, color_, transparency_, texture_);
	}
}

void ManipManager::AddGround(const matrices::Vector3& upLeft, const matrices::Vector3& upRight, const matrices::Vector3& downRight, const matrices::Vector3& downLeft)
{
	double p1[3];double p2[3];double p3[3];double p4[3];
	matrices::vect3ToArray(p1, upLeft);matrices::vect3ToArray(p2, upRight);matrices::vect3ToArray(p3, downRight);matrices::vect3ToArray(p4, downLeft);
	//pWorldManager_->AddObstacle(p1, p2, p3, p4, true);
	for(std::vector<ObstacleVisitor_ABC*>::iterator it = listeners_.begin(); it != listeners_.end(); ++it)
	{
		(*it)->OnGroundCreated(upLeft, upRight, downRight, downLeft, color_, transparency_, texture_);
	}
}

RobotI* ManipManager::CreateRobot(enums::robot::eRobots robotType, const matrices::Matrix4& transform)
{
	double transf[16];
	matrices::matrixTo16Array(transf, transform);
	return pWorldManager_->CreateRobot(robotType, transf);
}

RobotI* ManipManager::CreateRobot(enums::robot::eRobots robotType, const matrices::Matrix4& transform, const manip_core::WorldManagerI::T_TreeValues values)
{
	double transf[16];
	matrices::matrixTo16Array(transf, transform);
	return pWorldManager_->CreateRobot(robotType, transf, values);
}

/*RobotI* ManipManager::CreateRobot(const manip_core::joint_def_t* root , const matrices::Matrix4& transform)
{
	double transf[16];
	matrices::matrixTo16Array(transf, transform);
	return pWorldManager_->CreateRobot(*root, transf);
}

RobotI* ManipManager::CreateRobot(const manip_core::joint_def_t* root, const matrices::Matrix4& transform, const manip_core::WorldManagerI::T_TreeValues values)
{
	double transf[16];
	matrices::matrixTo16Array(transf, transform);
	return pWorldManager_->CreateRobot(*root, transf, values);
}*/

void ManipManager::Initialize(bool collision)
{
	pWorldManager_->Initialize(collision);
}
	
void ManipManager::GenerateChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, NUMBER height, const unsigned int depth)
{
	// stop condition, depth = 0, flat rectangle
	if(0 == depth)
	{
		Vector3 nUL (upLeft.x(), upLeft.y(), height);
		Vector3 nBR (bottomRight.x(), bottomRight.y(), height);
		Vector3 upRight(bottomRight.x(), upLeft.y(), height);
		Vector3 downLeft(upLeft.x(), bottomRight.y(), height);
		AddObstacle(nUL, upRight, nBR, downLeft);
	}
	else
	{
		Vector3 dx((bottomRight.x() - upLeft.x()) / 2., 0, 0);
		Vector3 dy(0, (upLeft.y() - bottomRight.y()) / 2., 0);
		//NUMBER dHeight = (1 == depth) ? 0.3 : 0.;
		unsigned int newDepth = depth - 1;
		GenerateChess(upLeft, upLeft + dx - dy, height, newDepth);
		if(depth != 1)
		{
			GenerateChess(upLeft + dx, bottomRight + dy, height, newDepth);
			GenerateChess(upLeft - dy, bottomRight - dx, height, newDepth);
		}
		GenerateChess(upLeft + dx - dy, bottomRight, height, newDepth);
	}
}

void ManipManager::GenerateUnevenChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, NUMBER height, const unsigned int depth)
{
	// stop condition, depth = 0, flat rectangle
	if(0 == depth)
	{
		Vector3 nUL (upLeft.x(), upLeft.y(), height);
		Vector3 nBR (bottomRight.x(), bottomRight.y(), height);
		Vector3 upRight(bottomRight.x(), upLeft.y(), height);
		Vector3 downLeft(upLeft.x(), bottomRight.y(), height);
		AddObstacle(nUL, upRight, nBR, downLeft);
	}
	else
	{
		Vector3 dx((bottomRight.x() - upLeft.x()) / 2., 0, 0);
		Vector3 dy(0, (upLeft.y() - bottomRight.y()) / 2., 0);
		//NUMBER dHeight = (1 == depth) ? 0.3 : 0.;
		unsigned int newDepth = depth - 1;
		GenerateUnevenChess(upLeft, upLeft + dx - dy, height, newDepth);
		if(depth != 1)
		{
			GenerateUnevenChess(upLeft + dx, bottomRight + dy, height +0.2, newDepth);
			GenerateUnevenChess(upLeft - dy, bottomRight - dx, height -0.2, newDepth);
		}
		GenerateUnevenChess(upLeft + dx - dy, bottomRight, height, newDepth);
	}
}

	
void ManipManager::GenerateVerticalChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, const unsigned int depth)
{
	// stop condition, depth = 0, flat rectangle
	if(0 == depth)
	{
		Vector3 upRight(bottomRight.x(), bottomRight.y(), upLeft.z());
		Vector3 downLeft(upLeft.x(), upLeft.y(), bottomRight.z());
		AddObstacle(upLeft, upRight, bottomRight, downLeft);
	}
	else
	{
		Vector3 dz(0, 0, (upLeft.z() - bottomRight.z()) / 2.);
		Vector3 dy(0, (upLeft.y() - bottomRight.y()) / 2., 0);
		//NUMBER dHeight = (1 == depth) ? 0.3 : 0.;
		unsigned int newDepth = depth - 1;
		GenerateVerticalChess(upLeft, upLeft - dy - dz, newDepth);
		if(depth != 1)
		{
			GenerateVerticalChess(upLeft - dy, bottomRight + dz, newDepth);
			GenerateVerticalChess(upLeft - dz, bottomRight + dy, newDepth);
		}
		GenerateVerticalChess(upLeft - dy - dz, bottomRight, newDepth);
	}
}

void ManipManager::GenerateXInclinedPlank(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight)
{
		Vector3 upRight(bottomRight.x(), upLeft.y(), upLeft.z());
		Vector3 downLeft(upLeft.x(), bottomRight.y(), bottomRight.z());
		AddObstacle(upLeft, upRight, bottomRight, downLeft);
}

void ManipManager::GenerateYInclinedPlank(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight)
{
		Vector3 upRight(bottomRight.x(), upLeft.y(), bottomRight.z());
		Vector3 downLeft(upLeft.x(), bottomRight.y(), upLeft.z());
		AddObstacle(upLeft, upRight, bottomRight, downLeft);
}

void ManipManager::GenerateStair(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, NUMBER heightInit, NUMBER heightFinal, const unsigned int depth)
{
	NUMBER xLength = bottomRight.x() - upLeft.x();
	NUMBER yLength = upLeft.y() - bottomRight.y();
	bool xIsLonger = xLength > yLength;
	NUMBER deltaHeight = (heightFinal - heightInit) / ((NUMBER) depth);
	NUMBER deltaCaseWidth = xIsLonger ? (xLength / ((NUMBER) depth)) : (yLength / ((NUMBER) depth));
	// stop condition, depth = 0, flat rectangle
	for(unsigned int i = 0; i < depth; ++i)
	{
		if(xIsLonger)
		{
			Vector3 uL(upLeft.x() + deltaCaseWidth * i, upLeft.y(), heightInit + i * deltaHeight);
			Vector3 bR(upLeft.x() + deltaCaseWidth * (i+1), bottomRight.y(), heightInit + i * deltaHeight);
			GenerateXInclinedPlank(uL, bR);
		}
		else
		{
			Vector3 uL(upLeft.x()     , bottomRight.y() + deltaCaseWidth * (i + 1), heightInit + i * deltaHeight);
			Vector3 bR(bottomRight.x(), bottomRight.y() + deltaCaseWidth * i      , heightInit + i * deltaHeight);
			GenerateXInclinedPlank(uL, bR);
		}
	}
}

void ManipManager::GenerateStairChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, NUMBER heightInit, NUMBER heightFinal, const unsigned int depth, const unsigned int chessdepth)
{
	NUMBER xLength = bottomRight.x() - upLeft.x();
	NUMBER yLength = upLeft.y() - bottomRight.y();
	bool xIsLonger = xLength > yLength;
	NUMBER deltaHeight = (heightFinal - heightInit) / ((NUMBER) depth);
	NUMBER deltaCaseWidth = xIsLonger ? (xLength / ((NUMBER) depth)) : (yLength / ((NUMBER) depth));
	// stop condition, depth = 0, flat rectangle
	for(unsigned int i = 0; i < depth; ++i)
	{
		if(xIsLonger)
		{
			Vector3 uL(upLeft.x() + deltaCaseWidth * i, upLeft.y(), 0);
			Vector3 bR(upLeft.x() + deltaCaseWidth * (i+1), bottomRight.y(), 0);
			GenerateChess(uL, bR, heightInit + i * deltaHeight, chessdepth);
		}
		else
		{
			Vector3 uL(upLeft.x()     , bottomRight.y() + deltaCaseWidth * (i + 1), 0);
			Vector3 bR(bottomRight.x(), bottomRight.y() + deltaCaseWidth * i      , 0);
			GenerateChess(uL, bR, heightInit + i * deltaHeight, chessdepth);
		}
	}
}

void ManipManager::RegisterObstacleCreatedListenerI(ObstacleVisitor_ABC* listener)
{
	listeners_.push_back(listener);
}

void ManipManager::UnRegisterObstacleCreatedListenerI(ObstacleVisitor_ABC* /*listener*/)
{
	// TODO
}

PostureManager* ManipManager::GetPostureManager()
{
	if(!pPostureManager_)
	{
		pPostureManager_ = new PostureManager(pWorldManager_->GetPostureManager());
		pPostureManager_->AddPostureCriteria(enums::postureCriteria::toeOffBoundary);
		//pPostureManager_->AddPostureCriteria(enums::postureCriteria::toeOffJointLimit);
	}
	return pPostureManager_;
}

const IKSolverApp& ManipManager::GetIkSolver() const
{
	return ikSolver_;
}
