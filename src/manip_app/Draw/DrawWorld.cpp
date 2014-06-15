
#include "DrawWorld.h"
#include "DrawObstacle.h"

#include "world/World.h"
#include "MatrixDefs.h"


#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include <vector>

using namespace std;
using namespace matrices;

struct PImpl
{
	PImpl()
//		: textHandler_(std::string("../textures"))
	{
		//NOTHING
	}

	~PImpl()
	{
		//NOTHING
	}

	typedef vector<DrawObstacle> T_Obstacle;
	typedef T_Obstacle::iterator T_ObstacleIT;
	typedef T_Obstacle::const_iterator T_ObstacleCIT;
	T_Obstacle drawObstacles_;
	T_Obstacle drawWalls_;
    T_Obstacle drawGround_;
};



//TODO listener in case of adding new trees
DrawWorld::DrawWorld(manip_core::ManipManager& manager)
	: pImpl_(new PImpl)
{
	manager.RegisterObstacleCreatedListenerI(this);
}

DrawWorld::~DrawWorld()
{
	// TODO
}

void DrawWorld::Draw() const
{
	//pImpl_->textHandler_.EnableTexture(textures::wood);

	//dsSetTexture(0);
	dsSetTexture(0);

	for(PImpl::T_ObstacleCIT it = pImpl_->drawGround_.begin(); it!= pImpl_->drawGround_.end(); ++it)
	{
		it->Draw();
	}
	//dsSetTexture(0);
	for(PImpl::T_ObstacleCIT it = pImpl_->drawWalls_.begin(); it!= pImpl_->drawWalls_.end(); ++it)
	{
		it->Draw();
	}
	//pImpl_->textHandler_.DisableTextures();
	//dsSetTexture(2);
	//dsSetTexture(0);
//	dsSetColorAlpha (0, 0, 1, 1);
	//pImpl_->textHandler_.EnableTexture(textures::wood);
	for(PImpl::T_ObstacleCIT it = pImpl_->drawObstacles_.begin(); it!= pImpl_->drawObstacles_.end(); ++it)
	{
		it->Draw();
		//it->DrawWithTexture();
	}
	//dsSetTexture(0);
}

void DrawWorld::OnObstacleCreated(const matrices::Vector3& p1, const matrices::Vector3& p2, const matrices::Vector3& p3, const matrices::Vector3& p4, float* color, const float transparency, const int texture)
{
	pImpl_->drawObstacles_.push_back(DrawObstacle(p1, p2, p3, p4, color, transparency, texture));
}

void DrawWorld::OnWallCreated(const matrices::Vector3& p1, const matrices::Vector3& p2, const matrices::Vector3& p3, const matrices::Vector3& p4, float* color, const float transparency, const int texture)
{
	pImpl_->drawWalls_.push_back(DrawObstacle(p1, p2, p3, p4, color, transparency, texture));
}

void DrawWorld::OnGroundCreated(const matrices::Vector3& p1, const matrices::Vector3& p2, const matrices::Vector3& p3, const matrices::Vector3& p4, float* color, const float transparency, const int texture)
{
	pImpl_->drawGround_.push_back(DrawObstacle(p1, p2, p3, p4, color, transparency, texture));
}

