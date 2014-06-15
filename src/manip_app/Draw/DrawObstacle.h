
#ifndef _CLASS_DRAWOBSTACLE
#define _CLASS_DRAWOBSTACLE

#include <vector>
#include "MatrixDefs.h"

class Obstacle;

class DrawObstacle {

public:
	typedef std::vector<DrawObstacle> T_DrawObstacle;


public:
	 DrawObstacle(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/);
	 DrawObstacle(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/, float* color, float transparency = 1.f, int texture = 0);
	~DrawObstacle();
	
	 void DrawWithTexture()const;
	 void Draw()const;
	 void DrawRed()const;

private:
	void Init(const matrices::Vector3& /*upLeft*/, const matrices::Vector3& /*upRight*/, const matrices::Vector3& /*downRight*/, const matrices::Vector3& /*downLeft*/);

private:
	float pos_[3];
	float center_[3];
	float normal_[3];
	float r_, g_, bl_;
	float transparency_;
	float posSphere_[3];
	double a_[3];
	double b_[3];
	double c_[3];
	double d_[3];
	float R_[12];
	float ID_[12];
	float radius_;
	float sides_[3];
	int texture_;
}; // class DrawObstacle

#endif //_CLASS_DRAWOBSTACLE