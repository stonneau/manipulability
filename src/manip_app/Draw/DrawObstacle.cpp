
#include "DrawObstacle.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;
#include "Simulation.h"

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
//#include <GL/glui.h>

using namespace matrices;


DrawObstacle::DrawObstacle(const matrices::Vector3& p1, const matrices::Vector3& p2, const matrices::Vector3& p3, const matrices::Vector3& p4)
	: transparency_(1.f)
	, texture_(0)
{
	Init(p1, p2, p3, p4);
	r_ = 1.f;
	g_ = 1.f;
	bl_ = 1.f;
}

DrawObstacle::DrawObstacle(const matrices::Vector3& p1, const matrices::Vector3& p2, const matrices::Vector3& p3, const matrices::Vector3& p4, float* color, float transparency, int texture)
	: transparency_(transparency)
	, texture_(texture)
{
	Init(p1, p2, p3, p4);
	r_ = color[0];
	g_ = color[1];
	bl_ = color[2];
}

void DrawObstacle::Init(const matrices::Vector3& p1, const matrices::Vector3& p2, const matrices::Vector3& p3, const matrices::Vector3& p4)
{
	
	matrices::vect3ToArray(a_,p1);
	matrices::vect3ToArray(b_,p2);
	matrices::vect3ToArray(c_,p3);
	matrices::vect3ToArray(d_,p4);
	
	float w_ = (float)((p3 - p4).norm());
	float h_ = (float)((p1 - p4).norm());
	matrices::Vector3 center = p1 + ((p4 - p1) + (p2 - p1)) / 2 ;
	float offset = 0;
	if(Simulation::GetInstance()->simpParams_.obstacleOffset_)
	{
		offset = w_;
	}
	pos_[0] = center.x() + offset / 2;
	pos_[1] = center.y(); // - offset / 2;
	pos_[2] = center.z();// - 0.01 / 2;
	center_[0] = center.x();
	center_[1] = center.y();
	center_[2] = center.z();

	sides_[0] = w_ + 0.01;
	sides_[1] = h_ + 0.01;
	//sides_[2] =  0.03f;
	sides_[2] =  offset;

	Vector3 normal = (p3 - p4).cross(p1 - p4);
	
	Matrix4 basis_ = Matrix4::Zero();
	Vector3 x = (p3 - p4); x.normalize();
	Vector3 y = (p1 - p4); y.normalize();
	normal.normalize();
	basis_.block(0,0,3,1) = x;
	basis_.block(0,1,3,1) = y;
	basis_.block(0,2,3,1) = normal;
	basis_.block(0,3,3,1) = p4;
	basis_(3,3) = 1;
	
	matrices::matrixToArray(R_, basis_);

	// sphere related stuff	
	radius_ = std::min(w_, h_) / 2.f + 0.05;
	matrices::matrixID(ID_);

	posSphere_[0] = center.x() + radius_;
	posSphere_[1] = center.y();
	posSphere_[2] = center.z();
	normal= normal / 3;
	normal_[0] = normal.x() + center_[0];
	normal_[1] = normal.y() + center_[1];
	normal_[2] = normal.z() + center_[2];

}

DrawObstacle::~DrawObstacle()
{
	// TODO
}

void DrawObstacle::Draw() const
{
//	dsSetColor(1.0f, 1.0f, 1.0f);
	//dsSetColor(r_, g_, bl_);
	dsSetColorAlpha(r_, g_, bl_,transparency_);
	dsSetTexture(texture_);
	//dsSetColorAlpha (7.f / 255.f, 188.f / 255.f, 248.f / 255.f, 0.3f);
	dsDrawBox (pos_, R_, sides_);
	dsSetColorAlpha(0,0, 0,transparency_);
	dsDrawLineD(a_, b_);
	dsDrawLineD(b_, c_);
	dsDrawLineD(c_, d_);
	dsDrawLineD(a_, d_);
	dsSetColorAlpha(r_, g_, bl_,transparency_);

	// renforce les arr�ts
	if(Simulation::GetInstance()->simpParams_.drawNormals_)
	{
		dsSetColorAlpha(1.f, 0, 0,0.7);
		dsDrawLine(center_, normal_);
		dsSetColorAlpha(r_, g_, bl_,transparency_);
	}
	dsSetColorAlpha(r_, g_, bl_,1);
}

void DrawObstacle::DrawWithTexture() const
{
	float id[12];
	matrices::matrixID(id);
	dsSetColor(1.0f, 1.0f, 1.0f);
	dsSetColor(r_, g_, bl_);
	dsDrawSphere(posSphere_,id,radius_);
	/*dsSetColor(1.0f, 0.f, 1.0f);
	glBegin(GL_QUADS);
	glNormal3d(1, 0, 0);
	glTexCoord2f(0, 1);
	glVertex3dv(a_);
	glTexCoord2f(0, 0);
	glVertex3dv(b_);
	glTexCoord2f(1, 0);
	glVertex3dv(c_);
	glTexCoord2f(1, 1);
	glVertex3dv(d_);
	glEnd();*/
}

void DrawObstacle::DrawRed() const
{
	dsSetColor(1.0f, 0.78f, 0.05f);
	dsDrawBox (pos_, R_, sides_);
}
