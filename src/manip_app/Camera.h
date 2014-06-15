
#ifndef _CLASS_CAMERA
#define _CLASS_CAMERA

#include "MatrixDefs.h"
#include "drawstuff/drawstuff.h"

#include "API/RobotI.h"

struct Camera_ABC
{
public:
	Camera_ABC(const matrices::Vector3& xyz, const matrices::Vector3& hpr)
	: free_(false)
	{
		matrices::vect3ToArray(xyz_, xyz);
		matrices::vect3ToArray(xyzInit_, xyz);
		matrices::vect3ToArray(hpr_, hpr);
		matrices::vect3ToArray(hprInit_, hpr);
	}

	~Camera_ABC()
	{
		// NOTHING
	}

	virtual void OnCharacterMotion(const matrices::Vector3& xyz)
	{	
		// NOTHING
	}
	
	virtual void ToggleFree()
	{	
		free_ = !free_;
	}

	virtual void Update()
	{	
		//dsSetViewpoint(xyz_, hpr_);
	}

	virtual void DrawArrow()
	{

	}


	virtual void Reset()
	{	
		if(!free_)
		{
			for(int i =0; i <3; ++i)
			{
				xyz_[i] = xyzInit_[i];
				hpr_[i] = hprInit_[i];
			}
			dsSetViewpoint(xyzInit_, hprInit_);
		}
	}

	float xyz_[3];
	float hpr_[3];
	float xyzInit_[3];
	float hprInit_[3];
	bool free_;
};

struct CameraFollow : public Camera_ABC
{
public:
	CameraFollow(const matrices::Vector3& xyz, const matrices::Vector3& hpr);

	~CameraFollow()
	{
		// NOTHING
	}

	virtual void OnCharacterMotion(const matrices::Vector3& xyz)
	{	
		moved_ = true;
		for(int i = 0; i < 3; ++i)
		{
			xyz_[i] += (float)xyz(i);
		}
	}

	virtual void Update()
	{	
		if(moved_ && !free_)
		{
			dsSetViewpoint(xyz_, hpr_);
			moved_ = false;
		}
	}
		
	bool moved_;
	float arrowOffset_ [3];
	float idMatrix[12];
	float arrow_[3];
	float arrowExtr_[3];
	float v1_[3];
	float v2_[3];
	float v3_[3];
	matrices::Vector3 unitz;
	matrices::Vector3 oldDir_;
	matrices::Vector3 screenArrowPosition_;
	
	virtual void DrawArrow();
	
};

#endif //_CLASS_CAMERA