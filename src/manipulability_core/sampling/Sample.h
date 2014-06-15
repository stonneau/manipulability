
#ifndef _CLASS_SAMPLE
#define _CLASS_SAMPLE

#include <vector>
#include "MatrixDefs.h"

#include "kinematic/Jacobian.h"
#include "Exports.h"


class Tree;

class Sample {

public:
	typedef std::vector<NUMBER> LAngles;

public:
	Sample(Tree& /*tree*/);
	//Sample(Jacobian& /*jacobian*/, const matrices::Vector3& /*position*/);
	~Sample();

private:
//	Sample& Sample::operator =(const Sample&);
//	Sample(const Sample&);

public:
	const LAngles& AngleValues(){return angles_;}
	void LoadIntoTree(Tree& /*tree*/) const;
	const matrices::Vector3& GetPosition() const;
	
	NUMBER velocityManipulabiliy(const matrices::Vector3& /*direction*/) const;
	NUMBER forceManipulabiliy   (const matrices::Vector3& /*direction*/) const ;

private:
	matrices::Matrix3 jacobianProd_;
	matrices::Matrix3 jacobianProdInverse_;
	LAngles angles_;
	matrices::Vector3 position_;

private:
};

#endif //_CLASS_SAMPLE