
#ifndef _CLASS_WORLDPARSER
#define _CLASS_WORLDPARSER

#include "ManipManager.h"
#include "MatrixDefs.h"

#include <xeumeuleu/xml.hpp>

#include <string>

/*<world>
  <robot type="0" x="-8" y="1" z="3">
	<autorotate val="true"> optional
	<rotation x="-90" y="0" z="0"/> optional
	<tree id="0"> optional
		<joint id="0" angle="0"/>
		<joint id="1" angle="0"/>
		<joint id="2" angle="0"/>
		<joint id="3" angle="0"/>
	</tree>
  </robot>
  <obstacles>
	<obstacle> 
		<p0 x="" y="" z=""/>
		<p1 x="" y="" z=""/>
		<p2 x="" y="" z=""/>
		<p3 x="" y="" z=""/>
	</obstacle>
	...
	<obstacle> 
		<p0 x="" y="" z=""/>
		<p1 x="" y="" z=""/>
		<p2 x="" y="" z=""/>
		<p3 x="" y="" z=""/>
	</obstacle>
	...
	<verticalChess> 
		<depth x="" y="" z=""/>
		<p0 x="" y="" z=""/>
		<p3 x="" y="" z=""/>
	</verticalChess>
	...
	<prise> 
		<p0 x="" y="" z=""/>
	</prise>
  </obstacles>
</world>*/
struct WorldParser
{
public:
	 WorldParser();
	~WorldParser();

	void CreateWorld(const std::string& filename);

private:
	typedef std::vector<std::pair<double, matrices::Vector3> > T_Waypoints;
private:
	void CreateRobot       (const int /*type*/, const float /*x*/, const float /*y*/, const float /*z*/
											  , const float /*rx*/, const float /*ry*/, const float /*rz*/) const;
	void CreateRobot       (const std::string& /*type*/, const float /*x*/, const float /*y*/, const float /*z*/
											  , const float /*rx*/, const float /*ry*/, const float /*rz*/) const;
	void CreateObstacle    (const std::string& /*name*/, xml::xistream& /*xis*/) const;
	void ConfigureTree     (const std::string& /*name*/, xml::xistream& /*xis*/);
	void ConfigureTreeJoint(const std::string& /*name*/, xml::xistream& /*xis*/, int tree);
	void AddRootTrajectoryPoint(const std::string& /*name*/, xml::xistream& /*xis*/, T_Waypoints& /*splinePoints*/) const;

private:
	manip_core::ManipManager& manager_;
	typedef std::vector<double> T_JointValues;
	typedef std::vector<T_JointValues> T_TreeValues;
	T_TreeValues values_;
};

#endif //_CLASS_WORLDPARSER
