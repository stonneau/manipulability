
#ifndef _CLASS_TREE_FACTORY
#define _CLASS_TREE_FACTORY

#include "MatrixDefs.h"
#include "kinematic/Tree.h"
#include "kinematic/Enums.h"


namespace factories{

struct TreeFactoryPimpl;

class TreeFactory {
public:
	  TreeFactory();
	 ~TreeFactory();

public:
	Tree* CreateTree(const manip_core::enums::eMembers /*member*/, const matrices::Vector3& /*rootPosition*/, const Tree::TREE_ID /*id*/) const;

private:
	std::auto_ptr<TreeFactoryPimpl> pImpl_;
	 
}; // ComFactory

} // namespace factories

#endif // _CLASS_TREE_FACTORY