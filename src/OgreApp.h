#include <OgreRoot.h>
#include <Bites/OgreApplicationContext.h>
#include <Bites/OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>
#include <OgreTimer.h>
#include <OgreManualObject.h>

#include <pcl/PolygonMesh.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <eos/core/Mesh.hpp>

class OgreApp : public OgreBites::ApplicationContext, public OgreBites::InputListener
{
public:
	OgreApp();

	Ogre::SceneManager* mScene;

	void setup(void);
	void add_mesh(pcl::PolygonMesh mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void add_mesh(eos::core::Mesh mesh);
	void update_mesh(eos::core::Mesh mesh);
	void add_or_update_mesh(eos::core::Mesh mesh);

private:
	bool keyPressed(const OgreBites::KeyboardEvent& evt);

	Ogre::Root *mRoot;
};