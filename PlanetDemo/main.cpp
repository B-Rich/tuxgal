
#include "PlanetDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

        PlanetDemo* planetDemo = new PlanetDemo;

        planetDemo->initPhysics(); 
		planetDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,800,600,"Bullet Character Demo. http://www.continuousphysics.com/Bullet/phpBB2/", planetDemo);
}

