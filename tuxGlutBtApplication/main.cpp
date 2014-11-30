#include "tuxGlutBtApplication.h"
#include "GLDebugDrawer.h"

GLDebugDrawer gDebugDrawer;

int main(int argc,char** argv) {

    tuxGlutBtApplication* app = new tuxGlutBtApplication;

    app->initPhysics();
    app->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

    return glutmain(argc, argv, 800, 600,"Tux Galaxy", app);
}

