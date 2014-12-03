#include <iostream>

#include "OgreRoot.h"
#include "OgreRenderSystem.h"
#include "OgreRenderWindow.h"
#include "OgreWindowEventUtilities.h"
#include "OgreMeshManager.h"
#include "OgreSceneNode.h"
#include "OgreEntity.h"
#include "tuxOgreApplication.h"

int main(int argc, char *argv[])
{
    float start_x = .0;
    float start_y = .0;
    float start_z = .0;

    if (argc > 1) {
        start_x = atof(argv[1]);
        std::cout << "start_x: " << start_x << std::endl;
    }
    else {
        start_y = 400.0;
    }

    if (argc > 2) {
        start_y = atof(argv[2]);
        std::cout << "start_y: " << start_y << std::endl;
    }

    if (argc > 3) {
        start_z = atof(argv[3]);
        std::cout << "start_z: " << start_z << std::endl;
    }

    tuxOgreApplication *app = new tuxOgreApplication("Gravity Demo", 800, 600);

    if (!app->initOgre()) {
        std::cerr << "Error initialzing Ogre" << std::endl;
        return 1;
    }

    app->initScene();
    app->initBasicLight();
    app->initInput();

    app->addGroup("Mesh", ".");
    app->initPlanet();

    Ogre::Root *lRoot = app->getRoot();
    Ogre::RenderWindow *lWindow = app->getWindow();
    lRoot->addFrameListener(app);

    lRoot->clearEventTimes();

    while (!lWindow->isClosed()) {
        lWindow->update(false);
        lWindow->swapBuffers();
        lRoot->renderOneFrame();
        Ogre::WindowEventUtilities::messagePump();
    }

    delete app;

    return 0;
}


