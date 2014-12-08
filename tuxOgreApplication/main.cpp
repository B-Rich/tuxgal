#include <iostream>

#include "OgreRoot.h"
#include "OgreRenderSystem.h"
#include "OgreRenderWindow.h"
#include "OgreWindowEventUtilities.h"
#include "OgreMeshManager.h"
#include "OgreSceneNode.h"
#include "OgreEntity.h"
#include "tuxOgreApplication.h"

int main(void)
{
    tuxOgreApplication *app = new tuxOgreApplication("Tux Galaxy", 800, 600);

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


