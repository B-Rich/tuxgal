#ifndef TUX_OGRE_APPLICATION
#define TUX_OGRE_APPLICATION

#include "OgreFrameListener.h"
#include "tuxInputManager.h"
#include "tuxGeoCoords.h"
#include "tuxWorld.h"
#include "tuxCharacterObject.h"

class tuxOgreApplication : public Ogre::FrameListener {
public:
    tuxOgreApplication(
        Ogre::String windowTitle,
        unsigned int sizeX,
        unsigned int sizeY,
        bool         fullScreen = false
        )
        : mWindowTitle(windowTitle),
          mSizeX(sizeX), mSizeY(sizeY),
          mFullScreen(fullScreen),
          mRoot(0), mWindow(0), mScene(0), mCamera(0),
          m_cameraPosition(30.0, 30.0, 30.0),
          m_cameraHeight(4.0) { }

    Ogre::SceneManager* getSceneManager() const { return mScene; }
    Ogre::Root* getRoot() { return mRoot; }
    Ogre::RenderWindow* getWindow() { return mWindow; }

    bool initOgre();
    void initScene();
    void initBasicLight();
    bool initInput();
    void initPlanet();

    void addGroup(Ogre::String name, Ogre::String dir);
    Ogre::SceneNode* loadMesh(Ogre::String name);

    void movePlayer(btScalar walkSpeed, btScalar turnSpeed, btScalar jumpSpeed);

    void updateCamera();

    virtual bool frameStarted(const Ogre::FrameEvent& evt);

private:
    btScalar getDeltaTimeMicroseconds() {
        btScalar dt = (btScalar) m_clock.getTimeMicroseconds();
        m_clock.reset();
        return dt;
    }

    void addCubeObject(
        Ogre::String name,
        btVector3    pos,
        btScalar     size,
        btScalar     mass,
        btScalar     friction
        );

    void addCubeObject(
        Ogre::String name,
        tuxGeoCoords pos,
        btScalar     size,
        btScalar     mass,
        btScalar     friction
        ) {
        addCubeObject(name, static_cast<btVector3>(pos), size, mass, friction);
    }

    Ogre::String mWindowTitle;
    unsigned int mSizeX, mSizeY;
    bool mFullScreen;
    Ogre::Root *mRoot;
    Ogre::RenderWindow *mWindow;
    Ogre::SceneManager *mScene;
    Ogre::Camera *mCamera;

    btVector3 m_cameraPosition;
    btVector3 m_cameraTargetPosition;
    btScalar m_cameraHeight;

    btClock m_clock;

    Ogre::AnimationState *m_playerAnimState;

    tuxInputManager *m_inputManager;

    tuxWorld *m_world;
    tuxCharacterObject *m_player;
};

#endif // TUX_OGRE_APPLICATION

