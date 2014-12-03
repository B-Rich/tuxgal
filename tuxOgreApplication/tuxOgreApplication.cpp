#include <memory>
#include <exception>
#include "OgreRoot.h"
#include "OgreRenderSystem.h"
#include "OgreRenderWindow.h"
#include "OgreWindowEventUtilities.h"
#include "OgreManualObject.h"
#include "OgreSceneNode.h"
#include "OgreMeshManager.h"
#include "OgreEntity.h"
#include "btBulletDynamicsCommon.h"
#include "tuxObject.h"
#include "tuxPlanetObject.h"
#include "tuxOgreApplication.h"

bool tuxOgreApplication::initOgre() {
    Ogre::String lConfigFileName = "";
    Ogre::String lPluginsFileName = "";
    Ogre::String lLogFileName = "tuxgal.log";
    mRoot = new Ogre::Root(lConfigFileName, lPluginsFileName, lLogFileName);

    typedef std::vector<Ogre::String> Strings;
    Strings lPluginNames;
    bool lIsInDebugMode = OGRE_DEBUG_MODE;
    lPluginNames.push_back("RenderSystem_GL");
    lPluginNames.push_back("Plugin_ParticleFX");
    //lPluginNames.push_back("Plugin_CgProgramManager");
    lPluginNames.push_back("Plugin_OctreeSceneManager");
    {
        Strings::iterator lIter = lPluginNames.begin();
        Strings::iterator lIterEnd = lPluginNames.end();
        for (; lIter != lIterEnd; lIter++) {
            Ogre::String &lPluginName = (*lIter);
            if (lIsInDebugMode) {
                lPluginName.append("_d");
            }
            mRoot->loadPlugin(lPluginName);
        }
    }

    const Ogre::RenderSystemList &lRenderSystemList = mRoot->getAvailableRenderers();
    if (lRenderSystemList.size() == 0) {
        Ogre::LogManager::getSingleton().logMessage("Sorry, no rendersystem was found.");
        return false;
    }
    Ogre::RenderSystem *lRenderSystem = lRenderSystemList[0];
    mRoot->setRenderSystem(lRenderSystem);

    bool lCreateAWindowAutomatically = false;
    Ogre::String lWindowTitle = "";
    Ogre::String lCustomCapacities = "";
    mRoot->initialise(lCreateAWindowAutomatically, lWindowTitle, lCustomCapacities);

    Ogre::NameValuePairList lParams;
    lParams["FSAA"] = "0";
    lParams["vsync"] = "true";
    mWindow = mRoot->createRenderWindow(mWindowTitle, mSizeX, mSizeY, mFullScreen, &lParams);

    return true;
}

void tuxOgreApplication::initScene() {
    mScene = mRoot->createSceneManager(Ogre::ST_GENERIC, "mySceneManager");
    Ogre::SceneNode *lRootSceneNode = mScene->getRootSceneNode();

    mCamera = mScene->createCamera("myCamera");
    Ogre::SceneNode *lCameraNode = lRootSceneNode->createChildSceneNode("myCameraNode");
    lCameraNode->attachObject(mCamera);

    float lViewportWidth = 0.88f;
    float lViewportHeight = 0.88f;
    float lViewportLeft = (1.0f - lViewportWidth) * 0.5f;
    float lViewportTop = (1.0f - lViewportHeight) * 0.5f;
    unsigned short lMainViewportZOrder = 100;
    Ogre::Viewport *lViewport = mWindow->addViewport(mCamera, lMainViewportZOrder, lViewportLeft, lViewportTop, lViewportWidth, lViewportHeight);
    lViewport->setAutoUpdated(true);
    lViewport->setBackgroundColour(Ogre::ColourValue(1, 0, 1));
    float lRatio = float(lViewport->getActualWidth()) / float(lViewport->getActualHeight());
    mCamera->setAspectRatio(lRatio);
    mCamera->setNearClipDistance(1.5f);
    mCamera->setFarClipDistance(3000.0f);
    mCamera->setPosition(Ogre::Vector3(0, 0, 1000));
    mCamera->lookAt(Ogre::Vector3(0, 0, 0));

    Ogre::Animation::setDefaultInterpolationMode(Ogre::Animation::IM_LINEAR);
    Ogre::Animation::setDefaultRotationInterpolationMode(Ogre::Animation::RIM_LINEAR);

    mWindow->setActive(true);
    mWindow->setAutoUpdated(false);
}

void tuxOgreApplication::initBasicLight() {
    Ogre::SceneNode *lRootSceneNode = mScene->getRootSceneNode();
    Ogre::SceneNode *lLightNode = 0;
    Ogre::Light *lLight = mScene->createLight("pointLight");
    lLight->setType(Ogre::Light::LT_POINT);
    lLight->setPosition(Ogre::Vector3(0, 1500, 2000));
    lLight->setDiffuseColour(0.6f, 0.6f, 0.6f);
    lLight->setSpecularColour(1.0f, 1.0f, 1.0f);
    lLightNode = lRootSceneNode->createChildSceneNode();
    lLightNode->attachObject(lLight);

    Ogre::ColourValue lAmbientColour(0.2f, 0.f, 0.2f, 1.0f);
    mScene->setAmbientLight(lAmbientColour);
}

bool tuxOgreApplication::initInput() {
    bool result = false;

    m_inputManager = tuxInputManager::getSingletonPtr();
    if (m_inputManager) {
        m_inputManager->init(getWindow());
        result = true;
    }

    return result;
}

void tuxOgreApplication::initPlanet() {

    m_world = new tuxWorld;
    m_world->init();

    btVector3 gravityCenter(btVector3(0.0, 0.0, 0.0));
    m_world->setGravityCenter(gravityCenter);

    tuxPlanetObject *planet = new tuxPlanetObject(gravityCenter, 300.0);
    if (planet) {
        Ogre::SceneManager *sceneManager = getSceneManager();
        Ogre::SceneNode *rootNode = sceneManager->getRootSceneNode();
        Ogre::Entity *entity = sceneManager->createEntity(
            "planet",
            "sphere.mesh"
            );
        Ogre::SceneNode *node = rootNode->createChildSceneNode();
        node->attachObject(entity);
        // Mesh has scale 100
        node->scale(3.0, 3.0, 3.0);
        planet->attachNode(node);
        m_world->addObject(planet);
    }

    tuxCharacterObject *player = new tuxCharacterObject(
        btVector3(0.0, 320.0, 0.0),
        10.0,
        10.0
        );
    if (player) {
        Ogre::SceneManager *sceneManager = getSceneManager();
        Ogre::SceneNode *rootNode = sceneManager->getRootSceneNode();
        Ogre::Entity *entity = sceneManager->createEntity(
            "player",
            "penguin.mesh"
            );
        Ogre::SceneNode *node = rootNode->createChildSceneNode();
        node->attachObject(entity);

        m_playerAnimState = entity->getAnimationState("amuse");
        m_playerAnimState->setEnabled(true);
        m_playerAnimState->setLoop(true);

        player->attachNode(node);
        m_world->addObject(player);
        player->getBody()->setActivationState(DISABLE_DEACTIVATION);
        m_player = player;
    }
}

void tuxOgreApplication::addGroup(Ogre::String name, Ogre::String dir) {
    Ogre::ResourceGroupManager &lRgMgr = Ogre::ResourceGroupManager::getSingleton();
    lRgMgr.createResourceGroup(name);
    bool lIsRecursive = false;
    lRgMgr.addResourceLocation(dir, "FileSystem", name, lIsRecursive);
    lRgMgr.initialiseResourceGroup(name);
    lRgMgr.loadResourceGroup(name);
}

Ogre::SceneNode* tuxOgreApplication::loadMesh(Ogre::String name) {
    Ogre::SceneNode *lRootSceneNode = mScene->getRootSceneNode();
    Ogre::Entity *lEntity = mScene->createEntity(name);
    Ogre::SceneNode *lNode = 0;

    if (lEntity) {
        lNode = lRootSceneNode->createChildSceneNode();
        lNode->attachObject(lEntity);
    }

    return lNode;
}

void tuxOgreApplication::updateCamera() {
//#define DISABLE_CAMERA 1
#ifdef DISABLE_CAMERA
    return;
#endif //DISABLE_CAMERA

    btTransform characterWorldTrans;

    //look at the vehicle
    btCollisionObject *player = m_player->getCollisionObject();
    characterWorldTrans = player->getWorldTransform();
    btVector3 up = characterWorldTrans.getBasis()[1];
    btVector3 backward = -characterWorldTrans.getBasis()[2];
    up.normalize();
    backward.normalize();

    m_cameraTargetPosition = characterWorldTrans.getOrigin();
    m_cameraPosition = m_cameraTargetPosition + up * 10.0 + backward * 200.0;

    //use the convex sweep test to find a safe position for the camera (not blocked by static geometry)
    btSphereShape cameraSphere(0.2);
    btTransform cameraFrom, cameraTo;
    cameraFrom.setIdentity();
    cameraFrom.setOrigin(characterWorldTrans.getOrigin());
    cameraTo.setIdentity();
    cameraTo.setOrigin(m_cameraPosition);

    btCollisionWorld::ClosestConvexResultCallback cb(
        characterWorldTrans.getOrigin(), cameraTo.getOrigin()
        );
    cb.m_collisionFilterMask = btBroadphaseProxy::StaticFilter;

    m_world->getDynamicsWorld()->convexSweepTest(
        &cameraSphere,
        cameraFrom,
        cameraTo,
        cb
        );
    if (cb.hasHit()) {

        btScalar minFraction  = cb.m_closestHitFraction;
        m_cameraPosition.setInterpolate3(
            cameraFrom.getOrigin(),cameraTo.getOrigin(),minFraction
            );
    }

    mCamera->setPosition(
        Ogre::Vector3(
            m_cameraPosition[0],
            m_cameraPosition[1],
            m_cameraPosition[2]
            )
        );
    mCamera->lookAt(
        Ogre::Vector3(
            m_cameraTargetPosition[0],
            m_cameraTargetPosition[1],
            m_cameraTargetPosition[2]
            )
        );
}

bool tuxOgreApplication:: frameStarted(const Ogre::FrameEvent& evt) {
    bool result = false;

    OIS::Keyboard *keyboard = m_inputManager->getKeyboard();
    keyboard->capture();
    if (keyboard->isKeyDown(OIS::KC_Q)) {
        exit(0);
    }

    btDynamicsWorld *dynamicsWorld = m_world->getDynamicsWorld();
    if (dynamicsWorld) {
        dynamicsWorld->stepSimulation(1.0 / 60.0);
        m_world->applyGravity();
        m_playerAnimState->addTime(evt.timeSinceLastFrame);
        const btVector3 localForward(0.0, 0.0, -1.0);
        btTransform trans = m_player->getBody()->getWorldTransform();
        btVector3 forwardDir = trans.getBasis() * localForward;
        m_player->getBody()->setLinearVelocity(-1.1 * 16.0 * forwardDir);
        m_world->applyTransform();
        updateCamera();
        result = true;
    }

    return result;
}

