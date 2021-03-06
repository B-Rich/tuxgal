#include <stdio.h>
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#include "tuxBoxObject.h"
#include "tuxGlutBtApplication.h"

static bool gForward = false;
static bool gBackward = false;
static bool gLeft = false;
static bool gRight = false;
static bool gJump = false;

tuxGlutBtApplication::tuxGlutBtApplication()
    : m_cameraHeight(4.0) {
    m_cameraPosition = btVector3(30.0, 30.0, 30.0);
}

tuxGlutBtApplication::~tuxGlutBtApplication() {
    delete m_world;
}

void tuxGlutBtApplication::addCubeObject(
    btVector3 pos,
    btScalar size,
    btScalar mass,
    btScalar friction 
    ) {

    tuxBoxObject *object = new tuxBoxObject(
        pos,
        size,
        size,
        size,
        mass,
        friction
        );
    if (object) {
        m_world->addObject(object);
    }
}

void tuxGlutBtApplication::initPhysics() {
    m_world = new tuxWorld;
    m_world->init();
    m_dynamicsWorld = m_world->getDynamicsWorld();

    btVector3 gravityCenter(btVector3(0, -20, 0));

    tuxPlanetObject *planet = new tuxPlanetObject(gravityCenter, 40);
    if (planet) {
        m_world->setStaticObject(planet);
    }

    tuxCharacterObject *player = new tuxCharacterObject(btVector3(0, 0, 0));
    if (player) {
        m_world->addObject(player);
        player->getBody()->setActivationState(DISABLE_DEACTIVATION);
        m_player = player;
    }

    addCubeObject(btVector3(10, 0, 0), 3, 1, 0.5);
    addCubeObject(btVector3(-10, 0, 0), 3, 1, 0.5);
    addCubeObject(btVector3(0, 0, 10), 3, 1, 0.5);
    addCubeObject(btVector3(0, 0, -10), 3, 1, 0.5);

    setCameraDistance(56.0);
}

void tuxGlutBtApplication::renderme() {
    updateCamera();
    BtApplication::renderme();
}

void tuxGlutBtApplication::clientMoveAndDisplay()
{
    btDynamicsWorld *dynamicsWorld = m_world->getDynamicsWorld();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float dt = getDeltaTimeMicroseconds() * 0.000001;

    if (dynamicsWorld) {
        m_world->applyGravity();

        //during idle mode, just run 1 simulation step maximum
        int maxSimSubSteps = m_idle ? 1 : 2;
        if (m_idle) {
            dt = 1.0 / 420.0;
        }

        //set walk speed for our character
        btScalar walkSpeed = btScalar(1.1) * 4.0;// * dt;

        //rotate view
        if (gLeft) {
            m_player->turn(0.01);
        }

        if (gRight) {
            m_player->turn(-0.01);
        }

        if (gForward) {
            m_player->move(-walkSpeed);
        }

        else if (gBackward) {
            m_player->move(walkSpeed);
        }

        else {
            m_player->move(0.0);
        }

        if (gJump) {
            m_player->jump(10);
            gJump = false;
        }

        m_player->applyMovement();

        int numSimSteps = dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

        //optional but useful: debug drawing
        if (dynamicsWorld) {
            dynamicsWorld->debugDrawWorld();
        }

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
        if (!numSimSteps) {
            printf("Interpolated transforms\n");
        }
        else {
            if (numSimSteps > maxSimSubSteps) {
                //detect dropping frames
                printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
            }
            else {
                printf("Simulated (%i) steps\n",numSimSteps);
            }
        }
#endif //VERBOSE_FEEDBACK

    }

#ifdef USE_QUICKPROF
    btProfiler::beginBlock("render");
#endif //USE_QUICKPROF

    renderme();

#ifdef USE_QUICKPROF
    btProfiler::endBlock("render");
#endif

    glFlush();
    glutSwapBuffers();
}

void tuxGlutBtApplication::displayCallback()
{
    btDynamicsWorld *dynamicsWorld = m_world->getDynamicsWorld();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    renderme();

    //optional but useful: debug drawing
    if (dynamicsWorld) {
        dynamicsWorld->debugDrawWorld();
    }

    glFlush();
    glutSwapBuffers();
}

void tuxGlutBtApplication::specialKeyboard(int key, int x, int y)
{
    switch (key) {
        case GLUT_KEY_UP:
            gForward = true;
            break;

        case GLUT_KEY_DOWN:
            gBackward = true;
            break;

        case GLUT_KEY_LEFT:
            gLeft = true;
            break;

        case GLUT_KEY_RIGHT:
            gRight = true;
            break;

        case GLUT_KEY_F1:
            gJump = true;
            break;

        default:
            BtApplication::specialKeyboard(key,x,y);
            break;
    }
}

void tuxGlutBtApplication::specialKeyboardUp(int key, int x, int y)
{
   switch (key) {
       case GLUT_KEY_UP:
            gForward = false;
            break;

       case GLUT_KEY_DOWN:
            gBackward = false;
            break;

       case GLUT_KEY_LEFT:
            gLeft = false;
            break;

       case GLUT_KEY_RIGHT:
            gRight = false;
            break;

        default:
            BtApplication::specialKeyboardUp(key,x,y);
            break;
    }
}

void tuxGlutBtApplication::updateCamera()
{
//#define DISABLE_CAMERA 1
#ifdef DISABLE_CAMERA
    BtApplication::updateCamera();
    return;
#endif //DISABLE_CAMERA

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    btTransform characterWorldTrans;

    //look at the vehicle
    btCollisionObject *player = m_player->getCollisionObject();
    characterWorldTrans = player->getWorldTransform();
    btVector3 up = characterWorldTrans.getBasis()[1];
    btVector3 backward = -characterWorldTrans.getBasis()[2];
    up.normalize();
    backward.normalize();

    m_cameraTargetPosition = characterWorldTrans.getOrigin();
    m_cameraPosition = m_cameraTargetPosition + up * 10.0 + backward * 12.0;

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

    m_dynamicsWorld->convexSweepTest(&cameraSphere, cameraFrom, cameraTo, cb);
    if (cb.hasHit()) {

        btScalar minFraction  = cb.m_closestHitFraction;
        m_cameraPosition.setInterpolate3(
            cameraFrom.getOrigin(),cameraTo.getOrigin(),minFraction
            );
    }

    //update OpenGL camera settings
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(
        m_cameraPosition[0],
        m_cameraPosition[1],
        m_cameraPosition[2],
        m_cameraTargetPosition[0],
        m_cameraTargetPosition[1],
        m_cameraTargetPosition[2],
        m_cameraUp.getX(),
        m_cameraUp.getY(),
        m_cameraUp.getZ()
        );
}

