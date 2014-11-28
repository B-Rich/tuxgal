#include <stdio.h>
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#include "tuxGlutBtApplication.h"

static bool gForward = 0;
static bool gBackward = 0;
static bool gLeft = 0;
static bool gRight = 0;
static bool gJump = 0;

tuxGlutBtApplication::tuxGlutBtApplication()
    : m_cameraHeight(4.) {
    m_cameraPosition = btVector3(30, 30, 30);
}

tuxGlutBtApplication::~tuxGlutBtApplication() {
    delete m_world;
}

void tuxGlutBtApplication::addCharacterObject(
    btVector3 pos,
    btScalar width,
    btScalar height,
    btScalar mass,
    btScalar friction 
    ) {

    tuxCharacterObject *object = new tuxCharacterObject(
        pos,
        width,
        height,
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
    m_world->setGravityCenter(gravityCenter);

    tuxPlanetObject *planet = new tuxPlanetObject(gravityCenter, 40);
    if (planet) {
        m_world->addObject(planet);
    }

    tuxCharacterObject *player = new tuxCharacterObject(btVector3(0, 0, 0));
    if (player) {
        m_world->addObject(player);
        m_player = player;
    }

    addCharacterObject(btVector3(10, 0, 0), 1, 5, 1, 0.5);
    addCharacterObject(btVector3(-10, 0, 0), 1, 5, 1, 0.5);
    addCharacterObject(btVector3(0, 0, 10), 1, 5, 1, 0.5);
    addCharacterObject(btVector3(0, 0, -10), 1, 5, 1, 0.5);

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

        //set walkDirection for our character
        btTransform xform;
        btCollisionObject *player = m_player->getCollisionObject();
        xform = player->getWorldTransform ();

        btVector3 forwardDir = xform.getBasis()[2];
        //printf("forwardDir=%f,%f,%f\n",forwardDir[0],forwardDir[1],forwardDir[2]);
        btVector3 upDir = xform.getBasis()[1];
        btVector3 strafeDir = xform.getBasis()[0];
        forwardDir.normalize();
        upDir.normalize();
        strafeDir.normalize();

        btVector3 walkDirection = btVector3(0.0, 0.0, 0.0);
        btScalar walkVelocity = btScalar(1.1) * 4.0; // 4 km/h -> 1.1 m/s
        btScalar walkSpeed = walkVelocity;// * dt;

        //rotate view
        if (gLeft) {
            btMatrix3x3 orn = player->getWorldTransform().getBasis();
            orn *= btMatrix3x3(btQuaternion(btVector3(0, 1, 0), 0.01));
            player->getWorldTransform().setBasis(orn);
        }

        if (gRight) {
            btMatrix3x3 orn = player->getWorldTransform().getBasis();
            orn *= btMatrix3x3(btQuaternion(btVector3(0, 1, 0), -0.01));
            player->getWorldTransform().setBasis(orn);
        }

        if (gForward) {
            walkDirection += forwardDir;
            m_player->getBody()->setLinearVelocity(walkDirection * walkSpeed);
        }

        if (gBackward) {
            walkDirection -= forwardDir;
            m_player->getBody()->setLinearVelocity(walkDirection * walkSpeed);
        }

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

