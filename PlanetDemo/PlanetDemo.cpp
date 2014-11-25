/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "PlanetDemo.h"
#ifdef DYNAMIC_CHARACTER_CONTROLLER
#include "DynamicCharacterController.h"
#else
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#endif

const int maxProxies = 32766;
const int maxOverlap = 65535;

static int gForward = 0;
static int gBackward = 0;
static int gLeft = 0;
static int gRight = 0;
static int gJump = 0;
static float c_friction = 0.01;




PlanetDemo::PlanetDemo()
:
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	m_character = 0;
	m_cameraPosition = btVector3(30,30,30);
}


void PlanetDemo::initPlanet(btVector3 initialPosition, btScalar radius)
{
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(
        initialPosition.getX(),
        initialPosition.getY(),
        initialPosition.getX()
        ));

    btScalar groundMass(0.); //the mass is 0, because the ground is immovable
    btVector3 localGroundInertia(0, 0, 0);

    btCollisionShape *groundShape = new btSphereShape(radius);
    m_collisionShapes.push_back(groundShape);
    btDefaultMotionState *groundMotionState =
        new btDefaultMotionState(groundTransform);

    groundShape->calculateLocalInertia(groundMass, localGroundInertia);

    btRigidBody::btRigidBodyConstructionInfo groundRBInfo(
        groundMass,
        groundMotionState,
        groundShape,
        localGroundInertia
        );
    groundRBInfo.m_friction = c_friction;
    btRigidBody *groundBody = new btRigidBody(groundRBInfo);

    //add the body to the dynamics world
    m_dynamicsWorld->addRigidBody(groundBody);
    //m_dynamicsWorld->addCollisionObject(groundBody);
}

void PlanetDemo::initPlayer(btVector3 initialPosition)
{
    btCollisionShape* newRigidShape = new btCapsuleShape(1.75, 1.75);
    m_collisionShapes.push_back(newRigidShape);

    //set the initial position and transform. We set tranform none
    btTransform startTransform;
    startTransform.setIdentity();

    //set the mass of the object. a mass of "0" means an immovable object
    btScalar mass = 1.0f;
    btVector3 localInertia(0, 0, 0);

    startTransform.setOrigin(initialPosition);
    newRigidShape->calculateLocalInertia(mass, localInertia);

    //actually construct the body and add it to the dynamics world
    btDefaultMotionState *myMotionState =
        new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        mass,
        myMotionState,
        newRigidShape,
        localInertia
        );
    rbInfo.m_friction = c_friction;
    m_player = new btRigidBody(rbInfo);
    m_player->setAngularFactor(0);
    m_player->setRestitution(1);

    m_dynamicsWorld->addRigidBody(m_player);
    //m_dynamicsWorld->addCollisionObject(m_player);
}

void PlanetDemo::initPhysics()
{
	//btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	//m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	btAxisSweep3* sweepBP = new btAxisSweep3(worldMin,worldMax);
	m_overlappingPairCache = sweepBP;

	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration=0.0001f;
	
#ifdef DYNAMIC_CHARACTER_CONTROLLER
	m_character = new DynamicCharacterController ();
#else
	
	btTransform startTransform;
	startTransform.setIdentity ();
	//startTransform.setOrigin (btVector3(0.0, 4.0, 0.0));
	startTransform.setOrigin (btVector3(10.210098,-1.6433364,16.453260));


	m_ghostObject = new btPairCachingGhostObject();
	m_ghostObject->setWorldTransform(startTransform);
	sweepBP->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	btScalar characterHeight=1.75;
	btScalar characterWidth =1.75;
	btConvexShape* capsule = new btCapsuleShape(characterWidth,characterHeight);
	m_ghostObject->setCollisionShape (capsule);
	m_ghostObject->setCollisionFlags (btCollisionObject::CF_CHARACTER_OBJECT);

	btScalar stepHeight = btScalar(0.35);
	m_character = new btKinematicCharacterController (m_ghostObject,capsule,stepHeight);
#endif

	////////////////

	/// Create some basic environment

	m_dynamicsWorld->setGravity(btVector3(0,0,0));
        btVector3 center(0, -30, 0);
        m_gravityCenter = center;
        initPlanet(center, 30);
        initPlayer(btVector3(0, 10, 0));
        m_player->setLinearVelocity(btVector3(3, 3, 3));

	///only collide with static for now (no interaction with dynamic objects)
	m_dynamicsWorld->addCollisionObject(m_ghostObject,btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);

	m_dynamicsWorld->addAction(m_character);


	///////////////

	clientResetScene();

	setCameraDistance(56.f);

}


void PlanetDemo::processPhysics()
{
    btCollisionObjectArray objects = m_dynamicsWorld->getCollisionObjectArray();
    for (int i = 0; i < objects.size(); i++) {

        btCollisionObject *obj = objects[i];
        btRigidBody *body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {

            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);
            btVector3 direction = m_gravityCenter - trans.getOrigin();
            direction.normalize();
            body->setGravity(9.8 * direction);
        }
    }
}

//to be implemented by the demo
void PlanetDemo::renderme()
{
	updateCamera();

	BtApplication::renderme();
}



void	PlanetDemo::debugDrawContacts()
{
//	printf("numPairs = %d\n",m_customPairCallback->getOverlappingPairArray().size());
	{
		btManifoldArray	manifoldArray;
		btBroadphasePairArray& pairArray = m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
		int numPairs = pairArray.size();

		for (int i=0;i<numPairs;i++)
		{
			manifoldArray.clear();

			const btBroadphasePair& pair = pairArray[i];
			
			btBroadphasePair* collisionPair = m_overlappingPairCache->getOverlappingPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);
			if (!collisionPair)
				continue;

			if (collisionPair->m_algorithm)
				collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

			for (int j=0;j<manifoldArray.size();j++)
			{
				btPersistentManifold* manifold = manifoldArray[j];
				for (int p=0;p<manifold->getNumContacts();p++)
				{
					const btManifoldPoint&pt = manifold->getContactPoint(p);

					btVector3 color(255,255,255);
					m_dynamicsWorld->getDebugDrawer()->drawContactPoint(pt.getPositionWorldOnB(),pt.m_normalWorldOnB,pt.getDistance(),pt.getLifeTime(),color);
				}
			}
		}
	}

}

void PlanetDemo::clientMoveAndDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	/* Character stuff &*/
	if (m_character)
	{
		
	}

	debugDrawContacts();


	if (m_dynamicsWorld)
	{
                processPhysics();

		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0/420.f;

		///set walkDirection for our character
		btTransform xform;
		xform = m_ghostObject->getWorldTransform ();

		btVector3 forwardDir = xform.getBasis()[2];
	//	printf("forwardDir=%f,%f,%f\n",forwardDir[0],forwardDir[1],forwardDir[2]);
		btVector3 upDir = xform.getBasis()[1];
		btVector3 strafeDir = xform.getBasis()[0];
		forwardDir.normalize ();
		upDir.normalize ();
		strafeDir.normalize ();

		btVector3 walkDirection = btVector3(0.0, 0.0, 0.0);
		btScalar walkVelocity = btScalar(1.1) * 16.0; // 4 km/h -> 1.1 m/s
		btScalar walkSpeed = walkVelocity * dt;

		//rotate view
		if (gLeft)
		{
			btMatrix3x3 orn = m_ghostObject->getWorldTransform().getBasis();
			orn *= btMatrix3x3(btQuaternion(btVector3(0,1,0),0.01));
			m_ghostObject->getWorldTransform ().setBasis(orn);
		}

		if (gRight)
		{
			btMatrix3x3 orn = m_ghostObject->getWorldTransform().getBasis();
			orn *= btMatrix3x3(btQuaternion(btVector3(0,1,0),-0.01));
			m_ghostObject->getWorldTransform ().setBasis(orn);
		}

		if (gForward)
			walkDirection += forwardDir;

		if (gBackward)
			walkDirection -= forwardDir;	


		m_character->setWalkDirection(walkDirection*walkSpeed);


		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);

		//optional but useful: debug drawing
		if (m_dynamicsWorld)
			m_dynamicsWorld->debugDrawWorld();

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
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



void PlanetDemo::displayCallback(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	debugDrawContacts();

	glFlush();
	glutSwapBuffers();
}

void PlanetDemo::clientResetScene()
{
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_ghostObject->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());

	//TODO: m_character->reset (m_dynamicsWorld);
	m_character->reset ();
	///WTF
	m_character->warp (btVector3(10.210001,-2.0306311,16.576973));
	
}

void PlanetDemo::specialKeyboardUp(int key, int x, int y)
{
   switch (key)
    {
    case GLUT_KEY_UP:
	{
		gForward = 0;
	}
	break;
	case GLUT_KEY_DOWN:
	{
		gBackward = 0;
	}
	break;
	case GLUT_KEY_LEFT:
	{
		gLeft = 0;
	}
	break;
	case GLUT_KEY_RIGHT:
	{
		gRight = 0;
	}
	break;
	default:
		BtApplication::specialKeyboardUp(key,x,y);
        break;
    }
}


void PlanetDemo::specialKeyboard(int key, int x, int y)
{

//	printf("key = %i x=%i y=%i\n",key,x,y);

    switch (key)
    {
    case GLUT_KEY_UP:
	{
		gForward = 1;
	}
	break;
	case GLUT_KEY_DOWN:
	{
		gBackward = 1;
	}
	break;
	case GLUT_KEY_LEFT:
	{
		gLeft = 1;
	}
	break;
	case GLUT_KEY_RIGHT:
	{
		gRight = 1;
	}
	break;
	case GLUT_KEY_F1:
	{
		if (m_character && m_character->canJump())
			gJump = 1;
	}
	break;
	default:
		BtApplication::specialKeyboard(key,x,y);
        break;
    }

//	glutPostRedisplay();


}

void	PlanetDemo::updateCamera()
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
	characterWorldTrans = m_ghostObject->getWorldTransform();
	btVector3 up = characterWorldTrans.getBasis()[1];
	btVector3 backward = -characterWorldTrans.getBasis()[2];
	up.normalize ();
	backward.normalize ();

	m_cameraTargetPosition = characterWorldTrans.getOrigin();
	m_cameraPosition = m_cameraTargetPosition + up * 10.0 + backward * 12.0;
	
	//use the convex sweep test to find a safe position for the camera (not blocked by static geometry)
	btSphereShape cameraSphere(0.2f);
	btTransform cameraFrom,cameraTo;
	cameraFrom.setIdentity();
	cameraFrom.setOrigin(characterWorldTrans.getOrigin());
	cameraTo.setIdentity();
	cameraTo.setOrigin(m_cameraPosition);
	
	btCollisionWorld::ClosestConvexResultCallback cb( characterWorldTrans.getOrigin(), cameraTo.getOrigin() );
	cb.m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
		
	m_dynamicsWorld->convexSweepTest(&cameraSphere,cameraFrom,cameraTo,cb);
	if (cb.hasHit())
	{

		btScalar minFraction  = cb.m_closestHitFraction;//btMax(btScalar(0.3),cb.m_closestHitFraction);
		m_cameraPosition.setInterpolate3(cameraFrom.getOrigin(),cameraTo.getOrigin(),minFraction);
	}




	//update OpenGL camera settings
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    gluLookAt(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],
		      m_cameraTargetPosition[0],m_cameraTargetPosition[1], m_cameraTargetPosition[2],
			  m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());



}


PlanetDemo::~PlanetDemo()
{
	//cleanup in the reverse order of creation/initialization
	if (m_character)
	{
		m_dynamicsWorld->removeCollisionObject(m_ghostObject);
	}
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

