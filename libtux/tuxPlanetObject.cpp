#include "tuxPlanetObject.h"

tuxPlanetObject::tuxPlanetObject(btVector3 pos, btScalar radius) {
    btTransform planetTransform;
    planetTransform.setIdentity();
    planetTransform.setOrigin(pos);

    btScalar planetMass(0.0); //the mass is 0, because the planet is immovable
    btVector3 localGroundInertia(0.0, 0.0, 0.0);

    btCollisionShape *planetShape = new btSphereShape(radius);
    btDefaultMotionState *planetMotionState =
        new btDefaultMotionState(planetTransform);

    planetShape->calculateLocalInertia(planetMass, localGroundInertia);

    btRigidBody::btRigidBodyConstructionInfo planetRBInfo(
        planetMass,
        planetMotionState,
        planetShape,
        localGroundInertia
        );
    btRigidBody *planetBody = new btRigidBody(planetRBInfo);
    if (planetBody) {
        planetBody->setUserPointer(this);
        init(planetShape, planetBody);
    }
}

