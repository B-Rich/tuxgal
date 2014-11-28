#include "tuxPlanetObject.h"

tuxPlanetObject::tuxPlanetObject(btVector3 pos, btScalar radius) {
    btTransform planetTransform;
    planetTransform.setIdentity();
    planetTransform.setOrigin(pos);

    btScalar planetMass(0.); //the mass is 0, because the planet is immovable
    btVector3 localGroundInertia(0., 0., 0.);

    btCollisionShape *planetShape = new btSphereShape(radius);
    addCollisionShape(planetShape);
    btDefaultMotionState *planetMotionState =
        new btDefaultMotionState(planetTransform);

    planetShape->calculateLocalInertia(planetMass, localGroundInertia);

    btRigidBody::btRigidBodyConstructionInfo planetRBInfo(
        planetMass,
        planetMotionState,
        planetShape,
        localGroundInertia
        );
    btRigidBody *body = new btRigidBody(planetRBInfo);
    if (body) {
        body->setUserPointer(this);
        setBody(body);
    }
}

