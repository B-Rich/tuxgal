#include "tuxCharacterObject.h"

tuxCharacterObject::tuxCharacterObject(
    btVector3 pos,
    btScalar  width,
    btScalar  height,
    btScalar  mass,
    btScalar  friction
    ) {
    btCollisionShape* characterShape = new btCapsuleShape(width, height);

    //set the initial position and transform. We set tranform none
    btTransform startTransform;
    startTransform.setIdentity();

    btVector3 localInertia(0.0, 0.0, 0.0);

    startTransform.setOrigin(pos);
    characterShape->calculateLocalInertia(mass, localInertia);

    //actually construct the body and add it to the dynamics world
    btDefaultMotionState *characterMotionState =
        new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(
        mass,
        characterMotionState,
        characterShape,
        localInertia
        );
    rbInfo.m_friction = friction;
    btRigidBody *characterBody = new btRigidBody(rbInfo);
    if (characterBody) {
        characterBody->setAngularFactor(0.0);
        characterBody->setRestitution(1.0);
        characterBody->setUserPointer(this);
        init(characterShape, characterBody);
    }
}

