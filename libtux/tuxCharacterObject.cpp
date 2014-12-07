#include "tuxCharacterObject.h"

tuxCharacterObject::tuxCharacterObject(
    const btVector3 pos,
    const btScalar  width,
    const btScalar  height,
    const btScalar  mass,
    const btScalar  friction
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
        characterBody->setRestitution(0.0);
        characterBody->setUserPointer(this);
        init(characterShape, characterBody);
    }
}

void tuxCharacterObject::turn(const btScalar angle) {
    if (angle) {
        btMatrix3x3 basis = getBody()->getWorldTransform().getBasis();
        basis *= btMatrix3x3(btQuaternion(btVector3(0.0, 1.0, 0.0), angle));
        getCollisionObject()->getWorldTransform().setBasis(basis);
    }
}

void tuxCharacterObject::move(const btScalar speed) {
    btVector3 currVelocity = getBody()->getLinearVelocity();
    btVector3 upDir = getUpDir();
    btVector3 gravityVelocity = upDir * currVelocity.dot(upDir);
    btTransform trans = getBody()->getWorldTransform();
    const btVector3 localForward(0.0, 0.0, -1.0);
    btVector3 forwardDir = trans.getBasis() * localForward;
    getBody()->setLinearVelocity(gravityVelocity + forwardDir * speed);
}

