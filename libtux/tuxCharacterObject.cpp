#include "tuxCharacterObject.h"
#include "tuxWorld.h"

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
        m_upDir = btVector3(0.0, 1.0, 0.0);
        m_forwardDir = btVector3(0.0, 0.0, 1.0);
        init(characterShape, characterBody);
    }
}

void tuxCharacterObject::applyGravity(tuxWorld *world) {

    if (getInitialized()) {
        btRigidBody *body = getBody();

        if (body && body->getMotionState()) {
            btVector3 gravityCenter = world->getGravityCenter();

            // Get transformation
            btTransform trans;
            body->getMotionState()->getWorldTransform(trans);

            // Calculate and set gravity vector
            btVector3 n = trans.getOrigin() - gravityCenter;
            n.normalize();
            body->setGravity(-9.8 * n);

            // Get body up vector
            const btVector3 yAxis(0.0, 1.0, 0.0);
            btQuaternion q = trans.getRotation();
            btVector3 bodyAxis = btMatrix3x3(q) * yAxis;

            // Align body up vector along gravity vector
            const btScalar epsilon = 1.0e-5;
            btVector3 rotationAxis = bodyAxis.cross(n);
            btScalar crossLength = rotationAxis.length();
            if (crossLength > epsilon) {
                btScalar angle = btAsin(crossLength);
                rotationAxis /= crossLength;
                btQuaternion dq(rotationAxis, angle);
                trans.setRotation(dq * q);
                body->setWorldTransform(trans);
            }

            // Store up direction vector
            m_upDir = n;

            // Calulcate forward direction
            m_forwardDir -= n * n.dot(m_forwardDir);
            m_forwardDir.normalize();
        }
    }
}

