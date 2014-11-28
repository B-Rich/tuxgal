#include "tuxCharacterObject.h"
#include "tuxWorld.h"

tuxCharacterObject::tuxCharacterObject(
    btVector3 pos,
    btScalar width,
    btScalar height,
    btScalar mass,
    btScalar friction
    ) {
    btCollisionShape* newRigidShape = new btCapsuleShape(width, height);
    addCollisionShape(newRigidShape);

    //set the initial position and transform. We set tranform none
    btTransform startTransform;
    startTransform.setIdentity();

    btVector3 localInertia(0, 0, 0);

    startTransform.setOrigin(pos);
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
    rbInfo.m_friction = friction;
    btRigidBody *body = new btRigidBody(rbInfo);
    if (body) {
        body->setAngularFactor(0);
        body->setRestitution(1);
        body->setUserPointer(this);
        setBody(body);
    }
}

void tuxCharacterObject::applyGravity(tuxWorld *world) {

    btRigidBody *body = getBody();
    if (body) {
        if (body->getMotionState()) {
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
        }
    }
}

