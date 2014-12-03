#include "tuxDynamicObject.h"
#include "tuxWorld.h"

void tuxDynamicObject::applyGravity(tuxWorld *world) {

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
        }
    }
}

