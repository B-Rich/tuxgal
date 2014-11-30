#ifndef TUX_OBJECT_H
#define TUX_OBJECT_H

#include "btBulletDynamicsCommon.h"
#include "OgreSceneNode.h"

class tuxWorld;

class tuxObject {
public:
    btCollisionShape* getShape() const { return m_shape; }
    btRigidBody* getBody() const { return m_body; }
    btCollisionObject* getCollisionObject() const {
        return (btCollisionObject *) m_body;
    }

    void attachNode(Ogre::SceneNode *node);

    virtual void applyGravity(tuxWorld *world) = 0;
    void applyTransform();

protected:
    tuxObject()
        : m_initialized(false),
          m_attached(false),
          m_shape(0),
          m_body(0),
          m_node(0) { }

    void init(btCollisionShape *shape, btRigidBody *body);

    bool getInitialized() const { return m_initialized; }
    bool getConnected() const { return m_attached; }

private:
    bool m_initialized;
    bool m_attached;
    btCollisionShape *m_shape;
    btRigidBody *m_body;
    Ogre::SceneNode *m_node;
};

#endif // TUX_OBJECT_H

