#ifndef TUX_GLUT_BT_APPLICATION
#define TUX_GLUT_BT_APPLICATION

#include "tuxWorld.h"
#include "tuxPlanetObject.h"
#include "tuxCharacterObject.h"
#include "GlutBtApplication.h"

class tuxGlutBtApplication : public GlutBtApplication {
public:
    tuxGlutBtApplication();
    virtual ~tuxGlutBtApplication();

    void addCubeObject(
        btVector3 pos,
        btScalar size,
        btScalar mass,
        btScalar friction 
        );
    void initPhysics();

    virtual void clientMoveAndDisplay();
    virtual void updateCamera();
    virtual void displayCallback();
    virtual void specialKeyboard(int key, int x, int y);
    virtual void specialKeyboardUp(int key, int x, int y);

    void renderme();

    static BtApplication* create() {
        tuxGlutBtApplication* tuxApp = new tuxGlutBtApplication();
        tuxApp->myinit();
        tuxApp->initPhysics();
        return tuxApp;
    }

private:
    tuxWorld *m_world;
    tuxCharacterObject *m_player;
    btScalar m_cameraHeight;
};

#endif // TUX_GLUT_BT_APPLICATION

