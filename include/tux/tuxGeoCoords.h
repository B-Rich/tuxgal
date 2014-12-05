#ifndef TUX_GEO_COORDS_H
#define TUX_GEO_COORDS_H

#include "btBulletDynamicsCommon.h"

class tuxGeoCoords {
public:
    tuxGeoCoords(
        const btScalar r,
        const btScalar lat,
        const btScalar lon
        )
        : m_r(r),
          m_lat(lat),
          m_lon(lon) { }

    operator btVector3() const;

private:
    btScalar m_r;
    btScalar m_lat, m_lon;
};

#endif // TUX_GEO_COORDS

