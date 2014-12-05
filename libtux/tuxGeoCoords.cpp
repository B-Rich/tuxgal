#include "tuxGeoCoords.h"

tuxGeoCoords::operator btVector3() const {
    btVector3 coords(
        m_r * btCos(m_lat) * btCos(m_lon),
        m_r * btCos(m_lat) * btSin(m_lon),
        m_r * btSin(m_lat)
        );

    return coords;
}

