
#ifndef _ABSTRACT_SPATIAL_DS_H
#define _ABSTRACT_SPATIAL_DS_H

#include "Point.h"
#include "BoundingBox.h"

class AbstractSpatialDS
{
public:
    virtual bool Find(Point const& p) = 0;
    virtual void Insert(Point const& p) = 0;
    virtual void Delete(Point const& p) = 0;
};

#endif // _ABSTRACT_SPATIAL_DS_H
