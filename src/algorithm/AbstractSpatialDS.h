
#ifndef _ABSTRACT_SPATIAL_DS_H
#define _ABSTRACT_SPATIAL_DS_H

#include "Point.h"
#include "BoundingBox.h"

typedef std::vector<Point*> PointVec;

class AbstractSpatialDS
{
public:
    explicit AbstractSpatialDS(BoundingBox const& box) : _box(box) { }

    virtual bool Find(Point const& p) const = 0;
    virtual void Insert(Point const& p) = 0;
    virtual void Delete(Point const& p) = 0;
    virtual Point* NearestNeighbour(Point const& p) const = 0;
    virtual PointVec KNearestNeighbour(Point const& p, uint32 k) const = 0;
    virtual PointVec RangeSearch(BoundingBox const& box) const = 0;

protected:
    BoundingBox const _box;
};

#endif // _ABSTRACT_SPATIAL_DS_H
