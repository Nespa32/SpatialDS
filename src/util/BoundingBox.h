#ifndef _BOUNDING_BOX_H
#define _BOUNDING_BOX_H

#include <algorithm>
#include "Defines.h"
#include "Point.h"

class BoundingBox
{
public:
    BoundingBox() { }
    BoundingBox(uint32 lowerX, uint32 lowerY, uint32 upperX, uint32 upperY) :
        _lower(lowerX, lowerY), _upper(upperX, upperY) { }
    BoundingBox(uint32 upperX, uint32 upperY) :
        _upper(upperX, upperY) { }
    BoundingBox(Point const& lower, Point const& upper) :
        _lower(lower), _upper(upper) { }

    bool Contains(Point const& p) const
    {
        return _lower.x <= p.x && p.x <= _upper.x &&
            _lower.y <= p.y && p.y <= _upper.y;
    }

    bool Intersects(BoundingBox const& box) const
    {
        return _lower.x <= box._upper.x || _lower.y <= box._upper.y ||
               _upper.x >= box._lower.x || _upper.y >= box._lower.y;
    }

    Point GetCenter() const
    {
        uint32 centerX = _lower.x + (_upper.x - _lower.x) / 2;
        uint32 centerY = _lower.y + (_upper.y - _lower.y) / 2;
        return Point(centerX, centerY);
    }

    Point GetLower() const { return _lower; }
    Point GetUpper() const { return _upper; }

    BoundingBox UpdateLowerX(uint32 x) const { return BoundingBox(x, _lower.y, _upper.x, _upper.y); }
    BoundingBox UpdateUpperX(uint32 x) const { return BoundingBox(_lower.x, _lower.y, x, _upper.y); }
    BoundingBox UpdateLowerY(uint32 y) const { return BoundingBox(_lower.x, y, _upper.x, _upper.y); }
    BoundingBox UpdateUpperY(uint32 y) const { return BoundingBox(_lower.x, _lower.y, _upper.x, y); }
    BoundingBox UpdateLowerDim(uint32 dim, uint32 val) const;
    BoundingBox UpdateUpperDim(uint32 dim, uint32 val) const;

    float SqrDistTo(Point const& p) const
    {
        if (Contains(p))
            return 0.0f;

        float x = float(p.x);
        float y = float(p.y);
        // source: https://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        float dx = std::max({ _lower.x - x, 0.0f, x - _lower.x });
        float dy = std::max({ _lower.y - y, 0.0f, y - _lower.y });
        return dx*dx + dy*dy;
    }

private:
    Point _lower;
    Point _upper;
};

BoundingBox BoundingBox::UpdateLowerDim(uint32 dim, uint32 val) const
{
    Point newLower = _lower;
    newLower.SetDim(dim, val);
    return BoundingBox(newLower, _upper);
}

BoundingBox BoundingBox::UpdateUpperDim(uint32 dim, uint32 val) const
{
    Point newUpper = _upper;
    newUpper.SetDim(dim, val);
    return BoundingBox(_lower, newUpper);
}

#endif // _BOUNDING_BOX_H

