
#ifndef _POINT_H
#define _POINT_H

#include "Defines.h"

struct Point
{
public:
    Point() { }
    Point(uint32 pointX, uint32 pointY) : x(pointX), y(pointY) { }
    Point(Point const& p) : x(p.x), y(p.y) { }

    inline bool operator==(Point const& p) { return p.x == x && p.y == y; }

    uint32 GetDim(uint32 dim) const { return (dim == 0) ? x : y; }
    void SetDim(uint32 dim, uint32 val);

    float SqrDistTo(Point const& p) const
    {
        float dx = float(x) - float(p.x);
        float dy = float(y) - float(p.y);
        return dx*dx + dy*dy;
    }

public:
    uint32 x = 0;
    uint32 y = 0;
};

void Point::SetDim(uint32 dim, uint32 value)
{
    // @todo: improve
    if (dim == 0)
        x = value;
    else if (dim == 1)
        y = value;
}

#endif // _POINT_H
