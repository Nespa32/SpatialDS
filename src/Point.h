
#ifndef _POINT_H
#define _POINT_H

#include <cstdint>

typedef uint32_t uint32;

struct Point
{
public:
    Point() { }
    Point(uint32 pointX, uint32 pointY) : x(pointX), y(pointY) { }

    inline bool operator==(Point const& p) { return p.x == x && p.y == y; }

    uint32 GetDim(uint32 dim) const { return (dim == 0) ? x : y; }

public:
    uint32 x = 0;
    uint32 y = 0;
};

class BoundingBox
{
public:
    BoundingBox(uint32 lowerX, uint32 upperX, uint32 lowerY, uint32 upperY) :
        _lowerX(lowerX), _upperX(upperX), _lowerY(lowerY), _upperY(upperY) { }
    BoundingBox(uint32 upperX, uint32 upperY) :
        _lowerX(0), _upperX(upperX), _lowerY(0), _upperY(upperY) { }

    bool Contains(Point const& p)
    {
        return _lowerX <= p.x && p.x >= _upperX &&
            _lowerY <= p.x && p.y >= _upperY;
    }

    Point GetCenter() const
    {
        uint32 centerX = _lowerX + (_upperX - _lowerX) / 2;
        uint32 centerY = _lowerY + (_upperY - _lowerY) / 2;
        return Point(centerX, centerY);
    }

private:
    uint32 _lowerX;
    uint32 _upperX;
    uint32 _lowerY;
    uint32 _upperY;
};

#endif // _POINT_H
