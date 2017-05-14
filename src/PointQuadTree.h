
#ifndef _POINT_QUADTREE_H
#define _POINT_QUADTREE_H

#include "Point.h"

class PointQuadTree
{
public:
    // @todo: could templatize scale
    explicit PointQuadTree(uint32 maxX, uint32 maxY);

    bool Find(Point const& p);
    void Insert(Point const& p);
    void Delete(Point const& p);

public:
    struct TreeNode; // forward declaration
    struct TreeNode
    {
        ~TreeNode()
        {
            delete point;
            for (TreeNode* node : children)
                delete node;
        }

        Point* point = nullptr;
        TreeNode* parent = nullptr;
        TreeNode* children[4] = { nullptr };
    };

private:
    // initial region size
    uint32 _maxX;
    uint32 _maxY;

    TreeNode _root;
};

PointQuadTree::PointQuadTree(uint32 maxX, uint32 maxY) :
    _maxX(maxX), _maxY(maxY) { }

bool PointQuadTree::Find(Point const& p)
{
    TreeNode* ptr = &_root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->point == nullptr)
            return false; // reached a leaf

        if (*ptr->point == p)
            return true; // found the point

        // calc which quadrant p is on
        uint32 centerX = ptr->point->x;
        uint32 centerY = ptr->point->y;

        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p.x >= centerX) << 0) ||
            ((p.y >= centerY) << 1);

        if (p.x >= centerX)
            lowerX = centerX;
        else
            upperX = centerX;

        if (p.y >= centerY)
            lowerY = centerY;
        else
            upperY = centerY;

        ptr = ptr->children[idx];
    }

    return false;
}

void PointQuadTree::Insert(Point const& p)
{
    TreeNode* ptr = &_root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (true)
    {
        // free slot, just need to add the point
        if (ptr->point == nullptr)
        {
            ptr->point = new Point(p);
            return;
        }

        // same coordinates, point already exists
        if (*ptr->point == p)
            return;

        // init children
        // we lazy-create children to save some memory
        if (ptr->children[0] == nullptr)
        {
            for (TreeNode*& child : ptr->children)
            {
                child = new TreeNode();
                child->parent = ptr;
            }
        }

        // calc which quadrant p is on
        uint32 centerX = ptr->point->x;
        uint32 centerY = ptr->point->y;

        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p.x >= centerX) << 0) |
            ((p.y >= centerY) << 1);

        if (p.x >= centerX)
            lowerX = centerX;
        else
            upperX = centerX;

        if (p.y >= centerY)
            lowerY = centerY;
        else
            upperY = centerY;

        ptr = ptr->children[idx];
    }
}

void PointQuadTree::Delete(Point const& p)
{
    TreeNode* ptr = &_root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->point == nullptr)
            return; // leaf node, but point not found

        if (*ptr->point == p)
        {
            // @todo: delete logic
            // need to remove the node from the tree
            // and then reinsert every child node
            return;
        }

        // calc which quadrant p is on
        uint32 centerX = ptr->point->x;
        uint32 centerY = ptr->point->y;

        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p.x >= centerX) << 0) ||
            ((p.y >= centerY) << 1);

        if (p.x >= centerX)
            lowerX = centerX;
        else
            upperX = centerX;

        if (p.y >= centerY)
            lowerY = centerY;
        else
            upperY = centerY;

        ptr = ptr->children[idx];
    }
}

#endif // _POINT_QUADTREE_H
