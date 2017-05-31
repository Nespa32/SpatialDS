
#ifndef _PR_QUADTREE_H
#define _PR_QUADTREE_H

#include "AbstractSpatialDS.h"

// basic implementation of a Point Region Quad Tree
class PRQuadTree : public AbstractSpatialDS
{
public:
    // @todo: could templatize scale
    explicit PRQuadTree(uint32 maxX, uint32 maxY);

    bool Find(Point const& p) override;
    void Insert(Point const& p) override;
    void Delete(Point const& p) override;

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

PRQuadTree::PRQuadTree(uint32 maxX, uint32 maxY) :
    _maxX(maxX), _maxY(maxY) { }

bool PRQuadTree::Find(Point const& p)
{
    TreeNode* ptr = &_root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->children[0] == nullptr)
            return ptr->point && *ptr->point == p;

        // calc which quadrant p is on
        uint32 centerX = lowerX + (upperX - lowerX) / 2;
        uint32 centerY = lowerY + (upperY - lowerY) / 2;

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

void PRQuadTree::Insert(Point const& p)
{
    TreeNode* ptr = &_root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (true)
    {
        if (ptr->children[0] == nullptr)
        {
            // free slot, just need to add the point
            if (ptr->point == nullptr)
            {
                ptr->point = new Point(p);
                return;
            }
            // need to split
            else
            {
                // same coordinates, point already exists
                if (ptr->point && *ptr->point == p)
                    return;

                Point* point = ptr->point;
                ptr->point = nullptr;
                for (TreeNode*& child : ptr->children)
                {
                    child = new TreeNode();
                    child->parent = ptr;
                }

                // insert existing point
                Insert(*point);

                delete point;
            }
        }

        // calc which quadrant p is on
        uint32 centerX = lowerX + (upperX - lowerX) / 2;
        uint32 centerY = lowerY + (upperY - lowerY) / 2;

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

void PRQuadTree::Delete(Point const& p)
{
    TreeNode* ptr = &_root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->children[0] == nullptr)
        {
            if (ptr->point && *ptr->point == p)
                break;
            else
                return;
        }

        // calc which quadrant p is on
        uint32 centerX = lowerX + (upperX - lowerX) / 2;
        uint32 centerY = lowerY + (upperY - lowerY) / 2;

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

    if (ptr->point && *ptr->point == p)
    {
        Point* point = ptr->point;
        while (ptr->point && *ptr->point == p)
        {
            ptr->point = nullptr;
            if (ptr->parent == nullptr)
                break;

            bool otherNode = false;
            for (TreeNode* node : ptr->parent->children)
            {
                if (node->point || node->children[0])
                    otherNode = true;
            }

            if (otherNode == false)
            {
                ptr = ptr->parent;
                ptr->point = point;
            }
        }

        delete point;
    }
}

#endif // _PR_QUADTREE_H


