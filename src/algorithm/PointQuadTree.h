#ifndef _POINT_QUADTREE_H
#define _POINT_QUADTREE_H

#include <array>
#include <numeric>
#include <limits>
#include <vector>
#include <queue>
#include <list>
#include "AbstractSpatialDS.h"

class PointQuadTree : public AbstractSpatialDS
{
public:
    explicit PointQuadTree(BoundingBox const& box);

    bool Find(Point const& p) const override;
    void Insert(Point const& p) override;
    void Delete(Point const& p) override;
    Point* NearestNeighbour(Point const& p) const override;
    PointVec KNearestNeighbour(Point const& p, uint32 k) const override;
    PointVec RangeSearch(BoundingBox const& box) const override;

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
    // re-insert existing point back into the tree (memory already allocated)
    void Reinsert(Point* p);
    Point* NearestNeighbourImpl(Point const& p, float& closestSqrDist,
        TreeNode const* node, BoundingBox nodeBox) const;
    void RangeSearchImpl(BoundingBox const& box, PointVec& points,
        TreeNode const* node, BoundingBox nodeBox) const;

private:
    TreeNode _root;
};

PointQuadTree::PointQuadTree(BoundingBox const& box) : AbstractSpatialDS(box) { }

bool PointQuadTree::Find(Point const& p) const
{
    TreeNode const* ptr = &_root;
    BoundingBox box = _box;

    while (ptr)
    {
        Point const* center = ptr->point;
        if (center == nullptr)
            return false; // reached a leaf

        if (*center == p)
            return true; // found the point

        // calc which quadrant p is on
        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p.x >= center->x) << 0) |
            ((p.y >= center->x) << 1);

        if (p.x >= center->x)
            box = box.UpdateLowerX(center->x);
        else
            box = box.UpdateUpperX(center->x);

        if (p.y >= center->y)
            box = box.UpdateLowerY(center->y);
        else
            box = box.UpdateUpperY(center->y);

        ptr = ptr->children[idx];
    }

    return false;
}

void PointQuadTree::Insert(Point const& p)
{
    TreeNode* ptr = &_root;
    BoundingBox box = _box;

    while (true)
    {
        // free slot, just need to add the point
        if (ptr->point == nullptr)
        {
            ptr->point = new Point(p);
            return;
        }

        Point* center = ptr->point;
        // same coordinates, point already exists
        if (*center == p)
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
        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p.x >= center->x) << 0) |
            ((p.y >= center->y) << 1);

        if (p.x >= center->x)
            box = box.UpdateLowerX(center->x);
        else
            box = box.UpdateUpperX(center->x);

        if (p.y >= center->y)
            box = box.UpdateLowerY(center->y);
        else
            box = box.UpdateUpperY(center->y);

        ptr = ptr->children[idx];
    }
}

void PointQuadTree::Delete(Point const& p)
{
    TreeNode* ptr = &_root;
    BoundingBox box = _box;

    while (ptr)
    {
        Point* center = ptr->point;
        if (center == nullptr)
            return; // leaf node, but point not found

        if (*center == p)
        {
            // with point quad trees, there's no salvaging possible when removing a node
            // need to reinsert every single children back into the tree
            // remove parent reference
            if (TreeNode* parent = ptr->parent)
            {
                for (TreeNode*& child : parent->children)
                {
                    if (child == ptr)
                        child = nullptr;
                }
            }

            // now do a BFS on the children, re-add them to the tree
            std::list<TreeNode*> nodes;
            for (TreeNode* child : ptr->children)
            {
                if (child)
                    nodes.push_back(child);
            }

            while (!nodes.empty())
            {
                TreeNode* node = nodes.front();
                nodes.pop_front();

                if (Point*& point = node->point)
                {
                    Reinsert(point);
                    point = nullptr;
                }

                for (TreeNode* child : ptr->children)
                {
                    if (child)
                        nodes.push_back(child);
                }
            }

            delete ptr; // will delete entire node subtree
            return;
        }

        // calc which quadrant p is on
        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p.x >= center->x) << 0) |
            ((p.y >= center->y) << 1);

        if (p.x >= center->x)
            box = box.UpdateLowerX(center->x);
        else
            box = box.UpdateUpperX(center->x);

        if (p.y >= center->y)
            box = box.UpdateLowerY(center->y);
        else
            box = box.UpdateUpperY(center->y);

        ptr = ptr->children[idx];
    }
}

void PointQuadTree::Reinsert(Point* p)
{
    TreeNode* ptr = &_root;
    BoundingBox box = _box;

    while (true)
    {
        // free slot, just need to add the point
        if (ptr->point == nullptr)
        {
            ptr->point = p;
            return;
        }

        Point* center = ptr->point;
        // same coordinates, point already exists
        if (*center == *p)
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
        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        uint32 idx = ((p->x >= center->x) << 0) |
            ((p->y >= center->y) << 1);

        if (p->x >= center->x)
            box = box.UpdateLowerX(center->x);
        else
            box = box.UpdateUpperX(center->x);

        if (p->y >= center->y)
            box = box.UpdateLowerY(center->y);
        else
            box = box.UpdateUpperY(center->y);

        ptr = ptr->children[idx];
    }
}

Point* PointQuadTree::NearestNeighbour(Point const& p) const
{
    float closestSqrDist = std::numeric_limits<float>::max();

    return NearestNeighbourImpl(p, closestSqrDist, &_root, _box);
}


Point* PointQuadTree::NearestNeighbourImpl(Point const& p, float& closestSqrDist,
    TreeNode const* node, BoundingBox box) const
{
    if (node == nullptr)
        return nullptr;

    Point* center = node->point;
    if (!center)
        return nullptr; // empty leaf node

    float dist = center->SqrDistTo(p);
    closestSqrDist = std::min(closestSqrDist, dist);

    std::array<BoundingBox, 4> boxes;
    std::array<float, 4> dists;
    for (uint32 idx = 0; idx < 4; ++idx)
    {
        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        if ((idx >> 0) & 0x1)
            boxes[idx] = box.UpdateLowerX(center->x);
        else
            boxes[idx] = box.UpdateUpperX(center->x);

        if ((idx >> 1) & 0x1)
            boxes[idx] = boxes[idx].UpdateLowerY(center->y);
        else
            boxes[idx] = boxes[idx].UpdateUpperY(center->y);

        dists[idx] = boxes[idx].SqrDistTo(p);
    }

    std::array<uint32, 4> indices;
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](uint32 a, uint32 b)
    {
        return dists[a] < dists[b];
    });

    std::array<Point*, 5> res = { nullptr };
    for (uint32 idx : indices)
    {
        if (closestSqrDist > dists[idx])
            res[idx] = NearestNeighbourImpl(p, closestSqrDist, node->children[idx], boxes[idx]);
    }

    res[4] = center;

    std::array<float, 5> resDists;
    std::array<uint32, 5> resIndices;
    std::iota(resIndices.begin(), resIndices.end(), 0);

    for (uint32 idx = 0; idx < 5; ++idx)
    {
        if (Point* point = res[idx])
            resDists[idx] = point->SqrDistTo(p);
        else
            resDists[idx] = std::numeric_limits<float>::max();
    }

    std::sort(resIndices.begin(), resIndices.end(), [&](uint32 a, uint32 b)
    {
        return resDists[a] < resDists[b];
    });

    return res[resIndices[0]];
}

PointVec PointQuadTree::KNearestNeighbour(Point const& p, uint32 k) const
{
    // need an additional struct to represent points *AND* nodes seperately in
    //  a priority queue (polymorphism is overkill)
    struct QueueElement
    {
        float sqrDist = 0.0f;
        bool isPoint = false;
        // point
        Point* point = nullptr;
        // node
        PointQuadTree::TreeNode const* node = nullptr;
        BoundingBox box;

        explicit QueueElement(float d, Point* p) :
            sqrDist(d), isPoint(true), point(p) { }
        explicit QueueElement(float d, PointQuadTree::TreeNode const* n, BoundingBox b) :
            sqrDist(d), isPoint(false), node(n), box(b) { }

        QueueElement& operator=(QueueElement const& e) = default;
    };

    auto queueCmpFn = [](QueueElement const& l, QueueElement const& r)
    {
        return l.sqrDist > r.sqrDist;
    };

    PointVec points;

    std::priority_queue<QueueElement, std::vector<QueueElement>,
                        decltype(queueCmpFn)> queue(queueCmpFn);

    BoundingBox box = _box;

    queue.emplace(box.SqrDistTo(p), &_root, box);

    while (!queue.empty())
    {
        QueueElement const& e = queue.top();
        queue.pop();

        if (e.isPoint)
        {
            points.push_back(e.point);
            if (points.size() >= k)
                break; // all done
        }
        else
        {
            TreeNode const* node = e.node;
            BoundingBox box = e.box;

            // push point (counts as a child)
            if (Point* center = node->point)
            {
                queue.emplace(center->SqrDistTo(p), center);

                for (uint32 idx = 0; idx < 4; ++idx)
                {
                    BoundingBox b = box;
                    //   ---------
                    // Y | 2 | 3 |
                    // y | 0 | 1 |
                    //   ---------
                    //     x   X
                    if ((idx >> 0) & 0x1)
                        b = b.UpdateLowerX(center->x);
                    else
                        b = b.UpdateUpperX(center->x);

                    if ((idx >> 1) & 0x1)
                        b = b.UpdateLowerY(center->y);
                    else
                        b = b.UpdateUpperY(center->y);

                    queue.emplace(b.SqrDistTo(p), node->children[idx], b);
                }
            }
        }
    }

    return points;
}

PointVec PointQuadTree::RangeSearch(BoundingBox const& box) const
{
    PointVec points;

    RangeSearchImpl(box, points, &_root, _box);

    return points;
}

void PointQuadTree::RangeSearchImpl(BoundingBox const& box, PointVec& points,
    TreeNode const* node, BoundingBox nodeBox) const
{
    if (node == nullptr)
        return;

    if (box.Intersects(nodeBox) == false)
        return;

    Point* center = const_cast<Point*>(node->point);
    if (center == nullptr) // empty leaf node
        return;

    if (box.Contains(*center))
        points.push_back(center);

    for (uint32 idx = 0; idx < 4; ++idx)
    {
        BoundingBox b = box;
        //   ---------
        // Y | 2 | 3 |
        // y | 0 | 1 |
        //   ---------
        //     x   X
        if ((idx >> 0) & 0x1)
            b = b.UpdateLowerX(center->x);
        else
            b = b.UpdateUpperX(center->x);

        if ((idx >> 1) & 0x1)
            b = b.UpdateLowerY(center->y);
        else
            b = b.UpdateUpperY(center->y);

        RangeSearchImpl(box, points, node->children[idx], b);
    }
}

#endif // _POINT_QUADTREE_H
