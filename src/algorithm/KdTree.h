
#ifndef _KD_TREE_H
#define _KD_TREE_H

#include <cstring>
#include <limits>
#include <vector>
#include <queue>
#include "AbstractSpatialDS.h"

class KdTree : public AbstractSpatialDS
{
public:
    explicit KdTree(BoundingBox const& box);

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
            delete left;
            delete right;
        }

        Point point;
        uint32 dim = 0; // which cut dimension
        TreeNode* parent = nullptr; // is this needed?
        TreeNode* left = nullptr;
        TreeNode* right = nullptr;
    };

private:
    TreeNode* FindMin(uint32 dim, TreeNode* node) const;
	Point* NearestNeighbourImpl(Point const& p, float& closestSqrDist,
		TreeNode const* node, BoundingBox nodeBox) const;
    void RangeSearchImpl(BoundingBox const& box, PointVec& points,
        TreeNode const* node, BoundingBox nodeBox) const;

private:
    TreeNode* _root = nullptr;
};

KdTree::KdTree(BoundingBox const& box) : AbstractSpatialDS(box) { }

bool KdTree::Find(Point const& p) const
{
    TreeNode const* ptr = _root;
    BoundingBox box = _box;

    while (ptr)
    {
        Point const& center = ptr->point;
        if (center == p)
            return true;

        uint32 const dim = ptr->dim;
        uint32 const dimValue = center.GetDim(dim);

        bool rightHalf = (p.GetDim(dim) >= dimValue);
        if (rightHalf)
            box.UpdateLowerDim(dim, dimValue);
        else
            box.UpdateUpperDim(dim, dimValue);

        ptr = (rightHalf ? ptr->right : ptr->left);
    }

    return false;
}

void KdTree::Insert(Point const& p)
{
    TreeNode* parentPtr = nullptr;
    TreeNode* ptr = _root;
    BoundingBox box = _box;

    while (ptr)
    {
        Point const& center = ptr->point;
        if (center == p)
            return; // p already exists

        uint32 const dim = ptr->dim;
        uint32 const dimValue = center.GetDim(dim);

        bool rightHalf = (p.GetDim(dim) >= dimValue);
        if (rightHalf)
            box.UpdateLowerDim(dim, dimValue);
        else
            box.UpdateUpperDim(dim, dimValue);

        parentPtr = ptr;
        ptr = (rightHalf ? ptr->right : ptr->left);
    }

    // @todo: need a uniform method that gives "next dim"
    uint32 dim = parentPtr ? ((parentPtr->dim + 1) % 2) : 0;
    // parentPtr contains last valid node (if any)
    ptr = new TreeNode();
    ptr->dim = dim;
    ptr->point = p;
    ptr->parent = parentPtr;

    if (parentPtr == nullptr)
        _root = ptr;
    else
    {
        if (p.GetDim(dim) >= ptr->point.GetDim(dim))
            parentPtr->right = ptr;
        else
            parentPtr->left = ptr;
    }
}

void KdTree::Delete(Point const& p)
{
    TreeNode* parentPtr = nullptr;
    TreeNode* ptr = _root;
    BoundingBox box = _box;

    while (ptr)
    {
        Point const& center = ptr->point;
        if (center == p)
        {
            // ptr is a leaf node, can just cleanly remove
            if (ptr->left == nullptr && ptr->right == nullptr)
            {
                if (ptr->parent->left == ptr)
                    ptr->parent->left = nullptr;
                else if (ptr->parent->right == ptr)
                    ptr->parent->right = nullptr;
            }
            // not a leaf node, need to find min of the right tree
            else
            {
                TreeNode* node = FindMin(ptr->dim, ptr->right);

                // replace ptr by node (can be nullptr)
                if (ptr->parent->left == ptr)
                    ptr->parent->left = node;
                else if (ptr->parent->right == ptr)
                    ptr->parent->right = node;

                // if there's a replacement node, move it to correct position in tree
                if (node)
                {
                    // remove node's parent references
                    if (node->parent->left == node)
                        node->parent->left = nullptr;
                    else if (node->parent->right == node)
                        node->parent->right = nullptr;

                    // node might have children with other dims, need to re-insert them
                    // so keep references
                    TreeNode* nodeLeft = node->left;
                    TreeNode* nodeRight = node->right;

                    node->left = ptr->left;
                    node->right = ptr->right;

                    if (nodeLeft)
                        Insert(nodeLeft->point);
                    if (nodeRight)
                        Insert(nodeRight->point);

                    delete nodeLeft;
                    delete nodeRight;
                }
                else
                {
                    // no node found, need to re-insert children with other dims
                    if (ptr->left)
                        Insert(ptr->left->point);
                    if (ptr->right)
                        Insert(ptr->right->point);

                    delete ptr->left;
                    delete ptr->right;
                }
            }

            // actually delete the node
            delete ptr;

            return;
        }

        uint32 const dim = ptr->dim;
        uint32 const dimValue = center.GetDim(dim);

        bool rightHalf = (p.GetDim(dim) >= dimValue);
        if (rightHalf)
            box.UpdateLowerDim(dim, dimValue);
        else
            box.UpdateUpperDim(dim, dimValue);

        parentPtr = ptr;
        ptr = (rightHalf ? ptr->right : ptr->left);
    }
}

KdTree::TreeNode* KdTree::FindMin(uint32 dim, TreeNode* node) const
{
    if (node == nullptr)
        return nullptr;

    if (node->dim == dim)
    {
        TreeNode* min = FindMin(dim, node->left);
        return (min == nullptr) ? node : min;
    }
    else
    {
        TreeNode* l = FindMin(dim, node->left);
        TreeNode* r = FindMin(dim, node->right);
        if (l && (!r || l->point.GetDim(dim) <= r->point.GetDim(dim)))
            return l;
        else
            return r;
    }
}

Point* KdTree::NearestNeighbour(Point const& p) const
{
	float closestSqrDist = std::numeric_limits<float>::max();

	return NearestNeighbourImpl(p, closestSqrDist, _root, _box);
}

Point* KdTree::NearestNeighbourImpl(Point const& p, float& closestSqrDist,
        TreeNode const* node, BoundingBox box) const
{
    if (node == nullptr)
        return nullptr;

    Point center = node->point;
    float centerDist = center.SqrDistTo(p);
    // update closest known dist
    closestSqrDist = std::min(closestSqrDist, centerDist);

    uint32 dim = node->dim;
    uint32 val = center.GetDim(dim);
    BoundingBox leftBox = box.UpdateUpperDim(dim, val);
    BoundingBox rightBox = box.UpdateLowerDim(dim, val);

    float leftDist = leftBox.SqrDistTo(p);
    float rightDist = rightBox.SqrDistTo(p);

    // visit viable branches
    Point* centerRes = const_cast<Point*>(&node->point);
    Point* leftRes = nullptr;
    Point* rightRes = nullptr;
    if (closestSqrDist > leftDist && closestSqrDist > rightDist)
    {
        // visit most promising branch first
        if (leftDist > rightDist)
        {
            leftRes = NearestNeighbourImpl(p, closestSqrDist, node->left, leftBox);
            rightRes = NearestNeighbourImpl(p, closestSqrDist, node->right, rightBox);
        }
        else
        {
            rightRes = NearestNeighbourImpl(p, closestSqrDist, node->right, rightBox);
            leftRes = NearestNeighbourImpl(p, closestSqrDist, node->left, leftBox);
        }
    }
    else
    {
        if (closestSqrDist > leftDist)
            leftRes = NearestNeighbourImpl(p, closestSqrDist, node->left, leftBox);
        if (closestSqrDist > rightDist)
            rightRes = NearestNeighbourImpl(p, closestSqrDist, node->right, rightBox);
    }


    // return closest point
    // centerDist is already correct
    leftDist = (leftRes ? leftRes->SqrDistTo(p) : std::numeric_limits<float>::max());
    rightDist = (rightRes ? rightRes->SqrDistTo(p) : std::numeric_limits<float>::max());

    // @todo: this is ugly and error-prune
    Point* res = centerRes;
    if (centerDist > leftDist)
        res = leftRes;
    if (res == leftRes && leftDist > rightDist)
        res = rightRes;
    if (res == centerRes && centerDist > rightDist)
        res = rightRes;

    return res;
}

PointVec KdTree::KNearestNeighbour(Point const& p, uint32 k) const
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
        KdTree::TreeNode const* node = nullptr;
        BoundingBox box;

        explicit QueueElement(float d, Point* p) :
            sqrDist(d), isPoint(true), point(p) { }
        explicit QueueElement(float d, KdTree::TreeNode const* n, BoundingBox b) :
            sqrDist(d), isPoint(false), node(n), box(b) { }

        QueueElement& operator=(QueueElement const& e) = default;
    };

    auto queueCmpFn = [](QueueElement const& l, QueueElement const& r)
    {
        return l.sqrDist > r.sqrDist;
    };

    PointVec points;
    if (!_root)
        return points;

    std::priority_queue<QueueElement, std::vector<QueueElement>,
                        decltype(queueCmpFn)> queue(queueCmpFn);

    BoundingBox box = _box;

    queue.emplace(box.SqrDistTo(p), _root, box);

    while (!queue.empty())
    {
        QueueElement e = queue.top();
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
            Point* point = const_cast<Point*>(&node->point);
            queue.emplace(point->SqrDistTo(p), point);

            // push left/right node
            uint32 dim = node->dim;
            uint32 val = point->GetDim(dim);
            if (node->left)
            {
                BoundingBox leftBox = box.UpdateUpperDim(dim, val);
                queue.emplace(leftBox.SqrDistTo(p), node->left, leftBox);
            }

            if (node->right)
            {
                BoundingBox rightBox = box.UpdateLowerDim(dim, val);
                queue.emplace(rightBox.SqrDistTo(p), node->right, rightBox);
            }
        }
    }

    return points;
}

PointVec KdTree::RangeSearch(BoundingBox const& box) const
{
    PointVec points;

    RangeSearchImpl(box, points, _root, _box);

    return points;
}

void KdTree::RangeSearchImpl(BoundingBox const& box, PointVec& points,
    TreeNode const* node, BoundingBox nodeBox) const
{
    if (node == nullptr)
        return;

    if (box.Intersects(nodeBox) == false)
        return;

    Point* point = const_cast<Point*>(&node->point);
    if (box.Contains(*point))
        points.push_back(point);

    uint32 dim = node->dim;
    uint32 val = point->GetDim(dim);
    BoundingBox leftBox = box.UpdateUpperDim(dim, val);
    BoundingBox rightBox = box.UpdateLowerDim(dim, val);

    RangeSearchImpl(box, points, node->left, leftBox);
    RangeSearchImpl(box, points, node->right, rightBox);
}

#endif // _KD_TREE_H

