
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
    explicit KdTree(uint32 maxX, uint32 maxY);

    bool Find(Point const& p) override;
    void Insert(Point const& p) override;
    void Delete(Point const& p) override;

    Point* NearestNeighbour(Point const& p);
    std::vector<Point*> KNearestNeighbour(Point const& p, uint32 k);

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
    TreeNode* FindMin(uint32 dim, TreeNode* node);
	Point* NearestNeighbourImpl(Point const& p, float& closestSqrDist,
		TreeNode const* node, BoundingBox b);

private:
    // initial region size
    uint32 _maxX;
    uint32 _maxY;

    TreeNode* _root = nullptr;
};

KdTree::KdTree(uint32 maxX, uint32 maxY) : _maxX(maxX), _maxY(maxY) { }

bool KdTree::Find(Point const& p)
{
    TreeNode* ptr = _root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->point == p)
            return true;

        // calc which half p is on
        uint32 centerX = ptr->point.x;
        uint32 centerY = ptr->point.y;
        uint32 dim = ptr->dim;

        // @todo: improve
        if (dim == 0) // x
        {
            if (p.x >= centerX)
                lowerX = centerX;
            else
                upperX = centerX;
        }
        else if (dim == 1) // y
        {
            if (p.y >= centerY)
                lowerY = centerY;
            else
                upperY = centerY;
        }

        ptr = (p.GetDim(dim) >= ptr->point.GetDim(dim)) ? ptr->right : ptr->left;
    }

    return false;
}

void KdTree::Insert(Point const& p)
{
    TreeNode* parentPtr = nullptr;
    TreeNode* ptr = _root;
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->point == p)
            return;

        // calc which half p is on
        uint32 centerX = ptr->point.x;
        uint32 centerY = ptr->point.y;
        uint32 dim = ptr->dim;

        // @todo: improve
        if (dim == 0) // x
        {
            if (p.x >= centerX)
                lowerX = centerX;
            else
                upperX = centerX;
        }
        else if (dim == 1) // y
        {
            if (p.y >= centerY)
                lowerY = centerY;
            else
                upperY = centerY;
        }

        parentPtr = ptr;
        ptr = (p.GetDim(dim) >= ptr->point.GetDim(dim)) ? ptr->right : ptr->left;
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
    uint32 lowerX = 0;
    uint32 lowerY = 0;
    uint32 upperX = _maxX;
    uint32 upperY = _maxY;

    while (ptr)
    {
        if (ptr->point == p)
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

        // calc which half p is on
        uint32 centerX = ptr->point.x;
        uint32 centerY = ptr->point.y;
        uint32 dim = ptr->dim;

        // @todo: improve
        if (dim == 0) // x
        {
            if (p.x >= centerX)
                lowerX = centerX;
            else
                upperX = centerX;
        }
        else if (dim == 1) // y
        {
            if (p.y >= centerY)
                lowerY = centerY;
            else
                upperY = centerY;
        }

        parentPtr = ptr;
        ptr = (p.GetDim(dim) >= ptr->point.GetDim(dim)) ? ptr->right : ptr->left;
    }
}

KdTree::TreeNode* KdTree::FindMin(uint32 dim, TreeNode* node)
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

Point* KdTree::NearestNeighbour(Point const& p)
{
	float closestSqrDist = std::numeric_limits<float>::max();
	BoundingBox box(_maxX, _maxY);

	NearestNeighbourImpl(p, closestSqrDist, _root, box);
}

Point* KdTree::NearestNeighbourImpl(Point const& p, float& closestSqrDist,
        TreeNode const* node, BoundingBox box)
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

std::vector<Point*> KdTree::KNearestNeighbour(Point const& p, uint32 k)
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
        KdTree::TreeNode* node = nullptr;
        BoundingBox box;

        explicit QueueElement(float d, Point* p) :
            sqrDist(d), isPoint(true), point(p) { }
        explicit QueueElement(float d, KdTree::TreeNode* n, BoundingBox b) :
            sqrDist(d), isPoint(false), node(n), box(b) { }

        QueueElement& operator=(QueueElement const& e) = default;
    };

    auto queueCmpFn = [](QueueElement const& l, QueueElement const& r)
    {
        return l.sqrDist > r.sqrDist;
    };

    std::vector<Point*> points;
    if (!_root)
        return points;

    std::priority_queue<QueueElement, std::vector<QueueElement>,
                        decltype(queueCmpFn)> queue(queueCmpFn);

    BoundingBox box(_maxX, _maxY);

    queue.push(QueueElement(box.SqrDistTo(p), _root, box));

    while (!queue.empty())
    {
        QueueElement const& e = queue.top();

        if (e.isPoint)
        {
            points.push_back(e.point);
            if (points.size() >= k)
                break; // all done
        }
        else
        {
            TreeNode* node = e.node;
            BoundingBox box = e.box;

            // push point (counts as a child)
            Point* point = &node->point;
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

        queue.pop();
    }

    return points;
}

#endif // _KD_TREE_H

