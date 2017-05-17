
#ifndef _KD_TREE_H
#define _KD_TREE_H

#include <cassert>

class KdTree
{
public:
    explicit KdTree(uint32 maxX, uint32 maxY);

    bool Find(Point const& p);
    void Insert(Point const& p);
    void Delete(Point const& p);

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
                // @todo: handle case where min is not a leaf
                assert(node && node->left == nullptr && node->right == nullptr);

                // remove node's parent references
                if (node->parent->left == node)
                    node->parent->left = nullptr;
                else if (node->parent->right == node)
                    node->parent->right = nullptr;

                // replace ptr by node
                if (ptr->parent->left == ptr)
                    ptr->parent->left = node;
                else if (ptr->parent->right == ptr)
                    ptr->parent->right = node;

                node->left = ptr->left;
                node->right = ptr->right;
            }

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

#endif // _KD_TREE_H

