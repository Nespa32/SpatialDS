
#include <cstdio>
#include <cassert>
#include <random>
#include <chrono>
#include <typeinfo>

#include "PRQuadTree.h"
#include "PointQuadTree.h"
#include "KdTree.h"

class HighResClock
{
public:
    typedef std::chrono::high_resolution_clock Clock;

    explicit HighResClock() : _start(Clock::now()) { }

    uint32 GetDuration() const
    {
        using namespace std::chrono;

        Clock::time_point end = Clock::now();
        auto duration = duration_cast<milliseconds>(end - _start).count();
        return static_cast<uint32>(duration);
    }

private:
    Clock::time_point _start;
};

template<class T>
void TestAndMeasure();

// globals, used by TestAndMeasure
// initial seed for each tree
uint32 seed = 42;
// size of tree bounding box
uint32 box_size = 1e9;
// size of each data batch
uint32 batch_size = 1e6;
// number of loops
uint32 loops = 10;
// whether operation should be tested or not
bool op_find = true;
bool op_insert = true;
bool op_delete = true;
bool op_nn = true;
bool op_knn = true;
bool op_rangesearch = true;

int main(int argc, char** argv)
{
    // disable stdout buffering
    setbuf(stdout, nullptr);

    for (int i = 1; i < argc; ++i)
    {
        std::string option = argv[i];

        // all options have a following arg
        if (i + 1 >= argc)
        {
            printf("Missing argument after option %s\n", option.c_str());
            return 1;
        }

        char* arg = argv[++i]; // also 'consumes' argument
        uint32 value = atol(arg);

        if (option == "--seed")
            seed = value;
        else if (option == "--box_size")
            box_size = value;
        else if (option == "--batch_size")
            batch_size = value;
        else if (option == "--loops")
            loops = value;
        else if (option == "--find")
            op_find = bool(value);
        else if (option == "--insert")
            op_insert = bool(value);
        else if (option == "--delete")
            op_delete = bool(value);
        else if (option == "--nn")
            op_nn = bool(value);
        else if (option == "--knn")
            op_knn = bool(value);
        else if (option == "--rangesearch")
            op_rangesearch = bool(value);
        else
            printf("Unrecognized option '%s', skipping\n", option.c_str());
    }

    TestAndMeasure<PRQuadTree>();
    TestAndMeasure<PointQuadTree>();
    TestAndMeasure<KdTree>();
}

template<class T>
void TestAndMeasure()
{
    std::mt19937 gen;
    gen.seed(seed);

    std::uniform_int_distribution<> dis(0, box_size - 1);

    BoundingBox box(box_size, box_size);

    printf("### %s ###\n", typeid(T).name());

    PRQuadTree tree(box);

    uint32 batch_sum = 0;
    for (uint32 i = 0; i < loops; ++i)
    {
        batch_sum += batch_size;

        // batch is inserted into the tree
        std::vector<Point> batch(batch_size);
        for (uint32 j = 0; j < batch_size; ++j)
        {
            uint32 x = dis(gen);
            uint32 y = dis(gen);
            batch[j] = Point(x, y);
        }

        // not_batch is *not* inserted into the tree
        std::vector<Point> not_batch(batch_size);
        for (uint32 j = 0; j < batch_size; ++j)
        {
            uint32 x = dis(gen);
            uint32 y = dis(gen);
            not_batch[j] = Point(x, y);
        }

        // now, with each batch:
        // 1. insert batch points
        {
            HighResClock clock;

            for (Point const& p : batch)
                tree.Insert(p);

            if (op_insert)
            {
                printf("Insert in %u ms, batch %u, total %u\n",
                    clock.GetDuration(), i, batch_sum);
            }
        }

        // 2. find batch points
        if (op_find)
        {
            HighResClock clock;

            uint32 found = 0;
            for (Point const& p : batch)
            {
                if (tree.Find(p))
                    ++found;
            }

            printf("Find in %u ms, batch %u, total %u\n",
                clock.GetDuration(), i, batch_sum);

            assert(found == batch.size());
        }

        // 3. find non-batch points
        if (op_find)
        {
            HighResClock clock;

            uint32 found = 0;
            for (Point const& p : not_batch)
            {
                if (tree.Find(p))
                    ++found;
            }

            printf("Random Find in %u ms, batch %u, total %u, found %u\n",
                clock.GetDuration(), i, batch_sum, found);
        }

        // 4. do a NN search on batch points
        if (op_nn)
        {
            HighResClock clock;

            uint32 checksum = 0;
            for (Point const& p : batch)
            {
                if (Point* nn = tree.NearestNeighbour(p))
                    checksum += nn->x + nn->y; // overflow on purpose
            }

            printf("NearestNeighbour in %u ms, batch %u, total %u, checksum %u\n",
                clock.GetDuration(), i, batch_sum, checksum);
        }

        // 5. do a k-NN search, with k=1 on batch points
        if (op_nn)
        {
            HighResClock clock;

            uint32 checksum = 0;
            for (Point const& p : batch)
            {
                PointVec points = tree.KNearestNeighbour(p, 1);
                for (Point* nn : points)
                    checksum += nn->x + nn->y; // overflow on purpose
            }

            printf("K(1)-NearestNeighbour in %u ms, batch %u, total %u, checksum %u\n",
                clock.GetDuration(), i, batch_sum, checksum);
        }

        // 6. do a k-NN search, with k>1 on non-batch points
        if (op_knn)
        {
            HighResClock clock;

            uint32 checksum = 0;
            for (Point const& p : not_batch)
            {
                PointVec points = tree.KNearestNeighbour(p, 10);
                for (Point* nn: points)
                    checksum += nn->x + nn->y; // overflow on purpose
            }

            printf("K(10)-NearestNeighbour in %u ms, batch %u, total %u, checksum %u\n",
                clock.GetDuration(), i, batch_sum, checksum);
        }

        // 7. @todo: range search

        // 8. delete batch points
        if (op_delete)
        {
            HighResClock clock;

            for (Point const& p : batch)
                tree.Delete(p);

            printf("Delete in %u ms, batch %u, total %u\n",
                clock.GetDuration(), i, batch_sum);

            // to maintain the same number of elements, add not-batch points
            for (Point const& p : not_batch)
                tree.Insert(p);
        }
    }
}
