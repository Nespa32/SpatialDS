#include <stdio.h>
#include <random>
#include <chrono>

#include "QuadTree.h"
#include "PointQuadTree.h"

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

int main(int argc, char** argv)
{
    uint32 seed = 42;
    uint32 space_size = 1024;
    uint32 loops = 1e6;

    std::mt19937 gen;
    gen.seed(seed);

    std::uniform_int_distribution<> dis(0, space_size - 1);

    // measure PRQuadTree
    {
        HighResClock clock;

        PRQuadTree tree(space_size, space_size);

        for (uint32 i = 0; i < loops; ++i)
        {
            uint32 x = dis(gen);
            uint32 y = dis(gen);
            Point p(x, y);

            tree.Insert(p);
        }

        printf("PRQuadTree - %u inserts took %u ms\n",
            loops, clock.GetDuration());
    }

    // measure PointQuadTree
    {
        HighResClock clock;

        PointQuadTree tree(space_size, space_size);

        for (uint32 i = 0; i < loops; ++i)
        {
            uint32 x = dis(gen);
            uint32 y = dis(gen);
            Point p(x, y);

            tree.Insert(p);
        }

        printf("PointQuadTree - %u inserts took %u ms\n",
            loops, clock.GetDuration());
    }
}
