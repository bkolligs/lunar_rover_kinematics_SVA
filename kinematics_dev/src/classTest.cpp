#include "classTest.h"


classTest::classTest(int x, int y) : x_{x}, y_{y}
{

}

int classTest::add()
{
    return x_ + y_;
}
