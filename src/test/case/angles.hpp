#pragma once

#include "controller/eigen.hpp"
#include "test/test.hpp"

class AngleTest : public RegisteredTest<AngleTest>
{
public:

    static inline constexpr const char *TEST_NAME = "angles";

    inline static std::unique_ptr<AngleTest> create(Options &options)
    {
        return std::unique_ptr<AngleTest>(new AngleTest());
    }

private:

    inline bool run() override
    {
        Vector3d x = Vector3d(0, 0, M_PI/8);
        std::cout << quaternion_to_euler(euler_to_quaternion(x)) << std::endl;
        if (!quaternion_to_euler(euler_to_quaternion(x)).isApprox(x))
            return false;
        
        return true;
    }
};
