#include <gtest/gtest.h>
#include <motor_controller/PID.hpp>

using namespace motor_controller;

struct PIDTest : public ::testing::Test
{

    PID controller(PIDSettings settings)
    {
        PID pid = PID();
        pid.setPIDSettings(settings);
        return pid;
    }
};

TEST_F(PIDTest, it_yields_the_right_command_when_there_is_only_proportional)
{
    PIDSettings settings;
    settings.K = 0.1;
    auto pid = controller(settings);
    auto command = pid.update(10, 0, 1);
    ASSERT_FLOAT_EQ(command, -1);
}

TEST_F(PIDTest, it_overwrites_the_integral_part_of_the_pid)
{
    PIDSettings settings;
    settings.K = 0.1;
    settings.Ti = 2;
    settings.B = 1;
    settings.Ts = 1;
    auto pid = controller(settings);
    pid.update(10, 0, 1);
    ASSERT_FLOAT_EQ(pid.getState().I, -0.5);
    pid.setAccumulatedIntegral(10);
    ASSERT_FLOAT_EQ(pid.getState().I, 10);
}