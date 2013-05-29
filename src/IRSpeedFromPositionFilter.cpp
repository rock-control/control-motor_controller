#include <motor_controller/IRSpeedFromPositionFilter.hpp>

using namespace motor_controller;

bool IRSpeedFromPositionFilter::update(base::samples::Joints& joints, bool force)
{
    if (mLast.time.isNull())
    {
        mLast = joints;
        return false;
    }
    if (joints.states.size() != mLast.states.size())
        throw std::runtime_error("IRSpeedFromPositionFilter::update - the number of states in the sample changed");

    size_t size = joints.states.size();
    bool hasResult = true;
    for (size_t i = 0; i < size; ++i)
    {
        if (joints.states[i].hasSpeed() && !force)
            continue;
        if (!joints.states[i].hasPosition())
            throw std::runtime_error("IRSpeedFromPositionFilter::update - no position in sample");

        float current = (joints.states[i].position - mLast.states[i].position) / (joints.time.toSeconds() - mLast.time.toSeconds());
        if (mLast.states[i].hasSpeed())
            joints.states[i].speed = mFactor * current + (1 - mFactor) * mLast.states[i].speed;
        else
        {
            hasResult = false;
            joints.states[i].speed = current;
        }
    }

    mLast = joints;
    return hasResult;
}


