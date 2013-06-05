#include <motor_controller/IRSpeedFromPositionFilter.hpp>

using namespace motor_controller;

IRSingleSpeedFromPositionFilter::IRSingleSpeedFromPositionFilter(float factor)
    : mFactor(factor)
{
}

bool IRSingleSpeedFromPositionFilter::update(base::Time time, base::JointState& state)
{
    if (!state.hasPosition())
        throw std::runtime_error("IRSingleSpeedFromPositionFilter::update - no position in sample");

    if (mLastUpdate.isNull())
    {
        mLastUpdate = time;
        mLastState = state;
        return false;
    }

    float current = (state.position - mLastState.position) / (time - mLastUpdate).toSeconds();
    mLastUpdate = time;
    if (mLastState.hasSpeed())
    {
        state.speed = mFactor * current + (1 - mFactor) * mLastState.speed;
        mLastState = state;
        return true;
    }
    else
    {
        mLastState = state;
        mLastState.speed = current;
        return false;
    }
}


IRSpeedFromPositionFilter::IRSpeedFromPositionFilter(float factor, size_t size)
    : mFactor(factor)
{
    mFilters.resize(size, IRSingleSpeedFromPositionFilter(factor));
}

bool IRSpeedFromPositionFilter::update(base::samples::Joints& joints, bool force)
{
    if (mFilters.empty())
        mFilters.resize(joints.size(), IRSingleSpeedFromPositionFilter(mFactor));
    else if (joints.states.size() != mFilters.size())
        throw std::runtime_error("IRSpeedFromPositionFilter::update - the number of states in the sample changed");

    size_t size = joints.states.size();
    bool hasResult = true;
    for (size_t i = 0; i < size; ++i)
    {
        if (joints.states[i].hasSpeed() && !force)
            continue;
        if (!mFilters[i].update(joints.time, joints.states[i]))
            hasResult = false;
    }

    return hasResult;
}

