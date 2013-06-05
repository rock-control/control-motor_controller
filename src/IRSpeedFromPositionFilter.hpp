#ifndef MOTOR_CONTROLLER_IR_SPEED_FROM_POSITION_FILTER_HPP
#define MOTOR_CONTROLLER_IR_SPEED_FROM_POSITION_FILTER_HPP

#include <base/samples/Joints.hpp>

namespace motor_controller
{
    class IRSingleSpeedFromPositionFilter
    {
        float mFactor;
        base::Time mLastUpdate;
        base::JointState mLastState;

    public:
        IRSingleSpeedFromPositionFilter(float factor);
        bool update(base::Time time, base::JointState& state);
    };

    /** This simple class applies a IR filter on a JointState to estimate the
     * velocity. It will work only on the JointState objects that do not have
     * the velocity set
     */
    class IRSpeedFromPositionFilter
    {
        float mFactor;
        std::vector<IRSingleSpeedFromPositionFilter> mFilters;

    public:
        /**
         * @param factor the IR filter factor. A velocity update is factor *
         * new + (1-factor)*old
         */
        IRSpeedFromPositionFilter(float factor, size_t size = 0);

        /** Update the velocities in the given structure, modifying it, if
         * possible
         *
         * @param force if true, all velocities are updated. Otherwise, only
         *   the ones that are currently unset will be computed
         * @return true if the velocities have been updated (there was enough
         *   information to compute it) and false otherwise.
         */
        bool update(base::samples::Joints& joints, bool force = false);
    };
}

#endif

