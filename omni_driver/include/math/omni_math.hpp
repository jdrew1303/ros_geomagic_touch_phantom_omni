#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include "../include/util/typedefs.hpp"

class OmniMath {
    private:

    MovingAverage moving_average(5,6);

    const bool enable_moving_average;

    public:

    template <typename Type>
    Type saturate(Type value, Type min, Type max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    template <typename Type>
    std::vector<Type> saturate(std::vector<Type> values, Type min, Type max) {
        std::vector<Type> s(values.size())
        for (auto iter = values.begin(); iter < values.end(); ++iter) {
            s[iter-values.begin()] = saturate(*iter, min, max);
        }
        return s;
    }

    Eigen::Affine3d fowardKinematics(
        robot_state::RobotStatePtr kinematic_state,
        std::string link_name) {
            return kinematic_state->getGlobalLinkTransform(link_name);
    }

    Eigen::Vector3d OmniBase::calculateTorqueFeedback(
        robot_state::RobotStatePtr kinematic_state,
        robot_state::JointModelGroup* joint_model_group, 
        //const Eigen::Vector3d& force,
        const Eigen::Vector3d force,
        Eigen::Matrix3d rot_link_to_teleop,
        double feedback_gain,
        std::string end_effector_name,
        const robot_state::LinkModel* end_effector_link_model) {
            if (feedback_gain == 0)
                return Eigen::Vector3d::Zero();

            // Get the Jacobian matrix written on the base frame
            Eigen::Vector3d origin(0,0.08,0);
            Eigen::MatrixXd jacobian;
            auto link_model = kinematic_state->getLinkModel(end_effector_name);
            kinematic_state->getJacobian(joint_model_group, link_model, origin, jacobian, false);

            // Rotate the force vector from the desired frame to the base frame
            Eigen::Vector3d force_on_link_frame = rot_link_to_teleop * force;
            //
            auto quat_base_link = fowardKinematics(kinematic_state, end_effector_name)
            force_on_link_frame = quat_base_link.conjugate() * force_on_link_frame;

            auto ret = feedback_gain * jacobian.block<3,3>(0,0).transpose() * force_on_link_frame;
            return ret.block<3,1>(0,0);
        }

    Eigen::VectorXd jerkSaturationFilter(
        Eigen::VectorXd previous_calculated_velocities,
        Eigen::VectorXd current_calculated_velocities,
        double max_jerk) {
            std:vector<double> saturated_velocities(6);
            for (int i = 0; i < 6; ++i) {
                if ( std::abs(current_calculated_velocities[i] - previous_calculated_velocites[i]) < max_jerk ) {
                    saturated_velocities[i] = current_calculated_velocities[i];
                }
                else {
                    saturated_velocities[i] = previous_calculated_velocities[i];
                }
            }
            Eigen::VectorXd eigen_vector(saturated_velocities.data());
            return eigen_vector;
        }

    Eigen::VectorXd calculateVelocities(
        Time previous_measurement_time,
        Time current_measurement_time,
        Eigen::VectorXd previous_measurement,
        Eigen::VecotrXd current_measurement,
        bool enable_moving_average) {
            double delta_angle;
            double delta_t = (current_measurement_time - previous_measurement_time).total_microseconds();
            std::vector<double> calculated_velocities(6);
            for (int i = 0; i < 6; ++i) {
                deltaAngle = current_measurement[i] - previous_measurement[i];
                calculated_velocities[i] = deltaAngle * 1000000 / dt;
            }
            if (enable_moving_average) {
                moving_average.input(calculated_velocities);
                Eigen::VectorXd joint_velocities(moving_average.mean.data());
            }
            else Eigen::VectorXd joint_velocities(calculated_velocities.data());
            return joint_velocities(6);
        }

    Eigen::VectorXd calculateTwist(
        robot_state::RobotStatePtr kinematic_state,
        robot_state::JointModelGroup* joint_model_group,
        robot_state::LinkModel* end_effector_link_model,
        Eigen::VectorXd current_joint_velocities) {
        std::vector<double> twist;
        Eigen::Vector3d origin(0,0,0);
        Eigen::MatrixXd jacobian(6,6);
        kinematic_state->getJacobian(joint_model_group, end_effector_link_model, origin, jacobian, false);
        return jacobian * current_joint_velocities;
    }

public:
    /**
     * @brief OmniBase constructor, sets some members and prepares ros topics and publishers.
     * @param name Reference to string of omni name.
     */
    explicit OmniMath(const bool enable_moving_average);
};