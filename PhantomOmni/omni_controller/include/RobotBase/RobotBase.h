#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

class RobotBase {
private:
    Eigen::VectorXi type;
    Eigen::MatrixXd links;
    Eigen::MatrixXd jointH;
    Eigen::Matrix4d homogenous;
    Eigen::Matrix4d Tb;
    Eigen::Matrix4d Te;

protected:
    inline void setLinks(Eigen::MatrixXd links)
    {
        this->links = links;
    }

    inline void setJoints(Eigen::VectorXi type, Eigen::MatrixXd axes)
    {
        this->type = type;
        this->jointH = axes;
    }

    inline void setTransforms(Eigen::Matrix4d Tb, Eigen::Matrix4d Te)
    {
        this->Tb = Tb;
        this->Te = Te;
    }

public:
    RobotBase();

    Eigen::Matrix4d fwdKin(Eigen::VectorXd theta, int jointIndex);

    Eigen::Matrix4d fwdKin(Eigen::VectorXd theta);

    Eigen::Matrix3d makeSkewMatrix (Eigen::Vector3d vector);
};
