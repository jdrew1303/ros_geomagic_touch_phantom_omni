#include "RobotBase/RobotBase.h"

RobotBase::RobotBase() {}

Eigen::Matrix3d RobotBase::makeSkewMatrix(Eigen::Vector3d vector)
{
    Eigen::Matrix3d tempMatrix;
    tempMatrix <<    0,      -vector(2),     vector(1),
                  vector(2),      0,        -vector(0),
                  -vector(1), vector(0),        0      ;
    return tempMatrix;
}

Eigen::Matrix4d RobotBase::fwdKin( Eigen::VectorXd theta, int jointIndex )
{
    Eigen::Matrix3d Rot;
    Eigen::Vector3d pos;
    if ( type(0) == 0 )
    {
        Rot = (makeSkewMatrix(this->jointH.col(0)) * theta(0)).exp();
        pos = this->links.col(0);
    }
    else
    {
        Rot << 1,0,0,
               0,1,0,
               0,0,1;
        pos = this->links.col(0) + theta(1)*this->jointH.col(0);
    }

    for (int i = 1; i < jointIndex; i++)
    {
        if ( type(i) == 0 )
        {
            pos += Rot*this->links.col(i);
            Rot *= (makeSkewMatrix(this->jointH.col(i)) * theta(i)).exp();
        }
        else
        {
            pos += Rot * ( this->links.col(i) + theta(i)*this->jointH.col(i) );
        }
    }
    this->homogenous << Rot, pos, 0, 0, 0, 1;
    if (jointIndex == this->type.rows())
    {
        return this->Tb * this->homogenous *this->Te;
    }

    return this->Tb * this->homogenous;
}

Eigen::Matrix4d RobotBase::fwdKin(Eigen::VectorXd theta)
{
    return fwdKin(theta, this->type.rows());
}
