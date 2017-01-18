#include "node/Omni.h"

Omni::Omni() : RobotBase()
{
    //Parameters
    Eigen::VectorXd L(4);
    L << 125, 145, 135, 40;
    L = L / 1000;

    //Robot description
    Eigen::VectorXi jtype(6); // RRRRRRR robot
    jtype << 0, 0, 0, 0, 0, 0;

    // Some vectors
    Eigen::Vector3d x,y,z,n;
    x << 1,0,0;
    y << 0,1,0;
    z << 0,0,1;
    n << 0,0,0;

    //Joints
    Eigen::MatrixXd H(3,7);
    H << z, y, y, x, y, x;

    // P
    Eigen::MatrixXd P(3,4);
    P << L(0)* z,
         L(1)* x,
         L(2)* x,
         L(3)*-x;

    // Constant transforms
    Eigen::Matrix4d Tb, Te;
    Tb << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Te <<  1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;

    this->setLinks(P);
    this->setJoints(jtype, H);
    this->setTransforms(Tb, Te);
}
