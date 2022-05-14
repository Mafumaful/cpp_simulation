#include <iostream>
#include <Eigen/Dense>

class extend_filter_kalman
{
private:
    Eigen::MatrixXd H_lidar;
    Eigen::MatrixXd F_lidar;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
public:
    extend_filter_kalman(double t);
    ~extend_filter_kalman(){};
    void update_ekf(Eigen::Vector2d z, Eigen::Vector4d u, Eigen::Vector4d &x_hat, Eigen::Matrix4d &P_hat);
    Eigen::MatrixXd calculatejacobian (const Eigen::Vector4d& x_state);
};

extend_filter_kalman::extend_filter_kalman(double t)
{
    H_lidar = Eigen::MatrixXd(2,4);
    H_lidar<< 1, 0, 0, 0,
              0, 1, 0, 0;
    F_lidar = Eigen::MatrixXd(4,4);
    F_lidar<<1,t,0,0,
            0,1,0,0,
            0,0,1,t,
            0,0,0,1;
    Q = Eigen::MatrixXd(4,4);
    int noise =9;
    Q<<pow(t,4)/4*noise,pow(t,3)/2*noise,0,0,
        0,0,pow(t,4)/4*noise,pow(t,3)/2*noise,
        pow(t,3)/2*noise,pow(t,2)*noise,0,0,
        0,0,pow(t,3)/2*noise,pow(t,2)*noise;
    R = Eigen::MatrixXd(2,2);
    R = Eigen::MatrixXd::Identity(2,2)*0.025;
}

Eigen::MatrixXd extend_filter_kalman::calculatejacobian (const Eigen::Vector4d& x_state)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd(2,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    if (pow(px,2)+pow(py,2)<0.00001)
    {
        jacobian<<0,0,0,0,
                  0,0,0,0;
    }
    else
    {
        jacobian << px/sqrt(px*px+py*py),0, py/sqrt(px*px+py*py), 0,
                -py/sqrt(px*px+py*py), 0, px/sqrt(px*px+py*py), 0;
    }
    H_lidar = jacobian;
    return jacobian;
}

// update ekf
void extend_filter_kalman::update_ekf(Eigen::Vector2d z, Eigen::Vector4d u, Eigen::Vector4d &x_hat, Eigen::Matrix4d &P_hat)
{
    // predict
    x_hat = F_lidar * x_hat + u;
    P_hat = F_lidar * P_hat * F_lidar.transpose() + Q;

    // update
    Eigen::Vector2d z_pred ;
    z_pred<<sqrt(x_hat(0)*x_hat(0)+x_hat(1)*x_hat(1)),
                atan2(x_hat(1),x_hat(0));
    Eigen::MatrixXd S = H_lidar * P_hat * H_lidar.transpose() + R;
    Eigen::MatrixXd K = P_hat * H_lidar.transpose() * S.inverse();
    x_hat = x_hat + K * (z - z_pred);
    P_hat = (Eigen::MatrixXd::Identity(4,4) - K * H_lidar) * P_hat;
}

int main(int argc, char const *argv[])
{
    double dt =0.02;
    extend_filter_kalman* ekf = new extend_filter_kalman(dt);

    ekf->calculatejacobian(Eigen::Vector4d(1,1,1,1));
    Eigen::Matrix4d p_init;
    p_init<<1,0,0,0,
            0,1,0,0,
            0,0,1000,0,
            0,0,0,1000;
    Eigen::Vector4d x_init = Eigen::Vector4d::Zero();
    ekf->update_ekf(Eigen::Vector2d(1,1),Eigen::Vector4d::Zero(),x_init,p_init);

}