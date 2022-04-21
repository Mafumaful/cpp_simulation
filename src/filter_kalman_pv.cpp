#include "matplotlibcpp.h"
#include "kalman.hpp"
#include <cmath>
#include <random>

namespace plt = matplotlibcpp;

Vector3d generate_randown(float n)
{
    std::random_device rd;
    std::default_random_engine rng{rd()};
    std::normal_distribution<double> norm{0, n};
    Vector3d v(norm(rng), norm(rng), norm(rng));
    return v;
}

int main(int argc, char const *argv[])
{

    double dt = 0.01;
    double r_noise = 1;
    double q_noise = 0.05;
    // the true number
    Vector3d p, v, a;
    p << 0, 0, 0;
    v << 2, 3, 10;
    a << 0, 0, -9.8;
    vector<double> x, y, z;
    vector<double> f_x, f_y, f_z;
    vector<double> noise_x, noise_y, noise_z;

    // // position with noise, this is the data we can observe
    Vector3d p_noise = p + generate_randown(r_noise);
    Vector3d a_noise = a + generate_randown(r_noise);

    VectorXd x_hat_init = VectorXd::Zero(9);
    kalmanxd _kf(x_hat_init);

    MatrixXd H(6, 9);
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    MatrixXd R = MatrixXd::Identity(6, 6) * r_noise;

    MatrixXd F(9, 9);
    F << 1, 0, 0, dt, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 1, 0, 0, dt, 0, 0,
        0, 0, 0, 0, 1, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 1, 0, 0, dt,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    MatrixXd Q = MatrixXd::Identity(9, 9) * q_noise;
    MatrixXd P = MatrixXd::Identity(9, 9);

    _kf.initialize(Q, P, H, R);

    for (float i = 0; i < 5; i += dt)
    {
        // this is the true number to be plot
        v += a * dt;
        p += v * dt;

        // position with noise
        p_noise = p + generate_randown(r_noise);
        a_noise = a + generate_randown(r_noise);

        VectorXd vec(6);
        vec << p_noise, a_noise;
        VectorXd filtered = _kf.kalman_measure(vec, F);

        // store the pose to plot
        x.push_back(p[0]);
        y.push_back(p[1]);
        z.push_back(p[2]);

        f_x.push_back(filtered[0]);
        f_y.push_back(filtered[1]);
        f_z.push_back(filtered[2]);

        noise_x.push_back(p_noise[0]);
        noise_y.push_back(p_noise[1]);
        noise_z.push_back(p_noise[2]);
    }

    // plot
    map<string, string> keywords;
    keywords.insert(pair<string, string>("label", "ground true"));
    plt::plot3(x, y, z, keywords);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();

    map<string, string> keywords2;
    keywords2.insert(pair<string, string>("label", "filtered"));
    plt::plot3(f_x, f_y, f_z, keywords2);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();

    map<string, string> keywords3;
    keywords3.insert(pair<string, string>("label", "noise"));
    plt::plot3(noise_x, noise_y, noise_z, keywords3);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();

    plt::show();

    return 0;
}