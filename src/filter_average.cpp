#include "averagefilter.hpp"
#include "matplotlibcpp.h"
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

    Vector3d p, v, a, r, sun_pose;
    p << 0, 0, 0;
    v << 1, 0, 0;
    a << 0, 0, 0;
    r << 0, 0, 0;
    sun_pose << 5, 5, 5;

    std::vector<double> x, y, z;
    double dt = 0.01;
    double k = 50;
    double len = 0;

    // the true number
    vector<double> f_x, f_y, f_z;
    vector<double> noise_x, noise_y, noise_z;

    averagefilter _avg;
    _avg._set_number(100);
    Vector3d p_noise;

    for (float i = 0; i < 5; i += dt)
    {
        // this is the true number to be plot
        r = sun_pose - p;
        len = sqrt(r.transpose() * r);
        a = k * r / pow(len, 3);
        v += a * dt;
        p += v * dt;

        // position with noise
        p_noise = p + generate_randown(0.1);
        VectorXd filtered = _avg._filt(p_noise);

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