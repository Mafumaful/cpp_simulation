#include "matplotlibcpp.h"
#include <string>
#include <Eigen/Dense>

namespace plt = matplotlibcpp;
using namespace Eigen;
using namespace std;

class draw3D
{
private:
    vector<double> _x, _y, _z;
    string _str;

public:
    void input_vector(Vector3d);
    void plot(void);
    draw3D(string str);
    ~draw3D();
};

void draw3D::input_vector(Vector3d a)
{
    _x.push_back(a[0]);
    _y.push_back(a[1]);
    _z.push_back(a[2]);
}

void draw3D::plot()
{
    map<string, string> keywords3;
    keywords3.insert(pair<string, string>("label", _str));
    plt::plot3(_x, _y, _z, keywords3);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();

    plt::show();
}

draw3D::draw3D(string str) : _str(str)
{
}

draw3D::~draw3D()
{
}