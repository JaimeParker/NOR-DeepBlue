#include <iostream>
#include <GradientDescentSolver.h>

int main() {
    std::vector<double> a;
    a.push_back(1.0);
    a.push_back(2.0);
    a.push_back(3.0);
    a.push_back(4.0);

    GradientDescentSolver solver(a);


    return 0;
}
