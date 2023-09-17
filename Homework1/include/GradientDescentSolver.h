#ifndef DEEPBLUEPROJECT_GRADIENTDESCENTSOLVER_H
#define DEEPBLUEPROJECT_GRADIENTDESCENTSOLVER_H

#include <Eigen/Eigen>
#include <iostream>
#include <chrono>

class GradientDescentSolver {
public:
    GradientDescentSolver(const Eigen::VectorXd &init_vec);
    GradientDescentSolver(std::vector<double> &init_vec);
    GradientDescentSolver(int n);

    static double RosenbrockFunc(const Eigen::VectorXd &vectorXd);
    static Eigen::VectorXd RosenbrockDerivative(const Eigen::VectorXd &vectorXd);


    void Process();

private:
    Eigen::VectorXd m_init_vec;
    Eigen::VectorXd m_solved_vec;

    double m_step = 0.1;
    double m_precision = 1e-5;
    int m_max_iter = 100;
};


#endif //DEEPBLUEPROJECT_GRADIENTDESCENTSOLVER_H
