#include "../include/GradientDescentSolver.h"

GradientDescentSolver::GradientDescentSolver(const Eigen::VectorXd &init_vec) {
    m_init_vec = init_vec;

    Process();
}

GradientDescentSolver::GradientDescentSolver(std::vector<double> &init_vec) {
    int length = int(init_vec.size());
    m_init_vec.resize(length);

    for (int i = 0; i < length; i++){
        m_init_vec[i] = init_vec[i];
    }

    Process();
}

GradientDescentSolver::GradientDescentSolver(int n) {
    m_init_vec = Eigen::VectorXd::Random(n);

    Process();
}

double GradientDescentSolver::RosenbrockFunc(const Eigen::VectorXd &vectorXd) {
    double ans = 0.0;
    int n = int(vectorXd.size());
    for (int i = 0; i < n / 2; i++) {
        ans += 100 * pow(vectorXd[2 * i] * vectorXd[2 * i] - vectorXd[2 * i + 1], 2)
                + pow(vectorXd[2 * i] - 1, 2);
    }
    return ans;
}

Eigen::VectorXd GradientDescentSolver::RosenbrockDerivative(const Eigen::VectorXd &vectorXd) {
    int n = int(vectorXd.size());
    Eigen::VectorXd res(n);
    for (int i = 0; i < n / 2; i ++) {
        res[i] = 400 * pow(vectorXd[i], 3)
                - 400 * vectorXd[i] * vectorXd[i + 1] + 2 * vectorXd[i] - 2;
        res[i + 1] = -200 * pow(vectorXd[i], 2) + 200 * vectorXd[i + 1];
    }
    return res;
}

void GradientDescentSolver::Process() {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    if (m_init_vec.size() < 1) {
        std::cout << "Error! Init value is valid." << std::endl;
        std::exit(0);
    }

    int iter = 0;
    double tau = 0.1;
    while(iter < m_max_iter && RosenbrockDerivative(m_init_vec).norm() > m_precision) {
        Eigen::VectorXd direction = -1 * RosenbrockDerivative(m_init_vec);
        if (m_init_vec.size() != direction.size()) {
            std::cout << "Error! size is not equal";
            std::exit(1);
        }

        double left = RosenbrockFunc(m_init_vec + tau * direction);
        double right = RosenbrockFunc(m_init_vec)
                + m_step * tau  * direction.transpose() * RosenbrockDerivative(m_init_vec);
        if (left > right) tau = tau / 2;
        m_init_vec += tau * direction;
        // added by me
        if (RosenbrockFunc(m_init_vec) > RosenbrockFunc(m_init_vec - tau * direction)) {
            m_init_vec -= tau * direction;
        }
        iter += 1;

        // debug
        std::cout << "iter: " << iter << ", " << "value: " << RosenbrockFunc(m_init_vec) << std::endl;
        for (int i = 0; i < int(m_init_vec.size()); i++) {
            std::cout<< m_init_vec[i];
            if (i < int(m_init_vec.size()) - 1) std::cout<< ", ";
        }
        std::cout << std::endl << std::endl;

    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> time_duration = end - start;
    std::cout << "time used: " << time_duration.count() << " seconds" <<std::endl;
    std::cout << "min f(x) = " << RosenbrockFunc(m_init_vec) << std::endl;
    std::cout << "final vector: ";
    for (int i = 0; i < int(m_init_vec.size()); i++) {
        std::cout<< m_init_vec[i];
        if (i < int(m_init_vec.size()) - 1) std::cout<< ", ";
    }
    std::cout << std::endl;
    std::cout << "final iter: " << iter;
}




