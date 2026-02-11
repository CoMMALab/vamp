#pragma once
#include <Eigen/Dense>
#include <vector>


namespace vamp::planning {
    using row_matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using state = Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor>;

    // maybe optimize this, i.e. compute once and store somewhere
    inline static int comb(int n, int k) {
        int n_fact = 1;
        int k_fact = 1;
        int n_k_fact = 1;

        for (int i = 1; i <= n; i++) {
            if (i <= k) {
                k_fact *= i;
            }

            if (i <= (n - k)) {
                n_k_fact *= i;
            }
            n_fact *= i;
        }
        return n_fact / (k_fact * n_k_fact);
    }

    class Bezier {
        public:
            row_matrix anchors;
            int degree;
            std::array<int, 10> combs;

            Bezier(row_matrix anchors) noexcept {
                this->anchors = anchors;
                this->degree = anchors.rows() - 1;
                // compute bez comb
                for (int i = 0; i <= this->degree; i++) {
                    this->combs[i] = comb(this->degree, i);
                }
            }

            std::vector<state> generate_trajectory(int T) {
                std::vector<state> traj;
                for (int t = 0; t <= T; t++) {
                    state P(1, this->anchors.rows());
                    for (int i = 0; i <= this->degree; i++) {
                        P(0, i) = ((this->combs[i] * 
                        (pow(1.0 - t * 1.0 / T, this->degree - i)) * 
                        (pow(1.0 * t / T, i))));
                    }
                    state s = P * this->anchors;
                    traj.push_back(s);
                }
                return traj;
            }
            
            state evaluate(float t) {
                state P(1, this->anchors.rows());
                for (int i = 0; i <= this->degree; i++) {
                    P(0, i) = ((this->combs[i] * 
                    (pow(1.0 - t * 1.0, this->degree - i)) * 
                    (pow(1.0 * t, i))));
                }
                state s = P * this->anchors;
                return s;
            }

            Bezier derivative() {
                // analytically, derivative of curve B(t) is B'(t) = P_{n - 1}Q
                // where Q_i = n(C_{i + 1} - C_{i})
                row_matrix Q(this->degree, this->anchors.cols());
                for (int i = 0; i < this->degree; i++) {
                    Q.row(i) = this->degree * (this->anchors.row(i + 1) - this->anchors.row(i));
                }
                Bezier dB(Q);
                return dB;
            }
    };
}