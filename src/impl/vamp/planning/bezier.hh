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
            float time;

            Bezier() = default;

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

            // implements de casteljaus alg 
            Bezier subdivide(Bezier bez, float t) {
                // let C[i][j] be control point i on the jth iteration for the new curve
                // std::cout << "in subdivide" << std::endl;
                std::vector<std::vector<state>> C;
                for (int i = 0; i <= bez.degree; i++) {
                    C.push_back(std::vector<state>(bez.degree + 1));
                }
                // fill initial layer j = 0
                for (int i = 0; i <= bez.degree; i++) {
                    C[i][0] = state(bez.anchors.row(i));
                }
                // fill remaining layers j = 1, 2, ...
                for (int j = 1; j <= bez.degree; j++) {
                    for (int i = 0; i <= bez.degree - j; i++) {
                        C[i][j] = state(C[i][j - 1] * (1 - t) + C[i + 1][j - 1] * t);
                    }
                }

                // we want C[0][j]
                row_matrix new_anchors(bez.anchors.rows(), bez.anchors.cols());
                for (int j = 0; j <= bez.degree; j++) {
                    new_anchors.row(j) = C[0][j];
                }
                Bezier sub_bez(new_anchors);
                sub_bez.time = bez.time * t;
                // std::cout << "end subdivide" << std::endl;
                return sub_bez;
            }
    };
}