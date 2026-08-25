#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>


namespace vamp::planning {
    using row_matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using state = Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor>;

    // maybe optimize this, i.e. compute once and store somewhere
    inline static int comb(int n, int k) {
        if (k < 0 || k > n) return 0;
        if (k == 0 || k == n) return 1;
        if (k > n / 2) k = n - k; // Optimize by taking advantage of symmetry

        long long res = 1;
        for (int i = 1; i <= k; ++i) {
            res = res * (n - i + 1) / i;
        }
        return res;
    }

    class Bezier {
        public:
            row_matrix anchors;
            int degree;
            std::array<int, 24> combs;
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

            std::vector<state> generate_trajectory() {
                std::vector<state> traj;
                int time_ms = (int) (this->time * 1000);
                for (int t = 0; t <= time_ms; t++) {
                    state P(1, this->anchors.rows());
                    for (int i = 0; i <= this->degree; i++) {
                        P(0, i) = ((this->combs[i] * 
                        (pow(1.0 - t * 1.0 / time_ms, this->degree - i)) * 
                        (pow(1.0 * t / time_ms, i))));
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
                    Q.row(i) = this->degree * (this->anchors.row(i + 1) - this->anchors.row(i)) / this->time;
                }
                Bezier dB(Q);
                dB.time = this->time;
                return dB;
            }

            // implements de casteljaus alg 
            std::pair<Bezier, Bezier> subdivide(float t) {
                // let C[i][j] be control point i on the jth iteration for the new curve
                // std::cout << "in subdivide" << std::endl;
                std::vector<std::vector<state>> C;
                for (int i = 0; i <= this->degree; i++) {
                    C.push_back(std::vector<state>(this->degree + 1));
                }
                // fill initial layer j = 0
                for (int i = 0; i <= this->degree; i++) {
                    C[i][0] = state(this->anchors.row(i));
                }
                // fill remaining layers j = 1, 2, ...
                for (int j = 1; j <= this->degree; j++) {
                    for (int i = 0; i <= this->degree - j; i++) {
                        C[i][j] = state(C[i][j - 1] * (1 - t) + C[i + 1][j - 1] * t);
                    }
                }

                // we want C[0][j]
                row_matrix left_anchors(this->anchors.rows(), this->anchors.cols());
                for (int j = 0; j <= this->degree; j++) {
                    left_anchors.row(j) = C[0][j];
                }
                // also find the right subcurve
                row_matrix right_anchors(this->anchors.rows(), this->anchors.cols());
                for (int j = 0; j <= this->degree; j++) {
                    right_anchors.row(j) = C[j][this->degree - j];
                }
                Bezier left_sub_bez(left_anchors);
                Bezier right_sub_bez(right_anchors);
                left_sub_bez.time = this->time * t;
                right_sub_bez.time = this->time * (1 - t);
                return {left_sub_bez, right_sub_bez};
            }

            void reverse() {
                // reverse the order of the anchors
                row_matrix new_anchors(this->anchors.rows(), this->anchors.cols());
                for (int i = 0; i <= this->degree; i++) {
                    new_anchors.row(i) = this->anchors.row(this->degree - i);
                }
                this->anchors = new_anchors;
            }

            void smoothen(state v0_target, state a0_target, state v1_target, state a1_target, bool smooth_both) {
                // slightly alter the curve generated by fwd pass to correct for discontinuity in H.O.T.
                // correct for discontinuity before collision check
                
                // analytically calculated velocity correction, modifies x[1]
                this->anchors.row(1) = this->anchors.row(0) + v0_target * this->time / this->degree;
                if (smooth_both) {
                    this->anchors.row(this->degree - 1) = this->anchors.row(this->degree) - v1_target * this->time / this->degree;
                }

                // analytically calculated acceleration correction, modifies x[2]
                this->anchors.row(2) = this->anchors.row(1) + (this->time / this->degree) * (a0_target * std::pow(this->time, 2) / (this->degree - 1) + v0_target);
                if (smooth_both) {
                    this->anchors.row(this->degree - 2) = this->anchors.row(this->degree - 1) + (this->time / this->degree) * (a1_target * std::pow(this->time, 2) / (this->degree - 1) - v1_target);
                }
            }
    };
}