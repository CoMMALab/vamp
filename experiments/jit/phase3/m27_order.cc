// What would order-M per-sphere recurrence take? Measure accumulated error for M=3..6
// with BEST-CASE coefficients (least-squares fit over the whole edge). Also states cost:
// order-M ~ (2M-1)*3 ops/sphere vs FK effective ~10-14 ops/sphere.
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "fetch_base.hh"

using R = vamp::robots::FetchBase;
constexpr std::size_t rake = vamp::FloatVectorWidth, dim = R::dimension, ns = R::n_spheres;
using DataV = vamp::FloatVector<rake>;

int main()
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> u(0.f, 1.f), nd(-1.f, 1.f);
    for (float L : {0.5f, 1.0f, 1.5f})
    {
        const std::size_t n = 20;
        R::ConfigurationBlock<rake> s0;
        for (std::size_t j = 0; j < dim; ++j) s0[j] = static_cast<DataV>(u(rng));
        R::scale_configuration_block<rake>(s0);
        std::array<float, dim> st, v; float nr = 0;
        for (std::size_t j = 0; j < dim; ++j) { st[j] = s0[j].to_array()[0]; v[j] = nd(rng); nr += v[j]*v[j]; }
        nr = std::sqrt(nr); for (std::size_t j = 0; j < dim; ++j) v[j] *= L / nr;
        float dt = 1.0f / static_cast<float>(n * rake);
        // lane-0 true positions per sphere per coord along edge
        std::vector<std::array<std::vector<double>,3>> P(ns);
        for (std::size_t s=0;s<ns;++s) for(int c=0;c<3;++c) P[s][c].resize(n);
        R::Spheres<rake> sp;
        for (std::size_t r = 0; r < n; ++r) {
            R::ConfigurationBlock<rake> b;
            for (std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=st[j]+static_cast<float>(r*rake+l)*dt*v[j];b[j]=DataV(ln.data());}
            R::sphere_fk<rake>(b, sp);
            for (std::size_t s=0;s<ns;++s){P[s][0][r]=sp.x[s].to_array()[0];P[s][1][r]=sp.y[s].to_array()[0];P[s][2][r]=sp.z[s].to_array()[0];}
        }
        std::printf("L=%.2f n=%zu:", L, n);
        for (std::size_t M : {std::size_t(3),std::size_t(4),std::size_t(5),std::size_t(6)})
        {
            double gmax = 0;
            for (std::size_t s=0;s<ns;++s){
                // best coord by variation
                int bc=0; double bv=0; for(int c=0;c<3;++c){auto&a=P[s][c];double mn=*std::min_element(a.begin(),a.end()),mx=*std::max_element(a.begin(),a.end());if(mx-mn>bv){bv=mx-mn;bc=c;}}
                if (bv<1e-6) continue;
                // LSQ fit order-M coeffs over full edge on best coord: p_k = sum c_i p_{k-i}
                std::size_t rows=n-M; Eigen::MatrixXd A(rows,M); Eigen::VectorXd y(rows);
                for(std::size_t r=M;r<n;++r){for(std::size_t i=0;i<M;++i)A(r-M,i)=P[s][bc][r-1-i];y(r-M)=P[s][bc][r];}
                Eigen::VectorXd c = A.colPivHouseholderQr().solve(y);
                // run recurrence from M seeds on all 3 coords, measure accumulated error
                for(int cc=0;cc<3;++cc){std::vector<double> q(P[s][cc].begin(),P[s][cc].begin()+M); q.resize(n);
                    for(std::size_t r=M;r<n;++r){double acc=0;for(std::size_t i=0;i<M;++i)acc+=c(i)*q[r-1-i];q[r]=acc; gmax=std::max(gmax,std::abs(q[r]-P[s][cc][r]));}}
            }
            std::printf("  M=%zu err=%.1e(%zu ops/sph)", M, gmax, (2*M-1)*3);
        }
        std::printf("\n");
    }
    return 0;
}
