// Trick 1: renormalization buys back the phasor-cascade FP drift on long edges.
// Every R steps, pull each phasor back to the unit circle with a first-order correction
// w *= (3 - |w|^2)/2  (Newton step for 1/sqrt(|w|^2), ~4 ops/phasor, no sqrt). Cubic cascade.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

int main()
{
    std::mt19937 rng(7); std::uniform_real_distribution<float> u(-1.f, 1.f);
    constexpr std::size_t dim = 7; const int D = 3;   // cubic
    for (std::size_t n : {std::size_t(48), std::size_t(96), std::size_t(192)})
    {
        std::vector<std::array<double, dim>> theta(n); std::array<std::array<double,6>,dim> coef{};
        for (std::size_t j=0;j<dim;++j){ coef[j][0]=u(rng)*3.0; for(int d=1;d<=D;++d) coef[j][d]=u(rng)*0.05/std::pow((double)n,d-1); }
        for (std::size_t k=0;k<n;++k) for(std::size_t j=0;j<dim;++j){ double t=(double)k,v=0; for(int d=D;d>=0;--d)v=v*t+coef[j][d]; theta[k][j]=v; }
        std::vector<std::array<float,dim>> S(n),C(n);
        for(std::size_t k=0;k<n;++k) for(std::size_t j=0;j<dim;++j){S[k][j]=std::sin(theta[k][j]);C[k][j]=std::cos(theta[k][j]);}
        auto cmul=[](std::array<float,2> a,std::array<float,2> b){return std::array<float,2>{a[0]*b[0]-a[1]*b[1],a[0]*b[1]+a[1]*b[0]};};
        auto run=[&](int R){ // R = renorm interval (0 = never)
            std::vector<std::array<std::array<float,2>,dim>> W(D+1);
            for(std::size_t j=0;j<dim;++j) for(int m=0;m<=D;++m){double fd=0;for(int i=0;i<=m;++i){double s=((m-i)&1)?-1:1;double b=std::tgamma(m+1)/(std::tgamma(i+1)*std::tgamma(m-i+1));fd+=s*b*theta[i][j];}W[m][j]={(float)std::cos(fd),(float)std::sin(fd)};}
            float me=0;
            for(std::size_t k=0;k<n;++k){
                for(std::size_t j=0;j<dim;++j) me=std::max({me,std::abs(W[0][j][0]-C[k][j]),std::abs(W[0][j][1]-S[k][j])});
                for(int m=0;m<D;++m) for(std::size_t j=0;j<dim;++j) W[m][j]=cmul(W[m][j],W[m+1][j]);
                if(R>0 && ((k+1)%R==0)) for(int m=0;m<=D;++m) for(std::size_t j=0;j<dim;++j){auto&w=W[m][j];float f=(3.0f-(w[0]*w[0]+w[1]*w[1]))*0.5f;w[0]*=f;w[1]*=f;}
            }
            return me;
        };
        std::printf("n=%zu: err  no-renorm=%.1e | R=16:%.1e | R=8:%.1e | R=4:%.1e | R=1:%.1e\n",
                    n, run(0), run(16), run(8), run(4), run(1));
    }
    // cost of renorm (relative)
    return 0;
}
