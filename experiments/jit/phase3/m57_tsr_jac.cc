// Verify analytic vs autodiff TSR Jacobian (panda): same error (exact) + same Jacobian, and time.
#include <cstdio>
#include <cmath>
#include <array>
#include <random>
#include <algorithm>
#include <chrono>
#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
using namespace vamp;
constexpr std::size_t rake = FloatVectorWidth;
using V1 = FloatVector<rake,1>;
#include "tsr_both.inc"
// input: q(7) rTe(quat4+t3) wTr(quat4+t3) lb(6) ub(6) = 33 ; output: Jac(42)+err(6)=48
int main(){
  std::mt19937 rng(9); std::normal_distribution<float> nd(0,1); std::uniform_real_distribution<float> u(-2,2);
  const int N=100000;
  std::vector<std::array<V1,33>> xs(N);
  for(auto&x:xs){ auto set=[&](int i,float v){ alignas(FloatVectorAlignment) std::array<float,rake> a; a.fill(v); x[i]=V1(a.data()); };
    for(int j=0;j<7;j++) set(j,u(rng));
    for(int base:{7,14}){ float q[4]={nd(rng),nd(rng),nd(rng),nd(rng)}; float n=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
      for(int k=0;k<4;k++) set(base+k,q[k]/n); for(int k=0;k<3;k++) set(base+4+k,u(rng)*0.3f); }
    for(int j=21;j<33;j++) set(j,0.0f); }
  std::array<V1,48> oa,ob; double jmax=0,emax=0;
  for(int i=0;i<3000;i++){ K::tsr_analytic<rake>(xs[i],oa); K::tsr_autodiff<rake>(xs[i],ob);
    for(int k=0;k<48;k++){ double d=std::abs(oa[k].to_array()[0]-ob[k].to_array()[0]); if(k<42)jmax=std::max(jmax,d); else emax=std::max(emax,d);} }
  auto med=[&](auto fn){ std::vector<double> t; volatile float sk=0;
    for(int rp=0;rp<9;rp++){ std::array<V1,48> o; auto a=std::chrono::steady_clock::now(); for(int i=0;i<N;i++){fn(xs[i],o); sk+=o[0].to_array()[0];}
      auto z=std::chrono::steady_clock::now(); t.push_back(std::chrono::duration<double>(z-a).count()/N*1e9);} (void)sk; std::sort(t.begin(),t.end()); return t[t.size()/2]; };
  double ta=med([](const std::array<V1,33>&x,std::array<V1,48>&o){K::tsr_analytic<rake>(x,o);});
  double tb=med([](const std::array<V1,33>&x,std::array<V1,48>&o){K::tsr_autodiff<rake>(x,o);});
  std::printf("panda TSR err+Jac: max|error diff|=%.2e  max|Jacobian diff|=%.2e\n", emax, jmax);
  std::printf("  autodiff=%.1f ns  analytic=%.1f ns  speedup=%.2fx\n", tb, ta, tb/ta);
}
