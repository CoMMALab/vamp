// (c) Verify cricket's NATIVELY-generated sphere_fk_pretrig (emitted through the trace).
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "panda_native.hh"
using R = vamp::robots::PandaNative;
constexpr std::size_t rake = vamp::FloatVectorWidth, dim = R::dimension, ns = R::n_spheres;
using DataV = vamp::FloatVector<rake>;
int main(){
  std::mt19937 rng(11); std::uniform_real_distribution<float> u(0,1),nd(-1,1); float range=1.25f;
  std::size_t n=(std::size_t)std::ceil(range*32.0f/rake);
  R::ConfigurationBlock<rake> s0; for(std::size_t j=0;j<dim;++j) s0[j]=static_cast<DataV>(u(rng));
  R::scale_configuration_block<rake>(s0);
  std::array<float,dim> st,v; float nr=0; for(std::size_t j=0;j<dim;++j){st[j]=s0[j].to_array()[0];v[j]=nd(rng);nr+=v[j]*v[j];}
  nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j)v[j]*=range/nr; float dt=1.0f/(n*rake);
  std::vector<R::ConfigurationBlock<rake>> B(n);
  for(std::size_t r=0;r<n;++r)for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
    for(std::size_t l=0;l<rake;++l)ln[l]=st[j]+static_cast<float>(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
  std::array<float,dim> C8{},S8{}; for(std::size_t j=0;j<dim;++j){float d8=rake*dt*v[j];C8[j]=std::cos(d8);S8[j]=std::sin(d8);}
  R::Spheres<rake> os,orr; std::array<DataV,dim> ps{},pc{}; float me=0;
  for(std::size_t r=0;r<n;++r){R::sphere_fk<rake>(B[r],os);
    if(r==0)for(std::size_t j=0;j<dim;++j){ps[j]=::sin(B[0][j]);pc[j]=::cos(B[0][j]);}
    else for(std::size_t j=0;j<dim;++j){DataV c=DataV::fill(C8[j]),s=DataV::fill(S8[j]);DataV sn=ps[j]*c+pc[j]*s,cn=pc[j]*c-ps[j]*s;ps[j]=sn;pc[j]=cn;}
    R::sphere_fk_pretrig<rake>(B[r],ps,pc,orr);
    for(std::size_t sp=0;sp<ns;++sp){auto e=(os.x[sp]-orr.x[sp]).to_array();for(std::size_t l=0;l<rake;++l)me=std::max(me,std::abs(e[l]));}}
  constexpr int K=60000; auto med=[&](auto fn){std::vector<double> t;volatile float sk=0;for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();float ac=fn();auto z=std::chrono::steady_clock::now();sk+=ac;t.push_back(std::chrono::duration<double>(z-a).count()/K*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
  R::Spheres<rake> o;
  double tf=med([&]{float ac=0;for(int k=0;k<K;++k)for(std::size_t r=0;r<n;++r){R::sphere_fk<rake>(B[r],o);ac+=o.x[0].to_array()[0];}return ac;});
  double tr=med([&]{float ac=0;for(int k=0;k<K;++k){std::array<DataV,dim> lp{},lc{};
    for(std::size_t r=0;r<n;++r){if(r==0){for(std::size_t j=0;j<dim;++j){lp[j]=::sin(B[0][j]);lc[j]=::cos(B[0][j]);}}
      else{for(std::size_t j=0;j<dim;++j){DataV c=DataV::fill(C8[j]),s=DataV::fill(S8[j]);DataV sn=lp[j]*c+lc[j]*s,cn=lc[j]*c-lp[j]*s;lp[j]=sn;lc[j]=cn;}}
      R::sphere_fk_pretrig<rake>(B[r],lp,lc,o);ac+=o.x[0].to_array()[0];}}return ac;});
  std::printf("NATIVE (cricket-traced) panda n=%zu: stock FK=%.1f recur FK=%.1f speedup=%.2fx err=%.1e\n",n,tf,tr,tf/tr,me);
}
