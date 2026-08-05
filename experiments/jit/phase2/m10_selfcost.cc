#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "fetch_base.hh"      // full: FK + env + self
#include "fetch_envbase.hh"   // env-only: FK + env
constexpr std::size_t rake=vamp::FloatVectorWidth; using DataV=vamp::FloatVector<rake>;
using EnvV=vamp::collision::Environment<DataV>;
static EnvV scene(std::mt19937&rng,int n,float ext){vamp::collision::Environment<float> ef;
  std::uniform_real_distribution<float> p(-ext,ext),rad(0.02F,0.06F);
  for(int i=0;i<n;++i)ef.spheres.emplace_back(vamp::collision::factory::sphere::array({p(rng),p(rng),p(rng)},rad(rng)));ef.sort();return EnvV(ef);}
static auto mblk(std::mt19937&rng){std::uniform_real_distribution<float> u(0,1);std::normal_distribution<float> st(0,0.02F);
  vamp::robots::FetchBase::ConfigurationBlock<rake> b;for(std::size_t j=0;j<8;++j){float base=u(rng),d=st(rng);
  alignas(vamp::FloatVectorAlignment) std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=base+(float(l)-3.5F)*d;b[j]=DataV(ln.data());}
  vamp::robots::FetchBase::scale_configuration_block<rake>(b);return b;}
template<class F> static double med(std::vector<vamp::robots::FetchBase::ConfigurationBlock<rake>>&blk,F f){
  std::vector<double> t;volatile std::uint64_t s=0;for(int r=0;r<9;++r){auto a=std::chrono::steady_clock::now();std::uint64_t acc=0;
  for(auto&b:blk)acc+=f(b);auto z=std::chrono::steady_clock::now();s+=acc;t.push_back(std::chrono::duration<double>(z-a).count()/blk.size()*1e9);}(void)s;std::sort(t.begin(),t.end());return t[t.size()/2];}
int main(){
  constexpr std::size_t M=200000; std::mt19937 wrng(0x33);
  std::vector<vamp::robots::FetchBase::ConfigurationBlock<rake>> blk(M);for(auto&b:blk)b=mblk(wrng);
  vamp::robots::FetchBase::Spheres<rake> sph;
  double tfk=med(blk,[&](auto&b){vamp::robots::FetchBase::sphere_fk<rake>(b,sph);return sph.x[0].to_array()[0]>0;});
  for(auto[n,ext]:{std::pair<int,float>{20,2.0F},{200,0.6F},{800,0.6F}}){
    std::mt19937 srng(0xBEEF);auto env=scene(srng,n,ext);
    std::size_t coll=0;for(auto&b:blk)if(!vamp::robots::FetchBase::fkcc<rake>(env,b))++coll;
    double tenv=med(blk,[&](auto&b){return vamp::robots::FetchEnvbase::fkcc<rake>(env,b)?1:0;});
    double tfull=med(blk,[&](auto&b){return vamp::robots::FetchBase::fkcc<rake>(env,b)?1:0;});
    std::printf("n=%d ext=%.1f frac=%.3f | fk=%.1f  env_only=%.1f  full=%.1f ns | self_cost=%.1f  env_cost=%.1f\n",
      n,ext,double(coll)/M,tfk,tenv,tfull,tfull-tenv,tenv-tfk);
  }
}
