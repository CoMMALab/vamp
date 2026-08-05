#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "fetch_base.hh"
#include "fetch_staged.hh"
constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
using EnvV = vamp::collision::Environment<DataV>;

static EnvV make_localized(std::mt19937& rng){
  vamp::collision::Environment<float> ef;
  std::normal_distribution<float> nx(0.55F,0.13F),ny(0.0F,0.13F),nz(0.55F,0.13F);
  std::uniform_real_distribution<float> rad(0.02F,0.06F);
  for(int i=0;i<300;++i) ef.spheres.emplace_back(vamp::collision::factory::sphere::array({nx(rng),ny(rng),nz(rng)},rad(rng)));
  ef.sort(); return EnvV(ef);
}
template<class R>
static auto motion_block(std::mt19937& rng){
  std::uniform_real_distribution<float> u(0,1); std::normal_distribution<float> st(0,0.02F);
  typename R::template ConfigurationBlock<rake> b;
  for(std::size_t j=0;j<R::dimension;++j){float base=u(rng),d=st(rng);
    alignas(vamp::FloatVectorAlignment) std::array<float,rake> ln;
    for(std::size_t l=0;l<rake;++l) ln[l]=base+(float(l)-3.5F)*d; b[j]=DataV(ln.data());}
  R::template scale_configuration_block<rake>(b); return b;
}
int main(){
  std::mt19937 srng(0xBEEF), wrng(0x33);
  auto env = make_localized(srng);
  constexpr std::size_t M=200000;
  std::vector<vamp::robots::FetchBase::ConfigurationBlock<rake>> blk(M);
  for(auto&b:blk) b=motion_block<vamp::robots::FetchBase>(wrng);
  // correctness: base vs staged bit-identical
  std::size_t mm=0, coll=0;
  for(auto&b:blk){bool rb=vamp::robots::FetchBase::fkcc<rake>(env,b);
                  bool rs=vamp::robots::FetchStaged::fkcc<rake>(env,b);
                  if(rb!=rs)++mm; if(!rb)++coll;}
  auto tmed=[&](auto f)->double{std::vector<double> t;volatile std::uint64_t s=0;
    for(int r=0;r<7;++r){auto a=std::chrono::steady_clock::now();std::uint64_t acc=0;
      for(auto&b:blk)acc+=f(b);auto z=std::chrono::steady_clock::now();s+=acc;
      t.push_back(std::chrono::duration<double>(z-a).count()/M*1e9);}(void)s;
    std::sort(t.begin(),t.end());return t[t.size()/2];};
  double tb=tmed([&](auto&b){return vamp::robots::FetchBase::fkcc<rake>(env,b)?1:0;});
  double ts=tmed([&](auto&b){return vamp::robots::FetchStaged::fkcc<rake>(env,b)?1:0;});
  std::printf("scene=localized workload=motion M=%zu collision_frac=%.3f mismatches=%zu\n",M,double(coll)/M,mm);
  std::printf("base_fkcc=%.2f ns  staged_fkcc=%.2f ns  speedup=%.3f\n",tb,ts,tb/ts);
}
