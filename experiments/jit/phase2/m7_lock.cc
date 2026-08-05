#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "fetch_base.hh"
#include "fetch_locked.hh"
constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
template <typename R> double time_fk(const char* name){
  constexpr std::size_t m=200000; std::mt19937 rng(1); std::uniform_real_distribution<float> u(0,1);
  std::vector<typename R::template ConfigurationBlock<rake>> blk(m);
  for(auto&b:blk){for(std::size_t j=0;j<R::dimension;++j){alignas(vamp::FloatVectorAlignment) std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=u(rng);b[j]=DataV(ln.data());}R::template scale_configuration_block<rake>(b);}
  typename R::template Spheres<rake> sph; std::vector<double> t; volatile std::uint64_t sink=0;
  for(int r=0;r<7;++r){auto a=std::chrono::steady_clock::now();std::uint64_t acc=0;for(auto&b:blk){R::template sphere_fk<rake>(b,sph);acc+=sph.x[0].to_array()[0]>0;}auto z=std::chrono::steady_clock::now();sink+=acc;t.push_back(std::chrono::duration<double>(z-a).count()/m*1e9);}
  (void)sink;std::sort(t.begin(),t.end());double med=t[t.size()/2];
  std::printf("%s: dim=%zu n_spheres=%zu  t_fk=%.3f ns\n",name,R::dimension,R::n_spheres,med);
  return med;
}
int main(){
  double b=time_fk<vamp::robots::FetchBase>("base   ");
  double l=time_fk<vamp::robots::FetchLocked>("locked ");
  std::printf("torso-lock FK saving: %.1f%%\n",(b-l)/b*100.0);
}
