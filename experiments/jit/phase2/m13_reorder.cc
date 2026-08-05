// Reordering potential: check robot spheres in COLLISION-FREQUENCY order (scene-
// specific, profiled from a training pass) vs generation order. In rejection-heavy
// scenes, early-exit hits sooner -> less collision-traversal work. Isolates the
// collision reorder (both pay identical FK). Also reports mean spheres-to-first-hit.
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/fetch.hh>
#include <vamp/robots/baxter.hh>
constexpr std::size_t rake=vamp::FloatVectorWidth; using DataV=vamp::FloatVector<rake>;
using EnvV=vamp::collision::Environment<DataV>;
static EnvV localized(std::mt19937&rng,int n,float sd){vamp::collision::Environment<float> ef;
  std::normal_distribution<float> nx(0.55F,sd),ny(0.0F,sd),nz(0.55F,sd);std::uniform_real_distribution<float> rad(0.02F,0.06F);
  for(int i=0;i<n;++i)ef.spheres.emplace_back(vamp::collision::factory::sphere::array({nx(rng),ny(rng),nz(rng)},rad(rng)));ef.sort();return EnvV(ef);}
template<class R> static auto mblk(std::mt19937&rng){std::uniform_real_distribution<float> u(0,1);std::normal_distribution<float> st(0,0.02F);
  typename R::template ConfigurationBlock<rake> b;for(std::size_t j=0;j<R::dimension;++j){float base=u(rng),d=st(rng);
  alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=base+(float(l)-3.5F)*d;b[j]=DataV(ln.data());}
  R::template scale_configuration_block<rake>(b);return b;}
template<class R> static void run(const char* name,int n_obs,float sd){
  constexpr std::size_t ns=R::n_spheres; std::mt19937 srng(0xBEEF),tr(0x55),wr(0x33);
  auto env=localized(srng,n_obs,sd); typename R::template Spheres<rake> sph;
  // profile collision frequency per sphere on a training set
  std::vector<long> freq(ns,0); int T=20000,collcfg=0;
  for(int i=0;i<T;++i){auto b=mblk<R>(tr);R::template sphere_fk<rake>(b,sph);bool any=false;
    for(std::size_t j=0;j<ns;++j)if(vamp::sphere_environment_in_collision(env,sph.x[j],sph.y[j],sph.z[j],sph.r[j])){freq[j]++;any=true;}
    if(any)collcfg++;}
  std::vector<std::size_t> ord(ns);std::iota(ord.begin(),ord.end(),0);
  std::stable_sort(ord.begin(),ord.end(),[&](std::size_t a,std::size_t b){return freq[a]>freq[b];});
  // test set
  constexpr std::size_t M=100000; std::vector<typename R::template ConfigurationBlock<rake>> blk(M);
  for(auto&b:blk)b=mblk<R>(wr);
  // mean spheres-to-first-hit gen vs freq (collision configs only)
  double sum_gen=0,sum_freq=0; long nc=0;
  for(auto&b:blk){R::template sphere_fk<rake>(b,sph);
    long g=-1,f=-1;
    for(std::size_t k=0;k<ns;++k){std::size_t j=k;if(g<0&&vamp::sphere_environment_in_collision(env,sph.x[j],sph.y[j],sph.z[j],sph.r[j]))g=k+1;}
    for(std::size_t k=0;k<ns;++k){std::size_t j=ord[k];if(f<0&&vamp::sphere_environment_in_collision(env,sph.x[j],sph.y[j],sph.z[j],sph.r[j])){f=k+1;break;}}
    if(g>0){sum_gen+=g;sum_freq+=f;nc++;}}
  auto med=[&](auto kern){std::vector<double> t;volatile std::uint64_t s=0;for(int r=0;r<7;++r){auto a=std::chrono::steady_clock::now();std::uint64_t acc=0;for(auto&b:blk)acc+=kern(b);auto z=std::chrono::steady_clock::now();s+=acc;t.push_back(std::chrono::duration<double>(z-a).count()/M*1e9);}(void)s;std::sort(t.begin(),t.end());return t[t.size()/2];};
  double tg=med([&](auto&b){R::template sphere_fk<rake>(b,sph);for(std::size_t j=0;j<ns;++j)if(vamp::sphere_environment_in_collision(env,sph.x[j],sph.y[j],sph.z[j],sph.r[j]))return 0;return 1;});
  double tf=med([&](auto&b){R::template sphere_fk<rake>(b,sph);for(std::size_t k=0;k<ns;++k){std::size_t j=ord[k];if(vamp::sphere_environment_in_collision(env,sph.x[j],sph.y[j],sph.z[j],sph.r[j]))return 0;}return 1;});
  std::printf("%s n_obs=%d frac=%.3f | mean spheres-to-hit gen=%.1f freq=%.1f | fkcc gen=%.1f freq=%.1f ns speedup=%.3f\n",
    name,n_obs,double(collcfg)/T,sum_gen/nc,sum_freq/nc,tg,tf,tg/tf);
}
int main(){
  for(auto[n,sd]:{std::pair<int,float>{300,0.13F},{120,0.11F},{60,0.10F}}){
    run<vamp::robots::Fetch>("fetch",n,sd); run<vamp::robots::Baxter>("baxter",n,sd); run<vamp::robots::Panda>("panda",n,sd);
  }
}
