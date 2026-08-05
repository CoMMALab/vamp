// Self-collision pair-pruning headroom: fraction of link-pairs whose (conservative)
// bounding spheres are ALWAYS separated over a config region -> statically prunable.
// Compares full joint box vs a start->goal corridor (query-time knowledge = JIT lever).
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/robots/fetch.hh>
using R=vamp::robots::Fetch; constexpr std::size_t rake=vamp::FloatVectorWidth,ns=R::n_spheres,dim=R::dimension;
using DataV=vamp::FloatVector<rake>;
static auto blk_from(const std::array<float,dim>& q){R::ConfigurationBlock<rake> b;
  for(std::size_t j=0;j<dim;++j) b[j]=DataV(q[j]); return b;}
// signature link grouping
static std::vector<std::vector<std::size_t>> links(std::mt19937&rng){
  std::uniform_real_distribution<float> u(0,1); R::Spheres<rake> sph; std::vector<std::uint64_t> sig(ns,0);
  for(int base=0;base<6;++base){R::ConfigurationBlock<rake> b0;
    for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=u(rng);b0[j]=DataV(ln.data());}
    R::scale_configuration_block<rake>(b0);std::vector<std::array<float,3>> p0(ns);R::sphere_fk<rake>(b0,sph);
    for(std::size_t j=0;j<ns;++j)p0[j]={sph.x[j].to_array()[0],sph.y[j].to_array()[0],sph.z[j].to_array()[0]};
    for(std::size_t jt=0;jt<dim;++jt){auto b=b0;b[jt]=b[jt]+DataV(0.3F);R::sphere_fk<rake>(b,sph);
      for(std::size_t j=0;j<ns;++j){float dx=sph.x[j].to_array()[0]-p0[j][0],dy=sph.y[j].to_array()[0]-p0[j][1],dz=sph.z[j].to_array()[0]-p0[j][2];
        if(dx*dx+dy*dy+dz*dz>1e-8F)sig[j]|=(1ULL<<jt);}}}
  std::vector<std::uint64_t> keys; std::vector<std::vector<std::size_t>> out;
  for(std::size_t j=0;j<ns;++j){auto it=std::find(keys.begin(),keys.end(),sig[j]);
    if(it==keys.end()){keys.push_back(sig[j]);out.push_back({j});}else out[it-keys.begin()].push_back(j);}
  return out;}
// per-link enclosing bounding sphere at a config (lane 0)
static void link_bs(const std::vector<std::vector<std::size_t>>&L,R::Spheres<rake>&sph,std::vector<std::array<float,4>>&bs){
  for(std::size_t g=0;g<L.size();++g){float cx=0,cy=0,cz=0;for(auto j:L[g]){cx+=sph.x[j].to_array()[0];cy+=sph.y[j].to_array()[0];cz+=sph.z[j].to_array()[0];}
    float inv=1.f/L[g].size();cx*=inv;cy*=inv;cz*=inv;float r=0;for(auto j:L[g]){float dx=sph.x[j].to_array()[0]-cx,dy=sph.y[j].to_array()[0]-cy,dz=sph.z[j].to_array()[0]-cz;
      r=std::max(r,std::sqrt(dx*dx+dy*dy+dz*dz)+sph.r[j].to_array()[0]);}bs[g]={cx,cy,cz,r};}}
static int count_prunable(const std::vector<std::vector<std::size_t>>&L,std::vector<std::array<float,dim>>&cfgs,int&npairs){
  std::size_t n=L.size(); std::vector<float> mingap(n*n,1e30f); R::Spheres<rake> sph;
  for(auto&q:cfgs){auto b=blk_from(q);R::sphere_fk<rake>(b,sph);std::vector<std::array<float,4>> bs(n);link_bs(L,sph,bs);
    for(std::size_t a=0;a<n;++a)for(std::size_t c=a+1;c<n;++c){float dx=bs[a][0]-bs[c][0],dy=bs[a][1]-bs[c][1],dz=bs[a][2]-bs[c][2];
      float gap=std::sqrt(dx*dx+dy*dy+dz*dz)-bs[a][3]-bs[c][3];mingap[a*n+c]=std::min(mingap[a*n+c],gap);}}
  int prun=0;npairs=0;for(std::size_t a=0;a<n;++a)for(std::size_t c=a+1;c<n;++c){++npairs;if(mingap[a*n+c]>0.f)++prun;}return prun;}
int main(){
  std::mt19937 rng(0x11); auto L=links(rng);
  std::printf("fetch signature-links=%zu\n",L.size());
  // full box
  std::vector<std::array<float,dim>> full;std::uniform_real_distribution<float> u(0,1);
  for(int i=0;i<4000;++i){std::array<float,dim> q;for(std::size_t j=0;j<dim;++j)q[j]=u(rng);R::ConfigurationBlock<rake> b=blk_from(q);R::scale_configuration_block<rake>(b);for(std::size_t j=0;j<dim;++j)q[j]=b[j].to_array()[0];full.push_back(q);}
  int np;int pf=count_prunable(L,full,np);
  // corridor: random start->goal (scaled), jitter
  std::normal_distribution<float> nj(0,0.15F);
  auto scaled=[&](){std::array<float,dim> q;for(std::size_t j=0;j<dim;++j)q[j]=u(rng);R::ConfigurationBlock<rake> b=blk_from(q);R::scale_configuration_block<rake>(b);for(std::size_t j=0;j<dim;++j)q[j]=b[j].to_array()[0];return q;};
  double sum=0;int trials=8;
  for(int t=0;t<trials;++t){auto s=scaled(),g=scaled();std::vector<std::array<float,dim>> cor;
    for(int i=0;i<2000;++i){float tt=u(rng);std::array<float,dim> q;for(std::size_t j=0;j<dim;++j)q[j]=s[j]+tt*(g[j]-s[j])+nj(rng);cor.push_back(q);}
    int npc;int pc=count_prunable(L,cor,npc);sum+=double(pc)/npc;}
  std::printf("full-box:  prunable %d/%d = %.1f%%\n",pf,np,100.0*pf/np);
  std::printf("corridor:  prunable %.1f%% (mean of %d start-goal corridors)\n",100.0*sum/trials,trials);
}
