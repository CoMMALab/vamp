// Self-BVH hit-rate: cluster links by kinematic depth, and measure how often a
// coarse cluster-level bound culls a whole group of self-collision pair checks.
// Reports per-cluster-pair bound-overlap rate (motion rakes) and the reduction in
// bound-checks vs the current flat per-pair traversal. Tight cluster bounds = ceiling.
#include <array>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/robots/fetch.hh>
using R=vamp::robots::Fetch; constexpr std::size_t rake=vamp::FloatVectorWidth,ns=R::n_spheres,dim=R::dimension;
using DataV=vamp::FloatVector<rake>;
static auto motion(std::mt19937&rng){std::uniform_real_distribution<float> u(0,1);std::normal_distribution<float> st(0,0.02F);
  R::ConfigurationBlock<rake> b;for(std::size_t j=0;j<dim;++j){float base=u(rng),d=st(rng);
  alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=base+(float(l)-3.5F)*d;b[j]=DataV(ln.data());}
  R::scale_configuration_block<rake>(b);return b;}
int main(){
  std::mt19937 rng(0x11); std::uniform_real_distribution<float> u(0,1); R::Spheres<rake> sph;
  // signatures
  std::vector<std::uint64_t> sig(ns,0);
  for(int base=0;base<6;++base){R::ConfigurationBlock<rake> b0;for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;for(std::size_t l=0;l<rake;++l)ln[l]=u(rng);b0[j]=DataV(ln.data());}
    R::scale_configuration_block<rake>(b0);std::vector<std::array<float,3>> p0(ns);R::sphere_fk<rake>(b0,sph);
    for(std::size_t j=0;j<ns;++j)p0[j]={sph.x[j].to_array()[0],sph.y[j].to_array()[0],sph.z[j].to_array()[0]};
    for(std::size_t jt=0;jt<dim;++jt){auto b=b0;b[jt]=b[jt]+DataV(0.3F);R::sphere_fk<rake>(b,sph);
      for(std::size_t j=0;j<ns;++j){float dx=sph.x[j].to_array()[0]-p0[j][0],dy=sph.y[j].to_array()[0]-p0[j][1],dz=sph.z[j].to_array()[0]-p0[j][2];if(dx*dx+dy*dy+dz*dz>1e-8F)sig[j]|=(1ULL<<jt);}}}
  // links = distinct signatures
  std::vector<std::uint64_t> keys; std::vector<std::vector<std::size_t>> L;
  for(std::size_t j=0;j<ns;++j){auto it=std::find(keys.begin(),keys.end(),sig[j]);if(it==keys.end()){keys.push_back(sig[j]);L.push_back({j});}else L[it-keys.begin()].push_back(j);}
  std::size_t nl=L.size();
  // cluster by depth = popcount(signature): c0 depth<=1 (body), c1 2-4 (proximal), c2 >=5 (distal)
  std::vector<int> depth(nl), clus(nl);
  for(std::size_t g=0;g<nl;++g){depth[g]=std::bitset<64>(keys[g]).count();clus[g]=depth[g]<=1?0:(depth[g]<=4?1:2);}
  const char* cn[3]={"body","prox","dist"};
  for(std::size_t g=0;g<nl;++g)std::printf("link %zu: depth=%d spheres=%zu cluster=%s\n",g,depth[g],L[g].size(),cn[clus[g]]);
  // self-pairs = non parent-child pairs (exclude ancestor-descendant adjacency: subset & depth diff 1)
  auto adj=[&](std::size_t a,std::size_t b){std::uint64_t x=keys[a],y=keys[b];bool sub=((x&y)==x)||((x&y)==y);return sub&&std::abs(depth[a]-depth[b])<=1;};
  std::vector<std::pair<int,int>> pairs;
  for(std::size_t a=0;a<nl;++a)for(std::size_t b=a+1;b<nl;++b)if(!adj(a,b))pairs.push_back({(int)a,(int)b});
  // measure over motion configs
  constexpr int M=50000; std::mt19937 wr(0x33);
  long ov[3][3]={{0}}, tot=0; double bvh_sum=0;
  for(int s=0;s<M;++s){auto b=motion(wr);R::sphere_fk<rake>(b,sph);
    std::vector<std::array<float,4>> bs(nl);
    for(std::size_t g=0;g<nl;++g){float cx=0,cy=0,cz=0;for(auto j:L[g]){cx+=sph.x[j].to_array()[0];cy+=sph.y[j].to_array()[0];cz+=sph.z[j].to_array()[0];}float inv=1.f/L[g].size();cx*=inv;cy*=inv;cz*=inv;float r=0;for(auto j:L[g]){float dx=sph.x[j].to_array()[0]-cx,dy=sph.y[j].to_array()[0]-cy,dz=sph.z[j].to_array()[0]-cz;r=std::max(r,std::sqrt(dx*dx+dy*dy+dz*dz)+sph.r[j].to_array()[0]);}bs[g]={cx,cy,cz,r};}
    // cluster bounds (enclose all links in cluster)
    std::array<std::array<float,4>,3> cb; std::array<bool,3> has{false,false,false};
    for(int c=0;c<3;++c){float mnx=1e30,mny=1e30,mnz=1e30,mxx=-1e30,mxy=-1e30,mxz=-1e30;bool any=false;
      for(std::size_t g=0;g<nl;++g)if(clus[g]==c){any=true;mnx=std::min(mnx,bs[g][0]-bs[g][3]);mxx=std::max(mxx,bs[g][0]+bs[g][3]);mny=std::min(mny,bs[g][1]-bs[g][3]);mxy=std::max(mxy,bs[g][1]+bs[g][3]);mnz=std::min(mnz,bs[g][2]-bs[g][3]);mxz=std::max(mxz,bs[g][2]+bs[g][3]);}
      has[c]=any;cb[c]={(mnx+mxx)/2,(mny+mxy)/2,(mnz+mxz)/2,0.5f*std::sqrt((mxx-mnx)*(mxx-mnx)+(mxy-mny)*(mxy-mny)+(mxz-mnz)*(mxz-mnz))};}
    auto overlap=[&](const std::array<float,4>&p,const std::array<float,4>&q){float dx=p[0]-q[0],dy=p[1]-q[1],dz=p[2]-q[2];return std::sqrt(dx*dx+dy*dy+dz*dz)<p[3]+q[3];};
    // count flat vs bvh checks
    // cluster-pairs that contain self-pairs:
    long bvh=0; bool cp_over[3][3]={{false}};
    for(int c1=0;c1<3;++c1)for(int c2=c1;c2<3;++c2){bool any=false;for(auto&pr:pairs){int a=clus[pr.first],bb=clus[pr.second];if((a==c1&&bb==c2)||(a==c2&&bb==c1)){any=true;break;}}
      if(any){bvh++;/*one cluster check*/ cp_over[c1][c2]=overlap(cb[c1],cb[c2]);}}
    for(auto&pr:pairs){int a=clus[pr.first],bb=clus[pr.second];int c1=std::min(a,bb),c2=std::max(a,bb);
      if(cp_over[c1][c2])bvh++; // only descend if cluster bounds overlap
      // overlap stats at cluster level
    }
    for(int c1=0;c1<3;++c1)for(int c2=c1;c2<3;++c2)if(cp_over[c1][c2])ov[c1][c2]++;
    bvh_sum+=double(bvh)/pairs.size(); tot++;
  }
  std::printf("\nself-pairs (flat bound-checks per config) = %zu\n",pairs.size());
  for(int c1=0;c1<3;++c1)for(int c2=c1;c2<3;++c2){long np=0;for(auto&pr:pairs){int a=clus[pr.first],bb=clus[pr.second];int x=std::min(a,bb),y=std::max(a,bb);if(x==c1&&y==c2)np++;}
    if(np)std::printf("cluster-pair %s-%s: %ld self-pairs, bound-overlap rate=%.1f%%\n",cn[c1],cn[c2],np,100.0*ov[c1][c2]/tot);}
  std::printf("\nBVH bound-checks / flat = %.1f%%  (lower = better; 100%% = no gain)\n",100.0*bvh_sum/tot);
}
