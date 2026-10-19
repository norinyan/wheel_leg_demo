#include "wheel_leg_demo/teb_optimizer.hpp"
#include <ceres/ceres.h>
#include <array>
#include <cmath>
#include <algorithm>

namespace wheel_leg_demo {
double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
bool footprintFree(Pose2 p, Bounds b, const std::vector<Obstacle>& obs, const TebConfig& c) {
  const double cs=std::cos(p.yaw),sn=std::sin(p.yaw),hx=c.length/2+c.margin,hy=c.width/2+c.margin;
  const double ex=std::abs(cs)*hx+std::abs(sn)*hy,ey=std::abs(sn)*hx+std::abs(cs)*hy;
  if(p.x-ex<b.xmin||p.x+ex>b.xmax||p.y-ey<b.ymin||p.y+ey>b.ymax)return false;
  for(auto o:obs) {
    double dx=o.x-p.x,dy=o.y-p.y;
    if(std::abs(dx)>ex+o.length/2 || std::abs(dy)>ey+o.width/2)continue;
    if(std::abs(dx*cs+dy*sn)>hx+std::abs(cs)*o.length/2+std::abs(sn)*o.width/2)continue;
    if(std::abs(-dx*sn+dy*cs)>hy+std::abs(sn)*o.length/2+std::abs(cs)*o.width/2)continue;
    return false;
  }
  return true;
}
bool sweptFree(Pose2 a,Pose2 b,Bounds bounds,const std::vector<Obstacle>& obs,const TebConfig& c) {
  const double turn=wrap(b.yaw-a.yaw);
  int n=std::max(1,int(std::ceil(std::max(std::hypot(b.x-a.x,b.y-a.y)/.01,std::abs(turn)/.02))));
  for(int i=0;i<=n;i++){double t=double(i)/n;if(!footprintFree({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*turn},bounds,obs,c))return false;}
  return true;
}
namespace {
template<typename T>T positive(T x){return x>T(0)?x:T(0);}
template<typename T>T angle(T x){using std::sin;using std::cos;using std::atan2;return atan2(sin(x),cos(x));}
struct Motion {
  TebConfig c;
  template<typename T>bool operator()(const T* a,const T* b,const T* dt,T* r)const {
    using std::sin;using std::cos;using std::sqrt;using std::abs;
    T dx=b[0]-a[0],dy=b[1]-a[1],mid=(a[2]+b[2])/T(2);
    T v=(dx*cos(mid)+dy*sin(mid))/dt[0],w=angle(b[2]-a[2])/dt[0];
    r[0]=T(150)*(dx*sin(mid)-dy*cos(mid)); // nonholonomic differential-drive constraint
    r[1]=T(12)*positive(v-T(c.max_v));r[2]=T(12)*positive(-v);
    r[3]=T(12)*positive(abs(w)-T(c.max_w));
    r[4]=T(.12)*sqrt(dt[0]); // minimize total duration
    r[5]=T(.15)*sqrt(dx*dx+dy*dy+T(1e-9));
    return true;
  }
};
struct Accel {
  TebConfig c;
  template<typename T>bool operator()(const T* a,const T* b,const T* d,const T* t1,const T* t2,T* r)const {
    using std::cos;using std::sin;using std::abs;
    auto velocity=[](const T* x,const T* y,const T* dt){return ((y[0]-x[0])*cos((x[2]+y[2])/T(2))+(y[1]-x[1])*sin((x[2]+y[2])/T(2)))/dt[0];};
    T dt=(t1[0]+t2[0])/T(2),v1=velocity(a,b,t1),v2=velocity(b,d,t2);
    T w1=angle(b[2]-a[2])/t1[0],w2=angle(d[2]-b[2])/t2[0];
    r[0]=T(8)*positive(abs((v2-v1)/dt)-T(c.max_a));
    r[1]=T(8)*positive(abs((w2-w1)/dt)-T(c.max_alpha));return true;
  }
};
struct InitialVelocity {
  double v,w;TebConfig c;
  template<typename T>bool operator()(const T* a,const T* b,const T* dt,T* r)const {
    using std::cos;using std::sin;using std::abs;
    T speed=((b[0]-a[0])*cos(a[2])+(b[1]-a[1])*sin(a[2]))/dt[0];
    r[0]=T(8)*positive(abs(speed-T(v))-T(c.max_a)*dt[0]);
    r[1]=T(8)*positive(abs(angle(b[2]-a[2])/dt[0]-T(w))-T(c.max_alpha)*dt[0]);return true;
  }
};
struct Reference {
  Pose2 p;
  template<typename T>bool operator()(const T* x,T* r)const {r[0]=T(.25)*(x[0]-T(p.x));r[1]=T(.25)*(x[1]-T(p.y));return true;}
};
struct Clearance {
  Obstacle o;TebConfig c;
  template<typename T>bool operator()(const T* p,T* r)const {
    using std::abs;using std::sqrt;
    // Conservative circumscribed circle; final hard check uses a rotated rectangle.
    T dx=positive(abs(p[0]-T(o.x))-T(o.length/2)),dy=positive(abs(p[1]-T(o.y))-T(o.width/2));
    T distance=sqrt(dx*dx+dy*dy+T(1e-8));
    r[0]=T(45)*positive(T(std::hypot(c.length,c.width)/2+c.margin+.025)-distance);return true;
  }
};
}
Band optimizeBand(const std::vector<Pose2>& seed,double v0,double w0,Bounds bounds,
                  const std::vector<Obstacle>& obs,const TebConfig& c) {
  Band result;if(seed.size()<3)return result;
  std::vector<std::array<double,3>> p;std::vector<double> dt;
  for(auto s:seed)p.push_back({s.x,s.y,s.yaw});
  for(size_t i=1;i<p.size();i++)dt.push_back(std::clamp(std::max(std::hypot(p[i][0]-p[i-1][0],p[i][1]-p[i-1][1])/.22,std::abs(wrap(p[i][2]-p[i-1][2]))/.6),.08,1.2));
  ceres::Problem problem;
  for(size_t i=0;i<p.size();i++) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Reference,2,3>(new Reference{seed[i]}),nullptr,p[i].data());
    problem.SetParameterLowerBound(p[i].data(),0,bounds.xmin+c.length/2+c.margin);
    problem.SetParameterUpperBound(p[i].data(),0,bounds.xmax-c.length/2-c.margin);
    problem.SetParameterLowerBound(p[i].data(),1,bounds.ymin+c.width/2+c.margin);
    problem.SetParameterUpperBound(p[i].data(),1,bounds.ymax-c.width/2-c.margin);
    for(auto o:obs)problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Clearance,1,3>(new Clearance{o,c}),nullptr,p[i].data());
    if(i+1<p.size()) {
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Motion,6,3,3,1>(new Motion{c}),nullptr,p[i].data(),p[i+1].data(),&dt[i]);
      problem.SetParameterLowerBound(&dt[i],0,.05);problem.SetParameterUpperBound(&dt[i],0,2.5);
    }
    if(i+2<p.size())problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Accel,2,3,3,3,1,1>(new Accel{c}),nullptr,p[i].data(),p[i+1].data(),p[i+2].data(),&dt[i],&dt[i+1]);
  }
  problem.SetParameterBlockConstant(p.front().data());problem.SetParameterBlockConstant(p.back().data());
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<InitialVelocity,2,3,3,1>(new InitialVelocity{v0,w0,c}),nullptr,p[0].data(),p[1].data(),&dt[0]);
  ceres::Solver::Options options;options.max_num_iterations=c.iterations;options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;options.num_threads=1;options.max_solver_time_in_seconds=.03;options.minimizer_progress_to_stdout=false;
  ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
  result.cost=summary.final_cost;result.valid=summary.IsSolutionUsable();result.dt=dt;
  for(auto v:p)result.poses.push_back({v[0],v[1],v[2]});
  for(size_t i=0;i+1<p.size();i++)if(!sweptFree(result.poses[i],result.poses[i+1],bounds,obs,c))result.valid=false;
  return result;
}
}
