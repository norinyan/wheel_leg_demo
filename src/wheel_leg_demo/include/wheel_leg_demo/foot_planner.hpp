#pragma once
// Planar stair footstep search and C2 swing trajectories. Geometry only, no dynamics.
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <queue>
#include <string>
#include <vector>
namespace wheel_leg_demo {
struct FootBox { double x,y,z,hx,hy,hz; };
struct FootConfig {
  double stair=2,rise=.17,tread=.35,width=4,goal=3.3,y=-.9;
  double resolution=.05,max_step=.50,edge_margin=.025,clearance=.08;
  double duration=4.5,dt=.02,max_foot_v=.8,max_foot_a=3,max_body_v=.3,max_body_a=1;
  double max_joint_v=3,max_joint_a=20,timeout=120;
  double arm=.28,hip_x=.06,hip_y=.285,hip_z=.06,elbow_y=.05,toe=.05;
  double foot_l=.18,foot_w=.10,foot_h=.035,body_height=.43,lift=.13;
  bool block_first_tread=false;
  double lower_min=-6,upper_max=10.7,half_width=3;
  std::vector<FootBox> obstacles;
};
struct FootSample { double t=0,x=0,y=0,z=0;std::array<double,2> fx{},fz{};std::array<double,7> velocity{},acceleration{}; };
struct FootStep { int side;double from,to,height,edge,t; };
struct FootPlan { bool ok=false;std::string reason;int expanded=0;std::vector<FootStep> steps;std::vector<FootSample> samples; };
inline double footGround(double x,const FootConfig& c) {return x<c.stair?0:x<c.stair+c.tread?c.rise:2*c.rise;}
inline double footQ(double u) {u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));}
inline double footQd(double u) {u=std::clamp(u,0.,1.);return 30*u*u*(1-u)*(1-u);}
inline double footQdd(double u) {u=std::clamp(u,0.,1.);return 60*u*(1-u)*(1-2*u);}
inline double footHermite(double a,double b,double va,double vb,double aa,double ab,double dt,double u) {
  double d=b-a-va*dt-aa*dt*dt/2,v=vb*dt-va*dt-aa*dt*dt,ac=ab*dt*dt-aa*dt*dt;
  return a+va*dt*u+aa*dt*dt*u*u/2+(10*d-4*v+ac/2)*u*u*u+(-15*d+7*v-ac)*std::pow(u,4)+(6*d-3*v+ac/2)*std::pow(u,5);
}
inline double footMix(double a,double b,double u) {return a+(b-a)*footQ(u);}
inline double footEdge(double x,const FootConfig& c) {
  double lo=c.lower_min,hi=c.upper_max;
  if(x<c.stair)hi=c.stair;
  else if(x<c.stair+c.tread){lo=c.stair;hi=c.stair+c.tread;}
  else lo=c.stair+c.tread; // second tread and upper platform have equal height
  return std::min(x-c.foot_l/2-lo,hi-x-c.foot_l/2);
}
inline bool footLanding(double x,const FootConfig& c) {
  return !(c.block_first_tread&&x>=c.stair&&x<c.stair+c.tread) && footEdge(x,c)>=c.edge_margin-1e-9 && std::abs(c.y)+c.hip_y+c.elbow_y+c.foot_w/2+c.edge_margin<c.width/2;
}
inline std::array<double,3> footJoints(const FootSample& s,int side,const FootConfig& c) {
  const double dx=s.fx[side]-c.toe-s.x-c.hip_x,down=s.z+c.hip_z-s.fz[side];
  const double d=std::hypot(dx,down);
  if(d>2*c.arm-1e-5||d<.025)return {NAN,NAN,NAN};
  const double alpha=std::acos(d/(2*c.arm)),theta=std::atan2(-dx,down),a=theta+alpha,b=theta-alpha;
  return {a-1.45,b-a+2.55,-b-1.1};
}
inline std::vector<FootBox> footSolids(const FootConfig& c) {
  std::vector<FootBox> boxes=c.obstacles;
  boxes.push_back({(c.lower_min+c.stair)/2,0,-.1,(c.stair-c.lower_min)/2,c.half_width,.1});
  boxes.push_back({c.stair+c.tread/2,0,c.rise/2,c.tread/2,c.width/2,c.rise/2});
  boxes.push_back({c.stair+1.5*c.tread,0,c.rise,c.tread/2,c.width/2,c.rise});
  boxes.push_back({(c.stair+2*c.tread+c.upper_max)/2,0,c.rise-.1,(c.upper_max-c.stair-2*c.tread)/2,c.half_width,c.rise+.1});
  return boxes;
}
inline bool footBoxHit(const FootBox& a,const FootBox& b) {
  return std::abs(a.x-b.x)<a.hx+b.hx-1e-6 && std::abs(a.y-b.y)<a.hy+b.hy-1e-6 && std::abs(a.z-b.z)<a.hz+b.hz-1e-6;
}
// Conservative oriented boxes for the nine moving major parts. The axle/wheel
// parent-child pairs are mechanical joints and intentionally overlap.
inline bool footSelfValid(const FootSample& s,const FootConfig& c) {
  struct Part {FootBox b;double pitch;};
  std::vector<Part> parts{{{s.x,s.y,s.z-.18+c.lift,.0225,.21,.0175},0}};
  for(int side=0;side<2;side++) {
    double sign=side==0?1:-1;auto q=footJoints(s,side,c);if(!std::isfinite(q[0]))return false;
    double a=q[0]+1.45,b=a+q[1]-2.55,sx=s.x+c.hip_x,sz=s.z+c.hip_z;
    double ex=sx-c.arm*std::sin(a),ez=sz-c.arm*std::cos(a);
    parts.push_back({{s.x,s.y+sign*.21,s.z-.18+c.lift,.14,.04,.14},0});
    parts.push_back({{sx-.14*std::sin(a),s.y+sign*c.hip_y,sz-.14*std::cos(a),.022,.022,.105},a});
    parts.push_back({{ex-.12*std::sin(b),s.y+sign*(c.hip_y+c.elbow_y),ez-.12*std::cos(b),.019,.019,.08},b});
    parts.push_back({{s.fx[side],s.y+sign*(c.hip_y+c.elbow_y),s.fz[side],c.foot_l/2,c.foot_w/2,c.foot_h/2},0});
  }
  for(size_t i=0;i<parts.size();i++)for(size_t j=i+1;j<parts.size();j++) {
    if(i==0&&(j==1||j==5))continue;
    auto a=parts[i],b=parts[j];if(std::abs(a.b.y-b.b.y)>=a.b.hy+b.b.hy-1e-6)continue;
    bool separated=false;
    for(double angle:{a.pitch,a.pitch+1.5707963267948966,b.pitch,b.pitch+1.5707963267948966}) {
      double ux=std::cos(angle),uz=-std::sin(angle);
      auto extent=[&](Part p){return std::abs(ux*std::cos(p.pitch)-uz*std::sin(p.pitch))*p.b.hx+std::abs(ux*std::sin(p.pitch)+uz*std::cos(p.pitch))*p.b.hz;};
      if(std::abs((a.b.x-b.b.x)*ux+(a.b.z-b.b.z)*uz)>=extent(a)+extent(b)-1e-6){separated=true;break;}
    }
    if(!separated)return false;
  }
  return true;
}
inline bool footPoseValid(const FootSample& s,const FootConfig& c,std::string* reason=nullptr) {
  auto fail=[&](const std::string& r){if(reason)*reason=r;return false;};
  if(!std::isfinite(s.x+s.y+s.z+s.fx[0]+s.fx[1]+s.fz[0]+s.fz[1]))return fail("NONFINITE_POSE");
  std::vector<FootBox> robot{{s.x,s.y,s.z,.27,.15,.09},{s.x+.285,s.y,s.z+.025,.01,.09,.02},
    {s.x,s.y,s.z-.18+c.lift,.0225,.21,.0175}};
  // Envelopes cover rods, housings, bridges, wheels, spokes and feet.
  for(int side=0;side<2;side++) {
    double sign=side==0?1:-1,sy=s.y+sign*c.hip_y,fy=s.y+sign*(c.hip_y+c.elbow_y);
    auto q=footJoints(s,side,c);if(!std::isfinite(q[0]))return fail("IK_UNREACHABLE");
    if(std::abs(q[0])>3.14||std::abs(q[1])>3.14||std::abs(q[2])>6.28)return fail("JOINT_LIMIT");
    double a=q[0]+1.45,b=a+q[1]-2.55,sx=s.x+c.hip_x,sz=s.z+c.hip_z;
    double ex=sx-c.arm*std::sin(a),ez=sz-c.arm*std::cos(a);
    robot.push_back({s.x,s.y+sign*.21,s.z-.18+c.lift,.14,.04,.14});
    robot.push_back({s.x,s.y+sign*.254,s.z-.18+c.lift,.10,.004,.0075});
    robot.push_back({sx,sy,sz,.037,.028,.037});
    robot.push_back({sx,s.y+sign*.2175,sz,.03,.0675,.03});
    robot.push_back({ex,fy-sign*.025,ez,.034,.051,.034});
    robot.push_back({s.fx[side],fy,s.fz[side],c.foot_l/2,c.foot_w/2,c.foot_h/2});
    auto rod=[&](double x,double y,double z,double angle,double first,double last,double r){
      // 1cm spacing with a 5mm envelope padding covers the continuous rod.
      int n=std::max(1,int(std::ceil((last-first)/.01)));
      for(int i=0;i<=n;i++){double d=first+(last-first)*i/n;robot.push_back({x-d*std::sin(angle),y,z-d*std::cos(angle),r+.005,r,r+.005});}
    };
    rod(sx,sy,sz,a,.035,.245,.022);rod(ex,fy,ez,b,.04,.20,.019);rod(ex,fy,ez,b,.20,.28,.011);
  }
  auto solids=footSolids(c);
  for(const auto& r:robot) {
    if(r.x-r.hx<c.lower_min||r.x+r.hx>c.upper_max||std::abs(r.y)+r.hy>c.half_width)return fail("BOUNDARY");
    for(const auto& b:solids)if(footBoxHit(r,b))return fail("TERRAIN_OR_OBSTACLE_COLLISION");
  }
  if(!footSelfValid(s,c))return fail("SELF_COLLISION");
  return true;
}
inline std::vector<FootSample> footTransition(const FootSample& start,int side,double target,const FootConfig& c) {
  FootSample end=start;end.fx[side]=target;end.fz[side]=footGround(target,c)+c.foot_h/2;
  end.x=(end.fx[0]+end.fx[1])/2-c.hip_x-c.toe;
  end.z=std::min(end.fz[0],end.fz[1])-c.foot_h/2+c.body_height;
  double peak=std::max(start.fz[side],end.fz[side])+c.clearance;
  int n=std::max(4,4*int(std::ceil(c.duration/c.dt/4)));std::vector<FootSample> samples;
  for(int i=0;i<=n;i++) {
    double u=double(i)/n;FootSample s=start;s.t=start.t+u*c.duration;
    s.x=footMix(start.x,end.x,u);s.z=footMix(start.z,end.z,u);
    if(u<.25)s.fz[side]=footMix(start.fz[side],peak,u*4);
    else if(u<.75){s.fx[side]=footMix(start.fx[side],target,(u-.25)*2);s.fz[side]=peak;}
    else {s.fx[side]=target;s.fz[side]=footMix(peak,end.fz[side],(u-.75)*4);}
    auto derivatives=[&](int axis,double a,double b,double phase,double scale){
      s.velocity[axis]=(b-a)*footQd(phase)*scale/c.duration;
      s.acceleration[axis]=(b-a)*footQdd(phase)*scale*scale/(c.duration*c.duration);
    };
    derivatives(0,start.x,end.x,u,1);derivatives(2,start.z,end.z,u,1);
    if(u<.25)derivatives(4+side*2,start.fz[side],peak,u*4,4);
    else if(u<.75)derivatives(3+side*2,start.fx[side],target,(u-.25)*2,2);
    else derivatives(4+side*2,peak,end.fz[side],(u-.75)*4,4);
    samples.push_back(s);
  }
  return samples;
}
inline bool footMotionValid(const std::vector<FootSample>& samples,const FootConfig& c,std::string* why=nullptr) {
  std::array<double,13> old{},vel{};double oldt=0;
  for(size_t i=0;i<samples.size();i++) {
    const auto& s=samples[i];if(!footPoseValid(s,c,why))return false;
    auto l=footJoints(s,0,c),r=footJoints(s,1,c);
    std::array<double,13> p{s.x,s.y,s.z,s.fx[0],s.fz[0],s.fx[1],s.fz[1],l[0],l[1],l[2],r[0],r[1],r[2]};
    if(i) {
      double dt=s.t-oldt;if(dt<=0){if(why)*why="INVALID_TIME";return false;}
      for(size_t k=0;k<p.size();k++) {
        double v=(p[k]-old[k])/dt,a=(v-vel[k])/dt;
        double vmax=k<3?c.max_body_v:k<7?c.max_foot_v:c.max_joint_v;
        double amax=k<3?c.max_body_a:k<7?c.max_foot_a:c.max_joint_a;
        if(std::abs(v)>vmax+1e-6 || (i>1&&std::abs(a)>amax+1e-6)){if(why)*why="MOTION_LIMIT";return false;}
        vel[k]=v;
      }
    }
    old=p;oldt=s.t;
  }
  return true;
}
inline FootPlan planFeet(const FootSample& start,const FootConfig& c) {
  FootPlan out;
  if(c.resolution<=0||c.duration<=0||c.dt<=0||c.dt>.05||c.max_step<=0||c.edge_margin<0||c.clearance<=0){out.reason="INVALID_PARAMETERS";return out;}
  if(std::abs(start.fx[0]-start.fx[1])>.02||!footLanding(start.fx[0],c)||!footLanding(start.fx[1],c)||!footPoseValid(start,c)) {out.reason="INVALID_START";return out;}
  const double goal=c.goal+c.hip_x+c.toe;
  if(!footLanding(goal,c)||footGround(goal,c)!=2*c.rise){out.reason="INVALID_GOAL";return out;}
  std::vector<double> candidates{start.fx[0]};
  for(double x=start.fx[0]+c.resolution;x<goal-1e-6;x+=c.resolution)if(footLanding(x,c))candidates.push_back(x);
  candidates.push_back(goal);int count=candidates.size(),last=count-1;
  struct State {int left,right,next;bool operator<(const State& b)const{return std::array<int,3>{left,right,next}<std::array<int,3>{b.left,b.right,b.next};}};
  struct Item {double f,g;State state;bool operator<(const Item& b)const{return f>b.f;}};
  std::map<State,double> costs;std::map<State,State> parents;std::priority_queue<Item> open;
  auto heuristic=[&](State s){return (2*goal-candidates[s.left]-candidates[s.right])/c.max_step;};
  State initial{0,0,0};costs[initial]=0;open.push({heuristic(initial),0,initial});State finish{};bool found=false;
  while(!open.empty()&&out.expanded<10000) {
    auto item=open.top();open.pop();auto s=item.state;if(item.g>costs[s]+1e-9)continue;++out.expanded;
    if(s.left==last&&s.right==last){finish=s;found=true;break;}
    FootSample pose=start;pose.fx={candidates[s.left],candidates[s.right]};
    for(int i=0;i<2;i++)pose.fz[i]=footGround(pose.fx[i],c)+c.foot_h/2;
    pose.x=(pose.fx[0]+pose.fx[1])/2-c.hip_x-c.toe;pose.z=std::min(pose.fz[0],pose.fz[1])-c.foot_h/2+c.body_height;
    int old=s.next==0?s.left:s.right,other=s.next==0?s.right:s.left;
    for(int target=old+1;target<count;target++) {
      double dx=candidates[target]-candidates[old];if(dx>c.max_step+1e-9)break;
      if(candidates[target]<candidates[other]-1e-6||std::abs(candidates[target]-candidates[other])>c.max_step)continue;
      if(footGround(candidates[target],c)-footGround(candidates[old],c)>c.rise+1e-6)continue;
      auto segment=footTransition(pose,s.next,candidates[target],c);
      if(!footMotionValid(segment,c))continue;
      State next=s;if(s.next==0)next.left=target;else next.right=target;next.next=1-s.next;
      double g=item.g+1+.1*dx;
      if(!costs.count(next)||g<costs[next]){costs[next]=g;parents[next]=s;open.push({g+heuristic(next),g,next});}
    }
  }
  if(!found){out.reason="NO_FEASIBLE_FOOTSTEPS";return out;}
  std::vector<State> route{finish};while(parents.count(route.back()))route.push_back(parents[route.back()]);std::reverse(route.begin(),route.end());
  out.samples.push_back(start);
  for(size_t i=1;i<route.size();i++) {
    int side=route[i-1].next;double to=candidates[side==0?route[i].left:route[i].right];
    out.steps.push_back({side,out.samples.back().fx[side],to,footGround(to,c),footEdge(to,c),out.samples.back().t+c.duration});
    auto seg=footTransition(out.samples.back(),side,to,c);out.samples.insert(out.samples.end(),seg.begin()+1,seg.end());
  }
  out.ok=footMotionValid(out.samples,c,&out.reason);if(out.ok)out.reason="OK";return out;
}
}
