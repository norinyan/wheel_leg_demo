#include "wheel_leg_demo/foot_planner.hpp"
#include <iostream>
#include <stdexcept>
using namespace wheel_leg_demo;
void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
int main() {
  FootConfig c;FootSample start;start.x=1.6;start.y=-.9;start.z=.43;start.fx={1.71,1.71};start.fz={.0175,.0175};
  auto plan=planFeet(start,c);require(plan.ok,"Baseline must have a feasible plan");
  require(plan.steps.size()>3,"Footstep search must produce a sequence");
  for(size_t i=0;i<plan.steps.size();i++) {
    const auto& step=plan.steps[i];require(step.side==int(i%2),"Feet must alternate");
    require(step.edge>=c.edge_margin,"Whole sole needs tread edge clearance");
    require(step.to-step.from<=c.max_step+1e-8,"Step length limit");
  }
  auto end=plan.samples.back();require(std::abs(end.x-c.goal)<1e-8,"Body must reach X");
  for(int side=0;side<2;side++)require(end.fx[side]-c.foot_l/2>c.stair+2*c.tread,"Both feet must leave the stairs");
  // Interpolate more densely than transmitted samples, as the execution plugin does.
  int checked=0;
  for(size_t i=1;i<plan.samples.size();i++) {
    auto a=plan.samples[i-1],b=plan.samples[i];double dt=b.t-a.t;
    for(int j=0;j<=4;j++) {
      double u=j/4.;auto s=a;
      auto h=[&](double p,double q,int k){return footHermite(p,q,a.velocity[k],b.velocity[k],a.acceleration[k],b.acceleration[k],dt,u);};
      s.x=h(a.x,b.x,0);s.y=h(a.y,b.y,1);s.z=h(a.z,b.z,2);
      for(int side=0;side<2;side++){s.fx[side]=h(a.fx[side],b.fx[side],3+2*side);s.fz[side]=h(a.fz[side],b.fz[side],4+2*side);}
      require(footPoseValid(s,c),"Interpolated pose collision or unreachable");++checked;
    }
  }
  for(double u:{0.,1.})require(std::abs(footQd(u))+std::abs(footQdd(u))<1e-12,"Quintic endpoint velocity and acceleration must vanish");
  c.block_first_tread=true;auto blocked=planFeet(start,c);require(!blocked.ok&&blocked.reason=="NO_FEASIBLE_FOOTSTEPS","Forbidden first tread must prevent a plan");
  c.block_first_tread=false;c.max_step=.2;require(!planFeet(start,c).ok,"Insufficient step reach must fail");
  c.max_step=.5;auto collision=start;collision.z=.15;require(!footPoseValid(collision,c),"Wheel-ground penetration must be rejected");
  std::cout<<"PASS: baseline, alternation, edge/reach, "<<checked<<" dense poses, C2 endpoints, blocked tread, short step, penetration\n";
}
