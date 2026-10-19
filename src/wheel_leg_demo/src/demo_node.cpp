// Kinematic executor: stage 2 presets, stage 3.2 velocities, stage 3.3 planned feet.
#include "wheel_leg_demo/foot_planner.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace gazebo {
class WheelLegAnimation : public ModelPlugin {
  enum class Action { Idle, Deploy, Fold, Drive, Climb, PlannedClimb };
  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_;
  event::ConnectionPtr update_;
  std::mutex mutex_;
  std::atomic<bool> resetPending_{false};
  std::atomic<int64_t> wallTickMs_{0};
  rclcpp::TimerBase::SharedPtr heartbeat_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<gazebo_msgs::msg::ModelStates>::SharedPtr models_;
  rclcpp::Publisher<gazebo_msgs::msg::LinkStates>::SharedPtr links_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocityCommand_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr footTrajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr footFeedback_;
  wheel_leg_demo::FootConfig footConfig_;
  std::vector<wheel_leg_demo::FootSample> plannedFeet_;
  double yaw_=0,leftWheel_=0,rightWheel_=0,v_=0,w_=0,commandV_=0,commandW_=0,commandTime_=-100;
  double exitX_=3.3,exitY_=-.9,maxV_=.3,maxW_=.8,maxA_=.3,maxAlpha_=1.;
  bool external_=false;
  Action action_ = Action::Idle;
  bool leg_ = false;
  double x_=0,y_=-.9,z_=.32,lift_=0,wheel_=0;
  double startX_=0,startY_=-.9,startZ_=.32,stairX_=2,rise_=.17,tread_=.35;
  double entry_=1.6,legHeight_=.43,liftTarget_=.13,morphTime_=4,driveDistance_=1.6,driveSpeed_=.3,stepTime_=2,clearance_=.18;
  double arm_= .28, radius_= .14,hipX_=.06,hipY_=.285,hipZ_=.06,elbowY_=.05,footHalf_=.0175,toeOffset_=.05;
  double obstacleX_=1,obstacleY_=0,obstacleLength_=.3,obstacleWidth_=.4,robotWidth_=.78,robotLength_=.6;
  double lowerEnd_=2,upperStart_=2.7,upperEnd_=6.7,worldHalfWidth_=2;
  double last_=0,begin_=0,report_=-1,fromX_=0,fromZ_=0,fromLift_=0,fromWheel_=0;
  std::array<double,2> fx_{},fz_{},fromFx_{},fromFz_{};
  std::vector<double> footholds_;
  std::string state_="READY_WHEEL";
  static double smooth(double u) {u=std::clamp(u,0.0,1.0);return u*u*(3-2*u);}
  static double mix(double a,double b,double u) {return a+(b-a)*smooth(u);}
  double ground(double x) const {
    if(x<stairX_)return 0;
    if(x<stairX_+tread_)return rise_;
    return 2*rise_;
  }
  std::array<double,2> folded(double x,double z) const {
    return {x+hipX_+toeOffset_-arm_*std::sin(1.45)-arm_*std::sin(-1.1),
            z+hipZ_-arm_*std::cos(1.45)-arm_*std::cos(-1.1)};
  }
  bool ik(int side,double &a,double &b) const {
    const double dx=fx_[side]-toeOffset_-(x_+hipX_),down=z_+hipZ_-fz_[side];
    double d=std::hypot(dx,down);
    if(!std::isfinite(d)||d>2*arm_-1e-6||d<.025)return false;
    double alpha=std::acos(d/(2*arm_)),theta=std::atan2(-dx,down);
    a=theta+alpha;b=theta-alpha;return true;
  }
  void resetState() {
    external_=false;v_=w_=commandV_=commandW_=yaw_=leftWheel_=rightWheel_=0;
    action_=Action::Idle;leg_=false;x_=startX_;y_=startY_;z_=startZ_;lift_=wheel_=0;
    auto f=folded(x_,z_);fx_.fill(f[0]);fz_.fill(f[1]);state_="READY_WHEEL";report_=-1;
  }
  void pose(const std::string &link,double x,double y,double z,double pitch=0) {
    auto l=model_->GetLink(link);
    const double dx=x-x_,dy=y-y_;
    if(l)l->SetWorldPose(ignition::math::Pose3d(x_+std::cos(yaw_)*dx-std::sin(yaw_)*dy,
      y_+std::sin(yaw_)*dx+std::cos(yaw_)*dy,z,0,pitch,yaw_),false,false);
  }
  bool apply(double now) {
    std::array<double,2> a{},b{};
    for(int i=0;i<2;i++)if(!ik(i,a[i],b[i]))return false;
    sensor_msgs::msg::JointState q;
    {
    // This is a kinematic display, not a physical controller. Update cached
    // poses without entering physics notification; publish once after all
    // pose locks are released to avoid GUI/physics/transport lock inversion.
    model_->SetWorldPose(ignition::math::Pose3d(x_,y_,z_,0,0,yaw_), false, false);
    pose("wheel_carrier",x_,y_,z_-.18+lift_);
    pose("left_wheel",x_,y_+.21,z_-.18+lift_,wheel_+leftWheel_);
    pose("right_wheel",x_,y_-.21,z_-.18+lift_,wheel_+rightWheel_);
    q.header.stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
    q.name={"wheel_lift_joint","left_wheel_joint","right_wheel_joint"};q.position={lift_,wheel_+leftWheel_,wheel_+rightWheel_};
    for(int i=0;i<2;i++) {
      const std::string s=i==0?"left":"right";double sign=i==0?1:-1;
      const double sx=x_+hipX_,sz=z_+hipZ_,ex=sx-arm_*std::sin(a[i]),ez=sz-arm_*std::cos(a[i]);
      pose(s+"_upper_arm",sx,y_+sign*hipY_,sz,a[i]);
      pose(s+"_forearm",ex,y_+sign*(hipY_+elbowY_),ez,b[i]);
      pose(s+"_foot",fx_[i]-toeOffset_,y_+sign*(hipY_+elbowY_),fz_[i]);
      q.name.insert(q.name.end(),{s+"_shoulder_joint",s+"_elbow_joint",s+"_foot_joint"});
      q.position.insert(q.position.end(),{a[i]-1.45,b[i]-a[i]+2.55,-b[i]-1.1});
    }
    }
    model_->GetWorld()->PublishModelPose(model_);
    joints_->publish(q);return true;
  }
  static builtin_interfaces::msg::Time qTime(double now) {
    return rclcpp::Time(static_cast<int64_t>(now*1e9));
  }
  bool wheelPoseFree(double x,double y,double yaw) {
    const double hx=.3+.03,hy=.39+.03,cs=std::cos(yaw),sn=std::sin(yaw);
    double ex=std::abs(cs)*hx+std::abs(sn)*hy,ey=std::abs(sn)*hx+std::abs(cs)*hy;
    const bool upper=z_>.55;
    if(x-ex<(upper?upperStart_:-6.)||x+ex>(upper?upperEnd_:lowerEnd_)||std::abs(y)+ey>3.)return false;
    for(auto m:model_->GetWorld()->Models()) {
      if(m->GetName().find("obstacle")==std::string::npos)continue;
      auto box=m->BoundingBox();double ox=(box.Min().X()+box.Max().X())/2,oy=(box.Min().Y()+box.Max().Y())/2;
      if(box.Max().Z()<(upper?2*rise_:0)+.001)continue;
      double bx=(box.Max().X()-box.Min().X())/2,by=(box.Max().Y()-box.Min().Y())/2,dx=ox-x,dy=oy-y;
      if(std::abs(dx)>ex+bx||std::abs(dy)>ey+by)continue;
      if(std::abs(dx*cs+dy*sn)>hx+std::abs(cs)*bx+std::abs(sn)*by)continue;
      if(std::abs(-dx*sn+dy*cs)>hy+std::abs(sn)*bx+std::abs(cs)*by)continue;
      return false;
    }
    return true;
  }
  void snapshot(double now) {
    begin_=now;fromX_=x_;fromZ_=z_;fromLift_=lift_;fromWheel_=wheel_;fromFx_=fx_;fromFz_=fz_;
  }
  void request(const std::string &command,std_srvs::srv::Trigger::Response &reply) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(command=="reset") {resetState();reply.success=true;reply.message="Reset to initial wheel pose";return;}
    if(command=="prepare_upper"||command=="prepare_entry") {
      if(action_!=Action::Idle||std::abs(v_)>.01||std::abs(w_)>.01){reply.message="Stop before test initialization";return;}
      resetState();x_=command=="prepare_upper"?exitX_:entry_;y_=exitY_;z_=(command=="prepare_upper"?2*rise_:0)+startZ_;
      auto f=folded(x_,z_);fx_.fill(f[0]);fz_.fill(f[1]);
      state_=command=="prepare_upper"?"READY_UPPER_TEST: initialized at X, not climbed":"READY_ENTRY_TEST: initialized at E, not driven";
      reply.success=true;reply.message="Test initialization only; no route execution";return;
    }
    if(command=="stop") {
      external_=false;v_=w_=commandV_=commandW_=0;
      if(action_==Action::Deploy||action_==Action::Fold)state_="STOPPED_MORPH: reset required";
      else state_="STOPPED";
      action_=Action::Idle;reply.success=true;reply.message="Held current display pose";return;
    }
    if(action_!=Action::Idle||std::abs(v_)>.01||std::abs(w_)>.01) {reply.message="Another action is running";return;}
    if(state_.find("STOPPED_MORPH")==0||state_.find("FAILED")==0) {reply.message="Reset required";return;}
    external_=false;v_=w_=commandV_=commandW_=0;
    double now=model_->GetWorld()->SimTime().Double();
    if(command=="to_leg") {
      if(leg_) {reply.message="Already in leg mode";return;}
      action_=Action::Deploy;state_="MORPH_TO_LEG";
    } else if(command=="to_wheel") {
      if(!leg_) {reply.message="Already in wheel mode";return;}
      if(std::abs(fz_[0]-fz_[1])>.005) {reply.message="Feet must be on the same level; reset interrupted climb";return;}
      action_=Action::Fold;state_="MORPH_TO_WHEEL";
    } else if(command=="drive") {
      if(leg_) {reply.message="Drive requires wheel mode";return;}
      double end=x_+driveDistance_;
      if((x_<lowerEnd_ && end+robotLength_/2>lowerEnd_) || end+robotLength_/2>upperEnd_ ||
         (x_>=lowerEnd_ && x_-robotLength_/2<upperStart_)) {reply.message="Drive would leave flat ground";return;}
      bool intersectsY=std::abs(y_-obstacleY_)<robotWidth_/2+obstacleWidth_/2+.03;
      if(intersectsY && x_-robotLength_/2<obstacleX_+obstacleLength_/2 && end+robotLength_/2>obstacleX_-obstacleLength_/2) {
        reply.message="Preset straight drive intersects obstacle";return;
      }
      action_=Action::Drive;state_="DRIVING";
    } else if(command=="climb") {
      if(!leg_||std::abs(x_-entry_)>.06) {reply.message="Climb requires leg mode at stair entry; drive then to_leg";return;}
      action_=Action::Climb;state_="CLIMBING_PRESET";
    } else {reply.message="Unknown command";return;}
    snapshot(now);reply.success=true;reply.message="Accepted; query /wheel_leg_demo/status for completion";
  }
  void acceptFootTrajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto reject=[&](const std::string& why){if(action_==Action::Idle)state_="FAILED_FOOT_TRAJECTORY: "+why;};
    double now=model_->GetWorld()->SimTime().Double();
    if(action_!=Action::Idle||!leg_||std::abs(v_)>.01||std::abs(w_)>.01){return;}
    if(std::abs(now-rclcpp::Time(msg.header.stamp).seconds())>.5||msg.header.frame_id!="world"||
       msg.joint_names!=std::vector<std::string>{"body","left_foot_center","right_foot_center"}||msg.points.size()<2||msg.points.size()>20000){reject("INVALID_HEADER");return;}
    std::vector<wheel_leg_demo::FootSample> samples;
    for(const auto& point:msg.points) {
      if(point.transforms.size()!=3||point.velocities.size()!=3||point.accelerations.size()!=3){reject("INVALID_POINT");return;}
      wheel_leg_demo::FootSample s;s.t=rclcpp::Duration(point.time_from_start).seconds();
      s.x=point.transforms[0].translation.x;s.y=point.transforms[0].translation.y;s.z=point.transforms[0].translation.z;
      for(int i=0;i<3;i++) {
        auto p=point.transforms[i];const double targetY=s.y+(i==0?0:(i==1?1:-1)*(hipY_+elbowY_));
        if(std::abs(p.translation.y-targetY)>.005||std::abs(p.rotation.x)+std::abs(p.rotation.y)+std::abs(p.rotation.z)>1e-6||std::abs(p.rotation.w-1)>1e-6){reject("PLANAR_CONVENTION");return;}
        int axis=i==0?0:3+2*(i-1);s.velocity[axis]=point.velocities[i].linear.x;s.acceleration[axis]=point.accelerations[i].linear.x;
        if(i==0){s.velocity[1]=point.velocities[i].linear.y;s.acceleration[1]=point.accelerations[i].linear.y;s.velocity[2]=point.velocities[i].linear.z;s.acceleration[2]=point.accelerations[i].linear.z;}
        else {s.fx[i-1]=p.translation.x;s.fz[i-1]=p.translation.z;s.velocity[axis+1]=point.velocities[i].linear.z;s.acceleration[axis+1]=point.accelerations[i].linear.z;}
      }
      if(!samples.empty()&&(s.t<=samples.back().t||s.t-samples.back().t>.05)){reject("SAMPLE_TIME");return;}
      samples.push_back(s);
    }
    const auto& first=samples.front();double mismatch=std::abs(x_-first.x)+std::abs(y_-first.y)+std::abs(z_-first.z);
    for(int i=0;i<2;i++)mismatch+=std::abs(fx_[i]-first.fx[i])+std::abs(fz_[i]-first.fz[i]);
    if(mismatch>.01||std::abs(yaw_)>.005||std::abs(first.t)>1e-9||samples.back().t>footConfig_.timeout){reject("START_OR_DURATION");return;}
    std::string why;auto cfg=footConfig_;cfg.y=y_;
    if(!wheel_leg_demo::footMotionValid(samples,cfg,&why)){reject(why);return;}
    plannedFeet_=std::move(samples);external_=false;v_=w_=commandV_=commandW_=0;
    action_=Action::PlannedClimb;state_="CLIMBING_PLANNED";snapshot(now);
  }
  void plannedFootTick(double elapsed) {
    auto b=std::lower_bound(plannedFeet_.begin(),plannedFeet_.end(),elapsed,[](const auto& s,double t){return s.t<t;});
    wheel_leg_demo::FootSample s;
    if(b==plannedFeet_.begin())s=*b;
    else if(b==plannedFeet_.end())s=plannedFeet_.back();
    else {
      auto a=b-1;double dt=b->t-a->t,u=(elapsed-a->t)/dt;s=*a;
      auto value=[&](double p,double q,int axis){return wheel_leg_demo::footHermite(p,q,a->velocity[axis],b->velocity[axis],a->acceleration[axis],b->acceleration[axis],dt,u);};
      s.x=value(a->x,b->x,0);s.y=value(a->y,b->y,1);s.z=value(a->z,b->z,2);
      for(int i=0;i<2;i++){s.fx[i]=value(a->fx[i],b->fx[i],3+2*i);s.fz[i]=value(a->fz[i],b->fz[i],4+2*i);}
    }
    auto cfg=footConfig_;cfg.y=s.y;
    // Validate the interpolated pose against current Gazebo obstacles, too.
    cfg.obstacles.clear();
    for(auto m:model_->GetWorld()->Models())if(m->GetName().find("obstacle")!=std::string::npos){auto box=m->BoundingBox();auto mid=(box.Min()+box.Max())/2;auto half=(box.Max()-box.Min())/2;cfg.obstacles.push_back({mid.X(),mid.Y(),mid.Z(),half.X(),half.Y(),half.Z()});}
    std::string why;if(!wheel_leg_demo::footPoseValid(s,cfg,&why)){action_=Action::Idle;state_="FAILED_FOOT_GUARD: "+why;return;}
    x_=s.x;y_=s.y;z_=s.z;fx_=s.fx;fz_=s.fz;
    if(elapsed>=plannedFeet_.back().t){action_=Action::Idle;state_="DONE_PLANNED_CLIMB";}
  }

  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    double now=model_->GetWorld()->SimTime().Double();
    if(resetPending_.exchange(false) || now<last_) resetState();
    else if(now-last_<.01) return;
    const double dt=std::max(0.0,std::min(.05,now-last_));
    last_=now;
    if(external_&&action_==Action::Idle&&!leg_) {
      const bool fresh=now-commandTime_<.35 && now>=commandTime_;
      double tv=fresh?commandV_:0,tw=fresh?commandW_:0;
      v_+=std::clamp(tv-v_,-maxA_*dt,maxA_*dt);w_+=std::clamp(tw-w_,-maxAlpha_*dt,maxAlpha_*dt);
      double heading=yaw_+w_*dt/2,nx=x_+v_*std::cos(heading)*dt,ny=y_+v_*std::sin(heading)*dt,na=yaw_+w_*dt;
      if(!wheelPoseFree(nx,ny,na)){v_=w_=0;external_=false;state_="STOPPED_COLLISION_GUARD";}
      else {
        x_=nx;y_=ny;yaw_=std::atan2(std::sin(na),std::cos(na));
        leftWheel_+=(v_-w_*.21)*dt/radius_;rightWheel_+=(v_+w_*.21)*dt/radius_;
        auto f=folded(x_,z_);fx_.fill(f[0]);fz_.fill(f[1]);
        state_=fresh?"WHEEL_VELOCITY_EXECUTION":"STOPPED_COMMAND_TIMEOUT";
      }
    }
    double u=(now-begin_)/morphTime_;
    if(action_==Action::Deploy) {
      double h=ground(x_);
      if(u<.5) {
        for(int i=0;i<2;i++){fx_[i]=mix(fromFx_[i],x_+hipX_+toeOffset_,u*2);fz_[i]=mix(fromFz_[i],h+footHalf_,u*2);}
      } else {
        fx_.fill(x_+hipX_+toeOffset_);fz_.fill(h+footHalf_);
        z_=mix(fromZ_,h+legHeight_,(u-.5)*2);lift_=mix(0,liftTarget_,(u-.5)*2);
      }
      if(u>=1){leg_=true;action_=Action::Idle;state_="DONE_TO_LEG";}
    } else if(action_==Action::Fold) {
      double h=fromFz_[0]-footHalf_;
      if(u<.5) {z_=mix(fromZ_,h+startZ_,u*2);lift_=mix(fromLift_,0,u*2);}
      else {
        z_=h+startZ_;lift_=0;auto f=folded(x_,z_);
        for(int i=0;i<2;i++){fx_[i]=mix(fromFx_[i],f[0],(u-.5)*2);fz_[i]=mix(fromFz_[i],f[1],(u-.5)*2);}
      }
      if(u>=1){leg_=false;action_=Action::Idle;state_="DONE_TO_WHEEL";}
    } else if(action_==Action::Drive) {
      double t=(now-begin_)/(1.5*driveDistance_/driveSpeed_);
      x_=mix(fromX_,fromX_+driveDistance_,t);wheel_=fromWheel_+(x_-fromX_)/radius_;
      auto f=folded(x_,z_);fx_.fill(f[0]);fz_.fill(f[1]);
      if(t>=1){action_=Action::Idle;state_="DONE_DRIVE";}
    } else if(action_==Action::PlannedClimb) {
      plannedFootTick(now-begin_);
    } else if(action_==Action::Climb) {
      double elapsed=now-begin_;
      fx_=fromFx_;fz_=fromFz_;
      for(size_t n=0;n<footholds_.size()*2;n++) {
        double t=(elapsed-n*stepTime_)/stepTime_;if(t<=0)break;
        int side=n%2;double target=footholds_[n/2],height=ground(target)+footHalf_;
        if(t>=1){fx_[side]=target;fz_[side]=height;continue;}
        double peak=std::max(fz_[side],height)+clearance_;
        if(t<.25)fz_[side]=mix(fz_[side],peak,t*4);
        else if(t<.75){fx_[side]=mix(fx_[side],target,(t-.25)*2);fz_[side]=peak;}
        else {fx_[side]=target;fz_[side]=mix(peak,height,(t-.75)*4);}
        break;
      }
      x_=(fx_[0]+fx_[1])/2-hipX_-toeOffset_;z_=std::min(fz_[0],fz_[1])-footHalf_+legHeight_;
      if(elapsed>=footholds_.size()*2*stepTime_){action_=Action::Idle;state_="DONE_CLIMB";}
    }
    wallTickMs_.store(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    if(!apply(now)){action_=Action::Idle;state_="FAILED_UNREACHABLE: reset required";}
    if(now-report_>=.1||report_<0) {
      std_msgs::msg::String s;s.data=state_+" | mode="+(leg_?"leg":"wheel")+" | x="+std::to_string(x_)+" | kinematic display, simulation feedback";
      if(action_==Action::Climb) s.data+=" | step="+std::to_string(std::min(8,1+static_cast<int>((last_-begin_)/stepTime_)))+"/8";
      status_->publish(s);
      gazebo_msgs::msg::ModelStates models;
      models.name.push_back(model_->GetName());
      auto append=[](const ignition::math::Pose3d &p) {
        geometry_msgs::msg::Pose out;
        out.position.x=p.Pos().X();out.position.y=p.Pos().Y();out.position.z=p.Pos().Z();
        out.orientation.x=p.Rot().X();out.orientation.y=p.Rot().Y();out.orientation.z=p.Rot().Z();out.orientation.w=p.Rot().W();return out;
      };
      models.pose.push_back(append(model_->WorldPose()));models.twist.emplace_back();
      models.twist.back().linear.x=v_*std::cos(yaw_);models.twist.back().linear.y=v_*std::sin(yaw_);models.twist.back().angular.z=w_;
      models_->publish(models);
      trajectory_msgs::msg::MultiDOFJointTrajectory footFeedback;
      footFeedback.header.stamp=qTime(now);footFeedback.header.frame_id="world";
      footFeedback.joint_names={"body","left_foot_center","right_foot_center"};
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint footPoint;
      if(action_==Action::PlannedClimb||state_=="DONE_PLANNED_CLIMB")footPoint.time_from_start=rclcpp::Duration::from_seconds(std::max(0.,now-begin_));
      for(const std::string name:{"base_link","left_foot","right_foot"}) {
        auto p=name=="base_link"?model_->WorldPose():model_->GetLink(name)->WorldPose();
        auto pos=p.Pos();if(name!="base_link")pos+=p.Rot().RotateVector(ignition::math::Vector3d(toeOffset_,0,0));
        geometry_msgs::msg::Transform t;t.translation.x=pos.X();t.translation.y=pos.Y();t.translation.z=pos.Z();
        t.rotation.x=p.Rot().X();t.rotation.y=p.Rot().Y();t.rotation.z=p.Rot().Z();t.rotation.w=p.Rot().W();footPoint.transforms.push_back(t);
      }
      footFeedback.points.push_back(footPoint);footFeedback_->publish(footFeedback);
      nav_msgs::msg::Odometry odom;odom.header.stamp=qTime(now);odom.header.frame_id="world";odom.child_frame_id="base_link";
      odom.pose.pose=models.pose.back();odom.twist.twist.linear.x=v_;odom.twist.twist.angular.z=w_;odom_->publish(odom);
      gazebo_msgs::msg::LinkStates links;
      for(const auto &link:model_->GetLinks()) {
        links.name.push_back(link->GetScopedName());links.pose.push_back(append(link->WorldPose()));links.twist.emplace_back();
      }
      links_->publish(links);report_=now;
    }
  }
public:
  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override {
    model_=model;ros_=gazebo_ros::Node::Get(sdf);
    auto get=[&](const std::string &key,double fallback){return sdf->HasElement(key)?sdf->Get<double>(key):fallback;};
    maxV_=get("max_v",.3);maxW_=get("max_w",.8);maxA_=get("max_a",.3);maxAlpha_=get("max_alpha",1.);
    exitX_=get("exit_x",3.3);exitY_=get("exit_y",-.9);
    startX_=get("start_x",0);startY_=get("start_y",-.9);startZ_=get("start_z",.32);
    stairX_=get("stair_x",2);rise_=get("rise",.17);tread_=get("tread",.35);entry_=get("entry_x",1.6);
    morphTime_=get("morph_duration",4);driveDistance_=get("drive_distance",1.6);driveSpeed_=get("drive_speed",.3);
    stepTime_=get("step_duration",2);clearance_=get("swing_clearance",.18);legHeight_=get("leg_body_height",.43);liftTarget_=get("wheel_lift",.13);
    lowerEnd_=get("lower_end",2);upperStart_=get("upper_start",2.7);upperEnd_=get("upper_end",6.7);worldHalfWidth_=get("half_width",2);
    footholds_={stairX_+tread_/2,stairX_+1.5*tread_,get("platform_foot_1",2.9),get("platform_foot_2",3.12)};
    if(morphTime_<=0||driveDistance_<=0||driveSpeed_<=0||stepTime_<=0||std::abs(startY_)+robotWidth_/2>worldHalfWidth_)throw std::runtime_error("Invalid animation parameters");
    for(const std::string name:{"wheel_carrier","left_wheel","right_wheel","left_upper_arm","right_upper_arm","left_forearm","right_forearm","left_foot","right_foot"})
      if(!model_->GetLink(name))throw std::runtime_error("Missing animation link: "+name);
    footConfig_.stair=stairX_;footConfig_.rise=rise_;footConfig_.tread=tread_;footConfig_.width=worldHalfWidth_*2;
    footConfig_.upper_max=upperEnd_;footConfig_.lift=liftTarget_;footConfig_.body_height=legHeight_;
    footConfig_.max_foot_v=get("foot_max_foot_v",.8);footConfig_.max_foot_a=get("foot_max_foot_a",3);
    footConfig_.max_body_v=get("foot_max_body_v",.3);footConfig_.max_body_a=get("foot_max_body_a",1);
    footConfig_.max_joint_v=get("foot_max_joint_v",3);footConfig_.max_joint_a=get("foot_max_joint_a",20);footConfig_.timeout=get("foot_timeout",120);
    footFeedback_=ros_->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/wheel_leg_demo/foot_feedback",10);
    footTrajectory_=ros_->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/wheel_leg_demo/foot_trajectory",rclcpp::QoS(1).reliable(),
      [this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr m){acceptFootTrajectory(*m);});
    odom_=ros_->create_publisher<nav_msgs::msg::Odometry>("/wheel_leg_demo/odom",10);
    velocityCommand_=ros_->create_subscription<geometry_msgs::msg::TwistStamped>("/wheel_leg_demo/cmd_vel",10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        double stamp=rclcpp::Time(msg->header.stamp).seconds(),now=model_->GetWorld()->SimTime().Double();
        if(!std::isfinite(msg->twist.linear.x)||!std::isfinite(msg->twist.angular.z)||std::abs(now-stamp)>.35||leg_||action_!=Action::Idle)return;
        commandV_=std::clamp(msg->twist.linear.x,-maxV_,maxV_);commandW_=std::clamp(msg->twist.angular.z,-maxW_,maxW_);
        commandTime_=stamp;external_=true;
      });
    joints_=ros_->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
    models_=ros_->create_publisher<gazebo_msgs::msg::ModelStates>("/wheel_leg_demo/model_states",10);
    links_=ros_->create_publisher<gazebo_msgs::msg::LinkStates>("/wheel_leg_demo/link_states",10);
    status_=ros_->create_publisher<std_msgs::msg::String>("/wheel_leg_demo/status",10);
    for(const std::string command:{"to_leg","to_wheel","drive","climb","stop","reset","prepare_upper","prepare_entry"}) {
      services_.push_back(ros_->create_service<std_srvs::srv::Trigger>("/wheel_leg_demo/"+command,
        [this,command](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr reply){request(command,*reply);}));
    }
    resetState();last_=model_->GetWorld()->SimTime().Double();
    heartbeat_=ros_->create_wall_timer(std::chrono::milliseconds(250), [this]() {
      std_msgs::msg::String s;
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if(!lock.owns_lock()) {
        const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if(wallTickMs_.load()==0 || ms-wallTickMs_.load()<2000) return;
        s.data="SIMULATION_UPDATE_BLOCKED: animation update is waiting";
      }
      else {
        s.data=state_+" | mode="+(leg_?"leg":"wheel")+" | x="+std::to_string(x_);
        if(action_==Action::Climb) s.data+=" | step="+std::to_string(std::min(8,1+static_cast<int>((last_-begin_)/stepTime_)))+"/8";
        const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if(model_->GetWorld()->IsPaused()) s.data+=" | PAUSED";
        else if(wallTickMs_.load()>0 && ms-wallTickMs_.load()>2000) s.data+=" | SIM_TIME_NOT_ADVANCING";
        s.data+=" | kinematic display, simulation feedback";
      }
      status_->publish(s);
    });
    update_=event::Events::ConnectWorldUpdateBegin(std::bind(&WheelLegAnimation::tick,this));
    RCLCPP_INFO(ros_->get_logger(),"Stage 2 animation services ready. No planning or force control.");
  }
  // Reset can run under Gazebo world locks: defer state changes to tick().
  void Reset() override {resetPending_.store(true);}
};
GZ_REGISTER_MODEL_PLUGIN(WheelLegAnimation)
}
