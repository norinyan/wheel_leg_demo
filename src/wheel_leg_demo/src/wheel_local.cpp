#include "wheel_leg_demo/output_paths.hpp"
#include "wheel_leg_demo/wheel_local.hpp"
#include "wheel_leg_demo/teb_optimizer.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <optional>

namespace wheel_leg_demo {
using Trigger=std_srvs::srv::Trigger;
struct WheelLocal::Impl {
  rclcpp::Node* n;
  TebConfig c;
  Pose2 pose{};
  double z=.32,v=0,w=0,feedbackTime=-100,lastTime=-1,startTime=0,startFeedbackTime=0,invalidSince=-1;
  double commandV=0,commandW=0,progress=0,lastRecorded=-1;
  double positionTolerance=.055,yawTolerance=.035,stopV=.012,stopW=.025;
  double feedbackTimeout=.5,segmentTimeout=100,blockedTimeout=3;
  bool running=false,upper=false,haveFeedback=false,pendingObstacle=false;
  std::string state="IDLE",outputBase,output;
  std::vector<Pose2> lowerRef,upperRef,ref;
  std::vector<Obstacle> obstacles;
  std::optional<Obstacle> testObstacle;
  std::ofstream actualCsv,bandCsv;
  int planId=0;double minClearance=1e9,maxObservedV=0,maxObservedW=0;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr localPath,actualPath;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr remove;
  rclcpp::TimerBase::SharedPtr timer;
  nav_msgs::msg::Path traveled;
  gazebo::transport::NodePtr gz;
  gazebo::transport::PublisherPtr visual,requests;
  std::vector<std::string> lastVisuals;
  size_t drawGeneration=0;
  template<class T>T param(const std::string& name,T value){return n->declare_parameter("wheel."+name,value);}
  explicit Impl(rclcpp::Node* node):n(node) {
    c.max_v=param("max_v",.3);c.max_w=param("max_w",.8);c.max_a=param("max_a",.3);c.max_alpha=param("max_alpha",1.);
    c.margin=param("safety_margin",.05);c.horizon=param("horizon",2.2);c.spacing=param("spacing",.15);c.iterations=param("iterations",30);
    positionTolerance=param("position_tolerance",.055);yawTolerance=param("yaw_tolerance",.035);
    stopV=param("stop_v",.012);stopW=param("stop_w",.025);feedbackTimeout=param("feedback_timeout",.5);
    segmentTimeout=param("segment_timeout",100.);blockedTimeout=param("blocked_timeout",3.);
    param("case_name",std::string("auto"));
    outputBase=param("output_dir",wheel_leg_demo::defaultOutputDir("wheel_local"));
    auto values=n->get_parameter("obstacles").as_double_array();for(size_t i=0;i+5<values.size();i+=6)obstacles.push_back({values[i],values[i+1],values[i+3],values[i+4]});
    cmd=n->create_publisher<geometry_msgs::msg::TwistStamped>("/wheel_leg_demo/cmd_vel",10);
    localPath=n->create_publisher<nav_msgs::msg::Path>("/wheel_leg_demo/local_path",10);
    actualPath=n->create_publisher<nav_msgs::msg::Path>("/wheel_leg_demo/actual_path",10);
    status=n->create_publisher<std_msgs::msg::String>("/wheel_leg_demo/local_status",rclcpp::QoS(1).transient_local());
    odom=n->create_subscription<nav_msgs::msg::Odometry>("/wheel_leg_demo/odom",rclcpp::QoS(1),[this](nav_msgs::msg::Odometry::SharedPtr m){
      pose={m->pose.pose.position.x,m->pose.pose.position.y,2*std::atan2(m->pose.pose.orientation.z,m->pose.pose.orientation.w)};
      z=m->pose.pose.position.z;v=m->twist.twist.linear.x;w=m->twist.twist.angular.z;feedbackTime=rclcpp::Time(m->header.stamp).seconds();haveFeedback=true;
    });
    spawn=n->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");remove=n->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    for(std::string name:{"run_lower","run_upper","stop_local","add_local_obstacle","block_lower","block_upper","clear_test_obstacle"}) {
      services.push_back(n->create_service<Trigger>("/wheel_leg_demo/"+name,[this,name](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> res){request(name,*res);}));
    }
    timer=n->create_wall_timer(std::chrono::milliseconds(100),[this](){tick();});
  }
  Bounds bounds()const{return upper?Bounds{2.7,10.7,-3,3}:Bounds{-6,2,-3,3};}
  std::vector<Obstacle> nearby()const {
    std::vector<Obstacle> out;for(auto o:obstacles)if(upper?o.x>=2.7:o.x<2)out.push_back(o);
    if(testObstacle)out.push_back(*testObstacle);
    return out;
  }
  void recordObstacle() {
    if(!testObstacle||output.empty())return;
    auto o=*testObstacle;std::ofstream f(output+"/测试障碍.json");
    f<<"{\"x\":"<<o.x<<",\"y\":"<<o.y<<",\"length\":"<<o.length<<",\"width\":"<<o.width<<",\"simulation_time\":"<<n->now().seconds()<<"}\n";
  }
  void say(){std_msgs::msg::String msg;msg.data=state+" | segment="+(upper?"X-B":"A-E")+" | simulation feedback";status->publish(msg);}
  void send(double linear,double angular) {
    geometry_msgs::msg::TwistStamped m;m.header.stamp=n->now();m.header.frame_id="base_link";m.twist.linear.x=linear;m.twist.angular.z=angular;cmd->publish(m);
    commandV=linear;commandW=angular;
  }
  void finish(const std::string& reason) {
    state=reason;running=false;send(0,0);say();
    if(actualCsv.is_open())actualCsv.flush();
    if(bandCsv.is_open())bandCsv.flush();
    if(!output.empty()) {
      std::ofstream file(output+"/结果.csv");auto goal=ref.back();
      file<<"state,position_error_m,yaw_error_rad,simulation_duration_s,max_actual_v,max_actual_w,min_conservative_clearance_m\n"
          <<state<<','<<std::hypot(pose.x-goal.x,pose.y-goal.y)<<','<<std::abs(wrap(pose.yaw-goal.yaw))<<','<<n->now().seconds()-startTime<<','<<maxObservedV<<','<<maxObservedW<<','<<minClearance<<'\n';
    }
  }
  static double length(const std::vector<Pose2>& path){double s=0;for(size_t i=1;i<path.size();i++)s+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);return s;}
  Pose2 at(double arc)const {
    for(size_t i=1;i<ref.size();i++) {
      double d=std::hypot(ref[i].x-ref[i-1].x,ref[i].y-ref[i-1].y);
      if(arc<=d){double u=d>1e-8?arc/d:0;return {ref[i-1].x+u*(ref[i].x-ref[i-1].x),ref[i-1].y+u*(ref[i].y-ref[i-1].y),std::atan2(ref[i].y-ref[i-1].y,ref[i].x-ref[i-1].x)};}arc-=d;
    }return ref.back();
  }
  double closestArc()const {
    double arc=0,best=1e9,result=progress;
    for(size_t i=1;i<ref.size();i++) {
      auto a=ref[i-1],b=ref[i];double dx=b.x-a.x,dy=b.y-a.y,d=std::hypot(dx,dy);
      double u=std::clamp(((pose.x-a.x)*dx+(pose.y-a.y)*dy)/(d*d+1e-12),0.,1.);
      double s=arc+u*d,dist=std::hypot(pose.x-a.x-u*dx,pose.y-a.y-u*dy);
      if(s>=progress-.2&&dist<best){best=dist;result=s;}arc+=d;
    }return std::max(progress,result);
  }
  std::vector<Pose2> seed(double end,int side,const std::vector<Obstacle>& obs) {
    std::vector<Pose2> points{pose};int count=std::max(4,int(std::ceil((end-progress)/c.spacing)));
    for(int i=1;i<=count;i++)points.push_back(at(progress+(end-progress)*i/count));
    // Alternative left/right initial bands help escape an obstacle-centered local minimum.
    if(side)for(auto o:obs) {
      double radius=std::hypot(c.length,c.width)/2+c.margin+.08;
      bool relevant=false;for(auto p:points)if(std::abs(p.x-o.x)<o.length/2+radius&&std::abs(p.y-o.y)<o.width/2+radius)relevant=true;
      if(!relevant)continue;
      double target=o.y+side*(o.width/2+radius);
      for(size_t i=1;i+1<points.size();i++) {
        double dx=std::abs(points[i].x-o.x),span=o.length/2+radius+.5;
        if(dx<span) {double blend=std::min(1.,(span-dx)/.5);points[i].y+=(target-points[i].y)*blend;}
      }
    }
    for(size_t i=1;i+1<points.size();i++)points[i].yaw=std::atan2(points[i+1].y-points[i-1].y,points[i+1].x-points[i-1].x);
    points.front()=pose;if(end>=length(ref)-1e-5)points.back().yaw=ref.back().yaw;
    return points;
  }
  void draw(const nav_msgs::msg::Path& path) {
    if(!gz){gz.reset(new gazebo::transport::Node);gz->Init("two_steps");visual=gz->Advertise<gazebo::msgs::Visual>("~/visual");requests=gz->Advertise<gazebo::msgs::Request>("~/request");}
    if(!visual->HasConnections())return;
    for(const auto& name:lastVisuals){gazebo::msgs::Request msg;msg.set_id(0);msg.set_request("entity_delete");msg.set_data(name);requests->Publish(msg);}
    lastVisuals.clear();++drawGeneration;
    for(size_t i=1;i<path.poses.size();i++) {
      auto a=path.poses[i-1].pose.position,b=path.poses[i].pose.position;double d=std::hypot(b.x-a.x,b.y-a.y);if(d<1e-6)continue;
      std::string name="local_teb_"+std::to_string(drawGeneration)+"_"+std::to_string(i);lastVisuals.push_back(name);
      gazebo::msgs::Visual msg;msg.set_name(name);msg.set_parent_name("two_steps");msg.set_is_static(true);
      gazebo::msgs::Set(msg.mutable_pose(),ignition::math::Pose3d((a.x+b.x)/2,(a.y+b.y)/2,(upper?.34:0)+.065,0,0,std::atan2(b.y-a.y,b.x-a.x)));
      msg.mutable_geometry()->set_type(gazebo::msgs::Geometry::BOX);gazebo::msgs::Set(msg.mutable_geometry()->mutable_box()->mutable_size(),ignition::math::Vector3d(d,.035,.015));
      gazebo::msgs::Set(msg.mutable_material()->mutable_ambient(),ignition::math::Color(.1,1,.25,1));gazebo::msgs::Set(msg.mutable_material()->mutable_diffuse(),ignition::math::Color(.1,1,.25,1));visual->Publish(msg);
    }
  }
  void tick() {
    if(!running)return;
    double now=n->now().seconds();if(now==lastTime)return;
    double dt=lastTime<0?.05:std::clamp(now-lastTime,.001,.2);lastTime=now;
    if(!haveFeedback||now-feedbackTime>feedbackTimeout||feedbackTime-now>.2){finish("FAILED_FEEDBACK");return;}
    if(now-startTime>segmentTimeout){finish("FAILED_TIMEOUT");return;}
    if(feedbackTime>lastRecorded) {
      actualCsv<<feedbackTime-startFeedbackTime<<','<<pose.x<<','<<pose.y<<','<<pose.yaw<<','<<v<<','<<w<<','<<commandV<<','<<commandW<<'\n';
      lastRecorded=feedbackTime;
    }
    auto obs=nearby();auto goal=ref.back();double distance=std::hypot(pose.x-goal.x,pose.y-goal.y);
    maxObservedV=std::max(maxObservedV,std::abs(v));maxObservedW=std::max(maxObservedW,std::abs(w));
    for(auto o:obs)minClearance=std::min(minClearance,std::hypot(std::max(std::abs(pose.x-o.x)-o.length/2,0.),std::max(std::abs(pose.y-o.y)-o.width/2,0.))-std::hypot(c.length,c.width)/2);
    if(!footprintFree(pose,bounds(),obs,c)){finish("FAILED_FOOTPRINT");return;}
    double targetV=0,targetW=0;
    Band band;
    if(distance<positionTolerance) {
      double error=wrap(goal.yaw-pose.yaw);targetW=std::clamp(2*error,-c.max_w,c.max_w);
      if(std::abs(error)<yawTolerance&&std::abs(v)<stopV&&std::abs(w)<stopW){finish("DONE");return;}
    } else {
      progress=closestArc();double end=std::min(length(ref),progress+c.horizon);
      for(int side:{0,1,-1}) {
        auto candidate=optimizeBand(seed(end,side,obs),v,w,bounds(),obs,c);
        if(candidate.valid&&(!band.valid||candidate.cost<band.cost))band=std::move(candidate);
        if(side==0&&band.valid)break;
      }
      if(band.valid) {
        auto a=band.poses[0],b=band.poses[1];double mid=a.yaw+wrap(b.yaw-a.yaw)/2;
        targetV=((b.x-a.x)*std::cos(mid)+(b.y-a.y)*std::sin(mid))/band.dt[0];targetW=wrap(b.yaw-a.yaw)/band.dt[0];
        targetV=std::min(targetV,std::sqrt(2*c.max_a*std::max(0.,distance-.035)));
        nav_msgs::msg::Path p;p.header.frame_id="world";p.header.stamp=n->now();double time=0;
        for(size_t i=0;i<band.poses.size();i++) {
          auto v=band.poses[i];geometry_msgs::msg::PoseStamped point;point.header=p.header;point.pose.position.x=v.x;point.pose.position.y=v.y;point.pose.position.z=upper?.34:0;point.pose.orientation.z=std::sin(v.yaw/2);point.pose.orientation.w=std::cos(v.yaw/2);p.poses.push_back(point);
          bandCsv<<planId<<','<<time<<','<<v.x<<','<<v.y<<','<<v.yaw<<'\n';if(i<band.dt.size())time+=band.dt[i];
        }
        localPath->publish(p);if(planId%4==0)draw(p);planId++;
      } else {
        // No feasible band: stop immediately, retry for a bounded simulation interval.
        if(invalidSince<0)invalidSince=now;
        send(0,0);state="WAITING_NO_FEASIBLE_BAND";say();
        if(now-invalidSince>blockedTimeout)finish("FAILED_BLOCKED");
        return;
      }
    }
    targetV=std::clamp(targetV,0.,c.max_v);targetW=std::clamp(targetW,-c.max_w,c.max_w);
    double cv=commandV+std::clamp(targetV-commandV,-c.max_a*dt,c.max_a*dt);
    double cw=commandW+std::clamp(targetW-commandW,-c.max_alpha*dt,c.max_alpha*dt);
    double brakingHorizon=.3+std::abs(v)/c.max_a;
    Pose2 next{pose.x+cv*std::cos(pose.yaw+cw*brakingHorizon/2)*brakingHorizon,pose.y+cv*std::sin(pose.yaw+cw*brakingHorizon/2)*brakingHorizon,pose.yaw+cw*brakingHorizon};
    if(!sweptFree(pose,next,bounds(),obs,c)){send(0,0);state="STOPPING_FOR_OBSTACLE";say();if(invalidSince<0)invalidSince=now;if(now-invalidSince>blockedTimeout)finish("FAILED_BLOCKED");return;}
    invalidSince=-1;send(cv,cw);state="RUNNING";
    geometry_msgs::msg::PoseStamped actual;actual.header.frame_id="world";actual.header.stamp=n->now();actual.pose.position.x=pose.x;actual.pose.position.y=pose.y;actual.pose.position.z=upper?.34:0;actual.pose.orientation.z=std::sin(pose.yaw/2);actual.pose.orientation.w=std::cos(pose.yaw/2);
    traveled.header=actual.header;traveled.poses.push_back(actual);actualPath->publish(traveled);say();
  }
  void request(const std::string& name,Trigger::Response& reply) {
    if(name=="stop_local"){if(running)finish("STOPPED");else send(0,0);reply.success=true;reply.message="Stopped local planner";return;}
    if(name=="run_lower"||name=="run_upper") {
      if(running){reply.message="Local segment already running";return;}
      outputBase=n->get_parameter("wheel.output_dir").as_string();
      yawTolerance=n->get_parameter("wheel.yaw_tolerance").as_double();
      upper=name=="run_upper";ref=upper?upperRef:lowerRef;
      if(!haveFeedback||ref.empty()){reply.message="Need global reference and odometry";return;}
      if(std::hypot(pose.x-ref.front().x,pose.y-ref.front().y)>.15||std::abs(z-(upper?.66:.32))>.04){reply.message="Initialize at A or X first";return;}
      std::string test=testObstacle?(testObstacle->width>5?"blocked":"added_obstacle"):"baseline";
      auto caseName=n->get_parameter("wheel.case_name").as_string();
      if(caseName!="auto")test=caseName;
      output=outputBase+"/"+(upper?"upper_":"lower_")+test;std::filesystem::create_directories(output);recordObstacle();
      actualCsv.close();bandCsv.close();actualCsv.open(output+"/轨迹.csv");bandCsv.open(output+"/局部轨迹.csv");
      actualCsv<<"time_s,x_m,y_m,yaw_rad,actual_v_mps,actual_w_rps,cmd_v_mps,cmd_w_rps\n";bandCsv<<"plan_id,time_from_start_s,x_m,y_m,yaw_rad\n";
      std::ofstream reference(output+"/参考路径.csv");reference<<"x_m,y_m,yaw_rad\n";for(auto p:ref)reference<<p.x<<','<<p.y<<','<<p.yaw<<'\n';
      running=true;progress=0;startTime=n->now().seconds();startFeedbackTime=feedbackTime;lastTime=-1;invalidSince=-1;commandV=commandW=0;lastRecorded=-1;planId=0;minClearance=1e9;maxObservedV=maxObservedW=0;traveled.poses.clear();state="RUNNING";say();reply.success=true;reply.message="Accepted; query local_status";return;
    }
    if(pendingObstacle){reply.message="Obstacle operation pending";return;}
    if(name=="clear_test_obstacle") {
      if(!remove->service_is_ready()){reply.message="Gazebo delete service unavailable";return;}
      auto req=std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();req->name="local_test_obstacle";pendingObstacle=true;
      remove->async_send_request(req,[this](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future){auto res=future.get();if(res->success||res->status_message.find("not exist")!=std::string::npos)testObstacle.reset();pendingObstacle=false;});
      reply.success=true;reply.message="Requested obstacle removal";return;
    }
    if(testObstacle){reply.message="Clear existing test obstacle first";return;}
    if(!spawn->service_is_ready()){reply.message="Gazebo spawn service unavailable";return;}
    Obstacle ob;
    if(name=="block_lower")ob={-1.5,0,.6,6};
    else if(name=="block_upper")ob={6.2,0,.6,6};
    else {
      if(!haveFeedback||ref.empty()){reply.message="Start a segment before adding the local obstacle";return;}
      auto p=at(closestArc()+1.4);ob={p.x,p.y,.35,.45};
    }
    auto req=std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();req->name="local_test_obstacle";
    req->initial_pose.position.x=ob.x;req->initial_pose.position.y=ob.y;req->initial_pose.position.z=(name=="block_upper"|| (name=="add_local_obstacle"&&upper))?.64:.3;
    std::ostringstream xml;xml<<"<sdf version=\"1.6\"><model name=\"local_test_obstacle\"><static>true</static><link name=\"link\"><collision name=\"c\"><geometry><box><size>"<<ob.length<<' '<<ob.width<<" .6</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>"<<ob.length<<' '<<ob.width<<" .6</size></box></geometry><material><ambient>.8 .1 .7 1</ambient><diffuse>.8 .1 .7 1</diffuse></material></visual></link></model></sdf>";req->xml=xml.str();pendingObstacle=true;
    spawn->async_send_request(req,[this,ob](rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future){if(future.get()->success){testObstacle=ob;if(running)recordObstacle();}pendingObstacle=false;});reply.success=true;reply.message="Requested test obstacle; inspect Gazebo";
  }
};
WheelLocal::WheelLocal(rclcpp::Node* n):impl_(new Impl(n)){}
WheelLocal::~WheelLocal()=default;
bool WheelLocal::command(const std::string& name,std::string& message) {Trigger::Response r;impl_->request(name,r);message=r.message;return r.success;}
bool WheelLocal::obstaclesCleared()const{return !impl_->pendingObstacle&&!impl_->testObstacle;}
bool WheelLocal::running()const{return impl_->running;}
std::string WheelLocal::state()const{return impl_->state;}
void WheelLocal::reset(){auto& f=*impl_;if(f.running)f.finish("STOPPED");f.commandV=f.commandW=0;f.state="IDLE";f.say();}

void WheelLocal::setReference(const nav_msgs::msg::Path& path,size_t lower,size_t stairs) {
  if(impl_->running)impl_->finish("STOPPED_REFERENCE_CHANGED");
  impl_->lowerRef.clear();impl_->upperRef.clear();
  for(size_t i=0;i<path.poses.size();i++) {
    const auto& p=path.poses[i].pose;Pose2 point{p.position.x,p.position.y,2*std::atan2(p.orientation.z,p.orientation.w)};
    if(i<lower)impl_->lowerRef.push_back(point);else if(i>=lower+stairs)impl_->upperRef.push_back(point);
  }
}
}
