#include "wheel_leg_demo/output_paths.hpp"
#include "wheel_leg_demo/foot_local.hpp"
#include "wheel_leg_demo/foot_planner.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
namespace wheel_leg_demo {
struct FootLocal::Impl {
  using Trigger=std_srvs::srv::Trigger;
  rclcpp::Node* n;FootConfig c;FootPlan plan;FootSample actual;
  std::string base,dir,executor,state="IDLE";bool reference=false,havePose=false,haveFeet=false,running=false;
  double poseStamp=-1,feetStamp=-1,begin=0,wallMs=0,yaw=0;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr feedbackSub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers;
  rclcpp::Client<Trigger>::SharedPtr stop;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services;
  rclcpp::TimerBase::SharedPtr timer;
  gazebo::transport::NodePtr gz;gazebo::transport::PublisherPtr visuals,requests;
  std::vector<std::string> names;size_t generation=0;
  std::ofstream feedback;
  explicit Impl(rclcpp::Node* node):n(node) {
    auto d=[&](const std::string& key,double v){return n->declare_parameter("foot."+key,v);};
    c.resolution=d("resolution",.05);c.max_step=d("max_step",.5);c.edge_margin=d("edge_margin",.025);
    c.clearance=d("swing_clearance",.08);c.duration=d("step_duration",4.5);c.dt=d("sample_dt",.02);
    c.max_foot_v=d("max_foot_v",.8);c.max_foot_a=d("max_foot_a",3);c.max_body_v=d("max_body_v",.3);c.max_body_a=d("max_body_a",1);
    c.max_joint_v=d("max_joint_v",3);c.max_joint_a=d("max_joint_a",20);c.timeout=d("timeout",120);
    n->declare_parameter("foot.case_name",std::string("baseline"));n->declare_parameter("foot.block_first_tread",false);
    base=n->declare_parameter("foot.output_dir",wheel_leg_demo::defaultOutputDir("foot_local"));
    c.stair=n->get_parameter("stair_x").as_double();c.rise=n->get_parameter("rise").as_double();c.tread=n->get_parameter("tread").as_double();c.width=n->get_parameter("stair_width").as_double();
    auto exit=n->get_parameter("exit").as_double_array();c.goal=exit[0];c.y=exit[1];
    auto bounds=n->get_parameter("lower_bounds").as_double_array();c.lower_min=bounds[0];c.half_width=std::min(-bounds[2],bounds[3]);
    bounds=n->get_parameter("upper_bounds").as_double_array();c.upper_max=bounds[1];
    auto boxes=n->get_parameter("obstacles").as_double_array();for(size_t i=0;i+5<boxes.size();i+=6)c.obstacles.push_back({boxes[i],boxes[i+1],boxes[i+2],boxes[i+3]/2,boxes[i+4]/2,boxes[i+5]/2});
    auto qos=rclcpp::QoS(1).transient_local();
    trajectory=n->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/wheel_leg_demo/foot_trajectory",rclcpp::QoS(1).reliable());
    output=n->create_publisher<std_msgs::msg::String>("/wheel_leg_demo/foot_status",qos);
    markers=n->create_publisher<visualization_msgs::msg::MarkerArray>("/wheel_leg_demo/foot_markers",qos);
    stop=n->create_client<Trigger>("/wheel_leg_demo/stop");
    feedbackSub=n->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/wheel_leg_demo/foot_feedback",rclcpp::QoS(1),[this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr m){
      if(m->points.size()!=1||m->points[0].transforms.size()!=3)return;
      actual.t=rclcpp::Duration(m->points[0].time_from_start).seconds();
      auto p=m->points[0].transforms;actual.x=p[0].translation.x;actual.y=p[0].translation.y;actual.z=p[0].translation.z;
      yaw=2*std::atan2(p[0].rotation.z,p[0].rotation.w);poseStamp=feetStamp=rclcpp::Time(m->header.stamp).seconds();havePose=haveFeet=true;
      for(int i=0;i<2;i++){actual.fx[i]=p[i+1].translation.x;actual.fz[i]=p[i+1].translation.z;}
      if(running&&feedback)feedback<<std::setprecision(12)<<actual.t<<','<<actual.x<<','<<actual.y<<','<<actual.z<<','<<actual.fx[0]<<','<<actual.fz[0]<<','<<actual.fx[1]<<','<<actual.fz[1]<<'\n';
    });
    status=n->create_subscription<std_msgs::msg::String>("/wheel_leg_demo/status",10,[this](std_msgs::msg::String::SharedPtr m){
      executor=m->data;
      if(!running)return;
      if(executor.find("DONE_PLANNED_CLIMB")==0)finish("DONE");
      else if(executor.find("FAILED")==0||executor.find("STOPPED")==0||executor.find("READY_WHEEL")==0)finish("FAILED_EXECUTION: "+executor);
    });
    for(const std::string cmd:{"plan_footsteps","run_footsteps","stop_footsteps"})services.push_back(n->create_service<Trigger>("/wheel_leg_demo/"+cmd,
      [this,cmd](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){request(cmd,*r);}));
    timer=n->create_wall_timer(std::chrono::milliseconds(100),[this](){
      if(!running)return;
      double now=n->now().seconds();
      if(now<begin||now-poseStamp>.5||now-feetStamp>.5){halt();finish("FAILED_FEEDBACK");}
      else if(now-begin>c.timeout){halt();finish("FAILED_TIMEOUT");}
    });
    publish("IDLE");
  }
  void publish(const std::string& value){state=value;std_msgs::msg::String m;m.data=value+" | simulation feedback; kinematic execution";output->publish(m);}
  void halt(){if(stop->service_is_ready())stop->async_send_request(std::make_shared<Trigger::Request>());}
  void finish(const std::string& result) {
    running=false;feedback.close();
    double error=std::hypot(actual.x-c.goal,actual.y-c.y);
    bool reached=result=="DONE"&&error<=.10&&std::abs(actual.z-(2*c.rise+c.body_height))<.01;
    for(int i=0;i<2;i++)reached=reached&&actual.fx[i]-c.foot_l/2>=c.stair+2*c.tread&&std::abs(actual.fx[i]-(c.goal+c.hip_x+c.toe))<.03&&std::abs(actual.fz[i]-(2*c.rise+c.foot_h/2))<.005;
    std::string final=result=="DONE"&&!reached?"FAILED_ENDPOINT":result;publish(final);saveResult(final,error);
  }
  void saveResult(const std::string& result,double error=-1) {
    std::ofstream f(dir+"/结果.csv");f<<"state,planning_success,steps,expanded,planning_wall_ms,simulation_duration_s,position_error_m,forbidden_first_tread\n"<<result<<','<<plan.ok<<','<<plan.steps.size()<<','<<plan.expanded<<','<<wallMs<<','<<(begin>0?actual.t:0)<<','<<error<<','<<n->get_parameter("foot.block_first_tread").as_bool()<<'\n';
  }
  void savePlan() {
    std::ofstream f(dir+"/足步.csv");f<<"step,side,from_x_m,foot_center_x_m,foot_center_y_m,ground_z_m,edge_margin_m,touchdown_time_s\n";
    for(size_t i=0;i<plan.steps.size();i++){auto s=plan.steps[i];f<<i+1<<','<<(s.side==0?"left":"right")<<','<<s.from<<','<<s.to<<','<<c.y+(s.side==0?1:-1)*(c.hip_y+c.elbow_y)<<','<<s.height<<','<<s.edge<<','<<s.t<<'\n';}
    std::ofstream t(dir+"/规划轨迹.csv");t<<"t_s,body_x_m,body_y_m,body_z_m,left_x_m,left_z_m,right_x_m,right_z_m\n";
    for(auto s:plan.samples)t<<std::setprecision(12)<<s.t<<','<<s.x<<','<<s.y<<','<<s.z<<','<<s.fx[0]<<','<<s.fz[0]<<','<<s.fx[1]<<','<<s.fz[1]<<'\n';
    std::ofstream p(dir+"/参数.yaml");p<<"frame: world\nfeedback: Gazebo simulation\nalgorithm: footstep_A_star_and_piecewise_quintic\nresolution: "<<c.resolution<<"\nmax_step: "<<c.max_step<<"\nedge_margin: "<<c.edge_margin<<"\nswing_clearance: "<<c.clearance<<"\nstep_duration: "<<c.duration<<"\nsample_dt: "<<c.dt<<"\nmax_foot_v: "<<c.max_foot_v<<"\nmax_foot_a: "<<c.max_foot_a<<"\nmax_body_v: "<<c.max_body_v<<"\nmax_body_a: "<<c.max_body_a<<"\nmax_joint_v: "<<c.max_joint_v<<"\nmax_joint_a: "<<c.max_joint_a<<"\nblock_first_tread: "<<(n->get_parameter("foot.block_first_tread").as_bool()?"true":"false")<<"\n";
  }
  void draw() {
    visualization_msgs::msg::MarkerArray array;visualization_msgs::msg::Marker clear;clear.action=clear.DELETEALL;array.markers.push_back(clear);
    for(auto name:names){gazebo::msgs::Request m;m.set_id(0);m.set_request("entity_delete");m.set_data(name);requests->Publish(m);}names.clear();++generation;
    std::string group="planned_footsteps_"+std::to_string(generation);
    std::ostringstream sdf;sdf<<"<sdf version='1.6'><model name='"<<group<<"'><static>true</static>";
    for(size_t i=0;i<plan.steps.size();i++) {
      auto s=plan.steps[i];double y=c.y+(s.side==0?1:-1)*(c.hip_y+c.elbow_y);
      visualization_msgs::msg::Marker marker;marker.header.frame_id="world";marker.header.stamp=n->now();marker.ns="planned_footholds";marker.id=i;marker.type=marker.CUBE;marker.action=marker.ADD;marker.pose.position.x=s.to;marker.pose.position.y=y;marker.pose.position.z=s.height+.007;marker.pose.orientation.w=1;marker.scale.x=c.foot_l;marker.scale.y=c.foot_w;marker.scale.z=.008;marker.color.a=.65;marker.color.r=s.side==0?.05:1;marker.color.g=s.side==0?.8:.45;marker.color.b=s.side==0?1:.05;array.markers.push_back(marker);
      auto label=marker;label.ns="footstep_numbers";label.type=label.TEXT_VIEW_FACING;label.text=std::to_string(i+1);label.pose.position.z+=.08;label.scale.z=.075;array.markers.push_back(label);
      // One model contains all visual-only sole links, inserted atomically.
      sdf<<"<link name='step_"<<i+1<<"'><pose>"<<s.to<<" "<<y<<" "<<s.height+.012<<" 0 0 0</pose><visual name='sole'><geometry><box><size>"<<c.foot_l<<" "<<c.foot_w<<" .008</size></box></geometry><material><ambient>"<<(s.side==0?"0.05 0.8 1 1":"1 0.45 0.05 1")<<"</ambient><diffuse>"<<(s.side==0?"0.05 0.8 1 1":"1 0.45 0.05 1")<<"</diffuse></material><cast_shadows>false</cast_shadows></visual></link>";
    }
    sdf<<"</model></sdf>";
    if(!plan.steps.empty()){names.push_back(group);gazebo::msgs::Factory factory;factory.set_sdf(sdf.str());visuals->Publish(factory);}
    markers->publish(array);
  }
  void request(const std::string& cmd,Trigger::Response& r) {
    if(cmd=="stop_footsteps"){halt();if(running)finish("STOPPED");plan.ok=false;r.success=true;r.message="Stopped; replan before execution";return;}
    if(running){r.message="Foot trajectory is running";return;}
    const double now=n->now().seconds();
    if(!havePose||!haveFeet||now-poseStamp>.5||now-feetStamp>.5){r.message="Fresh simulation body and feet feedback required";return;}
    if(cmd=="plan_footsteps") {
      if(executor.find("mode=leg")==std::string::npos||executor.find("DONE_TO_LEG")!=0){r.message="Morph to leg at E before planning (DONE_TO_LEG required)";return;}
      auto entry=n->get_parameter("entry").as_double_array();
      if(std::hypot(actual.x-entry[0],actual.y-entry[1])>.10||std::abs(yaw)>.005){r.message="Planar stairs require entry alignment (yaw <=0.005 rad)";return;}
      std::string name=n->get_parameter("foot.case_name").as_string();if(name.empty()||name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos){r.message="Invalid case_name";return;}
      base=n->get_parameter("foot.output_dir").as_string();
      dir=base+"/"+name;std::filesystem::create_directories(dir);begin=0;
      c.edge_margin=n->get_parameter("foot.edge_margin").as_double();c.max_step=n->get_parameter("foot.max_step").as_double();
      c.block_first_tread=n->get_parameter("foot.block_first_tread").as_bool();
      c.y=actual.y;FootSample start=actual;start.t=0;
      auto clock=std::chrono::steady_clock::now();
      if(!reference){plan={};plan.reason="GLOBAL_STAIR_REFERENCE_REQUIRED";}else plan=planFeet(start,c);
      wallMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-clock).count();
      savePlan();draw();publish(plan.ok?"PLANNED":"FAILED_PLAN: "+plan.reason);saveResult(state);r.success=plan.ok;r.message=plan.reason;
    } else {
      if(!plan.ok||plan.samples.empty()){r.message="No valid footstep plan; plan first";return;}
      if(executor.find("DONE_TO_LEG")!=0){r.message="Executor must be idle in initial leg pose";return;}
      auto start=plan.samples.front();double mismatch=std::abs(actual.x-start.x)+std::abs(actual.y-start.y)+std::abs(actual.z-start.z);
      for(int i=0;i<2;i++)mismatch+=std::abs(actual.fx[i]-start.fx[i])+std::abs(actual.fz[i]-start.fz[i]);
      if(mismatch>.01){plan.ok=false;r.message="Pose changed; replan required";return;}
      if(trajectory->get_subscription_count()==0){r.message="Execution subscriber unavailable";return;}
      trajectory_msgs::msg::MultiDOFJointTrajectory msg;msg.header.frame_id="world";msg.header.stamp=n->now();msg.joint_names={"body","left_foot_center","right_foot_center"};
      for(auto s:plan.samples){trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;point.time_from_start=rclcpp::Duration::from_seconds(s.t);for(int i=0;i<3;i++){geometry_msgs::msg::Transform p;p.rotation.w=1;p.translation.x=i==0?s.x:s.fx[i-1];p.translation.y=s.y+(i==0?0:(i==1?1:-1)*(c.hip_y+c.elbow_y));p.translation.z=i==0?s.z:s.fz[i-1];point.transforms.push_back(p);geometry_msgs::msg::Twist v,a;
        int axis=i==0?0:3+2*(i-1);v.linear.x=s.velocity[axis];a.linear.x=s.acceleration[axis];
        if(i==0){v.linear.y=s.velocity[1];a.linear.y=s.acceleration[1];v.linear.z=s.velocity[2];a.linear.z=s.acceleration[2];}
        else {v.linear.z=s.velocity[axis+1];a.linear.z=s.acceleration[axis+1];}
        point.velocities.push_back(v);point.accelerations.push_back(a);}msg.points.push_back(point);}
      feedback.open(dir+"/实际轨迹.csv");feedback<<"t_s,body_x_m,body_y_m,body_z_m,left_x_m,left_z_m,right_x_m,right_z_m\n";
      begin=now;running=true;publish("RUNNING");trajectory->publish(msg);r.success=true;r.message="Accepted; completion on /wheel_leg_demo/foot_status";
    }
  }
};
FootLocal::FootLocal(rclcpp::Node* n):impl_(std::make_unique<Impl>(n)){}
FootLocal::~FootLocal()=default;
bool FootLocal::command(const std::string& name,std::string& message) {Impl::Trigger::Response r;impl_->request(name,r);message=r.message;return r.success;}
bool FootLocal::running()const{return impl_->running;}
std::string FootLocal::state()const{return impl_->state;}
void FootLocal::reset(){impl_->plan={};impl_->draw();impl_->publish("IDLE");}

void FootLocal::connectGazebo() {
  auto& f=*impl_;f.gz.reset(new gazebo::transport::Node);f.gz->Init("two_steps");
  f.visuals=f.gz->Advertise<gazebo::msgs::Factory>("~/factory");f.requests=f.gz->Advertise<gazebo::msgs::Request>("~/request");
}
void FootLocal::setReference(const nav_msgs::msg::Path& p,size_t lo,size_t stair) {
  auto& f=*impl_;f.reference=false;
  if(stair<2||lo+stair>p.poses.size())return;
  auto a=p.poses[lo].pose.position,b=p.poses[lo+stair-1].pose.position;
  auto e=f.n->get_parameter("entry").as_double_array();f.reference=std::hypot(a.x-e[0],a.y-e[1])<.01&&std::hypot(b.x-f.c.goal,b.y-f.c.y)<.01;
}
}
