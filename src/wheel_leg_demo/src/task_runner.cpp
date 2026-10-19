#include "wheel_leg_demo/output_paths.hpp"
#include "wheel_leg_demo/task_runner.hpp"
#include "wheel_leg_demo/wheel_local.hpp"
#include "wheel_leg_demo/foot_local.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
namespace wheel_leg_demo {
struct TaskRunner::Impl {
  using Trigger=std_srvs::srv::Trigger;
  enum class Phase {Idle,Global,Lower,Deploy,FootPlan,FootRun,Climb,Fold,Upper,Stopping,Resetting,Done,Failed,Stopped};
  rclcpp::Node* n;WheelLocal* wheel;FootLocal* foot;
  std::function<bool()> global;std::function<std::string()> globalReason;
  Phase phase=Phase::Idle;std::string state="READY",reason="",terminal="STOPPED",executor,root,dir;
  bool active=false,haveFeedback=false,remoteAcknowledged=false,stopSent=false,resetRequested=false,resetSent=false,logging=false;
  double x=0,y=0,z=0,yaw=0,v=0,w=0,stamp=-100,begin=0,phaseBegin=0,lastTick=-1;
  double commandTimeout,feedbackTimeout,stageTimeout,taskTimeout,lastRecorded=-1,upperDistance=0,previousX=0,previousY=0;
  std::array<double,2> fx{},fy{},fz{};
  std::vector<double> a,b,e,exit;
  std::ofstream events,trace;
  std::vector<rclcpp::Parameter> original;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr poses;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services;
  std::map<std::string,rclcpp::Client<Trigger>::SharedPtr> clients;
  std::optional<rclcpp::Client<Trigger>::SharedFuture> pending;
  std::chrono::steady_clock::time_point sentAt;
  explicit Impl(rclcpp::Node* node,WheelLocal* wh,FootLocal* ft,std::function<bool()> g,std::function<std::string()> why):n(node),wheel(wh),foot(ft),global(g),globalReason(why) {
    root=n->declare_parameter("task.output_dir",wheel_leg_demo::defaultOutputDir("task"));
    n->declare_parameter("task.case_name",std::string("run_1"));
    commandTimeout=n->declare_parameter("task.command_timeout",10.);feedbackTimeout=n->declare_parameter("task.feedback_timeout",.5);
    stageTimeout=n->declare_parameter("task.stage_timeout",130.);taskTimeout=n->declare_parameter("task.timeout",300.);
    output=n->create_publisher<std_msgs::msg::String>("/wheel_leg_demo/task_status",rclcpp::QoS(1).transient_local());
    status=n->create_subscription<std_msgs::msg::String>("/wheel_leg_demo/status",10,[this](std_msgs::msg::String::SharedPtr m){executor=m->data;});
    odom=n->create_subscription<nav_msgs::msg::Odometry>("/wheel_leg_demo/odom",rclcpp::QoS(1),[this](nav_msgs::msg::Odometry::SharedPtr m){v=m->twist.twist.linear.x;w=m->twist.twist.angular.z;});
    poses=n->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/wheel_leg_demo/foot_feedback",rclcpp::QoS(1),[this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr m){
      if(m->points.size()!=1||m->points[0].transforms.size()!=3)return;
      auto t=m->points[0].transforms;double nx=t[0].translation.x,ny=t[0].translation.y;
      if(active&&phase==Phase::Upper&&haveFeedback)upperDistance+=std::hypot(nx-x,ny-y);
      x=nx;y=ny;z=t[0].translation.z;yaw=2*std::atan2(t[0].rotation.z,t[0].rotation.w);
      for(int i=0;i<2;i++){fx[i]=t[i+1].translation.x;fy[i]=t[i+1].translation.y;fz[i]=t[i+1].translation.z;}
      stamp=rclcpp::Time(m->header.stamp).seconds();haveFeedback=true;
      if(logging&&trace&&stamp>lastRecorded){trace<<std::setprecision(12)<<stamp-begin<<','<<state<<','<<x<<','<<y<<','<<z<<','<<yaw<<','<<v<<','<<w<<','<<fx[0]<<','<<fy[0]<<','<<fz[0]<<','<<fx[1]<<','<<fy[1]<<','<<fz[1]<<'\n';lastRecorded=stamp;}
    });
    for(std::string name:{"to_leg","to_wheel","stop","reset"})clients[name]=n->create_client<Trigger>("/wheel_leg_demo/"+name);
    for(std::string name:{"start_demo","stop_demo","reset_demo","demo_status"})services.push_back(n->create_service<Trigger>("/wheel_leg_demo/"+name,[this,name](Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){request(name,*r);}));
    timer=n->create_wall_timer(std::chrono::milliseconds(100),[this](){tick();});publish();
  }
  static std::string quoted(const std::string& s){std::string out="\"";for(char c:s){if(c=='"')out+='"';out+=c;}return out+'"';}
  bool fresh()const {double now=n->now().seconds();return haveFeedback&&now-stamp<=feedbackTimeout&&stamp-now<.2;}
  bool stopped()const{return std::abs(v)<=.001&&std::abs(w)<=.001;}
  bool at(const std::vector<double>& p,double bodyHeight)const{return p.size()==4&&std::hypot(x-p[0],y-p[1])<=.10&&std::abs(z-p[2]-bodyHeight)<.02&&std::abs(std::atan2(std::sin(yaw-p[3]),std::cos(yaw-p[3])))<=.0872665;}
  bool mode(const std::string& value)const{return executor.find("mode="+value)!=std::string::npos;}
  void publish(){std_msgs::msg::String m;m.data=state+" | reason="+reason+" | simulation feedback; automatic planning task";output->publish(m);}
  void enter(Phase next,const std::string& text,const std::string& why="") {
    phase=next;state=text;reason=why;phaseBegin=n->now().seconds();remoteAcknowledged=false;
    if(logging&&events){events<<std::setprecision(12)<<phaseBegin-begin<<','<<state<<','<<quoted(reason)<<','<<x<<','<<y<<','<<z<<','<<yaw<<','<<v<<','<<w<<'\n';events.flush();}
    publish();RCLCPP_INFO(n->get_logger(),"Task %s: %s",state.c_str(),reason.c_str());
  }
  void restore(){if(!original.empty()){n->set_parameters(original);original.clear();}}
  void save(){
    if(!logging)return;
    std::ofstream f(dir+"/结果.csv");double error=b.size()==4?std::hypot(x-b[0],y-b[1]):-1;
    double angle=b.size()==4?std::abs(std::atan2(std::sin(yaw-b[3]),std::cos(yaw-b[3]))):-1;
    f<<"state,reason,simulation_duration_s,position_error_m,yaw_error_rad,upper_distance_m,final_v_mps,final_w_rps,final_mode\n"<<state<<','<<quoted(reason)<<','<<std::max(0.,n->now().seconds()-begin)<<','<<error<<','<<angle<<','<<upperDistance<<','<<v<<','<<w<<','<<(mode("leg")?"leg":"wheel")<<'\n';
    trace.flush();events.flush();logging=false;trace.close();events.close();
  }
  void done(Phase target,const std::string& text,const std::string& why="") {active=false;enter(target,text,why);save();restore();}
  bool send(const std::string& name) {
    auto client=clients.at(name);if(!client->service_is_ready())return false;
    pending=client->async_send_request(std::make_shared<Trigger::Request>()).future.share();sentAt=std::chrono::steady_clock::now();return true;
  }
  bool poll() {
    if(!pending)return true;
    if(pending->wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      auto reply=pending->get();pending.reset();
      if(!reply->success){abort("COMMAND_REJECTED: "+reply->message);return false;}
      remoteAcknowledged=true;return true;
    }
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-sentAt).count()>commandTimeout){pending.reset();abort("COMMAND_RESPONSE_TIMEOUT");}
    return false;
  }
  void stop(const std::string& why,bool reset=false) {
    std::string ignored;wheel->command("stop_local",ignored);
    if(foot->running())foot->command("stop_footsteps",ignored);
    pending.reset();resetRequested=reset;stopSent=false;resetSent=false;
    enter(Phase::Stopping,"STOPPING",why);
  }
  void abort(const std::string& why) {
    if(phase==Phase::Stopping||phase==Phase::Resetting){done(Phase::Failed,"FAILED",why);return;}
    terminal="FAILED";stop(why);
  }
  void request(const std::string& cmd,Trigger::Response& reply) {
    if(cmd=="demo_status"){reply.success=true;reply.message=state+": "+reason;return;}
    if(cmd=="stop_demo"||cmd=="reset_demo") {
      terminal="STOPPED";stop(cmd=="reset_demo"?"USER_RESET":"USER_STOP",cmd=="reset_demo");reply.success=true;reply.message="Accepted; query task_status";return;
    }
    if(active||phase==Phase::Stopping||phase==Phase::Resetting){reply.message="Task already active; stop or reset first";return;}
    if(!fresh()||wheel->running()||foot->running()||!mode("wheel")||!stopped()||executor.find("READY_WHEEL")!=0){reply.message="Reset to A and wait for fresh idle wheel feedback";return;}
    a=n->get_parameter("start").as_double_array();b=n->get_parameter("goal").as_double_array();e=n->get_parameter("entry").as_double_array();exit=n->get_parameter("exit").as_double_array();
    if(a.size()!=4||b.size()!=4||e.size()!=4||exit.size()!=4||!at(a,.32)){reply.message="Valid start/goal/entry/exit and wheel pose at A required";return;}
    auto name=n->get_parameter("task.case_name").as_string();
    if(name.empty()||name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos){reply.message="Invalid task.case_name";return;}
    stageTimeout=n->get_parameter("task.stage_timeout").as_double();taskTimeout=n->get_parameter("task.timeout").as_double();
    if(stageTimeout<=0||taskTimeout<=0){reply.message="Task timeouts must be positive";return;}
    dir=root+"/"+name;std::filesystem::create_directories(dir);
    std::ofstream parameters(dir+"/参数.yaml");parameters<<"frame: world\nfeedback: Gazebo simulation\nentry_yaw_tolerance_rad: 0.005\nwheel_yaw_tolerance_rad: 0.003\nstop_v_mps: 0.001\nstop_w_rps: 0.001\nstage_timeout_s: "<<stageTimeout<<"\ntask_timeout_s: "<<taskTimeout<<"\n";
    for(const auto& point:std::vector<std::pair<std::string,std::vector<double>>>{{"A",a},{"E",e},{"X",exit},{"B",b}}){parameters<<point.first<<": [";for(size_t i=0;i<point.second.size();i++){if(i)parameters<<", ";parameters<<point.second[i];}parameters<<"]\n";}
    parameters<<"first_tread_forbidden: "<<(n->get_parameter("foot.block_first_tread").as_bool()?"true":"false")<<"\n";
    for(const std::string key:{"output_dir","wheel.output_dir","wheel.case_name","wheel.yaw_tolerance","foot.output_dir","foot.case_name"})original.push_back(n->get_parameter(key));
    n->set_parameters({rclcpp::Parameter("output_dir",dir+"/global"),rclcpp::Parameter("wheel.output_dir",dir),rclcpp::Parameter("wheel.case_name",std::string("task")),rclcpp::Parameter("wheel.yaw_tolerance",.003),rclcpp::Parameter("foot.output_dir",dir),rclcpp::Parameter("foot.case_name",std::string("foot"))});
    events.open(dir+"/事件.csv");trace.open(dir+"/全程轨迹.csv");events<<"time_s,state,reason,x_m,y_m,z_m,yaw_rad,v_mps,w_rps\n";trace<<"time_s,state,x_m,y_m,z_m,yaw_rad,v_mps,w_rps,left_x_m,left_y_m,left_z_m,right_x_m,right_y_m,right_z_m\n";
    logging=true;active=true;begin=n->now().seconds();lastTick=begin;lastRecorded=-1;upperDistance=0;pending.reset();terminal="STOPPED";
    enter(Phase::Global,"PLANNING_GLOBAL");reply.success=true;reply.message="Accepted; task_status DONE means complete";
  }
  void tick() {
    double now=n->now().seconds();
    if(phase==Phase::Stopping) {
      if(!stopSent) {
        if(!stopped()&&fresh()&&now-phaseBegin<3&&executor.find("PAUSED")==std::string::npos)return;
        if(!send("stop")){done(Phase::Failed,"FAILED","STOP_SERVICE_UNAVAILABLE");return;}stopSent=true;
      }
      if(!poll())return;
      if(!resetRequested){done(terminal=="FAILED"?Phase::Failed:Phase::Stopped,terminal,reason);return;}
      if(logging){enter(Phase::Stopped,"CANCELLED_RESET","USER_RESET");save();restore();}
      active=false;
      std::string message;wheel->command("clear_test_obstacle",message);enter(Phase::Resetting,"RESETTING");return;
    }
    if(phase==Phase::Resetting) {
      if(!resetSent){if(!send("reset")){done(Phase::Failed,"FAILED","RESET_SERVICE_UNAVAILABLE");return;}resetSent=true;}
      if(!poll())return;
      auto start=n->get_parameter("start").as_double_array();
      if(fresh()&&stamp>=phaseBegin&&at(start,.32)&&executor.find("READY_WHEEL")==0&&wheel->obstaclesCleared()) {
        active=false;enter(Phase::Idle,"READY");save();restore();wheel->reset();foot->reset();return;
      }
      if(now-phaseBegin>commandTimeout)done(Phase::Failed,"FAILED","RESET_TIMEOUT");
      return;
    }
    if(!active)return;
    if(now<lastTick){abort("SIMULATION_TIME_RESET");return;}
    if(now==lastTick)return;
    lastTick=now;
    if(!fresh()){abort("STALE_SIMULATION_FEEDBACK");return;}
    if(now-begin>taskTimeout||now-phaseBegin>stageTimeout){abort("TASK_OR_STAGE_TIMEOUT");return;}
    if(!poll())return;
    std::string message;
    if(phase==Phase::Global) {
      if(!global()){abort("GLOBAL_PLANNING: "+globalReason());return;}
      if(!wheel->command("run_lower",message)){abort("LOWER_START: "+message);return;}
      enter(Phase::Lower,"DRIVING_LOWER");
    } else if(phase==Phase::Lower||phase==Phase::Upper) {
      auto s=wheel->state();if(s.find("FAILED")==0||s.find("STOPPED")==0){abort("WHEEL_LOCAL: "+s);return;}
      if(s!="DONE")return;
      if(!stopped())return;
      if(phase==Phase::Lower) {
        if(!at(e,.32)||std::abs(yaw-e[3])>.005){abort("ENTRY_ALIGNMENT");return;}
        enter(Phase::Deploy,"DEPLOYING_LEGS");if(!send("to_leg"))abort("MORPH_SERVICE_UNAVAILABLE");
      } else {
        if(!at(b,.32)||!mode("wheel")||upperDistance<1){abort("FINAL_POSE_OR_DISTANCE");return;}
        done(Phase::Done,"DONE","Reached B in wheel mode");
      }
    } else if(phase==Phase::Deploy||phase==Phase::Fold) {
      if(executor.find("FAILED")==0||executor.find("STOPPED")==0){abort("MORPH_EXECUTION: "+executor);return;}
      if(!remoteAcknowledged)return;
      if(phase==Phase::Deploy&&executor.find("DONE_TO_LEG")==0&&mode("leg"))enter(Phase::FootPlan,"PLANNING_FEET");
      else if(phase==Phase::Fold&&executor.find("DONE_TO_WHEEL")==0&&mode("wheel")&&stamp>=phaseBegin) {
        if(!wheel->command("run_upper",message)){abort("UPPER_START: "+message);return;}
        upperDistance=0;enter(Phase::Upper,"DRIVING_UPPER");
      }
    } else if(phase==Phase::FootPlan) {
      if(!foot->command("plan_footsteps",message)){abort("FOOT_PLANNING: "+message);return;}
      enter(Phase::FootRun,"STARTING_CLIMB");
    } else if(phase==Phase::FootRun) {
      if(!foot->command("run_footsteps",message)){abort("FOOT_START: "+message);return;}
      enter(Phase::Climb,"CLIMBING");
    } else if(phase==Phase::Climb) {
      auto s=foot->state();if(s.find("FAILED")==0||s.find("STOPPED")==0){abort("FOOT_EXECUTION: "+s);return;}
      if(s!="DONE")return;
      double end=n->get_parameter("stair_x").as_double()+2*n->get_parameter("tread").as_double();
      if(!at(exit,.43)||!mode("leg")){abort("STAIR_EXIT_POSE");return;}
      for(int i=0;i<2;i++)if(fx[i]-.09<end||std::abs(fz[i]-exit[2]-.0175)>.005){abort("FEET_NOT_ON_UPPER_PLATFORM");return;}
      enter(Phase::Fold,"FOLDING_LEGS");if(!send("to_wheel"))abort("MORPH_SERVICE_UNAVAILABLE");
    }
  }
};
TaskRunner::TaskRunner(rclcpp::Node* n,WheelLocal* w,FootLocal* f,std::function<bool()> g,std::function<std::string()> reason):impl_(std::make_unique<Impl>(n,w,f,g,reason)){}
TaskRunner::~TaskRunner()=default;
}
