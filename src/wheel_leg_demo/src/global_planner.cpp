#include "wheel_leg_demo/output_paths.hpp"
// Global route planner with optional stage 3.2 wheel-local component.
#include "wheel_leg_demo/wheel_local.hpp"
#include "wheel_leg_demo/foot_local.hpp"
#include "wheel_leg_demo/task_runner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <queue>
#include <sstream>
#include <chrono>
#include <sys/stat.h>

struct Point { double x,y,z,yaw=0; };
struct Box { double x,y,z,l,w,h; };
struct Map { double xmin,xmax,ymin,ymax,z; };
class Planner : public rclcpp::Node {
  using Trigger=std_srvs::srv::Trigger;
  double resolution_,margin_,radius_,length_,width_;
  Map lower_,upper_;
  Point a_,b_,e_,x_,actual_{};
  double stairX_,rise_,tread_,stairWidth_;
  bool feedback_=false,once_,useFeedback_,planned_=false;
  std::vector<Box> obstacles_;
  std::vector<std::vector<Point>> sections_;
  std::string reason_,out_,blocked_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,route_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr state_;
  rclcpp::Service<Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  gazebo::transport::NodePtr gz_;
  gazebo::transport::PublisherPtr visuals_,visualRequests_;
  size_t visualCount_=0,visualGeneration_=0;
  std::vector<std::string> visualNames_;
  std::unique_ptr<wheel_leg_demo::WheelLocal> wheelLocal_;
  std::unique_ptr<wheel_leg_demo::FootLocal> footLocal_;
  std::unique_ptr<wheel_leg_demo::TaskRunner> task_;
  std::vector<double> vec(const std::string &name,std::vector<double> v){return declare_parameter(name,v);}
  static double distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
  bool circleFree(Point p,const Map &m) const {
    if(p.x-radius_<m.xmin||p.x+radius_>m.xmax||p.y-radius_<m.ymin||p.y+radius_>m.ymax)return false;
    for(auto o:obstacles_) {
      if(o.z+o.h/2<=m.z+1e-6||o.z-o.h/2>=m.z+.7)continue;
      double dx=std::max(std::abs(p.x-o.x)-o.l/2,0.0),dy=std::max(std::abs(p.y-o.y)-o.w/2,0.0);
      if(std::hypot(dx,dy)<=radius_)return false;
    }return true;
  }
  bool straightFree(Point p,Point q,const Map &m,bool docking=false) const {
    int count=std::max(1,int(std::ceil(distance(p,q)/(resolution_/4))));
    for(int i=0;i<=count;i++) {
      double u=double(i)/count;Point c{p.x+(q.x-p.x)*u,p.y+(q.y-p.y)*u,m.z};
      if(!docking){if(!circleFree(c,m))return false;continue;}
      // Final E approach is +x with yaw=0. Use the actual padded rectangle.
      double hx=length_/2+margin_,hy=width_/2+margin_;
      if(c.x-hx<m.xmin||c.x+hx>m.xmax||c.y-hy<m.ymin||c.y+hy>m.ymax)return false;
      for(auto o:obstacles_)if(o.z+o.h/2>m.z+1e-6 && std::abs(c.x-o.x)<=hx+o.l/2 && std::abs(c.y-o.y)<=hy+o.w/2)return false;
    }return true;
  }
  std::vector<Point> search(Point start,Point goal,const Map &m) {
    if(!circleFree(start,m)||!circleFree(goal,m)){reason_="INVALID_ENDPOINT_OR_CLEARANCE";return {};}
    int nx=int(std::floor((m.xmax-m.xmin)/resolution_))+1,ny=int(std::floor((m.ymax-m.ymin)/resolution_))+1;
    auto point=[&](int id){return Point{m.xmin+(id%nx)*resolution_,m.ymin+(id/nx)*resolution_,m.z};};
    auto index=[&](Point p){return int(std::lround((p.y-m.ymin)/resolution_))*nx+int(std::lround((p.x-m.xmin)/resolution_));};
    int s=index(start),g=index(goal);
    if(!straightFree(start,point(s),m)||!straightFree(point(g),goal,m)){reason_="ENDPOINT_GRID_CONNECTOR_BLOCKED";return {};}
    std::vector<double> costs(nx*ny,1e30);std::vector<int> parent(nx*ny,-1);
    using Item=std::pair<double,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
    costs[s]=0;open.push({distance(point(s),goal),s});
    std::vector<bool> closed(nx*ny,false);
    while(!open.empty()) {
      int id=open.top().second;open.pop();if(closed[id])continue;closed[id]=true;if(id==g)break;
      for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
        if(!dx&&!dy)continue;
        int xx=id%nx+dx,yy=id/nx+dy;if(xx<0||xx>=nx||yy<0||yy>=ny)continue;
        int next=yy*nx+xx;if(closed[next]||!straightFree(point(id),point(next),m))continue;
        double cost=costs[id]+resolution_*std::hypot(dx,dy);
        if(cost<costs[next]){costs[next]=cost;parent[next]=id;open.push({cost+distance(point(next),point(g)),next});}
      }
    }
    if(!closed[g]){reason_="NO_PATH";return {};}
    std::vector<Point> raw{goal};for(int id=g;id!=-1;id=parent[id])raw.push_back(point(id));raw.push_back(start);std::reverse(raw.begin(),raw.end());
    // Collision-checked line-of-sight simplification, not trajectory smoothing.
    std::vector<Point> result{raw.front()};size_t i=0;
    while(i+1<raw.size()) {size_t j=raw.size()-1;while(j>i+1&&!straightFree(raw[i],raw[j],m))--j;result.push_back(raw[j]);i=j;}
    return result;
  }
  void headings(std::vector<Point>& p) {for(size_t i=0;i+1<p.size();i++)p[i].yaw=std::atan2(p[i+1].y-p[i].y,p[i+1].x-p[i].x);}
  void draw() {
    visualization_msgs::msg::MarkerArray all;
    visualization_msgs::msg::Marker clear;clear.action=clear.DELETEALL;all.markers.push_back(clear);
    for(size_t s=0;s<sections_.size();s++) {
      visualization_msgs::msg::Marker line;line.header.frame_id="world";line.header.stamp=now();line.ns="global_reference";line.id=s;line.type=line.LINE_STRIP;line.action=line.ADD;line.pose.orientation.w=1;line.scale.x=.04;
      line.color.r=s==1?1:.05;line.color.g=s==1?.5:.35;line.color.b=s==1?.05:1;line.color.a=1;
      for(auto p:sections_[s]){geometry_msgs::msg::Point q;q.x=p.x;q.y=p.y;q.z=p.z+.035;line.points.push_back(q);}all.markers.push_back(line);
    }markers_->publish(all);
    if(!visuals_||!visuals_->HasConnections())return;
    for(const auto& name:visualNames_){gazebo::msgs::Request msg;msg.set_id(0);msg.set_request("entity_delete");msg.set_data(name);visualRequests_->Publish(msg);}
    visualCount_=0;visualNames_.clear();++visualGeneration_;
    for(size_t s=0;s<sections_.size();s++)for(size_t i=1;i<sections_[s].size();i++) {
      auto a=sections_[s][i-1],b=sections_[s][i];double d=distance(a,b),dz=b.z-a.z;if(d<1e-6)continue;
      std::string name="global_reference_"+std::to_string(visualGeneration_)+"_"+std::to_string(visualCount_++);visualNames_.push_back(name);
      gazebo::msgs::Visual v;v.set_name(name);v.set_parent_name("two_steps");v.set_is_static(true);
      gazebo::msgs::Set(v.mutable_pose(),ignition::math::Pose3d((a.x+b.x)/2,(a.y+b.y)/2,(a.z+b.z)/2+.04,0,-std::atan2(dz,d),std::atan2(b.y-a.y,b.x-a.x)));
      v.mutable_geometry()->set_type(gazebo::msgs::Geometry::BOX);gazebo::msgs::Set(v.mutable_geometry()->mutable_box()->mutable_size(),ignition::math::Vector3d(std::hypot(d,dz),.045,.015));
      ignition::math::Color col=s==1?ignition::math::Color(1,.5,.05,1):ignition::math::Color(.05,.35,1,1);
      gazebo::msgs::Set(v.mutable_material()->mutable_ambient(),col);gazebo::msgs::Set(v.mutable_material()->mutable_diffuse(),col);visuals_->Publish(v);
    }
  }
  void save(double ms,bool ok) {
    std::filesystem::create_directories(out_);
    std::ofstream f(out_+"/路径.csv");f<<"section,mode,index,x_m,y_m,ground_z_m,yaw_rad\n";
    double total=0;size_t offset=0;std::ostringstream json;json<<"{\"frame\":\"world\",\"success\":"<<(ok?"true":"false")<<",\"sections\":[";
    for(size_t s=0;s<sections_.size();s++) {
      double len=0;for(size_t i=0;i<sections_[s].size();i++){auto p=sections_[s][i];f<<s<<','<<(s==1?"leg":"wheel")<<','<<i<<','<<p.x<<','<<p.y<<','<<p.z<<','<<p.yaw<<'\n';if(i)len+=distance(sections_[s][i-1],p);}
      total+=len;if(s)json<<',';json<<"{\"from\":\""<<(s==0?"A":s==1?"E":"X")<<"\",\"to\":\""<<(s==0?"E":s==1?"X":"B")<<"\",\"mode\":\""<<(s==1?"leg":"wheel")<<"\",\"first_index\":"<<offset<<",\"point_count\":"<<sections_[s].size()<<",\"horizontal_length_m\":"<<len<<"}";offset+=sections_[s].size();
    }
    json<<"],\"reason\":\""<<reason_<<"\"}";std_msgs::msg::String route;route.data=json.str();route_->publish(route);std::ofstream(out_+"/路段.json")<<json.str()<<'\n';
    std::ofstream summary(out_+"/结果.csv");summary<<"success,reason,horizontal_length_m,planning_wall_ms,resolution_m,margin_m,footprint_length_m,footprint_width_m,blocked_platform\n"<<ok<<','<<reason_<<','<<total<<','<<ms<<','<<resolution_<<','<<margin_<<','<<length_<<','<<width_<<','<<blocked_<<'\n';
    for(auto name:{"路径.csv","结果.csv","路段.json"})chmod((out_+"/"+name).c_str(),0664);
  }
public:
  Planner():Node("global_planner") {
    auto lo=vec("lower_bounds",{-6,2,-3,3}),up=vec("upper_bounds",{2.7,10.7,-3,3});lower_={lo[0],lo[1],lo[2],lo[3],0};upper_={up[0],up[1],up[2],up[3],.34};
    auto a=vec("start",{-4.5,-.9,0,0}),b=vec("goal",{9,-.9,.34,0}),e=vec("entry",{1.6,-.9,0,0}),x=vec("exit",{3.3,-.9,.34,0});a_={a[0],a[1],a[2],a[3]};b_={b[0],b[1],b[2],b[3]};e_={e[0],e[1],e[2],e[3]};x_={x[0],x[1],x[2],x[3]};
    resolution_=declare_parameter("resolution",.05);margin_=declare_parameter("safety_margin",.05);length_=declare_parameter("footprint_length",.6);width_=declare_parameter("footprint_width",.78);radius_=std::hypot(length_,width_)/2+margin_;
    stairX_=declare_parameter("stair_x",2.0);rise_=declare_parameter("rise",.17);tread_=declare_parameter("tread",.35);stairWidth_=declare_parameter("stair_width",4.0);upper_.z=2*rise_;
    auto obs=vec("obstacles",{-1.5,-.9,.3,.6,.8,.6,6.2,-.9,.64,.6,.8,.6});for(size_t i=0;i+5<obs.size();i+=6)obstacles_.push_back({obs[i],obs[i+1],obs[i+2],obs[i+3],obs[i+4],obs[i+5]});
    blocked_=declare_parameter("blocked_platform",std::string("none"));if(blocked_=="lower")obstacles_.push_back({-1.5,0,.3,.6,6,.6});if(blocked_=="upper")obstacles_.push_back({6.2,0,.64,.6,6,.6});
    out_=declare_parameter("output_dir",wheel_leg_demo::defaultOutputDir("global/baseline"));once_=declare_parameter("once",false);useFeedback_=declare_parameter("use_feedback_start",true);
    if(resolution_<=0||margin_<0||length_<=0||width_<=0)throw std::runtime_error("Invalid grid/footprint parameters");
    auto qos=rclcpp::QoS(1).transient_local();path_=create_publisher<nav_msgs::msg::Path>("/wheel_leg_demo/global_path",qos);markers_=create_publisher<visualization_msgs::msg::MarkerArray>("/wheel_leg_demo/global_markers",qos);route_=create_publisher<std_msgs::msg::String>("/wheel_leg_demo/global_route",qos);status_=create_publisher<std_msgs::msg::String>("/wheel_leg_demo/planning_status",qos);
    state_=create_subscription<gazebo_msgs::msg::ModelStates>("/wheel_leg_demo/model_states",10,[this](gazebo_msgs::msg::ModelStates::SharedPtr m){for(size_t i=0;i<m->name.size();i++)if(m->name[i]=="wheel_leg_robot"){auto p=m->pose[i];actual_={p.position.x,p.position.y,p.position.z>.55?upper_.z:lower_.z,2*std::atan2(p.orientation.z,p.orientation.w)};feedback_=true;}});
    service_=create_service<Trigger>("/wheel_leg_demo/plan_global",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> res){res->success=plan();res->message=reason_;});
    const bool taskMode=declare_parameter("task_enabled",false);
    if(declare_parameter("wheel_local_enabled",false)||taskMode)wheelLocal_=std::make_unique<wheel_leg_demo::WheelLocal>(this);
    if(declare_parameter("foot_local_enabled",false)||taskMode)footLocal_=std::make_unique<wheel_leg_demo::FootLocal>(this);
    if(taskMode)task_=std::make_unique<wheel_leg_demo::TaskRunner>(this,wheelLocal_.get(),footLocal_.get(),[this](){return plan();},[this](){return reason_;});
    if(!once_&&!taskMode){timer_=create_wall_timer(std::chrono::seconds(2),[this](){if(!planned_){plan();planned_=true;}else if(!visualCount_&&!sections_.empty())draw();});}
  }
  void connectGazebo() {
    gz_.reset(new gazebo::transport::Node);
    gz_->Init("two_steps");
    visuals_=gz_->Advertise<gazebo::msgs::Visual>("~/visual");
    visualRequests_=gz_->Advertise<gazebo::msgs::Request>("~/request");
    if(footLocal_)footLocal_->connectGazebo();
  }
  bool once()const{return once_;}
  bool plan() {
    auto begin=std::chrono::steady_clock::now();sections_.clear();reason_="OK";
    useFeedback_=get_parameter("use_feedback_start").as_bool();
    out_=get_parameter("output_dir").as_string();
    Point a=a_;
    if(useFeedback_&&feedback_)a=actual_;
    // Refresh task endpoints for service calls after ros2 param set.
    auto av=get_parameter("start").as_double_array(),bv=get_parameter("goal").as_double_array();
    bool validInput=av.size()==4&&bv.size()==4&&std::all_of(av.begin(),av.end(),[](double v){return std::isfinite(v);})&&std::all_of(bv.begin(),bv.end(),[](double v){return std::isfinite(v);});
    if(validInput){
      if(!useFeedback_||!feedback_)a={av[0],av[1],av[2],av[3]};
      b_={bv[0],bv[1],bv[2],bv[3]};
    }
    bool ok=false;
    if(!validInput)reason_="INVALID_ENDPOINT_FORMAT";
    else if(std::abs(a.z-lower_.z)>.001||std::abs(b_.z-upper_.z)>.001)reason_="WRONG_PLATFORM_HEIGHT";
    else if(std::abs(e_.y-x_.y)>1e-6||std::abs(e_.yaw)>1e-6||std::abs(x_.yaw)>1e-6||std::abs(e_.y)+width_/2+margin_>stairWidth_/2)reason_="INVALID_STAIR_PORTAL";
    else {
      Point approach{e_.x-.6,e_.y,lower_.z};auto lower=search(a,approach,lower_);
      if(!lower.empty()&&straightFree(approach,e_,lower_,true)) {
        lower.push_back(e_);auto upper=search(x_,b_,upper_);
        if(!upper.empty()) {
          std::vector<Point> stairs{e_,{stairX_,e_.y,0},{stairX_,e_.y,rise_},{stairX_+tread_,e_.y,rise_},{stairX_+tread_,e_.y,2*rise_},x_};
          headings(lower);headings(upper);upper.back().yaw=b_.yaw;sections_={lower,stairs,upper};ok=true;
        }
      }else if(!lower.empty())reason_="ENTRY_DOCKING_BLOCKED";
    }
    nav_msgs::msg::Path p;p.header.frame_id="world";p.header.stamp=now();
    for(auto sec:sections_)for(auto v:sec){geometry_msgs::msg::PoseStamped pose;pose.header=p.header;pose.pose.position.x=v.x;pose.pose.position.y=v.y;pose.pose.position.z=v.z;pose.pose.orientation.z=std::sin(v.yaw/2);pose.pose.orientation.w=std::cos(v.yaw/2);p.poses.push_back(pose);}
    path_->publish(p);
    if(wheelLocal_)wheelLocal_->setReference(p,sections_.empty()?0:sections_[0].size(),sections_.empty()?0:sections_[1].size());
    if(footLocal_)footLocal_->setReference(p,sections_.empty()?0:sections_[0].size(),sections_.empty()?0:sections_[1].size());
    draw();double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();save(ms,ok);
    std_msgs::msg::String status;status.data=(ok?"PLANNED: ":"FAILED: ")+reason_+" | reference path only; simulation scene";status_->publish(status);RCLCPP_INFO(get_logger(),"%s",status.data.c_str());return ok;
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<Planner>();
  const bool live=!node->once();
  int result=0;
  if(live) {
    gazebo::client::setup(argc,argv);
    node->connectGazebo();
    rclcpp::spin(node);
  } else {
    result=node->plan()?0:2;
  }
  rclcpp::shutdown();
  node.reset();
  if(live)gazebo::client::shutdown();
  return result;
}
