#include <path_searching/astar.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace cane_planner {
namespace {
using V = Eigen::Vector2d;
using Poly = std::vector<V>;

class GlobalGeometry {
public:
    Poly local;
    std::vector<Poly> obstacles;
    Eigen::Vector2d origin;
    double resolution=0.,margin=0.;
    Eigen::Vector2i size;
    std::vector<uint8_t> blocked;
    std::vector<int> occupancy_prefix;
    std::chrono::steady_clock::time_point deadline;
    struct Support {V axis;double lo,hi;};
    struct Shape {Poly vertices;std::vector<Support> axes;std::vector<std::vector<Support>> obstacle_axes;};
    std::map<double,Shape> shapes;
    using SweepKey=Eigen::Matrix<double,5,1>;
    std::unordered_map<SweepKey,bool,matrix_hash<SweepKey>> certificates;
    bool expired() const {return std::chrono::steady_clock::now()>=deadline;}
    const Shape& shape(double yaw) {
        auto found=shapes.find(yaw);if(found!=shapes.end())return found->second;
        Shape prepared;const Eigen::Rotation2Dd rotate(yaw);
        for(const auto& v:local)prepared.vertices.push_back(rotate*v);
        auto add=[&](V axis) {
            const double n=axis.norm();if(n<1e-10)return;axis/=n;
            Support support{axis,std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
            for(const auto& v:prepared.vertices){const double d=axis.dot(v);support.lo=std::min(support.lo,d);support.hi=std::max(support.hi,d);}
            prepared.axes.push_back(support);
        };
        add(V(1,0));add(V(0,1));
        for(size_t i=0;i<prepared.vertices.size();++i) {
            const V e=prepared.vertices[(i+1)%prepared.vertices.size()]-prepared.vertices[i];add(V(-e.y(),e.x()));
        }
        for(const auto& obstacle:obstacles) {
            prepared.obstacle_axes.emplace_back();
            auto add_obstacle_axis=[&](V axis) {
                const double n=axis.norm();if(n<1e-10)return;axis/=n;
                double rlo=std::numeric_limits<double>::infinity(),rhi=-rlo,olo=rlo,ohi=rhi;
                for(const auto& v:prepared.vertices){const double x=axis.dot(v);rlo=std::min(rlo,x);rhi=std::max(rhi,x);}
                for(const auto& v:obstacle){const double x=axis.dot(v);olo=std::min(olo,x);ohi=std::max(ohi,x);}
                prepared.obstacle_axes.back().push_back({axis,olo-rhi,ohi-rlo});
            };
            for(const auto& s:prepared.axes)add_obstacle_axis(s.axis);
            for(size_t i=0;i<obstacle.size();++i){const V e=obstacle[(i+1)%obstacle.size()]-obstacle[i];add_obstacle_axis(V(-e.y(),e.x()));}
        }
        return shapes.emplace(yaw,std::move(prepared)).first->second;
    }
    bool obstacleFree(const Shape& shape,const V& a,const V& b) const {
        for(size_t i=0;i<obstacles.size();++i)
            if(!obstacleFree(shape,shape.obstacle_axes[i],obstacles[i],a,b))return false;
        return true;
    }
    bool obstacleFree(const Shape& shape,const std::vector<Support>& axes,const Poly& obstacle,const V& a,const V& b) const {
        for(const auto& s:axes) {
            const double da=s.axis.dot(a),db=s.axis.dot(b);
            if(s.lo-std::max(da,db)>margin+1e-9 || std::min(da,db)-s.hi>margin+1e-9)return true;
        }
        const V d=b-a;V axis(-d.y(),d.x());const double n=axis.norm();
        if(n<1e-10)return false;
        axis/=n;
        double rlo=std::numeric_limits<double>::infinity(),rhi=-rlo,olo=rlo,ohi=rhi;
        for(const auto& v:shape.vertices){const double x=axis.dot(v);rlo=std::min(rlo,x);rhi=std::max(rhi,x);}
        for(const auto& v:obstacle){const double x=axis.dot(v);olo=std::min(olo,x);ohi=std::max(ohi,x);}
        const double da=axis.dot(a),db=axis.dot(b);
        return olo-(rhi+std::max(da,db))>margin+1e-9 || rlo+std::min(da,db)-ohi>margin+1e-9;
    }
    bool cellFree(const Shape& shape,const V& a,const V& b,const V& center) const {
        auto separated=[&](const Support& s) {
            const double r=.5*resolution*s.axis.cwiseAbs().sum();
            const double c=s.axis.dot(center),da=s.axis.dot(a),db=s.axis.dot(b);
            return c-r-(s.hi+std::max(da,db))>margin+1e-9 ||
                s.lo+std::min(da,db)-(c+r)>margin+1e-9;
        };
        for(const auto& axis:shape.axes)if(separated(axis))return true;
        const V d=b-a;V axis(-d.y(),d.x());const double n=axis.norm();
        if(n<1e-10)return false;
        axis/=n;
        Support extra{axis,std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
        for(const auto& v:shape.vertices){const double x=axis.dot(v);extra.lo=std::min(extra.lo,x);extra.hi=std::max(extra.hi,x);}
        return separated(extra);
    }
    bool edge(const V& a,const V& b,double yaw) {
        if(expired())return false;
        SweepKey key;key<<a.x(),a.y(),b.x(),b.y(),yaw;
        const auto found=certificates.find(key);
        if(found!=certificates.end())return found->second;
        const bool free=checkEdge(a,b,yaw);
        certificates.emplace(key,free);return free;
    }
    bool checkEdge(const V& a,const V& b,double yaw) {
        const auto& prepared=shape(yaw);const auto& p=prepared.vertices;
        if(!obstacleFree(prepared,a,b))return false;
        V lo=a.cwiseMin(b),hi=a.cwiseMax(b),slo=p.front(),shi=slo;
        for(const auto& v:p){slo=slo.cwiseMin(v);shi=shi.cwiseMax(v);}
        lo+=slo-V::Constant(margin);hi+=shi+V::Constant(margin);
        const Eigen::Vector2i first=((lo-origin)/resolution).array().floor().cast<int>();
        const Eigen::Vector2i last=((hi-origin)/resolution).array().floor().cast<int>();
        if((first.array()<0).any() || (last.array()>=size.array()).any())return false;
        const int stride=size.x()+1;
        auto count=[&](int x,int y){return occupancy_prefix[y*stride+x];};
        if(count(last.x()+1,last.y()+1)-count(first.x(),last.y()+1)-
           count(last.x()+1,first.y())+count(first.x(),first.y())==0)return true;
        for(int y=first.y();y<=last.y();++y)for(int x=first.x();x<=last.x();++x) {
            if(expired())return false;
            if(!blocked[y*size.x()+x])continue;
            const V cell=origin+resolution*V(x,y);
            if(!cellFree(prepared,a,b,cell+V::Constant(.5*resolution)))return false;
        }
        return true;
    }

};
}

bool Astar::snapshotGlobalReference(GlobalReferenceQuery& q) const {
    if(!collision_)return false;
    q.lower=origin_;q.upper=map_max_2d_;q.resolution=resolution_;
    q.clearance=corridor_edge_clearance_;q.weight=w_clearance_;q.sigma=clearance_sigma_;
    q.allocate_num=allocate_num_;
    Eigen::Vector2i size;std::string reason;const auto began=std::chrono::steady_clock::now();
    if(!collision_->getStaticCorridorGrid(q.lower,q.upper,q.grid.origin,q.grid.resolution,
        size,q.grid.blocked,reason))return false;
    q.grid.width=size.x();q.grid.height=size.y();
    q.grid_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    return true;
}

Astar::RecoveryResult Astar::searchGlobalReference(const V& start,const V& goal,const Poly& obstacle,
    std::chrono::steady_clock::time_point deadline) {
    GlobalReferenceQuery q;q.start=start;q.goal=goal;
    if(!obstacle.empty())q.obstacles.push_back(obstacle);
    if(!snapshotGlobalReference(q))return RecoveryResult();
    auto result=evaluateGlobalReference(q,deadline);result.seconds+=q.grid_seconds;return result;
}

Astar::RecoveryResult Astar::evaluateGlobalReference(const GlobalReferenceQuery& q,
    std::chrono::steady_clock::time_point deadline) {
    RecoveryResult result;const auto began=std::chrono::steady_clock::now();
    const auto& start=q.start;const auto& goal=q.goal;
    const double resolution_=q.resolution,corridor_edge_clearance_=q.clearance;
    const double w_clearance_=q.weight,clearance_sigma_=q.sigma;
    const int allocate_num_=q.allocate_num;
    const auto& origin_=q.lower;const auto& map_max_2d_=q.upper;
    auto finish=[&](RecoveryStatus status) {
        result.status=status;result.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
        if(status!=RecoveryStatus::FOUND)result.path.clear();
        return result;
    };
    if(!start.allFinite() || !goal.allFinite() ||
       q.obstacles.size()>ConvexCorridor::kMaxPedestrians ||
       std::any_of(q.obstacles.begin(),q.obstacles.end(),[](const Poly& p){return !validRecoveryPolygon(p);}) ||
       allocate_num_<2 || !std::isfinite(resolution_) || resolution_<.01 ||
       !origin_.allFinite() || !map_max_2d_.allFinite() || (map_max_2d_.array()<=origin_.array()).any() ||
       !std::isfinite(corridor_edge_clearance_) || corridor_edge_clearance_<0 ||
       !std::isfinite(w_clearance_) || w_clearance_<0 || !std::isfinite(clearance_sigma_) || clearance_sigma_<=0 ||
       !q.grid.origin.allFinite() || !std::isfinite(q.grid.resolution) || q.grid.resolution<.01 ||
       q.grid.width<=0 || q.grid.height<=0 || int64_t(q.grid.width)*q.grid.height>1000000 ||
       q.grid.blocked.size()!=size_t(q.grid.width)*q.grid.height)
        return finish(RecoveryStatus::INVALID_INPUT);
    // A spatial point reference; the current human-cane hull belongs to FIRI's
    // root constraint, and future hulls belong to timed pedestrian checks.
    GlobalGeometry geometry;geometry.local={V::Zero()};
    geometry.obstacles=q.obstacles;geometry.deadline=deadline;
    geometry.margin=corridor_edge_clearance_+resolution_; // routing room for the point reference
    // Search every selected-map bin, independent of any previous route/window.
    const V lower=origin_,upper=map_max_2d_;
    if((start.array()<lower.array()).any() || (start.array()>=upper.array()).any() ||
       (goal.array()<lower.array()).any() || (goal.array()>=upper.array()).any())return finish(RecoveryStatus::INVALID_INPUT);
    if(geometry.expired())return finish(RecoveryStatus::TIMEOUT);
    geometry.origin=q.grid.origin;geometry.resolution=q.grid.resolution;
    geometry.size=Eigen::Vector2i(q.grid.width,q.grid.height);geometry.blocked=q.grid.blocked;
    result.profile.grid_calls=1;result.profile.grid_seconds=q.grid_seconds;
    geometry.occupancy_prefix.assign((geometry.size.x()+1)*(geometry.size.y()+1),0);
    for(int y=0;y<geometry.size.y();++y) {
        if(geometry.expired())return finish(RecoveryStatus::TIMEOUT);
        int row=0;const int stride=geometry.size.x()+1;
        for(int x=0;x<geometry.size.x();++x) {
            row+=geometry.blocked[y*geometry.size.x()+x]!=0;
            geometry.occupancy_prefix[(y+1)*stride+x+1]=geometry.occupancy_prefix[y*stride+x+1]+row;
        }
    }
    CorridorDistanceField field;
    if(w_clearance_>0 && !field.build(geometry.origin,geometry.resolution,geometry.size,geometry.blocked))return finish(RecoveryStatus::NO_PATH);
    if(!geometry.edge(start,start,0.))return finish(geometry.expired()?RecoveryStatus::TIMEOUT:RecoveryStatus::NO_PATH);

    struct State {Eigen::Vector2i key;V position;double g;int parent;};
    struct Open {double f,g;int state;bool operator<(const Open& b)const{return f>b.f;}};
    std::vector<State> states;states.reserve(std::min(allocate_num_,100000));
    std::unordered_map<Eigen::Vector2i,int,matrix_hash<Eigen::Vector2i>> indices;
    std::priority_queue<Open> open;
    states.push_back({{0,0},start,0.,-1});indices.emplace(states[0].key,0);
    open.push({(goal-start).norm(),0.,0});
    auto edge_cost=[&](const V& a,const V& b) {
        ++result.profile.soft_calls;const auto start=std::chrono::steady_clock::now();
        const double length=(b-a).norm();double penalty=0.;
        if(w_clearance_>0) {
            const double step=std::min(.02,std::min(.5*geometry.resolution,clearance_sigma_/16.));
            const int count=std::max(1,int(std::ceil(length/step)));
            for(int i=0;i<count;++i) {
                if(geometry.expired())return std::numeric_limits<double>::infinity();
                const double distance=field.distance(a+(double(i)+.5)/count*(b-a));
                const double t=std::max(0.,1.-std::max(0.,distance-geometry.margin)/clearance_sigma_);
                penalty+=t*t/count;
            }
        }
        result.profile.soft_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        return length*(1.+w_clearance_*penalty);
    };
    auto transition=[&](const State& s,const V& p) {
        ++result.profile.hard_calls;const auto start=std::chrono::steady_clock::now();
        const bool free=geometry.edge(s.position,p,0.);
        result.profile.hard_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();return free;
    };
    int reached=-1;
    while(!open.empty() && !geometry.expired()) {
        const auto top=open.top();open.pop();const State current=states[top.state];
        if(top.g!=current.g)continue;
        ++result.profile.expanded;
        const V connector=goal-current.position;
        if(connector.norm()<=1.5*resolution_) {
            if(transition(current,goal)) {reached=top.state;break;}
        }
        for(int direction=0;direction<8;++direction) {
            static const std::array<Eigen::Vector2i,8> offsets{{{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}}};
            const Eigen::Vector2i cell=current.key+offsets[direction];
            const V p=start+resolution_*cell.cast<double>();
            if((p.array()<lower.array()).any() || (p.array()>upper.array()).any())continue;
            const Eigen::Vector2i key=cell;auto known=indices.find(key);
            const double lower_cost=current.g+(p-current.position).norm();
            if(known!=indices.end() && lower_cost>=states[known->second].g)continue;
            if(!transition(current,p))continue;
            const double cost=current.g+edge_cost(current.position,p);
            if(!std::isfinite(cost) || (known!=indices.end() && cost>=states[known->second].g))continue;
            int index;
            if(known==indices.end()) {
                if(states.size()>=size_t(std::max(0,allocate_num_)))return finish(RecoveryStatus::NO_PATH);
                index=states.size();indices.emplace(key,index);states.push_back({key,p,cost,top.state});
            } else {index=known->second;states[index].g=cost;states[index].parent=top.state;}
            open.push({cost+(goal-p).norm(),cost,index});
        }
    }
    if(geometry.expired())return finish(RecoveryStatus::TIMEOUT);
    if(reached<0)return finish(RecoveryStatus::NO_PATH);
    Poly detour;
    for(int i=reached;i>=0;i=states[i].parent)detour.push_back(states[i].position);
    std::reverse(detour.begin(),detour.end());
    if((detour.back()-goal).norm()>1e-9)detour.push_back(goal);
    else detour.back()=goal;
    result.path=std::move(detour);
    return finish(RecoveryStatus::FOUND);
}
}
