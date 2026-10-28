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
constexpr double pi = 3.14159265358979323846;

// Support intervals of footprint + translation segment. A separating axis is a
// certificate for the entire straight-edge sweep, not just endpoint samples.
bool separatedSweep(const Poly& shape, const V& a, const V& b,
                    const Poly& obstacle, double margin) {
    auto separated = [&](V axis) {
        const double length=axis.norm();if(length<1e-10)return false;axis/=length;
        double fmin=std::numeric_limits<double>::infinity(),fmax=-fmin;
        double omin=fmin,omax=-fmin;
        for(const auto& p:shape){const double d=axis.dot(p);fmin=std::min(fmin,d);fmax=std::max(fmax,d);}
        for(const auto& p:obstacle){const double d=axis.dot(p);omin=std::min(omin,d);omax=std::max(omax,d);}
        const double da=axis.dot(a),db=axis.dot(b);
        return omin-(fmax+std::max(da,db))>margin+1e-9 ||
               (fmin+std::min(da,db))-omax>margin+1e-9;
    };
    for(const auto* polygon:{&shape,&obstacle})for(size_t i=0;i<polygon->size();++i) {
        const V e=(*polygon)[(i+1)%polygon->size()]-(*polygon)[i];
        if(separated(V(-e.y(),e.x())))return true;
    }
    const V d=b-a;return separated(V(-d.y(),d.x()));
}

class LocalGeometry {
public:
    Poly local,obstacle;
    Eigen::Vector2d origin;
    double resolution=0.,margin=0.;
    Eigen::Vector2i size;
    std::vector<uint8_t> blocked;
    std::chrono::steady_clock::time_point deadline;
    std::map<double,Poly> shapes;
    using SweepKey=Eigen::Matrix<double,5,1>;
    std::unordered_map<SweepKey,bool,matrix_hash<SweepKey>> certificates;
    bool expired() const {return std::chrono::steady_clock::now()>=deadline;}
    const Poly& shape(double yaw) {
        auto found=shapes.find(yaw);if(found!=shapes.end())return found->second;
        Poly p;const Eigen::Rotation2Dd rotate(yaw);
        for(const auto& v:local)p.push_back(rotate*v);
        return shapes.emplace(yaw,std::move(p)).first->second;
    }
    bool edge(const V& a,const V& b,double yaw,bool check_static=true) {
        if(expired())return false;
        SweepKey key;key<<a.x(),a.y(),b.x(),b.y(),yaw;
        if(check_static) {
            const auto found=certificates.find(key);
            if(found!=certificates.end())return found->second;
            const bool free=checkEdge(a,b,yaw,true);
            certificates.emplace(key,free);return free;
        }
        return checkEdge(a,b,yaw,false);
    }
    bool checkEdge(const V& a,const V& b,double yaw,bool check_static) {
        const auto& p=shape(yaw);
        if(!separatedSweep(p,a,b,obstacle,margin))return false;
        if(!check_static)return true;
        V lo=a.cwiseMin(b),hi=a.cwiseMax(b),slo=p.front(),shi=slo;
        for(const auto& v:p){slo=slo.cwiseMin(v);shi=shi.cwiseMax(v);}
        lo+=slo-V::Constant(margin);hi+=shi+V::Constant(margin);
        const Eigen::Vector2i first=((lo-origin)/resolution).array().floor().cast<int>();
        const Eigen::Vector2i last=((hi-origin)/resolution).array().floor().cast<int>();
        if((first.array()<0).any() || (last.array()>=size.array()).any())return false;
        for(int y=first.y();y<=last.y();++y)for(int x=first.x();x<=last.x();++x) {
            if(expired())return false;
            if(!blocked[y*size.x()+x])continue;
            const V cell=origin+resolution*V(x,y);
            const Poly square{cell,cell+V(resolution,0),cell+V(resolution,resolution),cell+V(0,resolution)};
            if(!separatedSweep(p,a,b,square,margin))return false;
        }
        return true;
    }
    bool turn(const V& p,double from,double to,bool check_static=true) {
        const double angle=std::atan2(std::sin(to-from),std::cos(to-from));
        if(!edge(p,p,from,check_static) || !edge(p,p,to,check_static))return false;
        const double step=pi/36.;
        const double lo=std::min(from,from+angle),hi=std::max(from,from+angle);
        for(int i=int(std::ceil(lo/step));i*step<hi;++i)
            if(!edge(p,p,i*step,check_static))return false;
        return true;
    }
};
}

Astar::RecoveryResult Astar::repairLocalReference(const Poly& reference,const Poly& obstacle,
    const HumanCaneFootprint::Config& footprint,double root_yaw,
    std::chrono::steady_clock::time_point deadline) {
    RecoveryResult result;const auto began=std::chrono::steady_clock::now();
    auto finish=[&](RecoveryStatus status) {
        result.status=status;result.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
        if(status!=RecoveryStatus::FOUND)result.path.clear();
        return result;
    };
    if(!collision_ || reference.size()<2 || reference.size()>10000 || obstacle.empty() ||
       !validRecoveryPolygon(obstacle) || !HumanCaneFootprint::validConfig(footprint) ||
       !std::isfinite(root_yaw) || allocate_num_<2 || !std::isfinite(resolution_) || resolution_<.01 ||
       !std::isfinite(corridor_edge_clearance_) || corridor_edge_clearance_<0 ||
       !std::isfinite(w_clearance_) || w_clearance_<0 || !std::isfinite(clearance_sigma_) || clearance_sigma_<=0)
        return finish(RecoveryStatus::INVALID_INPUT);
    root_yaw=std::atan2(std::sin(root_yaw),std::cos(root_yaw));
    LocalGeometry geometry;geometry.local=HumanCaneFootprint::relative(footprint,0.);
    geometry.obstacle=obstacle;geometry.deadline=deadline;
    geometry.margin=corridor_edge_clearance_+resolution_; // one search bin of routing room
    std::vector<double> arc{0.},yaw;
    for(size_t i=0;i<reference.size();++i) {
        if(!reference[i].allFinite() || reference[i].cwiseAbs().maxCoeff()>1e4)return finish(RecoveryStatus::INVALID_INPUT);
        if(i){const V d=reference[i]-reference[i-1];if(d.norm()<1e-9)return finish(RecoveryStatus::INVALID_INPUT);
            arc.push_back(arc.back()+d.norm());yaw.push_back(std::atan2(d.y(),d.x()));}
    }
    size_t first=reference.size(),last=0;
    for(size_t i=0;i+1<reference.size();++i) {
        if(geometry.expired())return finish(RecoveryStatus::TIMEOUT);
        if(!geometry.edge(reference[i],reference[i+1],yaw[i],false) ||
           !geometry.turn(reference[i],i?yaw[i-1]:root_yaw,yaw[i],false)) {
            first=std::min(first,i);last=i;
        }
    }
    if(first==reference.size())return finish(RecoveryStatus::NO_PATH); // no reference obstruction to repair
    double radius=0.;for(const auto& v:geometry.local)radius=std::max(radius,v.norm());
    const double flank=std::max(1.,radius+resolution_);
    size_t entry=first,exit=last+1;
    while(entry>0 && arc[first]-arc[entry]<flank)--entry;
    while(exit+1<reference.size() && arc[exit]-arc[last+1]<flank)++exit;
    result.repair_begin=entry;result.repair_end=exit;
    const double entry_yaw=entry?yaw[entry-1]:root_yaw;
    const double exit_yaw=exit<yaw.size()?yaw[exit]:yaw.back();
    V lower=reference[entry],upper=lower;
    for(size_t i=entry;i<=exit;++i){lower=lower.cwiseMin(reference[i]);upper=upper.cwiseMax(reference[i]);}
    // Bound search to the affected route plus nearby free space. The grid halo
    // covers every allowed footprint and the truncated comfort-distance range.
    const double padding=std::max(1.5,2.*radius);
    lower-=V::Constant(padding);upper+=V::Constant(padding);
    const double halo=radius+geometry.margin+clearance_sigma_+resolution_;
    std::string reason;
    const auto grid_start=std::chrono::steady_clock::now();
    if(geometry.expired())return finish(RecoveryStatus::TIMEOUT);
    ++result.profile.grid_calls;
    if(!collision_->getStaticCorridorGrid(lower-V::Constant(halo),upper+V::Constant(halo),
            geometry.origin,geometry.resolution,geometry.size,geometry.blocked,reason))return finish(RecoveryStatus::NO_PATH);
    result.profile.grid_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-grid_start).count();
    CorridorDistanceField field;
    if(w_clearance_>0 && !field.build(geometry.origin,geometry.resolution,geometry.size,geometry.blocked))return finish(RecoveryStatus::NO_PATH);
    if(!geometry.edge(reference[entry],reference[entry],entry_yaw) ||
       !geometry.edge(reference[exit],reference[exit],exit_yaw))return finish(geometry.expired()?RecoveryStatus::TIMEOUT:RecoveryStatus::NO_PATH);

    struct State {Eigen::Vector3i key;V position;double yaw,g;int parent;};
    struct Open {double f,g;int state;bool operator<(const Open& b)const{return f>b.f;}};
    std::vector<State> states;states.reserve(std::min(allocate_num_,100000));
    std::unordered_map<Eigen::Vector3i,int,matrix_hash<Eigen::Vector3i>> indices;
    std::priority_queue<Open> open;
    states.push_back({{0,0,8},reference[entry],entry_yaw,0.,-1});indices.emplace(states[0].key,0);
    open.push({(reference[exit]-reference[entry]).norm(),0.,0});
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
    auto transition=[&](const State& s,const V& p,double heading) {
        ++result.profile.hard_calls;const auto start=std::chrono::steady_clock::now();
        const bool free=geometry.turn(s.position,s.yaw,heading) && geometry.edge(s.position,p,heading);
        result.profile.hard_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();return free;
    };
    int reached=-1;
    while(!open.empty() && !geometry.expired()) {
        const auto top=open.top();open.pop();const State current=states[top.state];
        if(top.g!=current.g)continue;
        ++result.profile.expanded;
        const V connector=reference[exit]-current.position;
        if(connector.norm()<=1.5*resolution_) {
            const double heading=connector.norm()>1e-9?std::atan2(connector.y(),connector.x()):current.yaw;
            if(transition(current,reference[exit],heading) && geometry.turn(reference[exit],heading,exit_yaw)) {reached=top.state;break;}
        }
        for(int direction=0;direction<8;++direction) {
            static const std::array<Eigen::Vector2i,8> offsets{{{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}}};
            const Eigen::Vector2i cell=current.key.head<2>()+offsets[direction];
            const V p=reference[entry]+resolution_*cell.cast<double>();const double heading=direction*pi/4.;
            if((p.array()<lower.array()).any() || (p.array()>upper.array()).any())continue;
            const Eigen::Vector3i key(cell.x(),cell.y(),direction);auto known=indices.find(key);
            const double lower_cost=current.g+(p-current.position).norm();
            if(known!=indices.end() && lower_cost>=states[known->second].g)continue;
            if(!transition(current,p,heading))continue;
            const double cost=current.g+edge_cost(current.position,p);
            if(!std::isfinite(cost) || (known!=indices.end() && cost>=states[known->second].g))continue;
            int index;
            if(known==indices.end()) {
                if(states.size()>=size_t(std::max(0,allocate_num_)))return finish(RecoveryStatus::NO_PATH);
                index=states.size();indices.emplace(key,index);states.push_back({key,p,heading,cost,top.state});
            } else {index=known->second;states[index].g=cost;states[index].parent=top.state;}
            open.push({cost+(reference[exit]-p).norm(),cost,index});
        }
    }
    if(geometry.expired())return finish(RecoveryStatus::TIMEOUT);
    if(reached<0)return finish(RecoveryStatus::NO_PATH);
    Poly detour;
    for(int i=reached;i>=0;i=states[i].parent)detour.push_back(states[i].position);
    std::reverse(detour.begin(),detour.end());
    if((detour.back()-reference[exit]).norm()>1e-9)detour.push_back(reference[exit]);
    else detour.back()=reference[exit];
    result.path.assign(reference.begin(),reference.begin()+entry);
    result.path.insert(result.path.end(),detour.begin(),detour.end());
    result.path.insert(result.path.end(),reference.begin()+exit+1,reference.end());
    return finish(RecoveryStatus::FOUND);
}
}
