// Attributed 2D FIRI adaptation of GCOPTER firi.hpp, Zhepei Wang (2021).
// Pinned upstream e0444f6d47b84f972ced91746b05feb36ce1fd4f; see FIRI_NOTICE.
// Whole-cell finite angular restrictive inflation + constrained 2D MVIE using
// installed NLopt SLSQP. Not unchanged upstream point-obstacle/3D L-BFGS code.
#include <path_searching/convex_corridor.h>
#include <path_searching/pedestrian_polygon.h>
#include <nlopt.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
namespace cane_planner {
constexpr size_t ConvexCorridor::kMaxPedestrians;
namespace {
using V = Eigen::Vector2d;
using M = Eigen::Matrix2d;
using C = ConvexCorridor;
using H = C::Halfspace;
using Poly = std::vector<V>;
using Cell = std::array<V, 4>;
using Clock = std::chrono::steady_clock;
struct Failure { C::FailureReason reason; };
void require(bool ok, C::FailureReason reason) { if (!ok) throw Failure{reason}; }
void deadline(Clock::time_point end) { require(Clock::now() < end, C::FailureReason::BUDGET); }
double slack(const H& h, const V& p) { return h.offset-h.normal.dot(p); }
Poly vertices(const std::vector<H>& hs, double inset = 0.) {
    Poly v;
    for (size_t i=0; i<hs.size(); ++i) for (size_t j=0; j<i; ++j) {
        M a; a.row(0)=hs[i].normal; a.row(1)=hs[j].normal;
        if (std::abs(a.determinant()) <= 1e-10) continue;
        const V p = a.inverse()*V(hs[i].offset-inset, hs[j].offset-inset);
        bool good = p.allFinite();
        for (const auto& h: hs) good = good && slack(h,p) >= inset-1e-8;
        for (const auto& q: v) if ((p-q).norm()<1e-8) good=false;
        if (good) v.push_back(p);
    }
    require(v.size()>=3, C::FailureReason::OUTPUT_CERTIFICATE);
    V c=V::Zero(); for (const auto& p:v) c+=p; c/=v.size();
    std::sort(v.begin(),v.end(),[&](const V&a,const V&b) {
        return std::atan2(a.y()-c.y(),a.x()-c.x()) < std::atan2(b.y()-c.y(),b.x()-c.x()); });
    return v;
}
struct Objective { Clock::time_point end; };
double objective(const std::vector<double>&x,std::vector<double>&g,void*data) {
    auto& d=*static_cast<Objective*>(data);
    // NLopt forwards C++ exceptions after stopping the native optimizer.
    if (Clock::now()>=d.end) throw nlopt::forced_stop();
    if (!g.empty()) std::fill(g.begin(),g.end(),0.);
    if (!g.empty()) { g[2]=-1./x[2]; g[4]=-1./x[4]; }
    return -std::log(x[2])-std::log(x[4]);
}
double ellipseConstraint(const std::vector<double>&x,std::vector<double>&g,void*data) {
    const auto& h=*static_cast<H*>(data); const double a=h.normal.x(),b=h.normal.y();
    const double u=a*x[2]+b*x[3], v=b*x[4], n=std::hypot(u,v);
    if (!g.empty()) { g[0]=a; g[1]=b; g[2]=a*u/n; g[3]=b*u/n; g[4]=b*v/n; }
    return a*x[0]+b*x[1]+n-h.offset;
}
void stage(C::Diagnostics& d, C::FailureStage s) {
    const int iteration=d.outer_iteration;
    d=C::Diagnostics(); d.stage=s; d.outer_iteration=iteration;
}
// Diagnostic-only evaluation, including a failed solver's last iterate.
// Invalid/nonfinite values stay explicitly unavailable, never a fabricated zero.
template<class Constraint>
void measureConstraints(std::vector<Constraint>& hs, const std::vector<double>& x,
                        double (*eval)(const std::vector<double>&,std::vector<double>&,void*),
                        C::Diagnostics& d) {
    d.constraint_violation_valid=false;
    d.max_constraint_violation=std::numeric_limits<double>::quiet_NaN();
    for(double v:x) if(!std::isfinite(v)) return;
    double worst=0.; std::vector<double> unused;
    for(auto& h:hs) { const double v=eval(x,unused,&h); if(!std::isfinite(v))return; worst=std::max(worst,v); }
    if(!hs.empty()) { d.max_constraint_violation=worst; d.constraint_violation_valid=true; }
}
template<class Optimizer>
void solve(Optimizer& opt, std::vector<double>&x, Clock::time_point end, C::Diagnostics& diag,
           bool mvie_candidate = false) {
    deadline(end); double cost;
    opt.set_maxtime(std::max(1e-6,std::chrono::duration<double>(end-Clock::now()).count()));
    try {
        const auto status=opt.optimize(x,cost);
        diag.solver_status=static_cast<int>(status); diag.solver_status_valid=true;
        require(status==nlopt::SUCCESS || status==nlopt::FTOL_REACHED || status==nlopt::XTOL_REACHED,
                status==nlopt::MAXTIME_REACHED ? C::FailureReason::BUDGET : C::FailureReason::NUMERICAL_FAILURE);
    } catch (const Failure&) { throw; }
      catch (const nlopt::forced_stop& e) {
        diag.solver_status=static_cast<int>(opt.last_optimize_result()); diag.solver_status_valid=true;
        diag.solver_exception=std::string(e.what()).substr(0,256); throw Failure{C::FailureReason::BUDGET};
      }
      catch (const nlopt::roundoff_limited& e) {
        diag.solver_status=static_cast<int>(opt.last_optimize_result()); diag.solver_status_valid=true;
        diag.solver_exception=std::string(e.what()).substr(0,256);
        // Only MVIE may pass its current iterate to the unchanged certificates.
        // This is not optimizer success, and separation remains fail-closed.
        require(mvie_candidate && diag.solver_status==nlopt::ROUNDOFF_LIMITED,
                C::FailureReason::NUMERICAL_FAILURE);
      }
      catch (const std::exception& e) {
        diag.solver_status=static_cast<int>(opt.last_optimize_result()); diag.solver_status_valid=true;
        diag.solver_exception=std::string(e.what()).substr(0,256); throw Failure{C::FailureReason::NUMERICAL_FAILURE};
      }
    deadline(end);
    for (double value:x) require(std::isfinite(value),C::FailureReason::NUMERICAL_FAILURE);
}
// Maximize min_v n.(v-center) / |L^T n| over unit normals with
// n.(v-seed_i) >= clearance. Offset is free, including negative support.
// Within each cell support sector the objective is a linear form divided by
// an ellipsoidal norm. Its only stationary directions are +/- Q^-1(v-center).
// All other maxima are on a sector or seed-feasibility boundary. Enumerating
// these finitely many directions is complete in 2D; no angular sampling or
// optimizer status is used as a geometric feasibility certificate.
template<class Obstacle>
H separate(const Obstacle& cell,const V&seed,const V&other,const V&center,const M&L,const C::Config&cfg,Clock::time_point end,C::Diagnostics& diag,const Poly& extra = {}) {
    stage(diag,C::FailureStage::OBSTACLE_SEPARATION_SOLVE);
    Poly seeds{seed,other};seeds.insert(seeds.end(),extra.begin(),extra.end());
    deadline(end);
    require(seed.allFinite() && other.allFinite() && center.allFinite() &&
            L.allFinite() && std::abs(L.determinant())>0,
            C::FailureReason::NUMERICAL_FAILURE);
    const M inverse=(L*L.transpose()).inverse();
    require(inverse.allFinite(),C::FailureReason::NUMERICAL_FAILURE);
    std::vector<V> candidates;
    auto add=[&](const V& n) {
        if(n.allFinite() && n.norm()>0) candidates.push_back(n.normalized());
    };
    for(size_t i=0;i<cell.size();++i) {
        require(cell[i].allFinite(),C::FailureReason::NUMERICAL_FAILURE);
        const V stationary=inverse*(cell[i]-center);
        add(stationary); add(-stationary);
        for(size_t j=0;j<i;++j) {
            const V d=cell[i]-cell[j]; const V tangent(-d.y(),d.x());
            add(tangent); add(-tangent);
        }
        for(const V& p : seeds) {
            const V d=cell[i]-p; const double distance=d.norm();
            if(distance<cfg.clearance) continue;
            const V axis=d/distance, tangent(-axis.y(),axis.x());
            const double cosine=cfg.clearance/distance;
            const double sine=std::sqrt(std::max(0.,(1.-cosine)*(1.+cosine)));
            add(cosine*axis+sine*tangent); add(cosine*axis-sine*tangent);
        }
    }
    double best=-std::numeric_limits<double>::infinity(); H result;
    bool found=false;
    for(const V& n:candidates) {
        deadline(end);
        double support=std::numeric_limits<double>::infinity();
        for(const V& v:cell) support=std::min(support,n.dot(v));
        const double b=support-cfg.clearance;
        // Independent normalized metric endpoint certificate (unchanged).
        bool feasible=true;for(const auto& p:seeds)feasible=feasible && n.dot(p)<=b+1e-9;
        if(!feasible)continue;
        const double denominator=(L.transpose()*n).norm();
        require(std::isfinite(denominator) && denominator>0,C::FailureReason::NUMERICAL_FAILURE);
        const double value=(support-n.dot(center))/denominator;
        require(std::isfinite(value),C::FailureReason::NUMERICAL_FAILURE);
        if(!found || value>best) { found=true; best=value; result={n,b}; }
    }
    require(found,C::FailureReason::REFERENCE_OCCUPIED);
    diag.stage=C::FailureStage::OUTPUT_CERTIFICATE;
    diag.constraint_violation_valid=true;
    diag.max_constraint_violation=0.;
    for(const auto& p:seeds)diag.max_constraint_violation=std::max(diag.max_constraint_violation,result.normal.dot(p)-result.offset);
    require(result.normal.allFinite() && std::isfinite(result.offset),C::FailureReason::OUTPUT_CERTIFICATE);
    for(const V& v:cell) require(result.normal.dot(v)-result.offset>=cfg.clearance-1e-7,
                                C::FailureReason::OUTPUT_CERTIFICATE);
    return result;
}
void certifyMvie(std::vector<H>& hs,const std::vector<double>& x,V& center,M& L) {
    for(double value:x)require(std::isfinite(value),C::FailureReason::OUTPUT_CERTIFICATE);
    // Retain the optimizer's positive-diagonal bounds explicitly, including
    // when roundoff returned a candidate rather than a successful solve.
    require(x[2]>=1e-7 && x[4]>=1e-7,C::FailureReason::OUTPUT_CERTIFICATE);
    std::vector<double> unused;
    for(auto&h:hs)require(ellipseConstraint(x,unused,&h)<=1e-7,C::FailureReason::OUTPUT_CERTIFICATE);
    center=V(x[0],x[1]);L<<x[2],0.,x[3],x[4]; double scale=1.;
    for(auto&h:hs)scale=std::min(scale,slack(h,center)/(L.transpose()*h.normal).norm());
    require(scale>0,C::FailureReason::OUTPUT_CERTIFICATE);L*=scale*(1.-1e-8);
}
void mvie(std::vector<H>&hs,V&center,M&L,const C::Config&cfg,Clock::time_point end,C::Diagnostics& diag) {
    stage(diag,C::FailureStage::MVIE_INIT_SLACK);
    double minimum=std::numeric_limits<double>::infinity(); bool finite=true;
    for(const auto& h:hs) { const double s=slack(h,center); finite=finite&&std::isfinite(s); minimum=std::min(minimum,s); }
    if(finite&&!hs.empty()) { diag.min_slack=minimum; diag.min_slack_valid=true; }
    double scale=1.;
    for(auto&h:hs) { const double s=slack(h,center); require(s>0,C::FailureReason::NUMERICAL_FAILURE);
        scale=std::min(scale,.9*s/(L.transpose()*h.normal).norm()); }
    L*=scale; std::vector<double>x{center.x(),center.y(),L(0,0),L(1,0),L(1,1)};
    nlopt::opt opt(nlopt::LD_SLSQP,5); Objective d{end}; opt.set_min_objective(objective,&d);
    const double inf=std::numeric_limits<double>::infinity(); opt.set_lower_bounds({-inf,-inf,1e-7,-inf,1e-7});
    for(auto&h:hs)opt.add_inequality_constraint(ellipseConstraint,&h,1e-9);
    opt.set_maxeval(cfg.optimizer_iterations);opt.set_ftol_abs(1e-9);opt.set_xtol_rel(1e-9);
    diag.stage=C::FailureStage::MVIE_SOLVE;
    try { solve(opt,x,end,diag,true); }
    catch(const Failure&) { measureConstraints(hs,x,ellipseConstraint,diag); throw; }
    measureConstraints(hs,x,ellipseConstraint,diag);
    diag.stage=C::FailureStage::OUTPUT_CERTIFICATE;
    certifyMvie(hs,x,center,L);
}
C::Segment inflate(const C::Grid&grid,const V&begin,const V&finish,const C::Config&cfg,Clock::time_point end,C::Diagnostics& diag,
                   const std::vector<Poly>& dynamic = {},const Poly& extra_world = {}) {
    diag=C::Diagnostics(); diag.stage=C::FailureStage::REGION_INIT;
    // Midpoint-local coordinates protect both endpoints of this original-edge seed.
    const V seed=(begin+finish)*.5, a_seed=begin-seed, b_seed=finish-seed;
    Poly protected_points;
    for(const auto& p:extra_world)protected_points.push_back(p-seed);
    const V lo=(grid.origin-seed).cwiseMax(V::Constant(-cfg.local_radius));
    const V hi=(grid.origin+grid.resolution*V(grid.width,grid.height)-seed).cwiseMin(V::Constant(cfg.local_radius));
    std::vector<H> bounds{{V(1,0),hi.x()-cfg.clearance},{V(-1,0),-lo.x()-cfg.clearance},
                         {V(0,1),hi.y()-cfg.clearance},{V(0,-1),-lo.y()-cfg.clearance}};
    for(auto&h:bounds) {
        require(h.offset>0 && slack(h,a_seed)>=0 && slack(h,b_seed)>=0,C::FailureReason::REFERENCE_OCCUPIED);
        for(const auto& p:protected_points)require(slack(h,p)>=0,C::FailureReason::REFERENCE_OCCUPIED);
    }
    std::vector<Cell> cells;
    for(int y=0;y<grid.height;++y) { deadline(end); for(int x=0;x<grid.width;++x) {
        if(!grid.blocked[y*grid.width+x])continue;
        V a=grid.origin-seed+grid.resolution*V(x,y), z=a+V::Constant(grid.resolution);
        if((z.array()<lo.array()).any() || (a.array()>hi.array()).any())continue;
        require(!((a.array()<=cfg.clearance).all()&&(z.array()>=-cfg.clearance).all()),C::FailureReason::REFERENCE_OCCUPIED);
        cells.push_back({a,V(z.x(),a.y()),z,V(a.x(),z.y())});
    }}
    std::vector<Poly> polygons;
    for(const auto& world:dynamic) {
        Poly local;for(const auto& p:world)local.push_back(p-seed);
        polygons.push_back(std::move(local));
    }
    V center=V::Zero(); M L=M::Identity()*.05; std::vector<H> hs;
    for(int it=0;it<cfg.iterations;++it) {
        diag.outer_iteration=it;
        hs=bounds; std::vector<size_t> remaining; for(size_t i=0;i<cells.size();++i)remaining.push_back(i);
        while(!remaining.empty()) {
            deadline(end); double best=std::numeric_limits<double>::infinity(); size_t id=remaining[0]; const M inv=L.inverse();
            for(size_t i:remaining)for(auto&v:cells[i]) { double d=(inv*(v-center)).norm();if(d<best){best=d;id=i;} }
            H h=separate(cells[id],a_seed,b_seed,center,L,cfg,end,diag,protected_points);hs.push_back(h);
            const auto old=remaining.size();
            remaining.erase(std::remove_if(remaining.begin(),remaining.end(),[&](size_t i){
                double gap=std::numeric_limits<double>::infinity();for(auto&v:cells[i])gap=std::min(gap,-slack(h,v));
                return gap>=cfg.clearance-1e-8;
            }),remaining.end());
            require(remaining.size()<old,C::FailureReason::NO_PROGRESS);
        }
        for(const auto& polygon:polygons) {
            deadline(end);
            bool excluded=false;
            for(const auto& h:hs) {
                double gap=std::numeric_limits<double>::infinity();
                for(const auto& p:polygon)gap=std::min(gap,-slack(h,p));
                excluded=excluded || gap>=cfg.clearance-1e-8;
            }
            if(!excluded) {
                try {hs.push_back(separate(polygon,a_seed,b_seed,center,L,cfg,end,diag,protected_points));}
                catch(const Failure& f) {
                    if(f.reason==C::FailureReason::REFERENCE_OCCUPIED)throw Failure{C::FailureReason::DYNAMIC_REFERENCE_BLOCKED};
                    throw;
                }
            }
        }
        mvie(hs,center,L,cfg,end,diag);
    }
    diag.stage=C::FailureStage::OUTPUT_CERTIFICATE;
    diag.min_slack_valid=diag.constraint_violation_valid=false;
    diag.min_slack=diag.max_constraint_violation=std::numeric_limits<double>::quiet_NaN();
    double minimum=std::numeric_limits<double>::infinity();
    double worst=0.; bool finite=true;
    for(const auto& h:hs) {
        const double seed_slack=std::min(slack(h,a_seed),slack(h,b_seed));
        const double ellipse_slack=slack(h,center)-(L.transpose()*h.normal).norm();
        finite=finite&&std::isfinite(seed_slack)&&std::isfinite(ellipse_slack);
        minimum=std::min(minimum,std::min(seed_slack,ellipse_slack));
        worst=std::max(worst,-ellipse_slack);
    }
    if(finite&&!hs.empty()) {
        diag.min_slack=minimum;diag.min_slack_valid=true;
        diag.max_constraint_violation=worst;diag.constraint_violation_valid=true;
    }
    Poly v=vertices(hs);require(C::polygonArea(v)>=1e-8,C::FailureReason::OUTPUT_CERTIFICATE);
    for(auto&h:hs) {
        require(slack(h,a_seed)>=-1e-8 && slack(h,b_seed)>=-1e-8,C::FailureReason::OUTPUT_CERTIFICATE);
        for(const auto& p:protected_points)require(slack(h,p)>=-1e-8,C::FailureReason::OUTPUT_CERTIFICATE);
        require(slack(h,center)-(L.transpose()*h.normal).norm()>=-1e-7,C::FailureReason::OUTPUT_CERTIFICATE);
    }
    for(auto&h:bounds)for(auto&p:v)require(slack(h,p)>=-1e-7,C::FailureReason::OUTPUT_CERTIFICATE);
    for(auto&cell:cells) {
        bool separated=false;for(auto&h:hs) { double gap=std::numeric_limits<double>::infinity();
            for(auto&p:cell) { gap=std::min(gap,-slack(h,p)); }
            separated=separated||gap>=cfg.clearance-1e-7; }
        require(separated,C::FailureReason::OUTPUT_CERTIFICATE);
    }
    for(const auto& polygon:polygons) {
        deadline(end);bool excluded=false;
        for(const auto& h:hs) {
            double gap=std::numeric_limits<double>::infinity();
            for(const auto& p:polygon)gap=std::min(gap,-slack(h,p));
            excluded=excluded || gap>=cfg.clearance-1e-7;
        }
        require(excluded,C::FailureReason::OUTPUT_CERTIFICATE);
    }
    C::Segment r;r.center=center+seed;r.vertices=v;for(auto&p:r.vertices)p+=seed;
    for(auto&h:hs) { h.offset+=h.normal.dot(seed); }
    r.halfspaces=hs;return r;
}
std::pair<double,double> overlap(const C::Segment&a,const C::Segment&b) {
    // Rebase intersection as well: area and line solves remain local.
    std::vector<H> hs=a.halfspaces;hs.insert(hs.end(),b.halfspaces.begin(),b.halfspaces.end());
    for(auto&h:hs)h.offset-=h.normal.dot(a.center);
    try {
        // 1e-6 m is 100 times the vertex feasibility tolerance (1e-8 m).
        // A strictly verified witness disk, not a physical footprint requirement.
        constexpr double interior=1e-6;
        double ar=C::polygonArea(vertices(hs));Poly inner=vertices(hs,2*interior);V c=V::Zero();for(auto&p:inner)c+=p;c/=inner.size();
        double depth=std::numeric_limits<double>::infinity();for(auto&h:hs)depth=std::min(depth,slack(h,c));
        require(std::isfinite(ar) && ar>0 && depth>=interior,C::FailureReason::NO_OVERLAP);return {ar,depth};
    }catch(const Failure&){throw Failure{C::FailureReason::NO_OVERLAP};}
}
// Clip original edges, never chords between distant path points. Intervals are
// closed and merged without a gap tolerance; only the existing 1e-8 m plane
// certificate tolerance is used. s0/s1 bound the test, not evidence of coverage.
bool coversPath(const Poly& path, const std::vector<double>& arc,
                const std::vector<const C::Segment*>& regions,
                double begin, double finish, Clock::time_point end) {
    for(size_t e=0;e+1<path.size();++e) {
        deadline(end);
        if(arc[e+1]<begin || arc[e]>finish)continue;
        const double length=arc[e+1]-arc[e];
        const double lo=std::max(0.,(begin-arc[e])/length);
        const double hi=std::min(1.,(finish-arc[e])/length);
        std::vector<std::pair<double,double>> intervals;
        for(const auto* r:regions) {
            deadline(end);
            double a=lo,b=hi;
            const V p=path[e]-r->center,d=path[e+1]-path[e];
            for(const auto& h:r->halfspaces) {
                const double room=(h.offset-h.normal.dot(r->center))-h.normal.dot(p)+1e-8;
                const double slope=h.normal.dot(d);
                if(slope>0.)b=std::min(b,room/slope);
                else if(slope<0.)a=std::max(a,room/slope);
                else if(room<0.) {a=1.;b=0.;break;}
            }
            if(a<=b)intervals.emplace_back(a,b);
        }
        std::sort(intervals.begin(),intervals.end());
        double covered=lo;
        bool found=false;
        for(const auto& interval:intervals) {
            if(interval.first>covered)break;
            covered=std::max(covered,interval.second);found=true;
        }
        if(!found || covered<hi)return false;
    }
    return true;
}

// GCOPTER shortCut's backward, earliest-predecessor selection, strengthened
// with original-path coverage and the same positive-interior join certificate.
// Retain endpoint regions and their original protected-seed s0/s1 metadata.
void pruneRegions(std::vector<C::Segment>& regions, const Poly& path,
                  const std::vector<double>& arc, Clock::time_point end,
                  C::Diagnostics& diagnostics) {
    deadline(end);
    std::vector<size_t> keep{regions.size()-1};
    for(size_t i=regions.size()-1;i>0;) {
        size_t selected=i-1;
        for(size_t j=0;j+1<i;++j) {
            deadline(end);
            stage(diagnostics,C::FailureStage::OVERLAP);
            try { overlap(regions[j],regions[i]); }
            catch(const Failure& f) {if(f.reason!=C::FailureReason::NO_OVERLAP)throw;continue;}
            stage(diagnostics,C::FailureStage::COVERAGE);
            if(coversPath(path,arc,{&regions[j],&regions[i]},regions[j].s0,regions[i].s1,end)) {
                selected=j;break;
            }
        }
        keep.push_back(selected);i=selected;
    }
    std::vector<C::Segment> retained;
    for(auto it=keep.rbegin();it!=keep.rend();++it)retained.push_back(std::move(regions[*it]));
    regions=std::move(retained);
    stage(diagnostics,C::FailureStage::COVERAGE);
    std::vector<const C::Segment*> refs;
    for(const auto& r:regions)refs.push_back(&r);
    require(coversPath(path,arc,refs,0.,arc.back(),end),C::FailureReason::OUTPUT_CERTIFICATE);
    deadline(end);
}
}
bool ConvexCorridor::contains(const Segment&s,const V&p,double tol) {
    if(!p.allFinite()||s.halfspaces.size()<3)return false;
    for(auto&h:s.halfspaces)if(slack(h,p)<-tol)return false;
    return true;
}
std::vector<Eigen::Vector2d> ConvexCorridor::referencePath(const Poly& route,size_t index,const V& root,double length) {
    if(index>=route.size() || route.size()-index<2 || !root.allFinite() || !std::isfinite(length) || length<0.)return {};
    Poly result{root};double remaining=length>0.?length:std::numeric_limits<double>::infinity();
    const V edge=route[index+1]-route[index];
    if(!edge.allFinite() || edge.squaredNorm()<1e-18)return {};
    const V projection=route[index]+std::max(0.,std::min(1.,(root-route[index]).dot(edge)/edge.squaredNorm()))*edge;
    auto append=[&](const V& p) {
        const V delta=p-result.back();const double distance=delta.norm();
        if(!p.allFinite())return false;
        if(distance<1e-9)return true;
        if(distance>remaining){result.push_back(result.back()+delta*(remaining/distance));remaining=0.;return false;}
        result.push_back(p);remaining-=distance;return remaining>1e-9;
    };
    if(!append(projection))return result;
    for(size_t i=index+1;i<route.size();++i)if(!append(route[i]))break;
    return result;
}
bool ConvexCorridor::containsFootprint(const Segment& s,const V& p,const Poly& relative,double tol) {
    if(!p.allFinite() || relative.empty() || s.halfspaces.size()<3)return false;
    for(const auto& v:relative)if(!v.allFinite())return false;
    for(const auto& h:s.halfspaces)
        if(slack(h,p)-HumanCaneFootprint::support(relative,h.normal)<-tol)return false;
    return true;
}
double ConvexCorridor::violation(const Segment&s,const V&p) {
    if(!p.allFinite()||s.halfspaces.size()<3)return std::numeric_limits<double>::infinity();
    double v=0.;for(auto&h:s.halfspaces)v=std::max(v,-slack(h,p));return v;
}
double ConvexCorridor::polygonArea(const Poly&v) {
    if(v.size()<3)return 0.;
    double a=0.;for(size_t i=0;i<v.size();++i){V p=v[i]-v[0],q=v[(i+1)%v.size()]-v[0];a+=p.x()*q.y()-p.y()*q.x();}return std::abs(a)*.5;
}
const char* ConvexCorridor::failureReasonName(FailureReason r) {
#define NAME(x) case FailureReason::x: return #x
    switch(r){NAME(NONE);NAME(INVALID_INPUT);NAME(REFERENCE_OCCUPIED);NAME(DEGENERATE_PATH);NAME(NO_OVERLAP);NAME(NO_PROGRESS);NAME(NUMERICAL_FAILURE);NAME(OUTPUT_CERTIFICATE);NAME(BUDGET);NAME(UNSUPPORTED_SCALE);NAME(UNSUPPORTED_BACKEND);NAME(DYNAMIC_REFERENCE_BLOCKED);NAME(DYNAMIC_DATA_INVALID);NAME(MAP_CHANGED);}
#undef NAME
    return "UNKNOWN";
}
const char* ConvexCorridor::failureStageName(FailureStage s) {
#define NAME(x) case FailureStage::x: return #x
    switch(s){NAME(NONE);NAME(INPUT);NAME(REGION_INIT);NAME(OBSTACLE_SEPARATION_SOLVE);NAME(MVIE_INIT_SLACK);NAME(MVIE_SOLVE);NAME(OUTPUT_CERTIFICATE);NAME(COVERAGE);NAME(OVERLAP);}
#undef NAME
    return "UNKNOWN";
}
std::string ConvexCorridor::diagnosticsText(const Result& r) {
    const auto& d=r.diagnostics; std::ostringstream ss; ss << std::setprecision(17);
    ss << " stage=" << failureStageName(d.stage) << " outer_iteration=" << d.outer_iteration
       << " solver_status_valid=" << d.solver_status_valid << " solver_status=" << d.solver_status
       << " solver_exception=" << std::quoted(d.solver_exception)
       << " min_slack_valid=" << d.min_slack_valid << " min_slack=";
    if(d.min_slack_valid)ss << d.min_slack; else ss << "NA";
    ss << " constraint_violation_valid=" << d.constraint_violation_valid << " max_constraint_violation=";
    if(d.constraint_violation_valid)ss << d.max_constraint_violation; else ss << "NA";
    return ss.str();
}
ConvexCorridor::Result ConvexCorridor::buildStatic(const Poly&path,const Grid&grid,const std::vector<Poly>& dynamic) const {
    return build(path,grid,dynamic,false);
}
ConvexCorridor::Result ConvexCorridor::buildPrefix(const Poly&path,const Grid&grid,const std::vector<Poly>& dynamic) const {
    return build(path,grid,dynamic,true);
}
ConvexCorridor::Result ConvexCorridor::build(const Poly&path,const Grid&grid,const std::vector<Poly>& dynamic,bool allow_prefix) const {
    Result out;out.has_dynamic_input=!dynamic.empty();out.diagnostics.stage=FailureStage::INPUT;const auto began=Clock::now();const auto&c=cfg_;
    std::vector<double> arc;Clock::time_point end;
    try {
        require(path.size()>=2 && grid.width>0 && grid.height>0 && grid.width<=10000 && grid.height<=10000 &&
                grid.blocked.size()==size_t(grid.width)*size_t(grid.height),FailureReason::INVALID_INPUT);
        require(HumanCaneFootprint::validConfig(c.footprint) && std::isfinite(c.root_yaw),FailureReason::INVALID_INPUT);
        for(double x:{c.local_radius,c.clearance,c.max_segment_length,c.max_seconds,grid.resolution})
            require(std::isfinite(x)&&x>0,FailureReason::INVALID_INPUT);
        require(c.iterations>0&&c.iterations<=100&&c.optimizer_iterations>0&&c.optimizer_iterations<=10000&&c.max_regions>0&&c.max_regions<=1000&&
                c.clearance<c.local_radius && c.max_segment_length<2*(c.local_radius-c.clearance),FailureReason::INVALID_INPUT);
        require(grid.origin.allFinite(),FailureReason::INVALID_INPUT);
        V upper=grid.origin+grid.resolution*V(grid.width,grid.height);
        require(grid.resolution>=.01 && grid.resolution*std::max(grid.width,grid.height)<=100. &&
                grid.origin.cwiseAbs().maxCoeff()<=1e4 && upper.cwiseAbs().maxCoeff()<=1e4 &&
                c.local_radius>=.01&&c.local_radius<=100.&&c.clearance>=1e-4&&c.max_segment_length>=1e-4,
                FailureReason::UNSUPPORTED_SCALE);
        arc.push_back(0.);
        for(size_t i=0;i<path.size();++i){require(path[i].allFinite(),FailureReason::INVALID_INPUT);require(path[i].cwiseAbs().maxCoeff()<=1e4,FailureReason::UNSUPPORTED_SCALE);
            if(i){double d=(path[i]-path[i-1]).norm();require(d>=1e-9,FailureReason::DEGENERATE_PATH);arc.push_back(arc.back()+d);}}
        end=began+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c.max_seconds));
        require(dynamic.size()<=kMaxPedestrians,FailureReason::DYNAMIC_DATA_INVALID);
        for(const auto& polygon:dynamic) {
            require(polygon.size()>=3 && polygon.size()<=32,FailureReason::DYNAMIC_DATA_INVALID);
            for(const auto& p:polygon)require(p.allFinite() && p.cwiseAbs().maxCoeff()<=1e4 &&
                (p-polygon.front()).norm()<=20,FailureReason::DYNAMIC_DATA_INVALID);
            // Strictly convex CCW input: one support plane represents the whole
            // obstacle, never unrelated point-wise separating planes.
            for(size_t i=0;i<polygon.size();++i) {
                const V a=polygon[(i+1)%polygon.size()]-polygon[i];
                require(a.norm()>1e-8,FailureReason::DYNAMIC_DATA_INVALID);
                for(size_t j=0;j<polygon.size();++j) {
                    if(j==i || j==(i+1)%polygon.size())continue;
                    const V b=polygon[j]-polygon[i];
                    require(a.x()*b.y()-a.y()*b.x()>1e-10,FailureReason::DYNAMIC_DATA_INVALID);
                }
            }
        }
        // Check the entire protected local reference before numerical inflation:
        // a later crossing must not be obscured by an earlier MVIE failure.
        for(size_t edge=0;edge+1<path.size() && !dynamic.empty() && !(allow_prefix && c.adaptive_sampling);++edge) {
            out.failure_segment_index=edge;out.failure_s=arc[edge];
            out.failure_position=path[edge];out.failure_position_valid=true;
            for(const auto& world:dynamic) {
                Poly local;for(const auto& p:world)local.push_back(p-path[edge]);
                Poly protected_points;
                if(edge==0 && c.footprint.enabled)
                    protected_points=HumanCaneFootprint::relative(c.footprint,c.root_yaw);
                try {separate(local,V::Zero(),path[edge+1]-path[edge],V::Zero(),M::Identity(),c,end,out.diagnostics,protected_points);}
                catch(const Failure& f) {
                    if(f.reason==FailureReason::REFERENCE_OCCUPIED)throw Failure{FailureReason::DYNAMIC_REFERENCE_BLOCKED};
                    throw;
                }
            }
        }
        out.min_overlap_area=out.min_overlap_depth=std::numeric_limits<double>::infinity();
        auto append=[&](Segment r) {
            deadline(end);
            require(out.segments.size()<size_t(c.max_regions),FailureReason::BUDGET);
            if(!out.segments.empty()) {
                stage(out.diagnostics,FailureStage::OVERLAP);
                const auto o=overlap(out.segments.back(),r);
                out.min_overlap_area=std::min(out.min_overlap_area,o.first);
                out.min_overlap_depth=std::min(out.min_overlap_depth,o.second);
            }
            out.segments.push_back(std::move(r));++out.generated_regions;
        };
        if(c.adaptive_sampling) {
            double position=0.;
            while(position<arc.back()) {
                deadline(end);
                require(out.segments.size()<size_t(c.max_regions),FailureReason::BUDGET);
                size_t edge=std::min(path.size()-2,size_t(std::upper_bound(arc.begin(),arc.end(),position)-arc.begin()-1));
                const V delta=path[edge+1]-path[edge];
                const V point=path[edge]+delta*((position-arc[edge])/(arc[edge+1]-arc[edge]));
                // Protect a bounded forward portion of the ORIGINAL polyline,
                // using reference points, not predicted human-cane envelopes.
                // Only the actual current pose belongs to this spatial corridor.
                double seed_finish=std::min(arc.back(),position+std::min(c.max_segment_length,.5*c.local_radius));
                out.failure_s=position;out.failure_position=point;out.failure_position_valid=true;
                out.failure_segment_index=out.generated_regions;
                Segment region;
                // A free bent polyline need not have a free convex hull. Split
                // the protected interval, down to this original straight edge,
                // before rejecting it. Keep the actual root hull at every retry.
                const double minimum_finish=std::min(seed_finish,arc[edge+1]);
                for(;;) {
                    Poly protected_world;
                    for(size_t e=edge;e+1<path.size() && arc[e]<seed_finish;++e) {
                        deadline(end);
                        const V d=path[e+1]-path[e];const double length=arc[e+1]-arc[e];
                        for(double s:{std::max(position,arc[e]),std::min(seed_finish,arc[e+1])})
                            protected_world.push_back(path[e]+d*((s-arc[e])/length));
                    }
                    if(out.segments.empty() && c.footprint.enabled)
                        for(const auto& p:HumanCaneFootprint::world(c.footprint,path.front(),c.root_yaw))protected_world.push_back(p);
                    if(protected_world.size()>2)protected_world=PedestrianPolygon::convexHull(std::move(protected_world));
                    try {region=inflate(grid,point,point,c,end,out.diagnostics,dynamic,protected_world);break;}
                    catch(const Failure& f) {
                        if((f.reason!=FailureReason::REFERENCE_OCCUPIED && f.reason!=FailureReason::DYNAMIC_REFERENCE_BLOCKED &&
                            f.reason!=FailureReason::OUTPUT_CERTIFICATE) ||
                           seed_finish<=minimum_finish+1e-9)throw;
                        seed_finish=std::max(minimum_finish,position+.5*(seed_finish-position));
                    }
                }
                stage(out.diagnostics,FailureStage::COVERAGE);
                double covered=position;
                for(size_t e=edge;e+1<path.size();++e) {
                    const V d=path[e+1]-path[e];const double length=arc[e+1]-arc[e];
                    double a=std::max(0.,(covered-arc[e])/length),b=1.;
                    const double start=a;
                    for(const auto& h:region.halfspaces) {
                        const double room=(h.offset-h.normal.dot(region.center))-h.normal.dot(path[e]-region.center)+1e-8;
                        const double slope=h.normal.dot(d);
                        if(slope>0)b=std::min(b,room/slope);
                        else if(slope<0)a=std::max(a,room/slope);
                        else if(room<0){a=1.;b=0.;break;}
                    }
                    if(a>start || b<start)break;
                    covered=b>=1.?arc[e+1]:arc[e]+length*b;
                    if(b<1.)break;
                }
                require(covered>position+1e-6,FailureReason::NO_PROGRESS);
                region.s0=position;region.s1=covered;
                append(std::move(region));
                if(covered>=arc.back())break;
                position=covered-std::min(.25,.5*(covered-position));
            }
            stage(out.diagnostics,FailureStage::COVERAGE);
            std::vector<const Segment*> refs;for(const auto& r:out.segments)refs.push_back(&r);
            require(coversPath(path,arc,refs,0.,arc.back(),end),FailureReason::OUTPUT_CERTIFICATE);
        } else for(size_t edge=0;edge+1<path.size();++edge) {
            const double length=arc[edge+1]-arc[edge];
            const double count=std::ceil(length/c.max_segment_length);
            require(count<=c.max_regions,FailureReason::BUDGET);
            const int pieces=static_cast<int>(count);
            for(int j=0;j<pieces;++j) {
                deadline(end);
                require(out.segments.size()<size_t(c.max_regions),FailureReason::BUDGET);
                const V a=path[edge]+(path[edge+1]-path[edge])*(double(j)/pieces);
                const V b=j+1==pieces?path[edge+1]:path[edge]+(path[edge+1]-path[edge])*(double(j+1)/pieces);
                out.failure_segment_index=out.segments.size();
                out.failure_s=arc[edge]+length*j/pieces;
                out.failure_position=a;out.failure_position_valid=true;
                Poly root;
                if(edge==0 && j==0 && c.footprint.enabled)root=HumanCaneFootprint::world(c.footprint,path.front(),c.root_yaw);
                Segment r=inflate(grid,a,b,c,end,out.diagnostics,dynamic,root);
                r.s0=out.failure_s;r.s1=arc[edge]+length*(j+1)/pieces;
                if(!out.segments.empty()) {
                    bool connected=true;
                    try { overlap(out.segments.back(),r); }
                    catch(const Failure& f) { if(f.reason!=FailureReason::NO_OVERLAP)throw;connected=false; }
                    if(!connected) {
                        require(out.segments.size()+2<=size_t(c.max_regions),FailureReason::BUDGET);
                        Segment connection=inflate(grid,a,a,c,end,out.diagnostics,dynamic);
                        connection.s0=connection.s1=r.s0;
                        append(std::move(connection));
                    }
                }
                append(std::move(r));
            }
        }
        stage(out.diagnostics,FailureStage::COVERAGE);
        if(!c.adaptive_sampling)pruneRegions(out.segments,path,arc,end,out.diagnostics);
        out.min_overlap_area=out.min_overlap_depth=std::numeric_limits<double>::infinity();
        for(size_t i=1;i<out.segments.size();++i) {
            deadline(end);
            stage(out.diagnostics,FailureStage::OVERLAP);
            const auto o=overlap(out.segments[i-1],out.segments[i]);
            out.min_overlap_area=std::min(out.min_overlap_area,o.first);
            out.min_overlap_depth=std::min(out.min_overlap_depth,o.second);
        }
        deadline(end);
        out.feasible=true;out.complete=true;out.covered_length=arc.back();
        require(out.feasible,FailureReason::BUDGET);out.failure_segment_index=-1;out.failure_position_valid=false;
        out.diagnostics=Diagnostics();
        if(out.segments.size()==1)out.min_overlap_area=out.min_overlap_depth=0.;
    }catch(const Failure&f){
        out.failure_reason=f.reason;
        // No prefix on invalid input, expired budget or numerical failure. A
        // geometric suffix failure may retain only independently certified
        // original-path coverage and positive-interior joins from the root.
        if(allow_prefix && c.adaptive_sampling && !out.segments.empty() &&
           (f.reason==FailureReason::REFERENCE_OCCUPIED || f.reason==FailureReason::DYNAMIC_REFERENCE_BLOCKED ||
            f.reason==FailureReason::OUTPUT_CERTIFICATE || f.reason==FailureReason::NO_PROGRESS ||
            f.reason==FailureReason::NO_OVERLAP)) {
            try {
                const double covered=out.segments.back().s1;
                std::vector<const Segment*> refs;for(const auto& r:out.segments)refs.push_back(&r);
                require(covered>1e-6 && coversPath(path,arc,refs,0.,covered,end),FailureReason::OUTPUT_CERTIFICATE);
                out.min_overlap_area=out.min_overlap_depth=std::numeric_limits<double>::infinity();
                for(size_t i=1;i<out.segments.size();++i) {
                    deadline(end);const auto o=overlap(out.segments[i-1],out.segments[i]);
                    out.min_overlap_area=std::min(out.min_overlap_area,o.first);
                    out.min_overlap_depth=std::min(out.min_overlap_depth,o.second);
                }
                deadline(end);out.feasible=true;out.covered_length=covered;
                out.truncation_reason=f.reason;out.failure_reason=FailureReason::NONE;
                if(out.segments.size()==1)out.min_overlap_area=out.min_overlap_depth=0.;
            }catch(const Failure& certificate){out.failure_reason=certificate.reason;out.feasible=false;}
        }
        if(!out.feasible){out.segments.clear();out.complete=false;out.covered_length=0.;out.min_overlap_area=out.min_overlap_depth=0.;}
    }
    out.elapsed_seconds=std::chrono::duration<double>(Clock::now()-began).count();return out;
}
}
