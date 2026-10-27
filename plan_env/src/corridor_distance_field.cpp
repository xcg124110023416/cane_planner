#include <plan_env/corridor_distance_field.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace cane_planner {
namespace {
// Lower envelope of parabolas: exact squared Euclidean transform in O(n).
void transform(const std::vector<double>& input, std::vector<double>& output,
               std::vector<int>& sites, std::vector<double>& limits, int count) {
    int k=0;sites[0]=0;
    limits[0]=-std::numeric_limits<double>::infinity();
    limits[1]=std::numeric_limits<double>::infinity();
    for(int q=1;q<count;++q) {
        double s;
        do {
            const int v=sites[k];
            s=((input[q]+double(q)*q)-(input[v]+double(v)*v))/(2.*(q-v));
            if(s>limits[k])break;
            --k;
        } while(true);
        sites[++k]=q;limits[k]=s;limits[k+1]=std::numeric_limits<double>::infinity();
    }
    k=0;
    for(int q=0;q<count;++q) {
        while(limits[k+1]<q)++k;
        output[q]=double(q-sites[k])*(q-sites[k])+input[sites[k]];
    }
}
}

bool CorridorDistanceField::build(const Eigen::Vector2d& origin, double resolution,
        const Eigen::Vector2i& size, const std::vector<uint8_t>& blocked) {
    distances_.clear();size_.setZero();spacing_=0.;
    if(!origin.allFinite() || !std::isfinite(resolution) || resolution<.01 ||
       (size.array()<=0).any() || resolution*size.maxCoeff()>100. ||
       int64_t(size.x())*size.y()>1000000 ||
       blocked.size()!=static_cast<size_t>(size.x())*size.y())return false;
    origin_=origin;spacing_=.5*resolution;size_=2*size+Eigen::Vector2i::Ones();
    const int width=size_.x(),height=size_.y();
    // Half-bin samples include every original corner, face midpoint and center.
    // The closest point of a whole cell to any sample lies on this lattice, so
    // its transformed distance is exact before interpolation (up to roundoff).
    distances_.assign(static_cast<size_t>(width)*height,1e20);
    for(int y=0;y<size.y();++y)for(int x=0;x<size.x();++x)if(blocked[y*size.x()+x])
        for(int dy=0;dy<=2;++dy)for(int dx=0;dx<=2;++dx)
            distances_[(2*y+dy)*width+2*x+dx]=0.;
    for(int x=0;x<width;++x)distances_[x]=distances_[(height-1)*width+x]=0.;
    for(int y=0;y<height;++y)distances_[y*width]=distances_[y*width+width-1]=0.;

    const int longest=std::max(width,height);
    std::vector<double> input(longest),output(longest),limits(longest+1);
    std::vector<int> sites(longest);
    for(int x=0;x<width;++x) {
        for(int y=0;y<height;++y)input[y]=distances_[y*width+x];
        transform(input,output,sites,limits,height);
        for(int y=0;y<height;++y)distances_[y*width+x]=output[y];
    }
    for(int y=0;y<height;++y) {
        for(int x=0;x<width;++x)input[x]=distances_[y*width+x];
        transform(input,output,sites,limits,width);
        for(int x=0;x<width;++x)distances_[y*width+x]=spacing_*std::sqrt(output[x]);
    }
    return true;
}

double CorridorDistanceField::distance(const Eigen::Vector2d& position) const {
    if(distances_.empty() || !position.allFinite())return std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector2d p=(position-origin_)/spacing_;
    if((p.array()<=0.).any() || (p.array()>=(size_-Eigen::Vector2i::Ones()).cast<double>().array()).any())return 0.;
    const int x=std::min(static_cast<int>(p.x()),size_.x()-2);
    const int y=std::min(static_cast<int>(p.y()),size_.y()-2);
    const double tx=p.x()-x,ty=p.y()-y;
    const size_t i=static_cast<size_t>(y)*size_.x()+x;
    return (1.-ty)*((1.-tx)*distances_[i]+tx*distances_[i+1])+
           ty*((1.-tx)*distances_[i+size_.x()]+tx*distances_[i+size_.x()+1]);
}
} // namespace cane_planner
