#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

struct StaticVoxelKey {
    int x, y, z;
    bool operator==(const StaticVoxelKey &v) const { return x==v.x && y==v.y && z==v.z; }
};
struct StaticVoxelHash {
    size_t operator()(const StaticVoxelKey &v) const {
        return size_t(uint32_t(v.x))*73856093u ^ size_t(uint32_t(v.y))*19349663u ^ size_t(uint32_t(v.z))*83492791u;
    }
};

// Static export only: never use this cloud to hide live collision obstacles.
// Confirmation requires elapsed sensor time AND distinct scans. Previously
// confirmed surfaces survive occlusion; only observed free rays remove them.
template<class Point> class StaticMapFilter {
    struct Cell { Point point; double first, last; unsigned hits=1, misses=0; bool confirmed=false; };
    using Key=StaticVoxelKey;
    using Set=std::unordered_set<Key,StaticVoxelHash>;
    std::unordered_map<Key,Cell,StaticVoxelHash> cells_;
    double last_stamp_=-std::numeric_limits<double>::infinity();
    double last_prune_=-std::numeric_limits<double>::infinity();
    double resolution_=0.15, confirmation_=1.5;
    size_t capacity_=2000000;
    size_t ray_offset_=0;
    Key key(double x,double y,double z) const {
        return {int(std::floor(x/resolution_)),int(std::floor(y/resolution_)),int(std::floor(z/resolution_))};
    }
    bool valid(double x,double y,double z) const {
        return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&
               std::abs(x)<100000&&std::abs(y)<100000&&std::abs(z)<100000;
    }
public:
    explicit StaticMapFilter(double resolution=.15,double confirmation=1.5,size_t capacity=2000000)
      :resolution_(resolution),confirmation_(confirmation),capacity_(capacity) {
        if(!std::isfinite(resolution)||resolution<.05||resolution>1||
           !std::isfinite(confirmation)||confirmation<1||confirmation>10||capacity==0)
            throw std::invalid_argument("Invalid static-map filter settings");
    }
    size_t size() const { return cells_.size(); }
    bool full() const { return cells_.size()>=capacity_; }
    template<class Cloud> bool update(const Cloud &cloud,const std::array<double,3> &origin,double stamp) {
        if(!std::isfinite(stamp)||stamp<=last_stamp_||stamp-last_stamp_<.19||
           !valid(origin[0],origin[1],origin[2])) return false;
        last_stamp_=stamp;
        std::unordered_map<Key,Point,StaticVoxelHash> hits;
        for(const auto &p:cloud) {
            if(!valid(p.x,p.y,p.z))continue;
            double dx=p.x-origin[0],dy=p.y-origin[1],dz=p.z-origin[2];
            if(dx*dx+dy*dy+dz*dz>2500.)continue; // bounded 50 m export range
            hits.emplace(key(p.x,p.y,p.z),p);
        }
        Set free;
        size_t ray_index=0;
        const size_t offset=hits.empty()?0:ray_offset_%hits.size();
        ray_offset_+=8000;
        // Exact voxel traversal: no clearing behind a measured surface, and
        // leave two endpoint voxels as a guard against grazing/noisy returns.
        for(const auto &hit:hits) {
            // Rotate the bounded ray budget so dense scans cannot permanently
            // starve a fixed subset of directions of clearing observations.
            if((ray_index++ + hits.size() - offset)%hits.size()>=8000)continue;
            const auto &p=hit.second;
            const double end[3]={p.x,p.y,p.z};
            Key k=key(origin[0],origin[1],origin[2]);
            int index[3]={k.x,k.y,k.z},step[3];
            double next[3],delta[3];
            for(int a=0;a<3;++a) {
                double d=end[a]-origin[a]; step[a]=(d>0)?1:-1;
                delta[a]=d==0?std::numeric_limits<double>::infinity():resolution_/std::abs(d);
                double edge=(index[a]+(step[a]>0?1:0))*resolution_;
                next[a]=d==0?std::numeric_limits<double>::infinity():(edge-origin[a])/d;
            }
            for(int n=0;n<1800;++n) {
                k={index[0],index[1],index[2]};
                if(std::max({std::abs(k.x-hit.first.x),std::abs(k.y-hit.first.y),std::abs(k.z-hit.first.z)})<=2)break;
                if(!hits.count(k)&&cells_.count(k))free.insert(k);
                double t=std::min({next[0],next[1],next[2]});
                if(t>=1.)break;
                // Advance ties together; corner/edge touches are not free evidence.
                for(int a=0;a<3;++a)if(next[a]<=t+1e-10){index[a]+=step[a];next[a]+=delta[a];}
            }
        }
        for(const auto &k:free) {
            auto it=cells_.find(k);
            if(it!=cells_.end()&&++it->second.misses>=2)cells_.erase(it);
        }
        for(const auto &hit:hits) {
            auto it=cells_.find(hit.first);
            if(it==cells_.end()) {
                if(cells_.size()<capacity_)cells_.emplace(hit.first,Cell{hit.second,stamp,stamp,1,0,false});
                continue;
            }
            auto &c=it->second;
            if(!c.confirmed&&stamp-c.last>.6){c.first=stamp;c.hits=0;}
            c.last=stamp;c.point=hit.second;c.misses=0;++c.hits;
            if(c.hits>=6&&stamp-c.first>=confirmation_)c.confirmed=true;
        }
        if(stamp-last_prune_>=1.) {
            for(auto it=cells_.begin();it!=cells_.end();) {
                if(!it->second.confirmed&&stamp-it->second.last>3.)it=cells_.erase(it);else ++it;
            }
            last_prune_=stamp;
        }
        return true;
    }
    template<class Cloud> void snapshot(Cloud &out) const {
        out.clear();out.reserve(cells_.size());
        for(const auto &item:cells_)if(item.second.confirmed)out.push_back(item.second.point);
    }
};
