#pragma once
#include "static_map_filter.hpp"
#include <vector>

// Terrain intensity is height above the local floor (terrain_analysis_ext).
// Rebuild the complete grid from the current static snapshot on every update;
// removed 3D trails must not survive in an append-only 2D occupancy map.
class StaticMapProjection {
    struct Ground { double z; };
    std::unordered_map<StaticVoxelKey,Ground,StaticVoxelHash> ground_;
    StaticVoxelKey tile(double x,double y) const {return {int(std::floor(x/.3)),int(std::floor(y/.3)),0};}
    bool floor(double x,double y,double &z) const {
        auto k=tile(x,y);
        for(int radius=0;radius<=1;++radius) {
            double sum=0;int count=0;
            for(int dx=-radius;dx<=radius;++dx)for(int dy=-radius;dy<=radius;++dy) {
                auto it=ground_.find({k.x+dx,k.y+dy,0});
                if(it!=ground_.end()){sum+=it->second.z;++count;}
            }
            if(count){z=sum/count;return true;}
        }
        return false;
    }
public:
    struct Grid {double x=0,y=0,resolution=.05;unsigned width=0,height=0;std::vector<int8_t> data;};
    template<class Cloud> void terrain(const Cloud &cloud) {
        std::unordered_map<StaticVoxelKey,Ground,StaticVoxelHash> frame;
        for(const auto &p:cloud) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
               !std::isfinite(p.intensity)||std::abs(p.x)>100000||std::abs(p.y)>100000||
               p.intensity<0||p.intensity>.1)continue;
            auto k=tile(p.x,p.y);
            double z=p.z-p.intensity;
            auto it=frame.find(k);
            if(it==frame.end()||z<it->second.z)frame[k]={z};
        }
        for(const auto &v:frame)if(ground_.size()<2000000||ground_.count(v.first))ground_[v.first]=v.second;
    }
    template<class Cloud> bool project(const Cloud &cloud,Grid &grid) const {
        if(!std::isfinite(grid.resolution)||grid.resolution<.05||grid.resolution>1.)
            throw std::invalid_argument("Static projected map resolution must be in [0.05, 1.0] m");
        if(cloud.empty()||ground_.empty())return false;
        double minx=1e100,miny=1e100,maxx=-1e100,maxy=-1e100;
        for(const auto &p:cloud) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
            minx=std::min(minx,double(p.x));miny=std::min(miny,double(p.y));
            maxx=std::max(maxx,double(p.x));maxy=std::max(maxy,double(p.y));
        }
        if(minx>maxx)return false;
        grid.x=std::floor(minx/grid.resolution)*grid.resolution-grid.resolution;
        grid.y=std::floor(miny/grid.resolution)*grid.resolution-grid.resolution;
        double w=std::ceil((maxx-grid.x)/grid.resolution)+2,h=std::ceil((maxy-grid.y)/grid.resolution)+2;
        if(w*h>16000000)throw std::runtime_error("Static projected map exceeds 16 million cells");
        grid.width=unsigned(w);grid.height=unsigned(h);grid.data.assign(grid.width*grid.height,-1);
        for(const auto &p:cloud) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
            unsigned x=unsigned(std::floor((p.x-grid.x)/grid.resolution));
            unsigned y=unsigned(std::floor((p.y-grid.y)/grid.resolution));
            if(x>=grid.width||y>=grid.height)continue;
            auto &cell=grid.data[size_t(y)*grid.width+x];
            double z;bool known=floor(p.x,p.y,z);
            if(!known||p.z-z>=.15)cell=100; // unknown floor: keep obstacle conservatively
            else if(p.z-z>=-.15&&cell!=100)cell=0;
        }
        return true;
    }
};
