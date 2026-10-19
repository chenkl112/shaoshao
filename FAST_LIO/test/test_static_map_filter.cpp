#include "static_map_filter.hpp"
#include "static_map_projection.hpp"
#include <cassert>
#include <iostream>
#include <vector>
#ifdef SCURM_TEST_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
#else
struct Point {float x=0,y=0,z=0,intensity=0;};
using Cloud=std::vector<Point>;
#endif
Point p(float x,float y=.03,float z=.03,float intensity=0){Point v;v.x=x;v.y=y;v.z=z;v.intensity=intensity;return v;}
Cloud cloud(std::initializer_list<Point> list){Cloud c;for(auto &v:list)c.push_back(v);return c;}
bool contains(const Cloud &c,float x){for(auto &v:c)if(std::abs(v.x-x)<.05)return true;return false;}
int main(int argc,char **argv) {
    (void)argc;(void)argv;
    const std::array<double,3> origin={{0,.03,.03}};
    StaticMapFilter<Point> filter;
    Cloud out;
    // Duplicate stamps (including 1 Hz timer republication) never confirm a map.
    auto scan=cloud({p(4.03)});
    for(int i=0;i<100;++i)filter.update(scan,origin,1.);
    filter.snapshot(out);assert(out.empty());
    for(int i=1;i<=8;++i)filter.update(scan,origin,1.+i*.2);
    filter.snapshot(out);assert(contains(out,4.03));
    // Occlusion: observing a foreground object must not erase the wall behind it.
    for(int i=0;i<5;++i)filter.update(cloud({p(2.03)}),origin,3.+i*.2);
    filter.snapshot(out);assert(contains(out,4.03));assert(!contains(out,2.03));
    // Clearing: rays returning farther away remove the previous occupied voxel.
    filter.update(cloud({p(8.03)}),origin,4.2);
    filter.snapshot(out);assert(contains(out,4.03));
    filter.update(cloud({p(8.03)}),origin,4.4);
    filter.snapshot(out);assert(!contains(out,4.03));
    // Moving object spends < confirmation time in each voxel; stationary wall remains.
    StaticMapFilter<Point> moving;
    for(int i=0;i<30;++i)moving.update(cloud({p(10.03,2.03),p(3.03+i*.3,4.03)}),origin,10.+i*.2);
    moving.snapshot(out);assert(out.size()==1);assert(contains(out,10.03));
    // Paused input / empty scans / leaving the FOV do not age out static walls.
    moving.update(Cloud{},origin,100.);moving.snapshot(out);assert(out.size()==1);
    // Discontinuous provisional observations cannot confirm a stopped/restarted stream.
    StaticMapFilter<Point> discontinuous;
    for(int i=0;i<30;++i)discontinuous.update(scan,origin,1.+i);
    discontinuous.snapshot(out);assert(out.empty());
    assert(!discontinuous.update(scan,origin,2.));
    assert(!discontinuous.update(scan,origin,std::numeric_limits<double>::quiet_NaN()));
    // Negative coordinates and diagonal ray ties.
    StaticMapFilter<Point> negative;
    for(int i=0;i<10;++i)negative.update(cloud({p(-3.03,-3.03,-3.03)}),{{0,0,0}},20.+i*.2);
    negative.snapshot(out);assert(out.size()==1);
    for(int i=0;i<2;++i)negative.update(cloud({p(-7.03,-7.03,-7.03)}),{{0,0,0}},23.+i*.2);
    negative.snapshot(out);assert(out.empty());
    // Invalid/too distant returns cannot create voxels; candidate storage is bounded.
    StaticMapFilter<Point> bounded(.15,1.5,2);
    bounded.update(cloud({p(1),p(2),p(3),p(99),p(std::numeric_limits<float>::infinity())}),origin,1.);
    assert(bounded.size()==2);assert(bounded.full());
    bounded.update(Cloud{},origin,5.);assert(bounded.size()==0);
    // 2D output is rebuilt: ground is free, wall is occupied, removed trail vanishes.
    StaticMapProjection projection;
    projection.terrain(cloud({p(1.03,.03,.03),p(2.03,.03,.03),p(3.03,.03,.03)}));
    StaticMapProjection::Grid grid;
    auto cell=[&](double x,double y){return grid.data.at(size_t(std::floor((y-grid.y)/grid.resolution))*grid.width+size_t(std::floor((x-grid.x)/grid.resolution)));};
    scan=cloud({p(1.03),p(2.03),p(3.03),p(2.03,.03,1.03),p(3.03,.03,1.03)});
    assert(projection.project(scan,grid));assert(cell(1.03,.03)==0);assert(cell(2.03,.03)==100);assert(cell(3.03,.03)==100);
    scan=cloud({p(1.03),p(2.03),p(3.03),p(2.03,.03,1.03)});
    assert(projection.project(scan,grid));assert(cell(3.03,.03)==0);assert(cell(2.03,.03)==100);
    // No observed floor: preserve obstacles conservatively, no invented free space.
    scan.push_back(p(5.03,.03,1.03));assert(projection.project(scan,grid));assert(cell(5.03,.03)==100);
    // A physically identical 82 cm passage loses less width on a 5 cm grid.
    // Dense floor observations are supplied explicitly; unobserved cells must
    // stay unknown rather than being filled to make the passage passable.
    StaticMapProjection passage;
    Cloud passage_points;
    for(int i=0;i<=30;++i)passage_points.push_back(p(.025f,-.375f+i*.025f,.03f));
    passage.terrain(passage_points);
    passage_points.push_back(p(.025f,-.41f,.5f));
    passage_points.push_back(p(.025f,.41f,.5f));
    auto free_width=[&](double resolution){
        StaticMapProjection::Grid output;output.resolution=resolution;
        assert(passage.project(passage_points,output));
        auto at=[&](double y){
            auto x=size_t(std::floor((.025-output.x)/resolution));
            auto row=size_t(std::floor((y-output.y)/resolution));
            return output.data.at(row*output.width+x);
        };
        assert(at(-.41)==100&&at(.41)==100); // neither wall may disappear
        assert(at(0)==0);
        size_t free_cells=0;
        const auto x=size_t(std::floor((.025-output.x)/resolution));
        for(unsigned y=0;y<output.height;++y)if(output.data[y*output.width+x]==0)++free_cells;
        assert(std::count(output.data.begin(),output.data.end(),int8_t(-1))>0);
        return free_cells*resolution;
    };
    assert(free_width(.05)>free_width(.15)+.1);
    for(double bad:{0.,-.05,.009,2.,std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::quiet_NaN()}) {
        StaticMapProjection::Grid invalid;invalid.resolution=bad;
        bool rejected=false;
        try{passage.project(passage_points,invalid);}catch(const std::invalid_argument&){rejected=true;}
        assert(rejected);
    }
    // Fine static export retains two distinct surfaces in the same old voxel.
    StaticMapFilter<Point> fine(.05),coarse(.15);
    auto fine_scan=cloud({p(4.06),p(4.11)});
    for(int i=0;i<10;++i){
        fine.update(fine_scan,origin,30.+i*.2);
        coarse.update(fine_scan,origin,30.+i*.2);
    }
    fine.snapshot(out);assert(out.size()==2);
    coarse.snapshot(out);assert(out.size()==1);
    // A fully observed 60 cm corridor remains occupied at both walls. The
    // corrected axis-centred circle (with the original 2 cm padding) has a
    // centreline; the old IMU-centred enclosing circle does not. A 58 cm
    // corridor must remain rejected with that same padding.
    auto corridor=[&](double width,double radius){
        StaticMapProjection map;
        Cloud observations;
        const double middle=.007;
        for(int x=0;x<=40;++x) {
            for(double y=middle-width/2+.005;y<middle+width/2;y+=.005)
                observations.push_back(p(float(.005+x*.005),float(y),.03f));
            observations.push_back(p(float(.005+x*.005),float(middle-width/2),.5f));
            observations.push_back(p(float(.005+x*.005),float(middle+width/2),.5f));
        }
        map.terrain(observations);
        StaticMapProjection::Grid g;g.resolution=.01;
        assert(map.project(observations,g));
        const auto x=unsigned(std::floor((.105-g.x)/g.resolution));
        std::vector<unsigned> walls;
        for(unsigned y=0;y<g.height;++y)if(g.data[y*g.width+x]==100)walls.push_back(y);
        assert(walls.size()==2);
        assert(std::count(g.data.begin(),g.data.end(),int8_t(-1))>0);
        for(unsigned y=0;y<g.height;++y)if(g.data[y*g.width+x]==0) {
            double clearance=1e9;
            for(auto wall:walls)clearance=std::min(clearance,std::abs(double(y)-wall)*g.resolution);
            if(clearance>radius)return true;
        }
        return false;
    };
    assert(corridor(.60,.276011+.02));
    assert(!corridor(.60,.315948+.02));
    assert(!corridor(.58,.276011+.02));
    StaticMapFilter<Point> centimetre(.01),five_cm(.05);
    auto close_surfaces=cloud({p(4.011),p(4.031)});
    for(int i=0;i<10;++i){centimetre.update(close_surfaces,origin,40.+i*.2);five_cm.update(close_surfaces,origin,40.+i*.2);}
    centimetre.snapshot(out);assert(out.size()==2);
    five_cm.snapshot(out);assert(out.size()==1);
#ifdef SCURM_TEST_PCL
    assert(argc==2);
    moving.snapshot(out);
    assert(pcl::io::savePCDFileBinary(argv[1],out)==0);
    Cloud loaded;assert(pcl::io::loadPCDFile(argv[1],loaded)==0);
    assert(loaded.size()==1&&contains(loaded,10.03));
#endif
    std::cout<<"PASS: temporal confirmation, duplicate/out-of-order timestamps, moving returns, occlusion, free-ray clearing, negative coordinates, bounds, fresh 2D projection, conservative unknown floor, narrow passage resolution, fine static export"<<std::endl;
}
