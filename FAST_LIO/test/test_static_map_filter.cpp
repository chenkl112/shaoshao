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
#ifdef SCURM_TEST_PCL
    assert(argc==2);
    moving.snapshot(out);
    assert(pcl::io::savePCDFileBinary(argv[1],out)==0);
    Cloud loaded;assert(pcl::io::loadPCDFile(argv[1],loaded)==0);
    assert(loaded.size()==1&&contains(loaded,10.03));
#endif
    std::cout<<"PASS: temporal confirmation, duplicate/out-of-order timestamps, moving returns, occlusion, free-ray clearing, negative coordinates, bounds, fresh 2D projection, conservative unknown floor"<<std::endl;
}
