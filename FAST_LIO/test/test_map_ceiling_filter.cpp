#include "map_ceiling_filter.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

#ifdef SCURM_TEST_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
#else
struct Point { float x, y, z, intensity; };
using Cloud = std::vector<Point>;
#endif

Point point(float x, float y, float z, float intensity = 7.f)
{
    Point p;
    p.x = x; p.y = y; p.z = z; p.intensity = intensity;
    return p;
}

int main(int argc, char **argv)
{
    MapCeilingFilter filter;
    for (const double bad : {0., -1., 51., std::numeric_limits<double>::infinity()})
    {
        bool rejected = false;
        try { filter.configure(true, bad); } catch (const std::invalid_argument &) { rejected = true; }
        assert(rejected);
    }
    filter.configure(true, 1.5);
    assert(!filter.initialize({{0., 0., 0.}}, {{0., 0., 0.}}));
    Cloud pending;
    pending.push_back(point(0, 0, 0));
    filter.filter(pending);
    assert(pending.empty());
    assert(filter.initialize({{0., 0., .5}}, {{0., 0., -9.81}}));
    // Reinitializing after the vehicle moves must not move the clipping plane.
    assert(filter.initialize({{0., 0., 50.}}, {{0., 0., -9.81}}));
    Cloud observation;
    observation.push_back(point(0, 0, -.5)); // floor
    observation.push_back(point(1, 0, 1.));  // wall
    observation.push_back(point(1, 0, 2.));  // exact upper boundary
    observation.push_back(point(1, 0, 2.5)); // ceiling
    observation.push_back(point(1, 0, std::numeric_limits<float>::quiet_NaN()));
    Cloud registered = observation, accumulated = observation;
    filter.filter(registered);
    filter.filter(accumulated);
    assert(observation.size() == 5); // LIO observations are unchanged.
    assert(registered.size() == 3 && accumulated.size() == 3);
    assert(accumulated[0].z == -.5 && accumulated[2].z == 2.);
    assert(accumulated[1].intensity == 7.f);
#ifdef SCURM_TEST_PCL
    assert(argc == 2);
    assert(registered.width == 3 && registered.height == 1);
    assert(pcl::io::savePCDFileBinary(argv[1], accumulated) == 0);
    Cloud reloaded;
    assert(pcl::io::loadPCDFile<Point>(argv[1], reloaded) == 0);
    assert(reloaded.size() == 3);
    for (const auto &p : reloaded) assert(filter.keep(p));
#else
    (void)argc; (void)argv;
#endif
    filter.configure(false, 1.5);
    Cloud disabled = observation;
    filter.filter(disabled);
    assert(disabled.size() == observation.size());
    filter.configure(true, 1.5);
    // A tilted initial sensor still clips along gravity, not its local Z axis.
    assert(filter.initialize({{10., 20., 30.}}, {{0., -6.9367175, -6.9367175}}));
    assert(filter.keep(point(10, 20.5, 30.5)));
    assert(!filter.keep(point(10, 21.2, 31.2)));
    std::cout << "PASS: fixed reference, gravity-aware clipping, boundary/floor retention, unchanged observations and exported cloud filtering" << std::endl;
}
