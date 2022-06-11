////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include "zones.hpp"
#include <stdexcept>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_3.h>
#include <CGAL/Homogeneous.h>
#include "danger_zone.hpp"
#include <tf2/LinearMath/Quaternion.h>

#ifdef CGAL_USE_GMP
#include <CGAL/Gmpzf.h>
typedef CGAL::Gmpzf ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif
// use an inexact kernel...
typedef CGAL::Homogeneous<double>                         K;
typedef K::Point_3                                        Point;
// ... and the EXACT traits class based on the inexcat kernel
typedef CGAL::Polytope_distance_d_traits_3<K, ET, double> Traits;
typedef CGAL::Polytope_distance_d<Traits>                 Polytope_distance;

double zones_t::get_range(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

double zones_t::get_range(
        const std::vector<point_3d> &shape1,
        const std::vector<point_3d> &shape2)
{
    // convert shape1 points
    std::vector<Point> Pp(shape1.size());

    for(auto Pv:shape1)
        Pp.push_back(Point(Pv.x,Pv.y,Pv.z));

    auto P = Pp.data();

    // convert shape2 points
    std::vector<Point> Qp(shape2.size());

    for(auto Qv:shape2)
        Qp.push_back(Point(Qv.x,Qv.y,Qv.z));

    auto Q = Qp.data();

    // see https://doc.cgal.org/latest/Polytope_distance_d/index.html
    Polytope_distance pd(P, P+shape1.size(), Q, Q+shape2.size());

    return sqrt( CGAL::to_double (pd.squared_distance_numerator()) /
        CGAL::to_double (pd.squared_distance_denominator()));
}

uint8_t zones_t::get_range_zone_id(double x, double y)
{
    auto distance = get_range(x, y);

    for (auto range_id : c_range_id)
    {
        if (range_id[c_index_radius] > distance)
        {
            return range_id[c_index_zone];
        }
    }

    return c_no_zone;
}

uint8_t zones_t::get_range_zone_id(
    const std::vector<point_3d> &shape1,
    const std::vector<point_3d> &shape2)
{
    auto distance = get_range(shape1, shape2);

    for (auto range_id : c_range_id)
    {
        if (range_id[c_index_radius] > distance)
        {
            return range_id[c_index_zone];
        }
    }

    return c_no_zone;
}

// rotate the cube around the origin, then transpose to position
void get_cube(
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w,
        tf2::Vector3 cube[] )
{
    tf2::Quaternion rotation(orient_x, orient_y, orient_z, orient_w);
    tf2::Vector3 position(pos_x, pos_y, pos_z);

    cube[0] = (tf2::quatRotate(rotation, tf2::Vector3(size_x/2, size_y/2, size_z/2))) + position;
    cube[1] = (tf2::quatRotate(rotation, tf2::Vector3(size_x/2, size_y/2, -size_z/2))) + position;
    cube[2] = (tf2::quatRotate(rotation, tf2::Vector3(size_x/2, -size_y/2, size_z/2))) + position;
    cube[3] = (tf2::quatRotate(rotation, tf2::Vector3(size_x/2, -size_y/2, -size_z/2))) + position;
    cube[4] = (tf2::quatRotate(rotation, tf2::Vector3(-size_x/2, size_y/2, size_z/2))) + position;
    cube[5] = (tf2::quatRotate(rotation, tf2::Vector3(-size_x/2, size_y/2, -size_z/2))) + position;
    cube[6] = (tf2::quatRotate(rotation, tf2::Vector3(-size_x/2, -size_y/2, size_z/2))) + position;
    cube[7] = (tf2::quatRotate(rotation, tf2::Vector3(-size_x/2, -size_y/2, -size_z/2))) + position;
}

uint8_t zones_t::get_range_zone_id(
        double pos_x, double pos_y, double pos_z,
        double size_x, double size_y, double size_z,
        double orient_x, double orient_y, double orient_z, double orient_w )
{
    tf2::Vector3 cube[vertices_in_cube];

    // rotate the cube around the origin, then transpose to position
    get_cube( pos_x, pos_y, pos_z,
        size_x, size_y, size_z,
        orient_x, orient_y, orient_z, orient_w,
        cube );

    Point P[8];
    int index = 0;

    // convert cube to CGAL points
    for(auto vertex:cube)
        P[index++] = Point(vertex.x(), vertex.y(), vertex.z());

    // fetch the ego shape from the node instance
    auto ego_shape = danger_zone_t::get_instance()->get_ego_shape();

    // convert ego to CGAL points
    std::vector<Point> Qp(ego_shape.size());

    for(auto Qv:ego_shape)
        Qp.push_back(Point(Qv.x,Qv.y,Qv.z));

    auto Q = Qp.data();

    // find the distance
    Polytope_distance pd(P, P+8, Q, Q+ego_shape.size());

    auto distance= sqrt( CGAL::to_double (pd.squared_distance_numerator()) /
        CGAL::to_double (pd.squared_distance_denominator()));

    // find the range
    for (auto range_id : c_range_id)
    {
        if (range_id[c_index_radius] > distance)
        {
            return range_id[c_index_zone];
        }
    }

    // return error code if zone not found
    return c_no_zone;
}

double zones_t::get_direction(double z, double x)
{
    return std::atan2(z, x);
}

uint8_t zones_t::get_direction_zone_id(double z, double x)
{
    auto angle = get_direction(z, x);

    for (auto direction_id : c_direction_id)
    {
        if (angle < direction_id[0])
        {
            return direction_id[1];
        }
    }

    return c_default_angle_id;
}

uint8_t zones_t::convert_zone_id_to_simulation_id(uint8_t zone_id)
{
    if (zone_id == c_no_zone)
    {
        return 4;
    }

    return zone_id - 1;
}

std::string zones_t::zone_id_str(uint8_t zone_id)
{
    switch (zone_id)
    {
    case c_green_zone:
        return "green";
    case c_yellow_zone:
        return "yellow";
    case c_red_zone:
        return "red";
    case c_no_zone:
        return "unknown";
    }

    throw std::runtime_error("Invalid zone_id: " + std::to_string(zone_id));
}

bool zones_t::is_object_moving_away(uint8_t from_zone_id, uint8_t to_zone_id)
{
    if (from_zone_id == c_no_zone)
    {
        return false;
    }

    return to_zone_id > from_zone_id;
}
