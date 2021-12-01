/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#pragma once

#include <cmath>

#include <memory>
#include <string>
#include <vector>

class zones_t
{
public:
    // Note the constants numbering is relevant (see: is_object_moving_away()).
    // and the no_zone must be 0 because that is the default value that the
    // database assigns to numeric values.
    static constexpr uint8_t c_no_zone = 0;
    static constexpr uint8_t c_red_zone = 1;
    static constexpr uint8_t c_yellow_zone = 2;
    static constexpr uint8_t c_green_zone = 3;

    struct Point3d
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        Point3d(double x, double y, double z) : x(x), y(y), z(z)
        {}
    };

public:
    /**
     * Return the zone_id based on the distance from the given coordinates.
     */
    static uint8_t get_range_zone_id(double x, double y);

    /**
     * Return the direction_id based on the distance from the given coordinates.
     */
    static uint8_t get_direction_zone_id(double z, double x);

    /**
     * Tells if an object moving from a zone to another is getting closer or further.
     */
    static bool is_object_moving_away(uint8_t from_zone_id, uint8_t to_zone_id);

    /**
     * In the app we use 0 to denote no_zone because Gaia DB assign 0
     * to numeric values by default. The simulator though uses 0 to denote
     * red area. This function converts from app to simulation id.
     */
    static uint8_t convert_zone_id_to_simulation_id(uint8_t zone_id);

    /**
     * Returns a string representation of a zone_id.
     */
    static std::string zone_id_str(uint8_t zone_id);

private:
    //(pi / 180)
    static constexpr double c_rad_per_deg = 0.0174533;

    static constexpr int c_index_radius = 0;
    static constexpr int c_index_zone = 1;
    static constexpr double c_red_zone_radius = 15.0;
    static constexpr double c_yellow_zone_radius = 30.0;
    static constexpr double c_green_zone_radius = 2000.0;

    static constexpr double c_range_id[3][2] = {
        // The order is important.
        {c_red_zone_radius, c_red_zone},
        {c_yellow_zone_radius, c_yellow_zone},
        {c_green_zone_radius, c_green_zone}};

    static constexpr double c_direction_id[6][2] = {
        {60 * c_rad_per_deg, 0},
        {120 * c_rad_per_deg, 1},
        {180 * c_rad_per_deg, 2},
        {240 * c_rad_per_deg, 3},
        {300 * c_rad_per_deg, 4},
        {360 * c_rad_per_deg, 5}};

    static constexpr double c_default_angle_id = 6;

private:
    /**
     * Find the distance from the sensor to the detected object.
     */
    static double get_range(double x, double y);

    /**
     * Find the shortest distance between two shapes.
     */
    static double get_range(
        const std::vector<Point3d> &shape1, 
        const std::vector<Point3d> &shape2);

    /**
     * Find the direction of the object relative to the sensor.
     */
    static double get_direction(double z, double x);
};
