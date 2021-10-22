/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "gaia_db.hpp"

#include "gaia/logger.hpp"

#include "zones.hpp"

using namespace gaia::danger_zone;

void initialize_zones()
{
    std::vector zones = {zones_t::c_green_zone, zones_t::c_yellow_zone, zones_t::c_red_zone};

    for (uint8_t zone_id : zones)
    {
        auto zone_it = zone_t::list().where(zone_expr::id == zone_id);

        if (zone_it.begin() == zone_it.end())
        {
            gaia_log::app().info("Creating {} zone with id: {}...", zones_t::zone_id_str(zone_id), zone_id);
            zone_t::insert_row(zone_id);
        }
    }
}

void initialize_object_classes()
{
    // The missing classes are created on the fly, with begin_logging_zone = red,
    // as they are detected by the sensors.
    std::map<std::string, uint8_t> class_to_zone = {
        {"Person", zones_t::c_yellow_zone},
        {"DumpTruck", zones_t::c_red_zone}};

    for (const auto& pair : class_to_zone)
    {
        auto obj_class_it = object_class_t::list().where(object_class_expr::id == pair.first);

        if (obj_class_it.begin() == obj_class_it.end())
        {
            gaia_log::app().info(
                "Creating object class: {}, begin logging zone: {}...",
                pair.first, zones_t::zone_id_str(pair.second));
            object_class_t::insert_row(pair.first.c_str(), pair.second);
        }
    }
}

void initialize_logging_state()
{
    auto logging_state_it = logging_state_t::list();

    // If we already have a record, delete it.
    if (logging_state_it.begin() != logging_state_it.end())
    {
        gaia_log::app().warn("Found an unexpected logging state record! Deleting that record...");
        logging_state_it.begin()->delete_row();
    }

    // Reinsert a default state record.
    gaia_log::app().info("Creating new logging state record...");
    logging_state_t::insert_row(0, 0, 0, 0);
}

gaia::danger_zone::object_class_t get_object_class(const char* class_id)
{
    auto obj_class_it = object_class_t::list().where(
        object_class_expr::id == class_id);

    object_class_t obj_class;

    if (obj_class_it.begin() == obj_class_it.end())
    {
        gaia_log::app().info(
            "Found new object class: {}; its begin logging zone will be set to: {}.",
            class_id, zones_t::zone_id_str(zones_t::c_red_zone));

        obj_class = object_class_t::get(
            object_class_t::insert_row(class_id, zones_t::c_red_zone));
    }
    else
    {
        obj_class = *obj_class_it.begin();
    }

    return obj_class;
}

object_t get_object(const char* object_id, const char* class_id)
{
    auto object_it = object_t::list().where(
        object_expr::id == object_id);

    object_t object;

    if (object_it.begin() == object_it.end())
    {
        gaia_log::app().info("Found new object: {}", object_id);

        // Ensure the object class is created.
        get_object_class(class_id);

        // The object_id is in the form: 'Person (12)'.
        object = object_t::get(
            object_t::insert_row(object_id, class_id, zones_t::c_no_zone));
    }
    else
    {
        object = *object_it.begin();
    }

    return object;
}

bool is_earlier(
    int32_t first_seconds, uint32_t first_nanoseconds,
    int32_t second_seconds, uint32_t second_nanoseconds)
{
    return (first_seconds < second_seconds
        || (first_seconds == second_seconds && first_nanoseconds < second_nanoseconds))
        ? true : false;
}

void check_and_log(
    int32_t current_seconds, uint32_t current_nanoseconds, int32_t logging_window_seconds)
{
    auto logging_state_it = logging_state_t::list();

    if (logging_state_it.begin() == logging_state_it.end())
    {
        gaia_log::app().warn("Logging state record is missing! Cannot perform logging!");
        return;
    }

    auto logging_state = *logging_state_it.begin();

    // If we're not in the process of logging, there we're done.
    if (!is_earlier(
        logging_state.begin_log_seconds(), logging_state.begin_log_nanoseconds(),
        logging_state.end_log_seconds(), logging_state.end_log_nanoseconds()))
    {
        return;
    }

    // Calculate the begin of the next logging window.
    int32_t next_begin_log_seconds = logging_state.begin_log_seconds() + logging_window_seconds;
    uint32_t next_begin_log_nanoseconds = logging_state.begin_log_nanoseconds();

    // If the logging should end up earlier than the next window,
    // then use the time when logging should end.
    if (is_earlier(logging_state.end_log_seconds(), logging_state.end_log_nanoseconds(),
        next_begin_log_seconds, next_begin_log_nanoseconds))
    {
        next_begin_log_seconds = logging_state.end_log_seconds();
        next_begin_log_nanoseconds = logging_state.end_log_nanoseconds();
    }

    // If it's not time to log, exit.
    if (is_earlier(current_seconds, current_nanoseconds,
        next_begin_log_seconds, next_begin_log_nanoseconds))
    {
        return;
    }

    // We can now issue a logging request.
    gaia_log::app().info(
        "Triggering a log request for interval: ({}, {}) - ({}, {})...",
        logging_state.begin_log_seconds(), logging_state.begin_log_nanoseconds(),
        next_begin_log_seconds, next_begin_log_nanoseconds);
    send_trigger_log_action_t::insert_row(
        logging_state.begin_log_seconds(), logging_state.begin_log_nanoseconds(),
        next_begin_log_seconds, next_begin_log_nanoseconds);

    // Update the logging state as well.
    // Our database operations are transactional,
    // so if this update fails, our logging request will fail as well.
    auto logging_state_w = logging_state.writer();
    logging_state_w.begin_log_seconds = next_begin_log_seconds;
    logging_state_w.begin_log_nanoseconds = next_begin_log_nanoseconds;
    logging_state_w.update_row();
}

void update_logging(
    int32_t event_seconds, uint32_t event_nanoseconds,
    int32_t seconds_past, int32_t seconds_forward)
{
    auto logging_state_it = logging_state_t::list();

    if (logging_state_it.begin() == logging_state_it.end())
    {
        gaia_log::app().warn("Logging state record is missing! Cannot perform logging!");
        return;
    }

    auto logging_state = *logging_state_it.begin();

    // If we're not in the process of logging, then set logging state fully.
    if (!is_earlier(
        logging_state.begin_log_seconds(), logging_state.begin_log_nanoseconds(),
        logging_state.end_log_seconds(), logging_state.end_log_nanoseconds()))
    {
        gaia_log::app().info(
            "Initiating a logging interval: ({}, {}) - ({}, {})...",
            event_seconds - seconds_past, event_nanoseconds,
            event_seconds + seconds_forward, event_nanoseconds);
        auto logging_state_w = logging_state.writer();
        logging_state_w.begin_log_seconds = event_seconds - seconds_past;
        logging_state_w.begin_log_nanoseconds = event_nanoseconds;
        logging_state_w.end_log_seconds = event_seconds + seconds_forward;
        logging_state_w.end_log_nanoseconds = event_nanoseconds;
        logging_state_w.update_row();
    }
    // If we're already logging, we only need to update the state
    // if this event would push the end of logging further.
    else if (is_earlier(
        logging_state.end_log_seconds(), logging_state.end_log_nanoseconds(),
        event_seconds + seconds_forward, event_nanoseconds))
    {
        gaia_log::app().info(
            "Extending current logging interval to: ({}, {})...",
            event_seconds + seconds_forward, event_nanoseconds);
        auto logging_state_w = logging_state.writer();
        logging_state_w.end_log_seconds = event_seconds + seconds_forward;
        logging_state_w.end_log_nanoseconds = event_nanoseconds;
        logging_state_w.update_row();
    }
}

void dump_zone(const zone_t& zone)
{
    printf("zone id:                %d\n", zone.id());
}

void dump_object(const object_t& object)
{
    printf("object id:              %s\n", object.id());
    printf("object class id:        %s\n", object.class_id());
    printf("object zone id:         %d\n", object.zone_id());
}

void dump_detection(const detection_t& detection)
{
    printf("detection frame id:     %s\n", detection.frame_id());
    printf("detection seconds:      %d\n", detection.seconds());
    printf("detection nanoseconds:  %d\n", detection.nanoseconds());
    printf("detection processed:    %d\n", static_cast<int>(detection.processed()));
}

void dump_d_object(const d_object_t& d_object)
{
    printf("d_object id:            %s\n", d_object.object_id());
    printf("d_object score:         %f\n", d_object.score());
    printf("d_object range:         %d\n", d_object.range_id());
    printf("d_object direction:     %d\n", d_object.direction_id());
    printf(
        "d_object pos:           x = %f, y = %f, z = %f\n",
        d_object.pos_x(), d_object.pos_y(), d_object.pos_z());
    printf(
        "d_object size:          x = %f, y = %f, z = %f\n",
        d_object.size_x(), d_object.size_y(), d_object.size_z());
    printf(
        "d_object orient:        x = %f, y = %f, z = %f, w = %f\n",
        d_object.orient_x(), d_object.orient_y(), d_object.orient_z(), d_object.orient_w());
    printf("d_object seconds:       %d\n", d_object.seconds());
    printf("d_object nanoseconds:   %d\n", d_object.nanoseconds());
    printf("d_object zone id:       %d\n", d_object.zone_id());
}

void dump_logging_state(const logging_state_t& logging_state)
{
    printf("logging state begin log seconds:        %d\n", logging_state.begin_log_seconds());
    printf("logging state begin log nanoseconds:    %d\n", logging_state.begin_log_nanoseconds());
    printf("logging state end log seconds:          %d\n", logging_state.end_log_seconds());
    printf("logging state end log nanoseconds:      %d\n", logging_state.end_log_nanoseconds());
}

void dump_zone_transition_event(const zone_transition_event_t& zone_transition_event)
{
    printf("zone transition event object id:    %s\n", zone_transition_event.object_id());
    printf("zone transition event from zone id: %d\n", zone_transition_event.from_zone_id());
    printf("zone transition event to zone id:   %d\n", zone_transition_event.to_zone_id());
    printf("zone transition event seconds:      %d\n", zone_transition_event.seconds());
    printf("zone transition event nanoseconds:  %d\n", zone_transition_event.nanoseconds());
}

void dump_zones()
{
    printf("--------------------------------------------------------\n");
    printf("ZONES---------------------------------------------------\n");
    for (const auto& zone : zone_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_zone(zone);
    }
    printf("--------------------------------------------------------\n");
}

void dump_objects()
{
    printf("--------------------------------------------------------\n");
    printf("OBJECTS-------------------------------------------------\n");
    for (const auto& object : object_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_object(object);
    }
    printf("--------------------------------------------------------\n");
}

void dump_detections()
{
    printf("--------------------------------------------------------\n");
    printf("DETECTIONS----------------------------------------------\n");
    for (const auto& detection : detection_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_detection(detection);
    }
    printf("--------------------------------------------------------\n");
}

void dump_d_objects()
{
    printf("--------------------------------------------------------\n");
    printf("D_OBJECTS-----------------------------------------------\n");
    for (const auto& d_object : d_object_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_d_object(d_object);
    }
    printf("--------------------------------------------------------\n");
}

void dump_logging_states()
{
    printf("--------------------------------------------------------\n");
    printf("LOGGING_STATE-------------------------------------------\n");
    for (const auto& logging_state : logging_state_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_logging_state(logging_state);
    }
    printf("--------------------------------------------------------\n");
}

void dump_zone_transition_events()
{
    printf("--------------------------------------------------------\n");
    printf("ZONE_TRANSITIION_EVENTS---------------------------------\n");
    for (const auto& zone_transition_event : zone_transition_event_t::list())
    {
        printf("--------------------------------------------------------\n");
        dump_zone_transition_event(zone_transition_event);
    }
    printf("--------------------------------------------------------\n");
}

void dump_db_state()
{
    printf("\n");
    dump_d_objects();
    dump_detections();
    dump_objects();
    dump_logging_states();
    dump_zone_transition_events();
}

void dump_db()
{
    printf("\n");
    dump_zones();
    dump_db_state();
}

template <class T_table>
void delete_all_rows()
{
    for (auto obj = *T_table::list().begin();
         obj;
         obj = *T_table::list().begin())
    {
        obj.delete_row();
    }
}

void clean_db()
{
    for (auto zone = *zone_t::list().begin();
         zone;
         zone = *zone_t::list().begin())
    {
        zone.objects().clear();
        zone.object_classes().clear();
        zone.delete_row();
    }

    for (auto object_class = *object_class_t::list().begin();
         object_class;
         object_class = *object_class_t::list().begin())
    {
        object_class.objects().clear();
        object_class.delete_row();
    }

    for (auto detection = *detection_t::list().begin();
         detection;
         detection = *detection_t::list().begin())
    {
        detection.d_objects().clear();
        detection.delete_row();
    }

    delete_all_rows<object_t>();
    delete_all_rows<d_object_t>();
    delete_all_rows<logging_state_t>();
    delete_all_rows<zone_transition_event_t>();
    delete_all_rows<send_trigger_log_action_t>();
}
