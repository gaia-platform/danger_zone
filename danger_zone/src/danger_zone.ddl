---------------------------------------------
-- Copyright (c) Gaia Platform LLC
-- All rights reserved.
---------------------------------------------

database danger_zone

table zone (
    id uint8 unique,
    objects references object[],
    object_classes references object_class[]
)

-- Represent the type of an object such as: Person, Truck, Sign, etc..
-- Each object class has its own criteria to start logging.
-- For instance, if Person has begin_logging_zone = yellow it means
-- that the logging will be triggered as soon as a Person crosses the
-- yellow zone.
table object_class (
    id string unique,
    objects references object[],
    zone_id uint8,
    begin_logging_zone references zone
        where object_class.zone_id = zone.id
)

table object (
    -- For now this ID is just a merge of class_id + a number (taken as-is from the simulation).
    id string unique,
    class_id string,
    object_class references object_class
        where object.class_id = object_class.id,
    zone_id uint8,
    zone references zone
        where object.zone_id = zone.id
)

table detection (
    d_objects references d_object[],

    -- Identifier of detection frame.
    frame_id string,

    -- Seconds/nanoseconds of detection frame.
    seconds int32,
    nanoseconds uint32,

    processed bool
)

table d_object (
    -- Does not create a real relationship to the object, to reduce contention on insertion/deletion.
    object_id string,

    -- Detection score.
    score float,

    -- Range and direction detected.
    range_id int32,
    direction_id int32,

    -- Position coordinates.
    pos_x float,
    pos_y float,
    pos_z float,

    -- Box dimensions.
    size_x float,
    size_y float,
    size_z float,

    -- Pose coordinates.
    orient_x float,
    orient_y float,
    orient_z float,
    orient_w float,

    -- Detection time, stored redundantly because we cannot easily retrieve the detection for a d_object.
    seconds int32,
    nanoseconds uint32,

    -- Zone, numeric values gets 0 as default value,
    -- which corresponds to zones_t::c_no_zone.
    zone_id uint8,

    detection references detection
)

-- This table tracks the state of our logging requests.
-- If last_log < end_log, we are logging.
-- If last_log >= end_log, we are not logging.
table logging_state (
    -- Indicates the last time we performed logging.
    -- There is no need to log anything before this time.
    last_log_seconds int32,
    last_log_nanoseconds uint32,

    -- Indicates when we should end our logging requests.
    -- Data between last_log and end_log still needs to be logged.
    end_log_seconds int32,
    end_log_nanoseconds uint32
)

---
--- Events
---

table zone_transition_event (
    object_id string,
    from_zone_id uint8,
    to_zone_id uint8,

    -- Detection time.
    seconds int32,
    nanoseconds uint32
)

--
-- Actions
--
-- Gaia rules are executed within a database transaction. If the transaction fail,
-- the rules engine will retry it as many times as specified in the gaia_log.conf
-- (3 times by default). For this reason rules need to be idempotent: their effect
-- on the database must be the same given the same initial database state.
--
-- Idempotency is broken as soon as non-transactional code is mixed with transactional
-- code. Sending ROS messages is an example of non-transactional code. For this reason
-- it is ideal to separate code that deal with the database from the code that deals with
-- external world. You can do this by creating rules which only purpose is to deal with
-- the external world and that are triggered by specific "action" records.
--

table send_trigger_log_action (
    begin_seconds int32,
    begin_nanoseconds uint32,
    end_seconds int32,
    end_nanoseconds uint32
)
