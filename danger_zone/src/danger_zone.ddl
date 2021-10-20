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
    nano_seconds uint32,

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
    nano_seconds uint32,

    -- Zone, numeric values gets 0 as default value,
    -- which corresponds to zones_t::c_no_zone.
    zone_id uint8,

    detection references detection
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
    nano_seconds uint32
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
    seconds int32,
    nano_seconds uint32,
    seconds_past int32,
    seconds_forward int32
)
