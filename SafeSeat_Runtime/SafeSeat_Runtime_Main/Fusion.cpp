#include "Fusion.h"


// ============================================================
// SAFESEAT SENSOR FUSION CORE
// STEP 5.2A-1 - FOUNDATION
//
// This file intentionally contains NO final decision logic yet.
//
// Implemented in this mini-step:
// - constructor
// - begin()
// - getReading()
// - all enum -> text helpers
// - a compile-safe update() shell
//
// Next mini-step:
// Step 5.2A-2 will add:
// - sensor health interpretation
// - occupancy evaluation
// - motion evaluation
//
// The Isolation Forest / OCSVM models remain OUTSIDE Fusion.
// ============================================================


// ============================================================
// CONSTRUCTOR
// ============================================================

FusionEngine::FusionEngine()
{
    reading =
        FusionReading{};
}


// ============================================================
// BEGIN
// ============================================================

void FusionEngine::begin()
{
    reading =
        FusionReading{};


    // Fusion starts conservatively.
    //
    // Until enough valid sensor evidence exists, SafeSeat must
    // not call the occupant SAFE merely because inputs are
    // missing or still warming up.
    reading.valid =
        false;


    reading.occupancy =
        FusionOccupancyState::UNKNOWN;


    reading.motion =
        FusionMotionState::UNKNOWN;


    reading.vitals =
        FusionVitalsState::UNKNOWN;


    reading.pressure =
        FusionPressureState::UNKNOWN;


    reading.temperature =
        FusionTemperatureState::UNKNOWN;


    reading.respiration =
        FusionRespirationState::UNKNOWN;


    reading.level =
        FusionLevel::WATCH;


    reading.confidence =
        0.0f;


    reading.triggerCamera =
        false;


    reading.triggerAlert =
        false;


    reading.lastUpdateMillis =
        millis();


    Serial.println();
    Serial.println(
        "[FUSION] Core initialized."
    );

    Serial.println(
        "[FUSION] Step 5.2A-1 foundation active."
    );

    Serial.println(
        "[FUSION] Decision engine not enabled yet."
    );
}


// ============================================================
// UPDATE SHELL
//
// IMPORTANT:
//
// Step 5.2A-1 does NOT classify the sensors yet.
//
// This shell exists so Fusion.cpp is complete and link-safe,
// while preserving a conservative WATCH state.
//
// The input is deliberately accepted now so the same public API
// remains stable as later evaluators are added.
// ============================================================

void FusionEngine::update(
    const FusionInput& input
)
{
    reading.lastUpdateMillis =
        input.timestampMillis
            !=
            0
                ? input.timestampMillis
                : millis();


    // --------------------------------------------------------
    // Reset outputs that must never remain latched accidentally
    // across future update cycles.
    // --------------------------------------------------------

    reading.triggerCamera =
        false;


    reading.triggerAlert =
        false;


    // --------------------------------------------------------
    // Foundation behavior:
    //
    // Do NOT claim SAFE.
    // Do NOT claim EMERGENCY.
    // Do NOT count anomaly evidence yet.
    //
    // Step 5.2A-2 and 5.2A-3 will populate the contextual
    // states and evidence summary.
    // --------------------------------------------------------

    reading.valid =
        false;


    reading.level =
        FusionLevel::WATCH;


    reading.confidence =
        0.0f;
}


// ============================================================
// READING ACCESS
// ============================================================

const FusionReading&
FusionEngine::getReading() const
{
    return reading;
}


// ============================================================
// SENSOR HEALTH TEXT
// ============================================================

const char*
FusionEngine::getSensorHealthText(
    FusionSensorHealth health
)
{
    switch (
        health
    )
    {
        case FusionSensorHealth::UNAVAILABLE:
            return "UNAVAILABLE";


        case FusionSensorHealth::INITIALIZING:
            return "INITIALIZING";


        case FusionSensorHealth::WARMING_UP:
            return "WARMING UP";


        case FusionSensorHealth::VALID:
            return "VALID";


        case FusionSensorHealth::DEGRADED:
            return "DEGRADED";


        case FusionSensorHealth::INVALID:
            return "INVALID";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// OCCUPANCY TEXT
// ============================================================

const char*
FusionEngine::getOccupancyText(
    FusionOccupancyState state
)
{
    switch (
        state
    )
    {
        case FusionOccupancyState::UNKNOWN:
            return "UNKNOWN";


        case FusionOccupancyState::EMPTY:
            return "EMPTY";


        case FusionOccupancyState::OCCUPIED:
            return "OCCUPIED";


        case FusionOccupancyState::CONFLICT:
            return "CONFLICT";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// MOTION TEXT
// ============================================================

const char*
FusionEngine::getMotionText(
    FusionMotionState state
)
{
    switch (
        state
    )
    {
        case FusionMotionState::UNKNOWN:
            return "UNKNOWN";


        case FusionMotionState::STILL:
            return "STILL";


        case FusionMotionState::LOW_MOTION:
            return "LOW MOTION";


        case FusionMotionState::MODERATE_MOTION:
            return "MODERATE MOTION";


        case FusionMotionState::HIGH_MOTION:
            return "HIGH MOTION";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// VITALS TEXT
// ============================================================

const char*
FusionEngine::getVitalsText(
    FusionVitalsState state
)
{
    switch (
        state
    )
    {
        case FusionVitalsState::UNKNOWN:
            return "UNKNOWN";


        case FusionVitalsState::NOT_READY:
            return "NOT READY";


        case FusionVitalsState::NORMAL:
            return "NORMAL";


        case FusionVitalsState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// PRESSURE TEXT
// ============================================================

const char*
FusionEngine::getPressureText(
    FusionPressureState state
)
{
    switch (
        state
    )
    {
        case FusionPressureState::UNKNOWN:
            return "UNKNOWN";


        case FusionPressureState::EMPTY:
            return "EMPTY";


        case FusionPressureState::NORMAL:
            return "NORMAL";


        case FusionPressureState::ASYMMETRIC:
            return "ASYMMETRIC";


        case FusionPressureState::CONTACT_LOSS:
            return "CONTACT LOSS";


        case FusionPressureState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// TEMPERATURE TEXT
// ============================================================

const char*
FusionEngine::getTemperatureText(
    FusionTemperatureState state
)
{
    switch (
        state
    )
    {
        case FusionTemperatureState::UNKNOWN:
            return "UNKNOWN";


        case FusionTemperatureState::INVALID:
            return "INVALID";


        case FusionTemperatureState::STABLE:
            return "STABLE";


        case FusionTemperatureState::CONTEXT_CHANGE:
            return "CONTEXT CHANGE";


        case FusionTemperatureState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// RESPIRATION TEXT
// ============================================================

const char*
FusionEngine::getRespirationText(
    FusionRespirationState state
)
{
    switch (
        state
    )
    {
        case FusionRespirationState::UNKNOWN:
            return "UNKNOWN";


        case FusionRespirationState::NOT_AVAILABLE:
            return "NOT AVAILABLE";


        case FusionRespirationState::NORMAL:
            return "NORMAL";


        case FusionRespirationState::IRREGULAR:
            return "IRREGULAR";


        case FusionRespirationState::NO_BREATH:
            return "NO BREATH";


        case FusionRespirationState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// FINAL LEVEL TEXT
// ============================================================

const char*
FusionEngine::getLevelText(
    FusionLevel level
)
{
    switch (
        level
    )
    {
        case FusionLevel::SAFE:
            return "SAFE";


        case FusionLevel::WATCH:
            return "WATCH";


        case FusionLevel::WARNING:
            return "WARNING";


        case FusionLevel::EMERGENCY:
            return "EMERGENCY";


        default:
            return "WATCH";
    }
}
