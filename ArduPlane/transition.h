/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include <GCS_MAVLink/GCS_MAVLink.h>

class QuadPlane;
class AP_MotorsMulticopter;
// Transition empty base class
class Transition
{
public:

    Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors) {};

    virtual void update() = 0;

    virtual void VTOL_update() = 0;

    virtual void force_transition_complete() = 0;

    virtual bool complete() const = 0;

    virtual void restart() = 0;

    virtual uint8_t get_log_transition_state() const = 0;

    virtual bool active_frwd() const = 0;

    virtual bool show_vtol_view() const = 0;

    virtual void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd) {};

    virtual bool set_FW_roll_limit(int32_t& roll_limit_cd) { return false; }

    virtual bool allow_update_throttle_mix() const { return true; }

    virtual bool update_yaw_target(float& yaw_target_cd) { return false; }

    virtual MAV_VTOL_STATE get_mav_vtol_state() const = 0;

    virtual bool set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd) { return false; }

    virtual bool allow_weathervane() { return true; }

    // Return true if forward throttle should be allowed for position control, see Q_FWD_THR_USE
    virtual bool allow_vfwd() const { return true; }

    virtual void set_last_fw_pitch(void) {}

    virtual bool allow_stick_mixing() const { return true; }

    virtual bool use_multirotor_control_in_fwd_transition() const { return false; }

protected:

    // references for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

};

// Transition for separate lift thrust quadplanes
class SLT_Transition : public Transition
{
public:

    using Transition::Transition;

    void update() override;

    void VTOL_update() override;

    void force_transition_complete() override;

    bool complete() const override { return transition_state == State::DONE; }

    void restart() override { transition_state = State::AIRSPEED_WAIT; }

    uint8_t get_log_transition_state() const override { return static_cast<uint8_t>(transition_state); }

    bool active_frwd() const override;

    bool show_vtol_view() const override;

    void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd) override;

    bool set_FW_roll_limit(int32_t& roll_limit_cd) override;

    bool allow_update_throttle_mix() const override;

    MAV_VTOL_STATE get_mav_vtol_state() const override;

    bool set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd) override;

    void set_last_fw_pitch(void) override;

    // attitude hold of Q_TRANS_HOLD: the VTOL attitude controller holds the
    // navigation roll and pitch and a soft heading target through the airspeed
    // wait of a forward transition from VTOL, and the surfaces follow its rates
    bool update_yaw_target(float& yaw_target_cd) override;

    bool use_multirotor_control_in_fwd_transition() const override;

protected:

    enum class State {
        AIRSPEED_WAIT = 0,
        TIMER         = 1,
        DONE          = 2,
    } transition_state;

    // true while the attitude hold of Q_TRANS_HOLD runs
    bool attitude_hold_active() const;

    // checks of Q_TRANS_MIN_T and Q_TRANS_LIFT that must pass before a
    // forward transition from VTOL may leave the airspeed wait
    bool completion_allowed(uint32_t now_ms, uint8_t &blocked) const;

    // QTRN log message, 10 Hz during a forward transition from VTOL
    void log_transition(float aspeed, uint8_t blocked);

    // this transition started from a VTOL mode, not from a Q-assist
    bool from_vtol;

    // heading target of the attitude hold, the time it was last updated and
    // the time of the last EKF yaw reset already applied to it
    float hold_yaw_cd;
    uint32_t hold_yaw_ms;
    uint32_t hold_yaw_reset_ms;

    // yaw gain scale and heading error of the attitude hold, for the log
    float hold_yaw_scale = 1.0;
    float hold_yaw_err_cd;

    // a message has been sent for a transition the completion checks held
    bool completion_held_msg;

    uint32_t last_log_ms;

    // timer start for transition
    uint32_t transition_start_ms;
    uint32_t transition_low_airspeed_ms;

    // last throttle value when active
    float last_throttle;

    // time and pitch angle whe last in a vtol or FW control mode
    uint32_t last_fw_mode_ms;
    int32_t last_fw_nav_pitch_cd;

    // tiltrotor tilt angle when airspeed wait transition stage completes
    float airspeed_reached_tilt;

    bool in_forced_transition;

};

