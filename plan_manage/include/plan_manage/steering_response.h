#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace cane_planner {

// A delayed sequence of absolute heading cues. Called only at support boundaries;
// waiting for a cue keeps the previously responded heading, without sleeping.
class SteeringResponse {
public:
    struct Cue { uint64_t sequence=0; double issued=0., due=0., yaw=0.; };

    bool setDelay(double seconds) {
        if(!std::isfinite(seconds) || seconds<0.)return false;
        delay_=seconds;return true;
    }
    void reset(double yaw) {
        pending_.clear();desired_=selected_=applied_=Cue();
        selected_.yaw=wrap(yaw);applied_.yaw=selected_.yaw;
        applied_at_=0.;last_epoch_=-1.;
    }
    bool command(double epoch,double actual_yaw,double requested_turn,double& turn) {
        if(!std::isfinite(epoch) || epoch<=0. || epoch<last_epoch_ ||
           !std::isfinite(actual_yaw) || !std::isfinite(requested_turn) ||
           !std::isfinite(epoch+delay_))return false;
        last_epoch_=epoch;
        desired_={desired_.sequence+1,epoch,epoch+delay_,wrap(actual_yaw+requested_turn)};
        pending_.push_back(desired_);
        while(!pending_.empty() && pending_.front().due<=epoch) {
            selected_=pending_.front();pending_.pop_front();
        }
        turn=wrap(selected_.yaw-actual_yaw);return true;
    }
    // Selection is an intention; acknowledge only after the plant actually moves.
    void acknowledge(double epoch) {
        if(selected_.sequence && selected_.sequence!=applied_.sequence) {
            applied_=selected_;applied_at_=epoch;
        }
    }
    double delay() const {return delay_;}
    const Cue& desired() const {return desired_;}
    const Cue& selected() const {return selected_;}
    const Cue& applied() const {return applied_;}
    double appliedAt() const {return applied_at_;}
    std::size_t pending() const {return pending_.size();}
private:
    static double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
    double delay_=.3,last_epoch_=-1.,applied_at_=0.;
    Cue desired_,selected_,applied_;
    std::deque<Cue> pending_;
};
}
