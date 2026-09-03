#ifndef PATH_SEARCHING_HUMAN_SAFETY_EVALUATOR_H_
#define PATH_SEARCHING_HUMAN_SAFETY_EVALUATOR_H_

#include <path_searching/timed_walking_corridor.h>

#include <Eigen/Core>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace cane_planner
{

enum class InterventionMethod { B0_HUMAN_ONLY, B1_BOUNDARY, B2_CONTINUOUS, OURS };
enum class InterventionState { FREE, GUIDANCE, CAUTION, STOP_ADVICE };

struct CandidateSafety
{
    std::size_t sample_index = 0;
    bool evaluated = false;
    bool safe = false;
    double first_exit_time = 0.0;
    double minimum_signed_margin = -std::numeric_limits<double>::infinity();
    double mean_abs_api = 0.0;
    double first_step_heading = 0.0;
    std::string rejection_reason;
    // Appended so the positional initialisers in the tests keep their meaning.
    // Width-only margin, plus the rollout time at which the 2-D margin bottomed
    // out; the latter is diagnostics only, telling whether the 2-D minimum comes
    // from a cell's entry/exit face (fraction near 0 or 1) or from a genuine
    // lateral pinch (in between).
    double minimum_lateral_margin = -std::numeric_limits<double>::infinity();
    double margin_min_time = 0.0;
};

struct MpcCandidateSnapshot
{
    std::vector<CandidateSafety> candidates;
    std::vector<std::vector<Eigen::Vector2d>> com_paths;
    std::vector<std::vector<double>> com_times;
    std::vector<std::vector<double>> api_sequences;
    std::vector<double> native_costs;
    std::vector<bool> native_valid;
    double rollout_start_time = 0.0;
    double rollout_horizon = 0.0;

    void clear() { *this = MpcCandidateSnapshot(); }
};

struct HumanSafetyConfig
{
    double corridor_tolerance = 0.02;
    // Count a candidate as safe only if MPPI also found it collision free.
    // The corridor test is blind to the static map, so without this the layer
    // reports every rollout safe while every one of them walks into a wall
    // (ours_pilot17: eta 1.00, state FREE, 200/200 static rejected, frozen).
    // Set false to restore the corridor-only judgement.
    bool require_native_valid = true;
    double p_safe = 0.8;
    double min_candidate_margin = 0.0;
    double eta_warn = 0.25;
    double T_warn = 0.5;
    double T_guide = 0.25;
    double J_guide = 0.15;
    // Once the guidance condition fires, hold the cue for at least this long.
    // Without it a cue that lasts one or two MPC frames is shorter than the
    // human's reaction delay, so the response model discards it and no heading
    // correction is ever applied.
    double guide_hold_time = 1.0;
    // Arc length along the selected candidate's CoM path used to build the
    // absolute heading setpoint. The setpoint must be an absolute bearing, not a
    // per-step increment: a per-step increment never shrinks as the human turns,
    // so the response model keeps asking for another increment every frame and
    // can hold a saturated turn indefinitely (measured: 3.88 full revolutions in
    // ours_pilot9.bag). A chord over ~2-3 steps of path is stable enough to act
    // as a setpoint the human can converge onto.
    double guide_lookahead_dist = 1.0;
    // Weight, per metre, on how far a candidate's lookahead point sits from the
    // corridor centreline when choosing which safe candidate the cue points at.
    //
    // Without it the choice is "least steering", which is measured from the
    // robot's *current* heading and therefore has no anchor: a heading a few
    // degrees off the route makes the straightest candidate the one that
    // continues off the route, the cue confirms it, and the next frame starts
    // further out. Measured on ours_pilot18, the setpoint walked from -91 to
    // -124 deg against a route whose bearing was -90 deg throughout, off a
    // straight route into the obstacle beside it. mean_abs_api is radians and
    // typically 0.01-0.1, so a weight near 1 lets a decimetre of offset outrank
    // straightness and leaves straightness to break near ties. 0 restores the
    // unanchored behaviour.
    double guide_route_weight = 1.0;
    double T_stop = 0.1;
    double stop_entry_window = 0.5;
    double stop_release_window = 1.0;
    double stop_entry_vote_ratio = 0.8;
    double stop_release_vote_ratio = 0.8;
};

struct HumanSafetyResult
{
    std::size_t total_count = 0;
    std::size_t safe_count = 0;
    double eta = 0.0;
    double coverage_end = 0.0;
    double evaluation_horizon = 0.0;
    double T_autonomy = 0.0;
    // Best achievable corridor margin this frame: max over candidates of their
    // worst signed margin (positive inside, negative outside). Graded, unlike
    // safe_count, which is a step function of "is the human still in the tube".
    double best_margin = -std::numeric_limits<double>::infinity();
    double median_margin = -std::numeric_limits<double>::infinity();
    double best_lateral_margin = -std::numeric_limits<double>::infinity();
    double median_lateral_margin = -std::numeric_limits<double>::infinity();
    // Where the best candidate's 2-D margin bottomed out, as a fraction of the
    // evaluation horizon.
    double best_margin_time_fraction = -1.0;
    double J_safe_star = std::numeric_limits<double>::infinity();
    int selected_candidate = -1;
    int robust_selected_candidate = -1;
    double selected_heading = 0.0;
    bool valid_evidence = false;
    int native_best_candidate = -1;
    std::vector<double> survival_times;
    std::vector<double> survival_fraction;
};

struct InterventionDecision
{
    InterventionMethod method = InterventionMethod::OURS;
    InterventionState state = InterventionState::FREE;
    bool heading_active = false;
    double heading_target = 0.0;
    bool stop_advice = false;
    std::string stop_reason;
    int selected_candidate = -1;
    bool dangerous = false;
    bool pending_stop = false;
    double danger_vote_ratio = 0.0;
    double recovery_vote_ratio = 0.0;
};

class HumanSafetyEvaluator
{
public:
    explicit HumanSafetyEvaluator(const HumanSafetyConfig &config = HumanSafetyConfig()) : config_(config) {}
    HumanSafetyResult evaluate(const MpcCandidateSnapshot &snapshot,
                               const TimedWalkingCorridor &corridor) const;
    CandidateSafety evaluateCandidate(std::size_t index,
                                      const std::vector<Eigen::Vector2d> &points,
                                      const std::vector<double> &times,
                                      const std::vector<double> &api,
                                      const TimedWalkingCorridor &corridor) const;
    // True when candidate `index` has a usable lookahead point, which it writes
    // to `point`; the arc length used is guide_lookahead_dist.
    bool lookaheadPoint(const MpcCandidateSnapshot &snapshot, int index,
                        Eigen::Vector2d *point) const;

    // The corridor supplies the anchor for the choice; see guide_route_weight.
    int selectGuidanceCandidate(const MpcCandidateSnapshot &snapshot,
                                const HumanSafetyResult &result,
                                const TimedWalkingCorridor &corridor) const;
    // Absolute heading the human should converge onto, taken from the candidate's
    // own CoM path. See guide_lookahead_dist for why this must not be a per-step
    // increment.
    double headingSetpoint(const MpcCandidateSnapshot &snapshot, int index) const;
    const HumanSafetyConfig &config() const { return config_; }

private:
    HumanSafetyConfig config_;
};

class InterventionPolicy
{
public:
    explicit InterventionPolicy(const HumanSafetyConfig &config = HumanSafetyConfig());
    void reset();
    InterventionDecision update(double now, const HumanSafetyResult &metrics,
                                InterventionMethod method = InterventionMethod::OURS);
    bool stopActive() const { return stop_active_; }

private:
    struct Vote { double time; bool dangerous; };
    bool guidanceHeld(double now) const { return now < guide_hold_until_; }
    double voteRatio(double now, double window, bool danger) const;
    void prune(double now, double window);
    HumanSafetyConfig config_;
    std::vector<Vote> votes_;
    bool stop_active_ = false;
    double guide_hold_until_ = -std::numeric_limits<double>::infinity();
    double last_time_ = -std::numeric_limits<double>::infinity();
};

const char *toString(InterventionState state);
const char *toString(InterventionMethod method);

} // namespace cane_planner

#endif
