#include <path_searching/human_safety_evaluator.h>

#include <algorithm>
#include <cmath>

namespace cane_planner
{

namespace
{
const double kEpsilon = 1e-9;

bool finite(const double value) { return std::isfinite(value); }
}

CandidateSafety HumanSafetyEvaluator::evaluateCandidate(
    const std::size_t index, const std::vector<Eigen::Vector2d> &points,
    const std::vector<double> &times, const std::vector<double> &api,
    const TimedWalkingCorridor &corridor) const
{
    CandidateSafety result;
    result.sample_index = index;
    result.evaluated = !points.empty() && points.size() == times.size();
    result.first_exit_time = times.empty() ? 0.0 : times.back();
    result.minimum_signed_margin = std::numeric_limits<double>::infinity();
    if (!result.evaluated)
    {
        result.rejection_reason = "INVALID_TRACE";
        result.minimum_signed_margin = -std::numeric_limits<double>::infinity();
        return result;
    }

    const std::size_t api_count = api.size();
    double api_sum = 0.0;
    for (const double value : api)
        api_sum += std::abs(value);
    result.mean_abs_api = api_count == 0 ? 0.0 : api_sum / static_cast<double>(api_count);

    bool covered = true;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        double margin = 0.0;
        if (!corridor.strictPointSafetyAtTime(times[i], points[i], config_.corridor_tolerance,
                                              &margin))
        {
            result.first_exit_time = times[i];
            result.safe = false;
            result.minimum_signed_margin = std::min(result.minimum_signed_margin, margin);
            result.rejection_reason = corridor.segmentAtTime(times[i]) ? "CORRIDOR_OR_DYNAMIC" : "NO_COVERAGE";
            covered = false;
            break;
        }
        result.minimum_signed_margin = std::min(result.minimum_signed_margin, margin);
    }
    if (covered)
    {
        result.safe = true;
        result.first_exit_time = times.back();
        if (!finite(result.minimum_signed_margin))
            result.minimum_signed_margin = 0.0;
    }
    return result;
}

HumanSafetyResult HumanSafetyEvaluator::evaluate(const MpcCandidateSnapshot &snapshot,
                                                 const TimedWalkingCorridor &corridor) const
{
    HumanSafetyResult result;
    result.total_count = snapshot.candidates.size();
    result.coverage_end = corridor.tEnd();
    const double trace_end = snapshot.rollout_start_time + snapshot.rollout_horizon;
    result.evaluation_horizon = std::min(trace_end, result.coverage_end);
    if (snapshot.rollout_horizon <= 0.0 && !snapshot.com_times.empty())
    {
        result.evaluation_horizon = std::min(result.coverage_end, snapshot.com_times.front().empty() ? 0.0 : snapshot.com_times.front().back());
    }

    result.survival_times.clear();
    result.survival_fraction.clear();
    for (const auto &candidate : snapshot.candidates)
    {
        if (candidate.evaluated && candidate.safe)
            ++result.safe_count;
        if (candidate.evaluated && candidate.safe && finite(candidate.mean_abs_api))
            result.J_safe_star = std::min(result.J_safe_star, candidate.mean_abs_api);
    }
    result.native_best_candidate = -1;
    double native_best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < snapshot.native_costs.size(); ++i)
    {
        if (std::isfinite(snapshot.native_costs[i]) &&
            snapshot.native_costs[i] < native_best_cost)
        {
            native_best_cost = snapshot.native_costs[i];
            result.native_best_candidate = static_cast<int>(i);
        }
    }
    result.eta = result.total_count == 0 ? 0.0 : static_cast<double>(result.safe_count) / result.total_count;
    result.valid_evidence = result.total_count > 0;

    std::vector<double> times;
    for (const auto &candidate : snapshot.candidates)
        if (candidate.evaluated) times.push_back(candidate.first_exit_time);
    std::sort(times.begin(), times.end());
    for (const double t : times)
    {
        result.survival_times.push_back(t);
        std::size_t alive = 0;
        for (const auto &candidate : snapshot.candidates)
            if (candidate.evaluated && candidate.first_exit_time + kEpsilon >= t) ++alive;
        result.survival_fraction.push_back(result.total_count == 0 ? 0.0 : static_cast<double>(alive) / result.total_count);
    }
    result.T_autonomy = 0.0;
    if (result.total_count > 0)
    {
        for (std::size_t i = 0; i < result.survival_times.size(); ++i)
            if (result.survival_fraction[i] + kEpsilon >= config_.p_safe)
                result.T_autonomy = std::min(result.evaluation_horizon, result.survival_times[i]);
        if (result.safe_count == result.total_count)
            result.T_autonomy = result.evaluation_horizon;
    }
    std::vector<double> margins;
    margins.reserve(snapshot.candidates.size());
    for (const auto &candidate : snapshot.candidates)
        if (candidate.evaluated && finite(candidate.minimum_signed_margin))
            margins.push_back(candidate.minimum_signed_margin);
    if (!margins.empty())
    {
        std::sort(margins.begin(), margins.end());
        result.best_margin = margins.back();
        result.median_margin = margins[margins.size() / 2];
        for (const auto &candidate : snapshot.candidates)
        {
            if (candidate.evaluated &&
                candidate.minimum_signed_margin == result.best_margin &&
                result.evaluation_horizon > kEpsilon)
            {
                result.best_margin_time_fraction =
                    candidate.margin_min_time / result.evaluation_horizon;
                break;
            }
        }
    }

    std::vector<double> lateral;
    lateral.reserve(snapshot.candidates.size());
    for (const auto &candidate : snapshot.candidates)
        if (candidate.evaluated && finite(candidate.minimum_lateral_margin))
            lateral.push_back(candidate.minimum_lateral_margin);
    if (!lateral.empty())
    {
        std::sort(lateral.begin(), lateral.end());
        result.best_lateral_margin = lateral.back();
        result.median_lateral_margin = lateral[lateral.size() / 2];
    }

    result.selected_candidate = selectGuidanceCandidate(snapshot, result);
    result.robust_selected_candidate = result.selected_candidate;
    const int heading_source =
        result.selected_candidate >= 0 ? result.selected_candidate : result.native_best_candidate;
    if (heading_source >= 0 &&
        static_cast<std::size_t>(heading_source) < snapshot.candidates.size())
    {
        result.selected_heading = headingSetpoint(snapshot, heading_source);
    }
    return result;
}

double HumanSafetyEvaluator::headingSetpoint(const MpcCandidateSnapshot &snapshot,
                                             const int index) const
{
    const std::size_t i = static_cast<std::size_t>(index);
    // Absolute bearing toward a lookahead point on the candidate's own CoM path.
    // `first_step_heading` is only the fallback: it is the heading *after one
    // step*, i.e. the current heading plus one api increment, so using it as the
    // response setpoint makes the error equal to that increment on every frame no
    // matter how far the human has already turned. That is a rate command with no
    // setpoint, and it is what let the response hold a saturated turn until the
    // corridor ran out of free space.
    if (i < snapshot.com_paths.size())
    {
        const auto &path = snapshot.com_paths[i];
        if (path.size() >= 2)
        {
            const double lookahead = std::max(0.0, config_.guide_lookahead_dist);
            double travelled = 0.0;
            std::size_t target = path.size() - 1;
            for (std::size_t k = 1; k < path.size(); ++k)
            {
                travelled += (path[k] - path[k - 1]).norm();
                if (travelled >= lookahead)
                {
                    target = k;
                    break;
                }
            }
            const Eigen::Vector2d chord = path[target] - path.front();
            if (chord.norm() > kEpsilon)
                return std::atan2(chord.y(), chord.x());
        }
    }
    return snapshot.candidates[i].first_step_heading;
}

int HumanSafetyEvaluator::selectGuidanceCandidate(const MpcCandidateSnapshot &snapshot,
                                                  const HumanSafetyResult &) const
{
    int fallback = -1;
    int robust = -1;
    double fallback_cost = std::numeric_limits<double>::infinity();
    double robust_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < snapshot.candidates.size(); ++i)
    {
        const auto &candidate = snapshot.candidates[i];
        if (!candidate.evaluated || !candidate.safe || !finite(candidate.mean_abs_api))
            continue;
        if (candidate.mean_abs_api < fallback_cost ||
            (candidate.mean_abs_api == fallback_cost && static_cast<int>(i) < fallback))
        { fallback = static_cast<int>(i); fallback_cost = candidate.mean_abs_api; }
        if (candidate.minimum_signed_margin + kEpsilon >= config_.min_candidate_margin &&
            (candidate.mean_abs_api < robust_cost ||
             (candidate.mean_abs_api == robust_cost && (robust < 0 || static_cast<int>(i) < robust))))
        { robust = static_cast<int>(i); robust_cost = candidate.mean_abs_api; }
    }
    return robust >= 0 ? robust : fallback;
}

InterventionPolicy::InterventionPolicy(const HumanSafetyConfig &config) : config_(config) {}
void InterventionPolicy::reset() { votes_.clear(); stop_active_ = false; guide_hold_until_ = -std::numeric_limits<double>::infinity(); last_time_ = -std::numeric_limits<double>::infinity(); }
void InterventionPolicy::prune(const double now, const double window)
{
    votes_.erase(std::remove_if(votes_.begin(), votes_.end(), [now, window](const Vote &v) { return now - v.time > window; }), votes_.end());
}
double InterventionPolicy::voteRatio(const double now, const double window, const bool danger) const
{
    if (window <= 0.0) return danger ? 1.0 : 0.0;
    std::size_t total = 0, matching = 0;
    for (const auto &vote : votes_)
        if (now - vote.time <= window) { ++total; if (vote.dangerous == danger) ++matching; }
    return total == 0 ? 0.0 : static_cast<double>(matching) / total;
}
InterventionDecision InterventionPolicy::update(const double now, const HumanSafetyResult &metrics, const InterventionMethod method)
{
    const bool dangerous = metrics.safe_count == 0 || metrics.T_autonomy < config_.T_stop;
    votes_.push_back({now, dangerous});
    prune(now, std::max(config_.stop_entry_window, config_.stop_release_window));
    const double danger_ratio = voteRatio(now, config_.stop_entry_window, true);
    const double recovery_ratio = voteRatio(now, config_.stop_release_window, false);
    if (!stop_active_ && danger_ratio >= config_.stop_entry_vote_ratio &&
        !votes_.empty() && now - votes_.front().time >= config_.stop_entry_window)
    {
        stop_active_ = true;
    }
    else if (stop_active_ && recovery_ratio >= config_.stop_release_vote_ratio &&
             !votes_.empty() && now - votes_.front().time >= config_.stop_release_window)
    {
        stop_active_ = false;
    }

    const bool guide_trigger =
        metrics.T_autonomy < config_.T_guide && metrics.J_safe_star > config_.J_guide;
    if (guide_trigger)
        guide_hold_until_ = now + std::max(0.0, config_.guide_hold_time);

    InterventionDecision decision;
    decision.method = method;
    decision.dangerous = dangerous;
    decision.stop_advice = stop_active_;
    decision.pending_stop = dangerous && !stop_active_;
    decision.danger_vote_ratio = danger_ratio;
    decision.recovery_vote_ratio = recovery_ratio;
    decision.selected_candidate = metrics.selected_candidate;
    if (stop_active_)
    {
        decision.state = InterventionState::STOP_ADVICE;
        decision.stop_reason = "DANGER_WINDOW";
    }
    else if (metrics.safe_count == 0)
    {
        decision.state = InterventionState::CAUTION;
    }
    else if (guide_trigger || guidanceHeld(now))
    {
        // Held for guide_hold_time so the cue outlives the human reaction delay.
        decision.state = InterventionState::GUIDANCE;
        decision.heading_active = true;
    }
    else if (metrics.eta < config_.eta_warn || metrics.T_autonomy < config_.T_warn)
    {
        decision.state = InterventionState::CAUTION;
    }
    else
    {
        decision.state = InterventionState::FREE;
    }
    if (method == InterventionMethod::B0_HUMAN_ONLY)
    {
        decision.state = InterventionState::FREE;
        decision.heading_active = false;
        decision.stop_advice = false;
        decision.pending_stop = false;
    }
    else if (method == InterventionMethod::B2_CONTINUOUS &&
             !stop_active_ && metrics.native_best_candidate >= 0)
    {
        decision.selected_candidate = metrics.native_best_candidate;
        decision.heading_active = true;
        decision.state = InterventionState::GUIDANCE;
    }
    if (stop_active_)
        decision.heading_active = false;
    decision.heading_target = metrics.selected_heading;
    last_time_ = now;
    return decision;
}

const char *toString(const InterventionState state) { switch (state) { case InterventionState::FREE: return "FREE"; case InterventionState::GUIDANCE: return "GUIDANCE"; case InterventionState::CAUTION: return "CAUTION"; case InterventionState::STOP_ADVICE: return "STOP_ADVICE"; } return "UNKNOWN"; }
const char *toString(const InterventionMethod method) { switch (method) { case InterventionMethod::B0_HUMAN_ONLY: return "B0"; case InterventionMethod::B1_BOUNDARY: return "B1"; case InterventionMethod::B2_CONTINUOUS: return "B2"; case InterventionMethod::OURS: return "OURS"; } return "UNKNOWN"; }

} // namespace cane_planner
