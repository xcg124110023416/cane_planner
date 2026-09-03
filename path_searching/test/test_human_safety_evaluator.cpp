#include <gtest/gtest.h>
#include <path_searching/human_safety_evaluator.h>

using namespace cane_planner;

namespace
{
TimedWalkingCorridor corridor()
{
    TimedWalkingCorridor c;
    TimedWalkingCorridorSegment s;
    s.t_start = 0.0;
    s.t_end = 2.0;
    s.start = Eigen::Vector2d(0.0, 0.0);
    s.end = Eigen::Vector2d(2.0, 0.0);
    s.forward = Eigen::Vector2d::UnitX();
    s.left = Eigen::Vector2d::UnitY();
    s.half_width = 0.5;
    c.segments.push_back(s);
    return c;
}
}

TEST(HumanSafetyEvaluator, StrictHelpersDistinguishCoverageAndSignedMargin)
{
    const TimedWalkingCorridor c = corridor();
    EXPECT_TRUE(c.hasCoverageAtTime(0.5));
    EXPECT_FALSE(c.hasCoverageAtTime(2.0));
    EXPECT_NEAR(0.2, c.signedMarginAtTime(1.0, Eigen::Vector2d(1.0, 0.3)), 1e-9);
    EXPECT_LT(c.signedMarginAtTime(1.0, Eigen::Vector2d(1.0, 0.7)), 0.0);
    EXPECT_FALSE(c.strictPointSafetyAtTime(2.0, Eigen::Vector2d(1.0, 0.0)));
    EXPECT_TRUE(c.strictPointSafetyAtTime(1.0, Eigen::Vector2d(1.0, 0.51), 0.02));
}

TEST(HumanSafetyEvaluator, ComputesMetricsAndFallbackSelection)
{
    HumanSafetyConfig config;
    config.p_safe = 0.5;
    config.min_candidate_margin = 0.15;
    HumanSafetyEvaluator evaluator(config);
    MpcCandidateSnapshot snapshot;
    snapshot.rollout_horizon = 2.0;
    snapshot.candidates = {{0, true, true, 2.0, 0.1, 0.4, 0.2, ""},
                           {1, true, true, 2.0, 0.2, 0.6, 0.4, ""},
                           {2, true, false, 1.0, -0.1, 0.1, 0.1, "bad"}};
    const HumanSafetyResult result = evaluator.evaluate(snapshot, corridor());
    EXPECT_EQ(3u, result.total_count);
    EXPECT_EQ(2u, result.safe_count);
    EXPECT_NEAR(2.0 / 3.0, result.eta, 1e-9);
    EXPECT_EQ(1, result.selected_candidate);
    EXPECT_NEAR(0.4, result.J_safe_star, 1e-9);
}

TEST(InterventionPolicy, RequiresSustainedDangerAndConservativeRecovery)
{
    HumanSafetyConfig config;
    config.stop_entry_window = 1.0;
    config.stop_release_window = 2.0;
    config.stop_entry_vote_ratio = 1.0;
    config.stop_release_vote_ratio = 1.0;
    InterventionPolicy policy(config);
    HumanSafetyResult danger;
    danger.safe_count = 0;
    danger.total_count = 1;
    danger.T_autonomy = 0.0;

    EXPECT_FALSE(policy.update(0.0, danger).stop_advice);
    EXPECT_FALSE(policy.update(0.5, danger).stop_advice);
    EXPECT_TRUE(policy.update(1.0, danger).stop_advice);

    HumanSafetyResult recovery;
    recovery.safe_count = 1;
    recovery.total_count = 1;
    recovery.eta = 1.0;
    recovery.T_autonomy = 2.0;
    recovery.J_safe_star = 0.0;
    EXPECT_TRUE(policy.update(1.5, recovery).stop_advice);
    EXPECT_TRUE(policy.update(2.5, recovery).stop_advice);
    EXPECT_TRUE(policy.update(3.0, recovery).stop_advice);
    EXPECT_FALSE(policy.update(3.5, recovery).stop_advice);
}

TEST(InterventionPolicy, B0RemainsUnassisted)
{
    InterventionPolicy policy;
    HumanSafetyResult result;
    result.safe_count = 1;
    result.total_count = 1;
    result.eta = 1.0;
    result.T_autonomy = 1.0;
    result.selected_candidate = 2;
    result.selected_heading = 0.8;
    const InterventionDecision decision = policy.update(0.0, result, InterventionMethod::B0_HUMAN_ONLY);
    EXPECT_EQ(InterventionState::FREE, decision.state);
    EXPECT_FALSE(decision.heading_active);
    EXPECT_FALSE(decision.stop_advice);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(HumanSafetyEvaluator, AggregatesLateralMarginAndMarginArgmin)
{
    HumanSafetyEvaluator evaluator;
    MpcCandidateSnapshot snapshot;
    snapshot.rollout_horizon = 2.0;

    CandidateSafety a;   // best 2-D margin, pinched right at the cell entry
    a.sample_index = 0;
    a.evaluated = true;
    a.safe = true;
    a.first_exit_time = 2.0;
    a.minimum_signed_margin = 0.00;
    a.minimum_lateral_margin = 0.30;
    a.margin_min_time = 0.05;
    a.mean_abs_api = 0.2;

    CandidateSafety b;   // worse 2-D margin, tighter laterally
    b.sample_index = 1;
    b.evaluated = true;
    b.safe = true;
    b.first_exit_time = 2.0;
    b.minimum_signed_margin = -0.05;
    b.minimum_lateral_margin = 0.10;
    b.margin_min_time = 1.00;
    b.mean_abs_api = 0.3;

    CandidateSafety c;   // not evaluated: must be ignored entirely
    c.sample_index = 2;
    c.evaluated = false;
    c.minimum_signed_margin = 5.0;
    c.minimum_lateral_margin = 5.0;

    snapshot.candidates = {a, b, c};
    const HumanSafetyResult result = evaluator.evaluate(snapshot, TimedWalkingCorridor());

    EXPECT_NEAR(0.00, result.best_margin, 1e-9);
    EXPECT_NEAR(0.30, result.best_lateral_margin, 1e-9);
    EXPECT_GT(result.best_lateral_margin, result.median_lateral_margin - 1e-9);
    // evaluation_horizon comes from the (empty) corridor, so the argmin fraction
    // is only reported when there is a horizon to normalise by.
    EXPECT_LE(result.best_margin_time_fraction, 0.0);
}

// A guidance cue that lasts a single frame is shorter than the human reaction
// delay, so the response model throws it away. The policy must hold the cue.
TEST(InterventionPolicy, HoldsGuidanceCueLongEnoughToBeActedOn)
{
    HumanSafetyConfig config;
    config.T_guide = 4.0;
    config.J_guide = 0.06;
    config.guide_hold_time = 1.0;
    config.stop_entry_window = 5.0;      // keep STOP out of the way
    config.stop_entry_vote_ratio = 1.0;
    InterventionPolicy policy(config);

    HumanSafetyResult calm;
    calm.total_count = 10;
    calm.safe_count = 10;
    calm.T_autonomy = 2.0;
    calm.J_safe_star = 0.01;            // below J_guide: no trigger
    calm.eta = 1.0;

    HumanSafetyResult trigger = calm;
    trigger.J_safe_star = 0.20;         // above J_guide: fires

    EXPECT_FALSE(policy.update(0.0, calm).heading_active);

    // One single triggering frame, then the condition goes away again.
    EXPECT_TRUE(policy.update(0.1, trigger).heading_active);
    for (double t : {0.2, 0.5, 0.9})
    {
        const auto held = policy.update(t, calm);
        EXPECT_TRUE(held.heading_active) << "cue dropped at t=" << t;
        EXPECT_EQ(InterventionState::GUIDANCE, held.state);
    }

    // Past the hold window it releases on its own.
    EXPECT_FALSE(policy.update(1.3, calm).heading_active);

    // STOP still overrides the held cue.
    HumanSafetyResult danger = calm;
    danger.safe_count = 0;
    danger.T_autonomy = 0.0;
    InterventionPolicy strict(config);
    strict.update(0.0, trigger);
    for (double t = 0.1; t <= 0.4; t += 0.1)
        strict.update(t, calm);
    EXPECT_TRUE(strict.update(0.45, calm).heading_active);
}

// The response model turns by compliance * (setpoint - current heading). If the
// setpoint is the candidate's heading *after one step* it always sits one api
// increment ahead of wherever the human currently is, so the error never shrinks
// and the correction never decays -- that is what let the response hold a
// saturated turn for 88 consecutive steps in ours_pilot9.bag. The setpoint has to
// be an absolute bearing taken from the candidate's own path.
TEST(HumanSafetyEvaluator, HeadingSetpointIsAnAbsoluteBearingNotAPerStepIncrement)
{
    HumanSafetyConfig config;
    config.p_safe = 0.5;
    config.min_candidate_margin = 0.0;
    config.guide_lookahead_dist = 1.0;
    HumanSafetyEvaluator evaluator(config);

    MpcCandidateSnapshot snapshot;
    snapshot.rollout_horizon = 2.0;
    // One safe candidate whose CoM path runs due +x. Its first_step_heading is
    // deliberately a nonsense value so the test fails if the fallback is used.
    snapshot.candidates = {{0, true, true, 2.0, 0.3, 0.1, 99.0, ""}};
    snapshot.com_paths = {{Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.5, 0.0),
                           Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.5, 0.0)}};

    const HumanSafetyResult straight = evaluator.evaluate(snapshot, corridor());
    ASSERT_EQ(0, straight.selected_candidate);
    EXPECT_NEAR(0.0, straight.selected_heading, 1e-9);

    // A path bending to +y gives a bearing strictly between the chord angles it
    // spans, and crucially one that does not depend on the current heading.
    snapshot.com_paths = {{Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.5, 0.1),
                           Eigen::Vector2d(1.0, 0.4), Eigen::Vector2d(1.4, 0.9)}};
    const HumanSafetyResult bending = evaluator.evaluate(snapshot, corridor());
    EXPECT_GT(bending.selected_heading, 0.0);
    EXPECT_LT(bending.selected_heading, M_PI_2);

    // With no path recorded there is nothing to take a bearing from, so the
    // per-step heading remains the documented fallback.
    snapshot.com_paths.clear();
    EXPECT_NEAR(99.0, evaluator.evaluate(snapshot, corridor()).selected_heading, 1e-9);
}

// The setpoint must be reachable in one lookahead: a bearing measured over a
// distance shorter than a single step would swing with the LIPM's lateral sway.
TEST(HumanSafetyEvaluator, HeadingSetpointUsesTheWholePathWhenShorterThanTheLookahead)
{
    HumanSafetyConfig config;
    config.p_safe = 0.5;
    config.guide_lookahead_dist = 5.0;   // longer than the path below
    HumanSafetyEvaluator evaluator(config);

    MpcCandidateSnapshot snapshot;
    snapshot.rollout_horizon = 2.0;
    snapshot.candidates = {{0, true, true, 2.0, 0.3, 0.1, 99.0, ""}};
    snapshot.com_paths = {{Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0)}};
    EXPECT_NEAR(M_PI_4, evaluator.evaluate(snapshot, corridor()).selected_heading, 1e-9);
}
