#ifndef PATH_SEARCHING_CONVEX_CORRIDOR_H
#define PATH_SEARCHING_CONVEX_CORRIDOR_H
#include <Eigen/Eigen>
#include <vector>
#include <cstdint>
#include <limits>
#include <string>
#include <path_searching/human_cane_footprint.h>
namespace cane_planner {
class ConvexCorridor {
public:
    struct Halfspace { Eigen::Vector2d normal = Eigen::Vector2d::Zero(); double offset = 0.; };
    struct Segment {
        // Sampling metadata: protected seed interval in legacy mode, or seed to
        // coverage exit in adaptive mode. Original-edge coverage is certified separately.
        double s0 = 0., s1 = 0.;
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        std::vector<Halfspace> halfspaces;
        std::vector<Eigen::Vector2d> vertices;
        bool static_feasible = true;
    };
    // Row-major y*width+x. Every nonzero cell (including unknown) is blocked.
    struct Grid {
        Eigen::Vector2d origin = Eigen::Vector2d::Zero();
        double resolution = 0.;
        int width = 0, height = 0;
        std::vector<uint8_t> blocked;
    };
    struct Config {
        double local_radius = 3., clearance = .002;
        double max_segment_length = 1.;
        double max_seconds = .5;
        int iterations = 4, optimizer_iterations = 150, max_regions = 100;
        bool adaptive_sampling = false;
        HumanCaneFootprint::Config footprint;
        double root_yaw = 0.;
    };
    enum class FailureReason {
        NONE, INVALID_INPUT, REFERENCE_OCCUPIED, DEGENERATE_PATH,
        NO_OVERLAP, NO_PROGRESS, NUMERICAL_FAILURE, OUTPUT_CERTIFICATE,
        BUDGET, UNSUPPORTED_SCALE, UNSUPPORTED_BACKEND,
        DYNAMIC_REFERENCE_BLOCKED, DYNAMIC_DATA_INVALID, MAP_CHANGED
    };
    enum class FailureStage {
        NONE, INPUT, REGION_INIT, OBSTACLE_SEPARATION_SOLVE, MVIE_INIT_SLACK,
        MVIE_SOLVE, OUTPUT_CERTIFICATE, COVERAGE, OVERLAP
    };
    struct Diagnostics {
        FailureStage stage = FailureStage::NONE;
        int outer_iteration = -1;  // zero-based, -1 when no inflation iteration ran
        int solver_status = 0;     // raw NLopt status; 0 is NOT a solver result
        bool solver_status_valid = false;
        std::string solver_exception; // empty means no exception; bounded on capture
        // Local-frame center/halfspace slack (metres); constraints are solver units.
        double min_slack = std::numeric_limits<double>::quiet_NaN();
        bool min_slack_valid = false;
        double max_constraint_violation = std::numeric_limits<double>::quiet_NaN();
        bool constraint_violation_valid = false;
    };
    struct Result {
        Diagnostics diagnostics;
        std::vector<Segment> segments;
        bool feasible = false;
        bool complete = false;
        double covered_length = 0.;
        FailureReason truncation_reason = FailureReason::NONE;
        bool has_dynamic_input = false; // static-only snapshot capture must reject
        FailureReason failure_reason = FailureReason::NONE;
        int failure_segment_index = -1;
        double failure_s = 0.;
        Eigen::Vector2d failure_position = Eigen::Vector2d::Zero();
        bool failure_position_valid = false;
        double min_overlap_area = 0., min_overlap_depth = 0., elapsed_seconds = 0.;
        int generated_regions = 0;
    };
    static const char* failureReasonName(FailureReason);
    static const char* failureStageName(FailureStage);
    static std::string diagnosticsText(const Result&);
    static bool contains(const Segment&, const Eigen::Vector2d&, double tol = 1e-6);
    // One common region must contain the entire relative polygon (conservative at joins).
    static bool containsFootprint(const Segment&, const Eigen::Vector2d&,
                                  const std::vector<Eigen::Vector2d>& relative, double tol = 1e-6);
    static double violation(const Segment&, const Eigen::Vector2d&);
    static double polygonArea(const std::vector<Eigen::Vector2d>&);
    // Exact root -> route projection -> original edges. length=0 selects the rest.
    static std::vector<Eigen::Vector2d> referencePath(const std::vector<Eigen::Vector2d>& route,
        size_t index,const Eigen::Vector2d& root,double length);
    using Polygon = std::vector<Eigen::Vector2d>;
    Result buildStatic(const std::vector<Eigen::Vector2d>&, const Grid&,
                       const std::vector<Polygon>& dynamic_polygons = {}) const;
    // Adaptive mode may retain a freshly certified connected prefix when a
    // farther geometric obstruction prevents full coverage. Callers must still
    // validate their entire local trajectory inside the returned regions.
    Result buildPrefix(const std::vector<Eigen::Vector2d>&, const Grid&,
                       const std::vector<Polygon>& dynamic_polygons = {}) const;
    void setConfig(const Config& c) { cfg_ = c; }
    const Config& getConfig() const { return cfg_; }
private:
    Result build(const std::vector<Eigen::Vector2d>&, const Grid&,
                 const std::vector<Polygon>&, bool allow_prefix) const;
    Config cfg_;
};
}
#endif
