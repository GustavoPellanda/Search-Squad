#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <squad_interfaces/msg/corridor_observation.hpp>
#include <squad_interfaces/msg/fuzzy_steering.hpp>

/*
Two-input, zero-order Sugeno steering for the Scout. Subscribe to the local
corridor_observation and publish corridor_steering, pairing the original scan-time
observation with its angular result. Errors are measured relative to the corridor:
positive lateral error is left of the axis; positive heading error points left.

This node implements the fuzzy calculation only. It does not publish cmd_vel,
regulate forward speed, apply ramps, enable motion or decide corridor completion.
CorridorFollower retains input-age, operating-domain and stop checks.
*/
class FuzzySteering : public rclcpp::Node
{
    public:

    FuzzySteering();

    private:

    using Observation = squad_interfaces::msg::CorridorObservation;
    using Memberships = std::array<double, 5>;
    using RuleStrengths = std::array<double, 25>;

    struct NormalizedErrors {
        double lateral;
        double heading;
    };

    struct FuzzifiedErrors {
        Memberships lateral;
        Memberships heading;
    };

    struct AggregatedOutputs {
        double weighted_sum = 0.0;
        double total_weight = 0.0;
    };

    void read_parameters();
    void observation_callback(Observation::ConstSharedPtr observation);
    bool inputs_are_valid(const Observation & observation) const;
    double calculate_angular_velocity(double lateral_error, double heading_error) const;
    NormalizedErrors normalize_inputs(double lateral_error, double heading_error) const;
    FuzzifiedErrors fuzzify_inputs(const NormalizedErrors & errors) const;
    Memberships calculate_memberships(double normalized_error) const;
    RuleStrengths activate_rules(const FuzzifiedErrors & errors) const;
    AggregatedOutputs aggregate_outputs(const RuleStrengths & strengths) const;
    double defuzzify(const AggregatedOutputs & outputs) const;

    // Shared YAML supplies scales in m/rad, angular limit in rad/s and dimensionless rules:
    double lateral_error_scale_ = 0.0;
    double heading_error_scale_ = 0.0;
    double max_angular_speed_ = 0.0;
    std::vector<double> rule_consequents_;
    std::string base_frame_;

    rclcpp::Subscription<Observation>::SharedPtr observation_subscription_;
    rclcpp::Publisher<squad_interfaces::msg::FuzzySteering>::SharedPtr steering_publisher_;
};


// Evaluate once per observation; navigation continues publishing commands at its own 20 Hz:
FuzzySteering::FuzzySteering() : Node("fuzzy_steering")
{
    base_frame_ = std::string(get_namespace()).substr(1) + "/base_footprint";
    read_parameters();
    steering_publisher_ = create_publisher<squad_interfaces::msg::FuzzySteering>(
        "corridor_steering", 1
    );
    observation_subscription_ = create_subscription<Observation>(
        "corridor_observation", 1,
        std::bind(&FuzzySteering::observation_callback, this, std::placeholders::_1)
    );
}


// Consequents are constant Sugeno outputs. Rows = lateral sets, columns = heading sets;
// both axes use NB (negative big), NS, ZE (zero), PS and PB (positive big), in this order:
void FuzzySteering::read_parameters()
{
    lateral_error_scale_ = declare_parameter("lateral_error_scale", 0.35);
    heading_error_scale_ = declare_parameter("heading_error_scale", 0.524);
    max_angular_speed_ = declare_parameter("max_angular_speed", 0.60);
    rule_consequents_ = declare_parameter<std::vector<double>>("steering_rules", {
         1.00,  1.00,  0.80,  0.35,  0.00,
         1.00,  0.65,  0.35,  0.00, -0.35,
         0.90,  0.50,  0.00, -0.50, -0.90,
         0.35,  0.00, -0.35, -0.65, -1.00,
         0.00, -0.35, -0.80, -1.00, -1.00
    });
    for(double limit : {lateral_error_scale_, heading_error_scale_, max_angular_speed_}) {
        if(!std::isfinite(limit) || limit <= 0.0) {
            throw std::invalid_argument(
                "Fuzzy scales and angular limit must be finite and positive"
            );
        }
    }
    if(max_angular_speed_ > 0.8 || rule_consequents_.size() != 25) {
        throw std::invalid_argument("Invalid fuzzy angular limit or rule count");
    }
    for(double consequent : rule_consequents_) {
        if(!std::isfinite(consequent) || std::abs(consequent) > 1.0) {
            throw std::invalid_argument("Fuzzy consequents must be finite and in [-1, 1]");
        }
    }
}


// Retain the original observation and timestamp, even on invalid input. An invalid
// result explicitly invalidates the follower's old input; silence would hide the failure:
void FuzzySteering::observation_callback(Observation::ConstSharedPtr observation)
{
    squad_interfaces::msg::FuzzySteering result;
    result.observation = *observation;
    result.valid = false;
    result.angular_velocity = 0.0;
    if(inputs_are_valid(*observation)) {
        const double angular = calculate_angular_velocity(
            observation->lateral_error, observation->heading_error
        );
        result.valid = std::isfinite(angular);
        result.angular_velocity = result.valid ? angular : 0.0;
    }
    steering_publisher_->publish(result);
}


// Check the mathematical inputs and their declared frame. This does not authorize
// movement: the follower also checks acquisition/receipt age, walls, clearance and odom:
bool FuzzySteering::inputs_are_valid(const Observation & observation) const
{
    return observation.input_valid && observation.axis_valid &&
        observation.header.frame_id == base_frame_ &&
        std::isfinite(observation.lateral_error) && std::isfinite(observation.heading_error);
}


// Read this method as the complete inference flow. The helpers below implement
// each stage in the same order; no winning-region selection or rounding is used:
double FuzzySteering::calculate_angular_velocity(double lateral_error, double heading_error) const
{
    // Convert m/rad to bounded, dimensionless inputs:
    const NormalizedErrors normalized = normalize_inputs(lateral_error, heading_error);

    // Calculate degrees of membership in every linguistic set:
    const FuzzifiedErrors fuzzified = fuzzify_inputs(normalized);

    // Apply AND to each lateral/heading pair to obtain the 25 rule strengths:
    const RuleStrengths strengths = activate_rules(fuzzified);

    // Combine the weighted constant outputs and their total strength:
    const AggregatedOutputs aggregated = aggregate_outputs(strengths);

    // Defuzzification returns one dimensionless value from all active rules:
    const double normalized_output = defuzzify(aggregated);

    // Scale to body yaw rate: positive turns left, negative turns right:
    return max_angular_speed_ * normalized_output;
}


// Normalization: x = clamp(e_lateral / scale_m, -1, 1), and likewise for heading.
// Clipping defines the outer shoulders; the follower separately enforces operating limits:
FuzzySteering::NormalizedErrors FuzzySteering::normalize_inputs(
    double lateral_error, double heading_error
) const
{
    NormalizedErrors errors;
    errors.lateral = std::clamp(lateral_error / lateral_error_scale_, -1.0, 1.0);
    errors.heading = std::clamp(heading_error / heading_error_scale_, -1.0, 1.0);
    return errors;
}


// Fuzzification describes both inputs independently. A value between two set centres
// belongs partly to both sets; these are matching degrees, not probabilities:
FuzzySteering::FuzzifiedErrors FuzzySteering::fuzzify_inputs(
    const NormalizedErrors & errors
) const
{
    FuzzifiedErrors fuzzified;
    fuzzified.lateral = calculate_memberships(errors.lateral);
    fuzzified.heading = calculate_memberships(errors.heading);
    return fuzzified;
}


// Set centres NB/NS/ZE/PS/PB = [-1, -0.5, 0, 0.5, 1]. Each triangle has half-width
// 0.5: mu(x) = max(0, 1 - |x - centre| / 0.5). Normalized clipping gives shoulders
// at the two ends. Degrees sum to one; x=0.25 gives ZE=0.5 and PS=0.5:
FuzzySteering::Memberships FuzzySteering::calculate_memberships(double normalized_error) const
{
    Memberships degrees{};
    for(std::size_t set = 0; set < degrees.size(); ++set) {
        const double center = -1.0 + 0.5 * set;
        degrees[set] = std::max(0.0, 1.0 - std::abs(normalized_error - center) / 0.5);
    }
    return degrees;
}


// Inference antecedents: "lateral is set i AND heading is set j".
// Product AND gives w_ij = mu_i(x) * mu_j(y), retaining every active rule:
FuzzySteering::RuleStrengths FuzzySteering::activate_rules(const FuzzifiedErrors & errors) const
{
    RuleStrengths strengths{};
    for(std::size_t lateral_set = 0; lateral_set < errors.lateral.size(); ++lateral_set) {
        for(std::size_t heading_set = 0; heading_set < errors.heading.size(); ++heading_set) {
            const std::size_t rule = lateral_set * errors.heading.size() + heading_set;
            strengths[rule] = errors.lateral[lateral_set] * errors.heading[heading_set];
        }
    }
    return strengths;
}


// Aggregation: every rule suggests its constant z_ij. Accumulate sum(w_ij * z_ij)
// and sum(w_ij). For example, aligned ZE suggests 0, while aligned PS suggests -0.35:
FuzzySteering::AggregatedOutputs FuzzySteering::aggregate_outputs(
    const RuleStrengths & strengths
) const
{
    AggregatedOutputs outputs;
    for(std::size_t rule = 0; rule < strengths.size(); ++rule) {
        outputs.weighted_sum += strengths[rule] * rule_consequents_[rule];
        outputs.total_weight += strengths[rule];
    }
    return outputs;
}


// Sugeno weighted-average defuzzification: u = sum(w * z) / sum(w).
// ZE/PS degrees of 0.5 each, with heading ZE=1, produce u=-0.175;
// the 0.60 rad/s output scale then gives omega=-0.105 rad/s before navigation ramps:
double FuzzySteering::defuzzify(const AggregatedOutputs & outputs) const
{
    if(outputs.total_weight <= 0.0 || !std::isfinite(outputs.total_weight)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return outputs.weighted_sum / outputs.total_weight;
}


// Observation callbacks evaluate one complete input/output pair at a time:
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzySteering>());
    rclcpp::shutdown();
    return 0;
}
