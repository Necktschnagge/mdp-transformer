
namespace keywords {

	static constexpr std::string_view states{ "states" };
	static constexpr std::string_view actions{ "actions" };
	static constexpr std::string_view probabilities{ "probabilities" };
	static constexpr std::string_view initial{ "initial" };
	static constexpr std::string_view rewards{ "rewards" };
	static constexpr std::string_view targets{ "targets" };

	static constexpr std::string_view task{ "task" };
	static constexpr std::string_view calc{ "calc" };
	static constexpr std::string_view mode{ "mode" };

	namespace value {
		static constexpr std::string_view classic{ "classic" };
		static constexpr std::string_view crinkle{ "crinkle" };
		static constexpr std::string_view quadratic{ "quadratic" };
		static constexpr std::string_view hVar_approach_percentage_cut{ "hVar-percentage-cut" };
		static constexpr std::string_view hVar_approach{ "hVar-approach" };
	}

	namespace checks {
		static constexpr std::string_view checks{ "checks" };

		static constexpr std::string_view no_unreachable_states{ "no-unreachable-states" };
		static constexpr std::string_view reaching_target_with_probability_1{ "reaching-target-with-probability-1" };
		static constexpr std::string_view no_negative_cycles{ "no-negative-cycles" };
		static constexpr std::string_view ignore_negative_cycles_on_target_states{ "ignore-negative-cycles-on-target-states" };

	}

}