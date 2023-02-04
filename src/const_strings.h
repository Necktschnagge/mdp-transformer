
namespace keywords {

	static constexpr std::string_view states{ "states" };
	static constexpr std::string_view actions{ "actions" };
	static constexpr std::string_view probabilities{ "probabilities" };
	static constexpr std::string_view initial{ "initial" };
	static constexpr std::string_view rewards{ "rewards" };
	static constexpr std::string_view targets{ "targets" };

	static constexpr std::string_view task{ "task" };
	static constexpr std::string_view calc{ "calc" };

	namespace checks {
		static constexpr std::string_view checks{ "checks" };

		static constexpr std::string_view no_unreachable_states{ "no-unreachable-states" };
		static constexpr std::string_view reaching_target_with_probability_1{ "reaching-target-with-probability-1" };
		static constexpr std::string_view only_positive_cycles{ "only-positive-cycles" };
		static constexpr std::string_view ignore_non_positive_cycles_on_target_states{ "ignore-non-positive-cycles-on-target-states" };

	}

}