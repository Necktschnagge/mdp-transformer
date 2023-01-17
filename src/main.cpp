#include "logger.h"

#include "utility.h"

#include <nlohmann/json.hpp>


namespace feature_toogle {


}

namespace keywords {
	static constexpr std::string_view states{ "states" };
	static constexpr std::string_view actions{ "actions" };
	static constexpr std::string_view probabilities{ "probabilities" };
	static constexpr std::string_view initial{ "initial" };
	static constexpr std::string_view rewards{ "rewards" };
	static constexpr std::string_view targets{ "targets" };
}

class mdp {
public:
	std::set<std::string> states;
	std::set<std::string> actions;
	std::map<std::string, std::map<std::string, std::map<std::string, std::string>>> probabilities;
	std::string initial;
	std::map<std::string, std::map<std::string, std::string>> rewards;
	std::set<std::string> targets;

};

class mdp_sanity : public std::logic_error {

public:

	template <class T>
	mdp_sanity(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw mdp_sanity(message);
	}
};

template <class IteratorA, class IteratorB>
bool sets_disjoint(IteratorA a_begin, IteratorA a_end, IteratorB b_begin, IteratorB b_end) {
	while (a_begin != a_end && b_begin != b_end) {
		if (*a_begin < *b_begin) {
			++a_begin;
			continue;
		}
		if (*a_begin > *b_begin) {
			++b_begin;
			continue;
		}
		if (*a_begin == *b_begin)
			return false;
	}
	return true;
}


bool check_valid_mdp(const nlohmann::json& input, mdp& fill_in) {
	try {
		mdp_sanity::check("mdp_is_object", input.is_object());

		// states
		const nlohmann::json::const_iterator i_states = input.find(keywords::states);
		mdp_sanity::check("mdp_has_states", i_states != input.cend());
		mdp_sanity::check("mdp_states_is_array", i_states.value().is_array());
		for (auto iter = i_states.value().cbegin(); iter != i_states.value().cend(); ++iter) {
			mdp_sanity::check("mdp_states_item_is_string", iter->is_string());
			fill_in.states.insert(iter->get<std::string>());
		}
		mdp_sanity::check("mdp_states_not_empty", fill_in.states.size() != 0);
		mdp_sanity::check("mdp_states_do_not_have_duplicates", fill_in.states.size() == i_states.value().size());

		// actions
		const nlohmann::json::const_iterator i_actions = input.find(keywords::actions);
		mdp_sanity::check("mdp_has_actions", i_actions != input.cend());
		mdp_sanity::check("mdp_actions_is_array", i_actions.value().is_array());
		for (auto iter = i_actions.value().cbegin(); iter != i_actions.value().cend(); ++iter) {
			mdp_sanity::check("mdp_actions_item_is_string", iter->is_string());
			fill_in.actions.insert(iter->get<std::string>());
		}
		mdp_sanity::check("mdp_actions_not_empty", fill_in.actions.size() != 0);
		mdp_sanity::check("mdp_actions_do_not_have_duplicates", fill_in.actions.size() == i_actions.value().size());

		// states cap actions = emtyset
		mdp_sanity::check("mdp_states_and_actions_are_disjoint", sets_disjoint(fill_in.states.cbegin(), fill_in.states.cend(), fill_in.actions.cbegin(), fill_in.actions.cend()));

		// probabilities
		const nlohmann::json::const_iterator i_probabilities = input.find(keywords::probabilities);
		mdp_sanity::check("mdp_has_probabilities", i_probabilities != input.cend());
		mdp_sanity::check("mdp_probabilities_is_object", i_probabilities.value().is_object());
		for (auto iter = i_probabilities.value().cbegin(); iter != i_probabilities.value().cend(); ++iter) {
			const std::string available_state = iter.key();
			mdp_sanity::check("mdp_probabilities_item_key_is_state", fill_in.states.find(available_state) != fill_in.states.cend());
			auto& fill_state = fill_in.probabilities[available_state];
			mdp_sanity::check("mdp_probabilities_item_is_object", iter.value().is_object());
			for (auto jter = iter.value().cbegin(); jter != iter.value().cend(); ++jter) {
				const std::string available_action = jter.key();
				mdp_sanity::check("mdp_probabilities_item_item_key_is_action", fill_in.actions.find(available_action) != fill_in.actions.cend());
				mdp_sanity::check("mdp_probabilities_item_item_is_object", jter.value().is_object());
				auto& fill_action = fill_state[available_action];
				for (auto kter = jter.value().cbegin(); kter != jter.value().cend(); ++kter) {
					const std::string next_state = kter.key();
					mdp_sanity::check("mdp_probabilities_item_item_item_key_is_state", fill_in.states.find(next_state) != fill_in.states.cend());
					mdp_sanity::check("mdp_probabilities_item_item_item_is_string", kter.value().is_string());
					auto& fill_next_state = fill_action[next_state];
					// -> ##### todo check that it is a valid rational number!
					fill_next_state = kter.value(); // replace by setting some rational value!
					// ### also check sum of probs...
				}
			}
		}

		// initial state
		const nlohmann::json::const_iterator i_initial = input.find(keywords::initial);
		mdp_sanity::check("mdp_has_initial_state", i_initial != input.cend());
		mdp_sanity::check("mdp_initial_is_string", i_initial.value().is_string());
		fill_in.initial = i_initial.value().get<std::string>();
		mdp_sanity::check("mdp_initial_is_state", fill_in.states.find(fill_in.initial) != fill_in.states.cend());

		// rewards
		const nlohmann::json::const_iterator i_rewards = input.find(keywords::rewards);
		mdp_sanity::check("mdp_has_i_rewards", i_rewards != input.cend());
		mdp_sanity::check("mdp_rewards_is_object", i_rewards.value().is_object());
		for (auto iter = i_rewards.value().cbegin(); iter != i_rewards.value().cend(); ++iter) {
			const std::string available_state = iter.key();
			mdp_sanity::check("mdp_rewards_item_key_is_state", fill_in.states.find(available_state) != fill_in.states.cend());
			auto& fill_state = fill_in.rewards[available_state];
			mdp_sanity::check("mdp_rewards_item_is_object", iter.value().is_object());
			for (auto jter = iter.value().cbegin(); jter != iter.value().cend(); ++jter) {
				const std::string available_action = jter.key();
				mdp_sanity::check("mdp_rewards_item_item_key_is_action", fill_in.actions.find(available_action) != fill_in.actions.cend());
				mdp_sanity::check("mdp_rewards_item_item_is_string", jter.value().is_string());
				auto& fill_action = fill_state[available_action];
				// -> ##### todo check that it is a valid rational number!
				fill_action = fill_action = jter.value(); // replace by setting some rational value!
			}
		}

		// targets
		const nlohmann::json::const_iterator i_targets = input.find(keywords::targets);
		mdp_sanity::check("mdp_has_targets", i_targets != input.cend());
		mdp_sanity::check("mdp_targets_is_array", i_targets.value().is_array());
		for (auto iter = i_targets.value().cbegin(); iter != i_targets.value().cend(); ++iter) {
			mdp_sanity::check("mdp_targets_item_is_string", iter->is_string());
			const auto strval{ iter->get<std::string>() };
			mdp_sanity::check("mdp_targets_item_is_state", fill_in.states.find(strval) != fill_in.states.cend());
			fill_in.targets.insert(strval);
		}
		mdp_sanity::check("mdp_targets_not_empty", fill_in.targets.size() != 0);
		mdp_sanity::check("mdp_targets_do_not_have_duplicates", fill_in.targets.size() == i_targets.value().size());


	}
	catch (mdp_sanity& e) {
		standard_logger()->error(e.what());
		return false;
	}
	return true;
}

int main(int argc, char* argv[])
{
	init_logger();

	if (argc > 1) {
		//on server
		(void)argv;
	}


	// json...
	nlohmann::json input = load_json("../../src/example-mdp.json");

	standard_logger()->trace(input.dump(3));

	mdp m;

	const bool ok{ check_valid_mdp(input, m) };

	if (!ok) {
		standard_logger()->error("invalid MDP");
	}
	else {
		standard_logger()->info("check MDP: ok");
	}


	// do all checks
	/*
	states:
		* is list
		* not empty, finite
		*
	actions:
		* is list
		* not emtpy, finite
		* no common names with states
		*
	probs:
		* is obj
		* for each state as arg & no other args:
			* if there is an entry:
				* is obj
				* for each arg:
					* arg is an action
					* with val:
						* is obj
						* all args are states
						* all values are Q-strings
		*
	initial:
		* is a state
		*
	rewards:
		* is obj
		* for each pair:
			* key is state
			* value is object with for each pair
				* key is action
				* value is reward_value (string)
		*
	targets:
		* is list
		* all elements are states
		* is not empty


		* no other keys in object...
	*/

	//additional checks for our assumptions

	// create unfold-mdp



	standard_logger()->info("     DONE     ");
	return 0;
}
