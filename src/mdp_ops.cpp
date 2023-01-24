#include "mdp_ops.h"

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
				rational_type probs_sum{ 0 };
				for (auto kter = jter.value().cbegin(); kter != jter.value().cend(); ++kter) {
					const std::string next_state = kter.key();
					mdp_sanity::check("mdp_probabilities_item_item_item_key_is_state", fill_in.states.find(next_state) != fill_in.states.cend());
					mdp_sanity::check("mdp_probabilities_item_item_item_is_string", kter.value().is_string());
					auto& transition_probability = fill_action[next_state];
					transition_probability = string_to_rational_type(kter.value());
					mdp_sanity::check("mdp_probabilities_item_item_item_is_not_negative", transition_probability >= rational_type(0));
					probs_sum += transition_probability;
				}
				mdp_sanity::check("mdp_probabilities_item_item_has_probability_sum_1", probs_sum == rational_type(1));
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
				auto& reward = fill_state[available_action];
				reward = string_to_rational_type(jter.value());
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


		* no other keys in object... ### not yet done
		*/

	}
	catch (mdp_sanity& e) {
		standard_logger()->error(e.what());
		return false;
	}
	return true;
}

nlohmann::json mdp_to_json(const mdp& m) {
	nlohmann::json j = nlohmann::json::object();
	j[keywords::states] = m.states;
	j[keywords::actions] = m.actions;
	for (const auto& state_paired_action_state_prob : m.probabilities) {
		for (const auto& action_paired_state_prob : state_paired_action_state_prob.second) {
			for (const auto& state_paired_prob : action_paired_state_prob.second) {
				if (state_paired_prob.second.denominator() == rational_type::int_type(1)) {
					j[keywords::probabilities][state_paired_action_state_prob.first][action_paired_state_prob.first][state_paired_prob.first] = state_paired_prob.second.numerator().str();
				}
				else {
					j[keywords::probabilities][state_paired_action_state_prob.first][action_paired_state_prob.first][state_paired_prob.first] = state_paired_prob.second.numerator().str() + "/" + state_paired_prob.second.denominator().str();
				}
			}
		}
	}
	j[keywords::initial] = m.initial;
	for (const auto& state_paired_action : m.rewards) {
		for (const auto& action_paired_reward : state_paired_action.second) {
			if (action_paired_reward.second.denominator() == rational_type::int_type(1)) {
				j[keywords::rewards][state_paired_action.first][action_paired_reward.first] = action_paired_reward.second.numerator().str();
			}
			else {
				j[keywords::rewards][state_paired_action.first][action_paired_reward.first] = action_paired_reward.second.numerator().str() + "/" + action_paired_reward.second.denominator().str();
			}
		}
	}
	j[keywords::targets] = m.targets;
	return j;
}
