#pragma once

#include "logger.h"
#include "custom_types.h"
#include "const_strings.h"


#include <nlohmann/json.hpp>

#include <set>
#include <map>
#include <list>
#include <string>


template <class IteratorA, class IteratorB>
inline bool sets_disjoint(IteratorA a_begin, IteratorA a_end, IteratorB b_begin, IteratorB b_end) {
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


class mdp {
public:
	std::set<std::string> states;
	std::set<std::string> actions;
	std::map<std::string, std::map<std::string, std::map<std::string, rational_type>>> probabilities;
	std::string initial;
	std::map<std::string, std::map<std::string, rational_type>> rewards;
	std::set<std::string> targets;


	rational_type min_reward() const {
		std::vector<rational_type> values;
		for (const auto& state_paired_rewards : rewards) {
			for (const auto& action_paired_reward : state_paired_rewards.second) {
				values.push_back(action_paired_reward.second);
			}
		}
		if (values.empty())
			return rational_type(0);
		rational_type min{values.front()};
		for (const auto& v : values) {
			if (v < min)
				min = v;
		}
		return min;
	}
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


void check_valid_mdp(const nlohmann::json& input, mdp& fill_in);

class further_expand_record {
public:
	std::string new_state_name;
	std::string old_state_name;
	rational_type accumulated_reward;

	further_expand_record(const std::string& old_state_name, rational_type accumulated_reward, const std::string& new_state_name) :
		new_state_name(new_state_name),
		old_state_name(old_state_name),
		accumulated_reward(accumulated_reward)
	{}

};

//#### mdp sanity checks should stop further calculations!

template<class _Modification>
inline mdp unfold(const mdp& m, const _Modification& func, rational_type threshold, const std::map<std::string, rational_type>& delta_max, std::vector<std::string>& ordered_variables) {

	std::list<further_expand_record> further_expand;
	/*
		* state is already created in n.states
		* todo:
			* check if it is target, if so, add to set of final states, do not expand else do the following
			* add probability distrs + rewards
			* create follow_up for every next_state that is not already created in n.states
	*/

	mdp n;

	n.actions = m.actions;

	const auto get_new_state_name = [](const std::string& s, const rational_type& r) { return s + "_" + r.numerator().str(); };

	const auto initial_state_name = std::string(get_new_state_name(m.initial, rational_type(0)));

	n.states.insert(initial_state_name);
	ordered_variables.push_back(initial_state_name);
	n.initial = initial_state_name;

	further_expand.emplace_back(m.initial, rational_type(0), initial_state_name);

	while (!further_expand.empty())
	{
		further_expand_record expand = further_expand.front();
		further_expand.pop_front();

		//check target state
		if (m.targets.find(expand.old_state_name) != m.targets.cend()) {
			// this is target state
			n.targets.insert(expand.new_state_name);
			continue; // do not expand target states. They will be final.
		}

		// here we are at some non-target state...

		for (const auto& choose_action : m.probabilities.at(expand.old_state_name)) {
			const auto& action_name = choose_action.first;
			const auto& distr = choose_action.second;

			rational_type step_reward = rational_type(0);
			try {
				step_reward = m.rewards.at(expand.old_state_name).at(action_name);
			}
			catch (const std::out_of_range&) {
				// ignore, it is 0 else alredy by definition line
			}
			rational_type m_next_rew = expand.accumulated_reward + step_reward;

			n.rewards[expand.new_state_name][action_name] = func(m_next_rew) - func(expand.accumulated_reward); // is always okay.
			// we need to check if we passed threshold + delta_max....

			for (const auto& choose_next_state : distr) {
				const auto& next_state_name = choose_next_state.first;
				const auto& prob = choose_next_state.second;

				std::string new_next_state_name = get_new_state_name(next_state_name, m_next_rew);
				// check if we passed threshold + delta_max....
				if (m_next_rew > threshold + delta_max.at(next_state_name)) {
					new_next_state_name = next_state_name;
				}

				// add the n_next_state to the todo-list (if we have not seen it before)
				if (n.states.find(new_next_state_name) == n.states.cend()) {
					n.states.insert(new_next_state_name);
					ordered_variables.push_back(new_next_state_name);
					further_expand.emplace_back(next_state_name, m_next_rew, new_next_state_name);
				};

				n.probabilities[expand.new_state_name][action_name][new_next_state_name] = prob;

			}

		}

	}
	return n;
}

nlohmann::json mdp_to_json(const mdp& m);

inline std::map<std::string, rational_type> calc_delta_max_state_wise(const mdp& m, bool ignore_target_states) {

	std::map<std::string, rational_type> result;

	for (const auto& state : m.states) {
		result[state] = rational_type(0);
	}

	if (ignore_target_states) {
		for (const auto& target : m.targets) {
		}
	}

	// We need to check for negative loops here.
	// If and only if we every see a delta of - min_reward * number_of_states, there is some negative loop.
	const rational_type negative_loop_delta_threshold{ rational_type(m.states.size()) * m.min_reward() };

	bool continue_loop = true;
	while (continue_loop) {
		continue_loop = false;
		for (const auto& state : m.states) {
			if (ignore_target_states && set_contains(m.targets, state)) {
				continue;
			}
			try {
				const auto& actions{ m.probabilities.at(state) }; // building MDP ensures an entry for every state, even if left in json

				for (const auto& action_tree : actions) {
					for (const auto& next_state_pair : action_tree.second) {
						rational_type update = std::min(result[state], result[next_state_pair.first] + m.rewards.at(state).at(action_tree.first)); // building MDP ensures an entry for every state,action, even if left in json
						if (result[state] != update)
							continue_loop = true;
						result[state] = update;
						if (update < negative_loop_delta_threshold) {
							throw found_negative_loop("Found negative loop while determining delta max.");
						}
						standard_logger()->trace(std::string("UPDATE delta_m for  >" + state + "<  :" + update.numerator().str() + "/" + update.denominator().str()));
					}
				}
			}
			catch (const std::out_of_range&) {
				throw calc_delta_max_error("Fatal internal error: The MDP that was build internally from json does not meet a constraint that is required for calculating delta max. Probably some out-of-range on a map occureed.");
			}
		}
	}

	for (auto& pair : result) // convert into positive values!
		pair.second *= rational_type(-1);

	return result;
}
