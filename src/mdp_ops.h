#pragma once

#include "logger.h"
#include "custom_types.h"
#include "const_strings.h"
#include "utility.h"


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




void check_valid_mdp_and_load_mdp_from_json(const nlohmann::json& input, mdp& fill_in);



//#### mdp sanity checks should stop further calculations!

template<class _Modification>
inline mdp unfold(const mdp& m, const _Modification& func, const std::map<std::string, rational_type>& delta_max, std::vector<std::string>& ordered_variables) { // do-check!

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
		if (m.targets.find(expand.original_state_name) != m.targets.cend()) {
			// this is target state
			n.targets.insert(expand.augmented_state_name);
			continue; // do not expand target states. They will be final.
		}

		// here we are at some non-target state...

		for (const auto& choose_action : m.probabilities.at(expand.original_state_name)) {
			const auto& action_name = choose_action.first;
			const auto& distr = choose_action.second;

			rational_type step_reward = rational_type(0);
			try {
				step_reward = m.rewards.at(expand.original_state_name).at(action_name);
			}
			catch (const std::out_of_range&) {
				// ignore, it is 0 else alredy by definition line
			}
			rational_type m_next_rew = expand.accumulated_reward + step_reward;

			n.rewards[expand.augmented_state_name][action_name] = func.func(m_next_rew) - func.func(expand.accumulated_reward); // is always okay.
			// we need to check if we passed threshold + delta_max....

			for (const auto& choose_next_state : distr) {
				const auto& next_state_name = choose_next_state.first;
				const auto& prob = choose_next_state.second;

				std::string new_next_state_name = get_new_state_name(next_state_name, m_next_rew);
				// check if we passed threshold + delta_max....
				if (m_next_rew > func.threshold() + delta_max.at(next_state_name)) { // or current state is alredy in cut mode (se how it is solved in stupid_unfold)
					new_next_state_name = next_state_name;
				}

				// add the n_next_state to the todo-list (if we have not seen it before)
				if (n.states.find(new_next_state_name) == n.states.cend()) {
					n.states.insert(new_next_state_name);
					ordered_variables.push_back(new_next_state_name);
					further_expand.emplace_back(next_state_name, m_next_rew, new_next_state_name);
				};

				n.probabilities[expand.augmented_state_name][action_name][new_next_state_name] = prob;

			}

		}

	}
	return n;
}

class mdp_view {

	const mdp* m;

public:
	mdp_view(const mdp& m) : m(&m) {}

	std::vector<std::string> get_actions_of(const std::string& state) {
		auto result = std::vector<std::string>(m->probabilities.at(state).size());
		std::transform(
			m->probabilities.at(state).cbegin(),
			m->probabilities.at(state).cend(),
			result.begin(),
			[](const auto& pair) {
				return pair.first;
			}
		);
		return result;
	}

};

class scheduler_container {
public:
	using scheduler = std::map<std::string, std::size_t>; // stationary scheduler: maps to each state the index of the action chosen in the state 

	std::map<std::string, std::vector<std::string>> available_actions_per_state;
	scheduler sched;

	/* returns true if no overflow */
	bool operator ++ () noexcept {
		for (auto iter = sched.begin(); iter != sched.end(); ++iter) {
			auto& state{ iter->first };
			auto& action_index{ iter->second };

			if (action_index + 1 < available_actions_per_state[state].size()) { // no overflow.
				action_index++;
				return true;
			}
			action_index = 0;
		}
		return false;
	}


	rational_type number_of_schedulers() noexcept {
		rational_type result{ 1 };
		for (auto pair : available_actions_per_state) {
			auto n = pair.second.empty() ? 1 : pair.second.size();
			result *= rational_type(n);
		}
		return result;
	}

	void init(const mdp& m) {
		mdp_view view = mdp_view(m);

		// fill quick table of available actions
		for (auto state = m.states.cbegin(); state != m.states.cend(); ++state) {
			available_actions_per_state[*state] = view.get_actions_of(*state);
			if (available_actions_per_state[*state].empty() && !set_contains(m.targets, *state)) {
				standard_logger()->error("There is some non target state which has no action enabled");
				throw 0;//### fix this!
			}
		}

		// select deterministically a start node for our optimization: select the first available action everywhere.
		for (auto non_trap_state_paired_actions = available_actions_per_state.cbegin();
			non_trap_state_paired_actions != available_actions_per_state.cend();
			++non_trap_state_paired_actions
			) {
			if (!non_trap_state_paired_actions->second.empty()) {
				sched[non_trap_state_paired_actions->first] = 0;
			}
		}
	}

};

inline mdp stupid_unfold(const mdp& m, const rational_type& cut_level, std::vector<std::string>& ordered_variables, std::map<std::string, std::pair<std::string, rational_type>>& augmentated_state_to_pair) { // do-check!

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

	const auto get_augmented_state_name = [](const std::string& s, const rational_type& r) { return s + "_" + r.numerator().str(); };

	const auto initial_state_name = std::string(get_augmented_state_name(m.initial, rational_type(0)));

	n.states.insert(initial_state_name);
	ordered_variables.push_back(initial_state_name);
	augmentated_state_to_pair[initial_state_name] = std::make_pair(m.initial, rational_type(0));
	n.initial = initial_state_name;

	further_expand.emplace_back(m.initial, rational_type(0), initial_state_name);

	while (!further_expand.empty())
	{
		further_expand_record expand = further_expand.front();
		further_expand.pop_front();

		//check target state
		if (m.targets.find(expand.original_state_name) != m.targets.cend()) {
			// this is target state
			n.targets.insert(expand.augmented_state_name);
			continue; // do not expand target states. They will be final.
		}

		// here we are at some non-target state...

		for (const auto& choose_action : m.probabilities.at(expand.original_state_name)) {
			const auto& action_name = choose_action.first;
			const auto& distr = choose_action.second;

			rational_type step_reward = rational_type(0);
			try {
				step_reward = m.rewards.at(expand.original_state_name).at(action_name);
			}
			catch (const std::out_of_range&) {
				// ignore, it is 0 else alredy by definition line
			}
			rational_type m_next_rew = expand.accumulated_reward + step_reward;

			n.rewards[expand.augmented_state_name][action_name] = step_reward; // func.func(m_next_rew) - func.func(expand.accumulated_reward); // FOR stupid_unfold use the value as it is
			// we need to check if we passed threshold + delta_max....

			for (const auto& choose_next_state : distr) {
				const auto& next_state_name = choose_next_state.first;
				const auto& prob = choose_next_state.second;

				std::string augmented_next_state_name = get_augmented_state_name(next_state_name, m_next_rew);
				// check if we passed threshold + delta_max....
				if (m_next_rew >= cut_level || expand.augmented_state_name == expand.original_state_name) { // enhance this line also in the normal unfold version.
					augmented_next_state_name = next_state_name;
					m_next_rew = cut_level;
				}

				// add the n_next_state to the todo-list (if we have not seen it before)
				if (n.states.find(augmented_next_state_name) == n.states.cend()) {
					n.states.insert(augmented_next_state_name);
					ordered_variables.push_back(augmented_next_state_name);
					augmentated_state_to_pair[augmented_next_state_name] = std::make_pair(next_state_name, m_next_rew);
					further_expand.emplace_back(next_state_name, m_next_rew, augmented_next_state_name);
				};

				n.probabilities[expand.augmented_state_name][action_name][augmented_next_state_name] = prob;

			}

		}

	}
	return n;
}

template<class _Modification>
inline mdp modified_stupid_unfold(const mdp& m, const rational_type& cut_level, std::vector<std::string>& ordered_variables, const _Modification& modify, scheduler_container& cont2) { // do-check!

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

	const auto get_augmented_state_name = [](const std::string& s, const rational_type& r) { return s + "_" + r.numerator().str(); };

	const auto initial_state_name = std::string(get_augmented_state_name(m.initial, rational_type(0)));



	// insert a new initial state: to get an initial negative reward.
	const auto pre_init_name = std::string("___pre___init___");
	const auto the_action{ *m.actions.cbegin() };

	cont2.available_actions_per_state[pre_init_name] = { the_action };
	cont2.sched[pre_init_name] = 0;

	n.initial = pre_init_name;
	n.states.insert(pre_init_name);
	ordered_variables.push_back(pre_init_name);

	n.probabilities[pre_init_name][the_action][initial_state_name] = rational_type(1);
	n.rewards[pre_init_name][the_action] = modify(rational_type(0));


	n.states.insert(initial_state_name);
	ordered_variables.push_back(initial_state_name);

	further_expand.emplace_back(m.initial, rational_type(0), initial_state_name);

	while (!further_expand.empty())
	{
		further_expand_record expand = further_expand.front();
		further_expand.pop_front();

		//check target state
		if (m.targets.find(expand.original_state_name) != m.targets.cend()) {
			// this is target state
			n.targets.insert(expand.augmented_state_name);
			continue; // do not expand target states. They will be final.
		}

		// here we are at some non-target state...

		for (const auto& choose_action : m.probabilities.at(expand.original_state_name)) {
			const auto& action_name = choose_action.first;
			const auto& distr = choose_action.second;

			rational_type step_reward = rational_type(0);
			try {
				step_reward = m.rewards.at(expand.original_state_name).at(action_name);
			}
			catch (const std::out_of_range&) {
				// ignore, it is 0 else alredy by definition line
			}
			rational_type m_next_rew = expand.accumulated_reward + step_reward;

			n.rewards[expand.augmented_state_name][action_name] = modify(m_next_rew) - modify(expand.accumulated_reward); // FOR stupid_unfold use the value as it is
			// we need to check if we passed threshold + delta_max....

			for (const auto& choose_next_state : distr) {
				const auto& next_state_name = choose_next_state.first;
				const auto& prob = choose_next_state.second;

				std::string augmented_next_state_name = get_augmented_state_name(next_state_name, m_next_rew);
				// check if we passed threshold + delta_max....
				if (m_next_rew >= cut_level || expand.augmented_state_name == expand.original_state_name) { // enhance this line also in the normal unfold version.
					augmented_next_state_name = next_state_name;
				}

				// add the n_next_state to the todo-list (if we have not seen it before)
				if (n.states.find(augmented_next_state_name) == n.states.cend()) {
					n.states.insert(augmented_next_state_name);
					ordered_variables.push_back(augmented_next_state_name);
					further_expand.emplace_back(next_state_name, m_next_rew, augmented_next_state_name);
				};

				n.probabilities[expand.augmented_state_name][action_name][augmented_next_state_name] = prob;

			}

		}

	}
	return n;
}


nlohmann::json mdp_to_json(const mdp& m);


/*
return 0 on ignored,
max delta otherwise
-m.negative_loop_delta_threshold() if there is a negative loop
*/
inline std::map<std::string, rational_type> calc_delta_max_state_wise(const mdp& m, bool ignore_target_states, bool error_on_negative_loop) { // do-check!
	// should only check for negative circles

	std::map<std::string, rational_type> result;

	for (const auto& state : m.states) {
		result[state] = rational_type(0);
	}

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
						rational_type update = std::max(
							m.negative_loop_delta_threshold(),
							std::min(result[state], result[next_state_pair.first] + m.rewards.at(state).at(action_tree.first)) // building MDP ensures an entry for every state,action, even if left in json
						);
						if (result[state] != update)
							continue_loop = true;
						result[state] = update;
						if (update <= m.negative_loop_delta_threshold()) {
							if (error_on_negative_loop) {
								throw found_negative_loop("Found negative loop while determining delta max.");
							}
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
