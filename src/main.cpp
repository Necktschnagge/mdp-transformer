#include "logger.h"
#include "utility.h"
#include "linear_system.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

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


class rational_parse_error : public std::logic_error {

public:

	template <class T>
	rational_parse_error(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw mdp_sanity(message);
	}
};

rational_type string_to_rational_type(const std::string& s) {
	if (s.empty())
		throw rational_parse_error("empty");
	if (s.front() == ' ')
		return string_to_rational_type(s.substr(1));
	if (s.back() == ' ')
		return string_to_rational_type(s.substr(0, s.size() - 1));
	if (s.front() == '(' && s.back() == ')')
		return string_to_rational_type(s.substr(1, s.size() - 2));
	std::string copy{ s };
	int64_t level{ 0 };
	for (auto iter = copy.begin(); iter != copy.end(); ++iter) {

		if (*iter == ')')
			--level;
		if (level < 0)
			throw rational_parse_error("too many ')'");
		if (level > 0)
			*iter = 'x';
		if (*iter == '(')
			++level;
	}
	for (auto iter = copy.rbegin(); iter != copy.rend(); ++iter) {
		if (*iter == '+') {
			return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) + string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
		}
		if (*iter == '-') {
			if (s.size() - 1 - (iter - copy.rbegin()) == 0) { // leading "-"
				return rational_type::int_type(0) - string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
			}
			else {
				return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) - string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
			}
		}
	}
	for (auto iter = copy.rbegin(); iter != copy.rend(); ++iter) {
		if (*iter == '*') {
			return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) * string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
		}
		if (*iter == '/') {
			return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) / string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
		}
	}
	rational_type::int_type value;
	try {
		value = rational_type::int_type(s);
	}
	catch (...) {
		throw rational_parse_error(std::string("Cannot convert \"") + s + "\" into integer");
	}
	return rational_type(value);
}

class mdp {
public:
	std::set<std::string> states;
	std::set<std::string> actions;
	std::map<std::string, std::map<std::string, std::map<std::string, rational_type>>> probabilities;
	std::string initial;
	std::map<std::string, std::map<std::string, rational_type>> rewards;
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

std::map<std::string, rational_type> calc_delta_max(const mdp& m) {


	std::map<std::string, rational_type> result;

	std::map<std::string, bool> needs_update;

	for (const auto& state : m.states) {
		result[state] = rational_type(0);
		needs_update[state] = true;
	}

	for (bool continue_loop = true; continue_loop;) {
		continue_loop = false;
		for (const auto& state : m.states) {
			for (const auto& action_tree : m.probabilities.at(state)) {
				for (const auto& next_state_pair : m.probabilities.at(state).at(action_tree.first)) {
					rational_type update = std::min(result[state], result[next_state_pair.first] + m.rewards.at(state).at(action_tree.first));
					if (result[state] != update)
						continue_loop = true;
					result[state] = update;
					standard_logger()->trace(std::string("UPDATE delta_m for  >" + state + "<  :" + update.numerator().str() + "/" + update.denominator().str()));
				}
			}
			// separate treatment for target states?
		}
	}

	for (auto& pair : result) // convert into positive values!
		pair.second *= rational_type(-1);

	return result;
}

template<class _Modification>
mdp unfold(const mdp& m, const _Modification& func, rational_type threshold, const std::map<std::string, rational_type>& delta_max) {

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

			for (const auto choose_next_state : distr) {
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
					further_expand.emplace_back(next_state_name, m_next_rew, new_next_state_name);
				};

				n.probabilities[expand.new_state_name][action_name][new_next_state_name] = prob;

			}

		}

	}
	return n;
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

	std::map<std::string, rational_type> delta_max = calc_delta_max(m);

	mdp n;

	rational_type threshold{ 12 };
	const auto crinkle =
		[&threshold](const rational_type& x) -> rational_type {
		return (x < threshold) ?
			rational_type(2) * x
			: x + threshold;
	};

	n = unfold(m, crinkle, threshold, delta_max);
	//#### will break if in m state names use underscore

	standard_logger()->info("got unfolded mdp:");
	standard_logger()->info(mdp_to_json(n).dump(3));


	solve_linear_system_dependency_order_optimized(linear_systems::matrix(), linear_systems::rational_vector(), linear_systems::id_vector(), linear_systems::id_vector());///####### only fo debug compile
	//additional checks for our assumptions #####


	// create unfold-mdp



	// find an optimal det memless scheduler


	standard_logger()->info("     DONE     ");
	return 0;
}
