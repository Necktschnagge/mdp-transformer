#include "logger.h"
#include "utility.h"
#include "linear_system.h"
#include "mdp_ops.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

#include <nlohmann/json.hpp>


namespace feature_toogle {


}


bool check_mdp_constraints(const mdp& m) {
	//### put in all the requirements here!!!
	try {
		mdp_sanity::check("mdp_is_object",m.actions.size() > 0);


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

std::map<std::string, rational_type> calc_delta_max_state_wise(const mdp& m) {


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

	std::map<std::string, rational_type> delta_max = calc_delta_max_state_wise(m);

	mdp n;

	rational_type threshold{ 12 };
	const auto crinkle =
		[&threshold](const rational_type& x) -> rational_type {
		return (x < threshold) ?
			rational_type(2) * x
			: x + threshold;
	};

	// create unfold-mdp
	n = unfold(m, crinkle, threshold, delta_max);
	//#### will break if in m state names use underscore

	
	standard_logger()->info("got unfolded mdp:");
	standard_logger()->info(mdp_to_json(n).dump(3));

	/// this is only compile check...
	auto rewards = linear_systems::rational_vector();
	solve_linear_system_dependency_order_optimized(linear_systems::matrix(), rewards, linear_systems::id_vector(), linear_systems::id_vector());///####### only fo debug compile
	//additional checks for our assumptions #####


	// find an optimal det memless scheduler
	// 
	//select a scheduler randomly..
	// A:
	// MDP + scheduler -> LGS
	// solve LGS...
	// locally select an optimal scheduler..
	// loop -> A, but
	// if there was no better sched decision found anywhere, stop.

	// check for all nodes with multiple optimal solutions...








	standard_logger()->info("     DONE     ");
	return 0;
}
