#include "logger.h"
#include "utility.h"
#include "linear_system.h"
#include "mdp_ops.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

#include <nlohmann/json.hpp>


namespace feature_toogle {


}

bool set_contains(const std::set<std::string>& set, const std::string& s) {
	return set.find(s) != set.cend();
}

std::set<std::string> calc_reachable_states(const mdp& m) {

	std::set<std::string> reachable_states;

	std::vector<std::string> to_expand;

	reachable_states.insert(m.initial);
	to_expand.push_back(m.initial);

	while (!to_expand.empty()) {

		const std::string state = to_expand.back();
		to_expand.pop_back();
		auto& transitions = m.probabilities.at(state);

		for (const auto& action_paired_distr : transitions) {
			for (const auto& next_state_paired_prob : action_paired_distr.second) {
				if (next_state_paired_prob.second != rational_type(0)) {
					const auto& next_state{ next_state_paired_prob.first };
					const auto [iter, insertion_took_place] = reachable_states.insert(next_state);
					if (insertion_took_place) {
						to_expand.push_back(next_state);
					}
				}
			}
		}
	}

	return reachable_states;
}

bool check_mdp_constraints_and_simplify(mdp& m) {

	// check for reachable states
	std::set<std::string> reachable_states = calc_reachable_states(m);

	const auto is_reachable{
		[&](const std::string& s) -> bool {
			return reachable_states.find(s) != reachable_states.cend();
		}
	};

	for (const auto& s : m.states) {
		if (reachable_states.find(s) == reachable_states.cend()) {
			standard_logger()->info(std::string("The following state is unreachable and is removed:   ") + s);
		}
	}
	m.states = reachable_states;

	// remove unreachables from target
	std::vector<decltype(m.targets.begin())> targets_to_remove;
	for (auto iter = m.targets.begin(); iter != m.targets.end(); ++iter) {
		if (reachable_states.find(*iter) == reachable_states.cend()) {
			standard_logger()->info(std::string("The following state is an unreachable target state, will be removed:   ") + *iter);
			targets_to_remove.push_back(iter);
		}
	}

	for (const auto& foreign_iter : targets_to_remove) {
		m.targets.erase(foreign_iter);
	}

	// remove unreachables from probability matrix
	for (auto iter = m.probabilities.begin(); iter != m.probabilities.end();) {
		if (reachable_states.find(iter->first) == reachable_states.cend()) {
			auto del{ iter };
			++iter;
			m.probabilities.erase(del);
		}
		else {
			++iter;
		}
	}

	// remove unreachable from next states...
	for (auto& state_paired_transitions : m.probabilities) {
		for (auto& action_paired_distr : state_paired_transitions.second) {
			for (auto iter = action_paired_distr.second.begin(); iter != action_paired_distr.second.end();) {
				if (!is_reachable(iter->first)) {
					auto del{ iter };
					++iter;
					action_paired_distr.second.erase(del);
				}
				else {
					++iter;
				}
			}
		}
	}

	// remove unreachable form rewards
	for (auto iter = m.rewards.begin(); iter != m.rewards.end();) {
		if (reachable_states.find(iter->first) == reachable_states.cend()) {
			auto del{ iter };
			++iter;
			m.rewards.erase(del);
		}
		else {
			++iter;
		}
	}

	// check probability for reaching target = 1
	
	// first set of all states which reach under every scheduler target with positive prob.
	// then we have this ones that can never reach target. -> X
	// find all states that can reach X with positive probability. There should be no such states.

	/*
	std::set<std::string> eventually_target = m.targets;
	bool rerun{ true };
	while (rerun) {
		rerun = false;
		for (auto& state_paired_transitions : m.probabilities) {
			// check if state_paired_transitions.first is eventually_target;
			if (!set_contains(eventually_target, state_paired_transitions.first)) {
				// check: for each action there is a probability 
				for (auto& action_paired_distr : state_paired_transitions.second) {
					for (auto iter = action_paired_distr.second.begin(); iter != action_paired_distr.second.end();) {
						if (!is_reachable(iter->first)) {
							auto del{ iter };
							++iter;
							action_paired_distr.second.erase(del);
						}
						else {
							++iter;
						}
					}
				}
			}
		}
	}
	*/
	
	//check poisitive cycles inside delta_max #####
	
	// check for integer rewards. ######
	
	return true;
}


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



int main(int argc, char* argv[])
{
	init_logger();

	if (argc > 1) {
		//on server
		(void)argv;
		standard_logger()->info(argv[1]);
		standard_logger()->info("Running on server. Doing nothing for now.");
		return 0;
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
	// select a scheduler randomly..
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
