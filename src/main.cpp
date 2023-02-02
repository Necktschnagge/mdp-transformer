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
			//#### check again safety of all ".at(...)"
			try {
				const auto& actions{ m.probabilities.at(state) }; // try is only supposed top catch this!! there are more .at()...

				for (const auto& action_tree : actions) {
					for (const auto& next_state_pair : m.probabilities.at(state).at(action_tree.first)) {
						rational_type update = std::min(result[state], result[next_state_pair.first] + m.rewards.at(state).at(action_tree.first));
						if (result[state] != update)
							continue_loop = true;
						result[state] = update;
						standard_logger()->trace(std::string("UPDATE delta_m for  >" + state + "<  :" + update.numerator().str() + "/" + update.denominator().str()));
					}
				}
			}
			catch (const std::out_of_range&) {

			}
			// separate treatment for target states?
		}
	}

	for (auto& pair : result) // convert into positive values!
		pair.second *= rational_type(-1);

	return result;
}

class mdp_view {

	mdp* m;

public:
	mdp_view(mdp& m) : m(&m) {}

	std::vector<std::string> get_actions_of(const std::string& state) {
		auto result = std::vector<std::string>(m->probabilities[state].size());
		std::transform(
			m->probabilities[state].cbegin(),
			m->probabilities[state].cend(),
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
	using scheduler = std::map<std::string, std::size_t>;

	scheduler sched;
	std::map<std::string, std::vector<std::string>> available_actions_per_state;

};

std::size_t get_index(const std::vector<std::string>& str_vec, const std::string& s) {
	for (std::size_t i{ 0 }; i < str_vec.size(); ++i) {
		if (str_vec[i] == s) {
			return i;
		}
	}
	throw std::logic_error("String not contained in string vector.");
}

void optimize_scheduler(mdp& m, const std::vector<std::string>& ordered_variables) {
	scheduler_container cont;

	mdp_view view = mdp_view(m);

	// fill quick table of available actions
	for (auto state = m.states.cbegin(); state != m.states.cend(); ++state) {
		cont.available_actions_per_state[*state] = view.get_actions_of(*state);
		if (cont.available_actions_per_state[*state].empty() && !set_contains(m.targets, *state)) {
			standard_logger()->error("There is some non target state which has no action enabled");
			throw 0;//### fix this!
		}
	}

	//+++++++++++

	// select deterministically a start node for our optimization: select the first available action everywhere.
	for (auto non_trap_state_paired_actions = cont.available_actions_per_state.cbegin();
		non_trap_state_paired_actions != cont.available_actions_per_state.cend();
		++non_trap_state_paired_actions
		) {
		if (!non_trap_state_paired_actions->second.empty()) {
			cont.sched[non_trap_state_paired_actions->first] = 0;
		}
	}

	while (true) {

		// create matrix
		linear_systems::matrix mat;
		linear_systems::rational_vector rew;
		for (const auto& var : ordered_variables) {
			const auto& line_var_id{ mat.size() };
			if (cont.available_actions_per_state[var].empty()) { // it is a target state
				rew.emplace_back(0);
				linear_systems::matrix_line line = { { std::make_pair(line_var_id, rational_type(1)) } };
				mat.emplace_back(std::move(line));
			}
			else { // it is no target state
				const auto& action_id{ cont.sched[var] };
				const auto action{ cont.available_actions_per_state[var][action_id] };
				rew.emplace_back(m.rewards[var][action]);
				linear_systems::matrix_line line;
				bool extra_diagonal_entry{ true };
				for (const auto& state_paired_rational : m.probabilities[var][action]) {
					const auto& var_id{ get_index(ordered_variables, state_paired_rational.first) };
					auto value{ state_paired_rational.second * rational_type(-1) };
					if (var_id == line_var_id) {
						value += rational_type(1);
						extra_diagonal_entry = false;
					}
					line.push_back(std::make_pair(var_id, value));
				}
				if (extra_diagonal_entry) {
					line.push_back(std::make_pair(line_var_id, 1));
				}
				mat.emplace_back(std::move(line));
			}
		}
		// Px = rew
		// target: xi = 0
		// others: xj = Pk xk + r 
		// .....->  (d_j - Pk) x = r
		//

		linear_systems::id_vector unresolved;
		linear_systems::id_vector resolved;
		std::transform(
			m.targets.begin(),
			m.targets.end(),
			std::back_inserter(resolved),
			[&](const std::string& s) -> linear_systems::var_id { return get_index(ordered_variables, s); }
		);
		std::sort(resolved.begin(), resolved.end());

		for (linear_systems::var_id i{ 0 }; i < mat.size(); ++i) {
			unresolved.push_back(i);
		}

		auto new_end = std::copy_if(
			unresolved.begin(),
			unresolved.end(),
			unresolved.begin(),
			[&](const linear_systems::var_id& var) -> bool {
				/* not contained in resolved */
				auto found = std::lower_bound(resolved.cbegin(), resolved.cend(), var);
				return found == resolved.cend() || *found != var;
			}
		);
		unresolved.erase(new_end, unresolved.end());

		/*
		for (const auto& decision : cont.sched) {
			standard_logger()->trace(std::string("At state  ") + decision.first + "  :  " + cont.available_actions_per_state[decision.first][decision.second]);
		}
		*/
		// solve matrix
		solve_linear_system_dependency_order_optimized(mat, rew, unresolved, resolved);

		linear_systems::rational_vector current_solution = rew;

		/*
		for (std::size_t i = 0; i < current_solution.size(); ++i) {
			standard_logger()->trace(std::string("curr-sol:  ") + std::to_string(i) + "  :  " +
				current_solution[i].numerator().str() + "/" + current_solution[i].denominator().str());
		}
		*/

		bool found_improvement{ false };

		// improve the scheduler...
		for (auto var = ordered_variables.cbegin(); var != ordered_variables.cend(); ++var) {
			if (cont.available_actions_per_state[*var].empty()) { // no action to choose...
				continue;
			}
			linear_systems::var_id var_id = var - ordered_variables.cbegin();
			auto select_action = cont.sched[*var];
			auto best_seen_value = current_solution[var_id];
			for (auto action = cont.available_actions_per_state[*var].cbegin(); action != cont.available_actions_per_state[*var].cend(); ++action) {
				std::size_t action_id = action - cont.available_actions_per_state[*var].cbegin();
				auto& distr = m.probabilities[*var][*action];
				rational_type accummulated{ m.rewards[*var][*action] };
				for (const auto& state_paired_prob : distr) {
					accummulated += state_paired_prob.second * current_solution[get_index(ordered_variables, state_paired_prob.first)];
				}
				if (accummulated > best_seen_value) {
					/*
					standard_logger()->trace(std::string("improve valued from  ") +
						best_seen_value.numerator().str() + "/" + best_seen_value.denominator().str() + "   to " +
						accummulated.numerator().str() + "/" + accummulated.denominator().str());
					*/
					standard_logger()->trace(std::string("improve decision at   ") + *var + "   ::   " +
						cont.available_actions_per_state[*var][select_action] + "   -->>   " +cont.available_actions_per_state[*var][action_id] 
						+ ":     " + best_seen_value.denominator().str());
					select_action = action_id;
					best_seen_value = accummulated;
					found_improvement = true;
				}
			}
			cont.sched[*var] = select_action;
		}

		if (!found_improvement) {
			standard_logger()->info("The following memoryless deterministic scheduler is optimal:");
			for (const auto& decision : cont.sched) {
				standard_logger()->info(std::string("At state  ") + decision.first + "  :  " + cont.available_actions_per_state[decision.first][decision.second]);
			}
			standard_logger()->info("The following expectations per state are optimal:");
			for (std::size_t i = 0; i < current_solution.size(); ++i) {
				standard_logger()->info(std::string("At state  ") + ordered_variables[i] + "  :  " + current_solution[i].numerator().str() + "/" + current_solution[i].denominator().str());
			}
			return;
		}
		standard_logger()->info("Found a scheduler improvement. Rerun stepwise improvement.");
	}
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

	rational_type threshold{ 10 };
	const rational_type factor = 1000;
	const auto crinkle =
		[&threshold, &factor](const rational_type& x) -> rational_type {
		return (x < threshold) ?
			factor * x
			: x + (factor - rational_type(1)) * threshold;
	};

	// create unfold-mdp
	std::vector<std::string> ordered_variables;
	n = unfold(m, crinkle, threshold, delta_max, ordered_variables);
	//#### will break if in m state names use underscore


	standard_logger()->info("got unfolded mdp:");
	standard_logger()->info(mdp_to_json(n).dump(3));


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

	optimize_scheduler(n, ordered_variables);

	standard_logger()->info("     DONE     ");
	return 0;
}
