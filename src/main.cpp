#include "logger.h"
#include "utility.h"
#include "linear_system.h"
#include "mdp_ops.h"
#include "feature_toggle.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

#include <nlohmann/json.hpp>

#include <future>
#include <random>


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



std::size_t get_index(const std::vector<std::string>& str_vec, const std::string& s) {
	for (std::size_t i{ 0 }; i < str_vec.size(); ++i) {
		if (str_vec[i] == s) {
			return i;
		}
	}
	throw std::logic_error("String not contained in string vector.");
}


void create_matrix(const mdp& m, const std::vector<std::string>& ordered_variables, const scheduler_container& cont, linear_systems::matrix& mat, linear_systems::rational_vector& rew, linear_systems::id_vector& unresolved, linear_systems::id_vector& resolved) {

	for (const auto& var : ordered_variables) {
		const auto& line_var_id{ mat.size() };
		if (cont.available_actions_per_state.at(var).empty()) { // it is a target state
			rew.emplace_back(0);
			linear_systems::matrix_line line = { { std::make_pair(line_var_id, rational_type(1)) } };
			mat.emplace_back(std::move(line));
		}
		else { // it is no target state
			const auto& action_id{ cont.sched.at(var) };
			const auto action{ cont.available_actions_per_state.at(var)[action_id] };
			rew.emplace_back(m.rewards.at(var).at(action));
			linear_systems::matrix_line line;
			bool extra_diagonal_entry{ true };
			for (const auto& state_paired_rational : m.probabilities.at(var).at(action)) {
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
}

template <bool WRITE_LOG = true>
void optimize_scheduler(mdp& m, const std::vector<std::string>& ordered_variables) { // do-check!
	scheduler_container cont;


	cont.init(m); // start with the "smallest" scheduler

	while (true) {

		linear_systems::matrix mat;
		linear_systems::rational_vector rew;
		linear_systems::id_vector unresolved;
		linear_systems::id_vector resolved;

		// create matrix
		create_matrix(m, ordered_variables, cont, mat, rew, unresolved, resolved);

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
					if constexpr (WRITE_LOG) standard_logger()->trace(std::string("improve decision at   ") + *var + "   ::   " +
						cont.available_actions_per_state[*var][select_action] + "   -->>   " + cont.available_actions_per_state[*var][action_id]
						+ ":     " + best_seen_value.denominator().str());
					select_action = action_id;
					best_seen_value = accummulated;
					found_improvement = true;
				}
			}
			cont.sched[*var] = select_action;
		}

		if (!found_improvement) {
			// check for multiple optimal schedulers...
			scheduler_container::multi_scheduler s;

			for (auto var = ordered_variables.cbegin(); var != ordered_variables.cend(); ++var) {
				if (cont.available_actions_per_state[*var].empty()) { // no action to choose...
					continue;
				}
				linear_systems::var_id var_id = var - ordered_variables.cbegin();
				auto best_seen_value = current_solution[var_id];

				//auto select_action = cont.sched[*var]; //?
				for (auto action = cont.available_actions_per_state[*var].cbegin(); action != cont.available_actions_per_state[*var].cend(); ++action) {
					std::size_t action_id = action - cont.available_actions_per_state[*var].cbegin();
					auto& distr = m.probabilities[*var][*action];
					rational_type accummulated{ m.rewards[*var][*action] };
					for (const auto& state_paired_prob : distr) {
						accummulated += state_paired_prob.second * current_solution[get_index(ordered_variables, state_paired_prob.first)];
					}
					if (accummulated == best_seen_value) {
						s[*var].push_back(action_id);
					}
				}
			}

			// output optimal schedulers
			if constexpr (WRITE_LOG) standard_logger()->info("The following memoryless deterministic scheduler(s) is/are optimal:");
			for (const auto& decision : s) {
				std::string schedulers_string;
				for (auto action_id : decision.second) {
					schedulers_string += cont.available_actions_per_state[decision.first][action_id] + "   ";
				}
				if constexpr (WRITE_LOG) standard_logger()->info(std::string("At state  ") + decision.first + "  :  " + schedulers_string);
			}
			if constexpr (WRITE_LOG) standard_logger()->info("The following expectations per state are optimal:");
			if constexpr (WRITE_LOG)
				for (std::size_t i = 0; i < current_solution.size(); ++i) {
					standard_logger()->info(std::string("At state  ") + ordered_variables[i] + "  :  " + current_solution[i].numerator().str() + "/" + current_solution[i].denominator().str());
				}
			return;
		}
		if constexpr (WRITE_LOG) standard_logger()->info("Found a scheduler improvement. Rerun stepwise improvement.");
	}
}


class application_errors {
public:
	static constexpr std::size_t count_error_codes{ 14 };
	inline static const std::array<std::string_view, count_error_codes> application_error_messages{ {
		std::string_view("Ordinary EXIT"), // 0
		std::string_view("Internal error: called with ZERO arguments") , // 1
		std::string_view("Call error: called with ZERO arguments"), // 2
		std::string_view("Could not open and parse all files correctly"), // 3
		std::string_view("Could not merge json files"), // 4
		std::string_view("Could build up an MDP from json files"), // 5
		std::string_view("Found a negative loop in an MDP"), // 6
		std::string_view("Fatal internal error"), // 7
		std::string_view("Json entry for task not valid"), // 8
		std::string_view("Found unreachable state(s)"), // 9
		std::string_view("Found state(s) where reaching target is not guaranteed"), // 10
		std::string_view("Crinkle values error"), // 11
		std::string_view("Unknown calc mode. Don't know what to do"), // 12
		std::string_view("Cannot parse rational value from json") // 13
		}
	};


};

void check_task_okay(const nlohmann::json& merged_json) { // do-check!

	// check for "task:"
	json_task_error::check("merged_json_is_object", merged_json.is_object());
	json_task_error::check("merged_containes_key_task", merged_json.contains(keywords::task));

	// check for "checks:"
	const auto& task_json{ merged_json.at(keywords::task) };
	json_task_error::check("task_contains_checks", task_json.contains(keywords::checks::checks));
	const auto& checks_json{ task_json.at(keywords::checks::checks) };
	json_task_error::check("checks_contain_no_unreachable_states", checks_json.contains(keywords::checks::no_unreachable_states));
	json_task_error::check("checks_contain_no_unreachable_states_bool", checks_json.at(keywords::checks::no_unreachable_states).is_boolean());
	json_task_error::check("checks_contain_only_positive_cycles", checks_json.contains(keywords::checks::no_negative_cycles));
	json_task_error::check("checks_contain_only_positive_cycles_bool", checks_json.at(keywords::checks::no_negative_cycles).is_boolean());
	json_task_error::check("checks_contain_reaching_target_with_probability_1", checks_json.contains(keywords::checks::reaching_target_with_probability_1));
	json_task_error::check("checks_contain_reaching_target_with_probability_1_bool", checks_json.at(keywords::checks::reaching_target_with_probability_1).is_boolean());
	json_task_error::check("checks_contain_ignore_non_positive_cycles_on_target_states", checks_json.contains(keywords::checks::ignore_negative_cycles_on_target_states));
	json_task_error::check("checks_contain_ignore_non_positive_cycles_on_target_states_bool", checks_json.at(keywords::checks::ignore_negative_cycles_on_target_states).is_boolean());

	// check for "calc:"
	json_task_error::check("task_containes_calc", task_json.contains(keywords::calc));
	const auto& calc_json{ task_json.at(keywords::calc) };
	json_task_error::check("calc_containes_mode", calc_json.contains(keywords::mode));
	const auto& task_calc_mode{ calc_json.at(keywords::mode) };
	json_task_error::check("task_calc_mode_is_string", task_calc_mode.is_string());

}

/**
*	removes unreachable states, throws error if error_on_exists_unreachable_state
*/
template <bool WRITE_LOG = true>
void remove_unreachable_states(mdp& m, bool error_on_exists_unreachable_state) { // do-check!
	std::string& initial_state{ m.initial };

	std::vector<std::string> reachables;
	reachables.push_back(initial_state);

	std::size_t next_expand{ 0 };

	while (next_expand < reachables.size()) {
		auto& actions_paired_distr{ m.probabilities[reachables[next_expand]] };
		for (auto& action_paired_distr : actions_paired_distr) {
		again_inner_loop:
			for (auto next_state_paired_probability = action_paired_distr.second.begin(); next_state_paired_probability != action_paired_distr.second.end(); ++next_state_paired_probability) {
				if (next_state_paired_probability->second != rational_type(0)) {
					auto& next_s{ next_state_paired_probability->first };
					auto found = std::find(
						reachables.cbegin(),
						reachables.cend(),
						next_s
					);
					if (found == reachables.cend()) {
						reachables.push_back(next_s);
					}
				}
				else {
					///#### log that a zero prob- transition was found
					action_paired_distr.second.erase(next_state_paired_probability);
					goto again_inner_loop; // because of iterator invalidation!!!
				}
			}
		}
		++next_expand;
	}

	std::vector<std::string> unreachables;

	std::sort(reachables.begin(), reachables.end());
	auto iter = reachables.cbegin();
	for (const auto& state : m.states) {
		if (iter == reachables.cend()) {
			unreachables.push_back(state);
			continue;
		}
		if (state == *iter) {
			++iter;
			continue;
		}
		if (state < *iter) {
			unreachables.push_back(state);
			continue;
		}
		if (*iter < state) {
			++iter;
			//## this is an internal error
			continue;
		}
	}
	if (!unreachables.empty()) {
		for (auto& state : unreachables) {
			if (error_on_exists_unreachable_state) {
				if constexpr (WRITE_LOG) standard_logger()->error("Found unreachable states:  ");
				if constexpr (WRITE_LOG) standard_logger()->error(std::string(".....") + state);
			}
			else {
				if constexpr (WRITE_LOG) standard_logger()->warn("Found unreachable states:  ");
				if constexpr (WRITE_LOG) standard_logger()->warn(std::string(".....") + state);
			}
		}
		if (error_on_exists_unreachable_state)
			throw found_unreachable_state(std::string("Found unreachable states:   ") + std::to_string(unreachables.size()));
	}

	// remove the unreachable states:
	for (const auto& state : unreachables) {
		m.states.erase(state);
		m.targets.erase(state);
		m.probabilities.erase(state); // next states already removed:: either probability 0 or exists only on right side of another unreachable state
		m.rewards.erase(state);
	}
}

template <bool WRITE_LOG = true>
bool check_reaching_target_is_guaranteed(mdp& m) { // do-check!
	std::map<std::string, bool> prob_to_target_is_positive;
	std::map<std::string, bool> is_target;
	for (const auto& state : m.states) {
		prob_to_target_is_positive[state] = false;
	}
	for (const auto& state : m.targets) {
		prob_to_target_is_positive[state] = true;
	}
	is_target = prob_to_target_is_positive;

	bool has_changes{ true };
	while (has_changes) {
		has_changes = false;

		for (auto& pair : prob_to_target_is_positive) {
			if (pair.second == true)
				continue;
			const bool update = std::accumulate(
				m.probabilities.at(pair.first).cbegin(),
				m.probabilities.at(pair.first).cend(),
				!m.probabilities.at(pair.first).empty(),
				[&](bool b, const std::map<std::string, std::map<std::string, rational_type>>::const_iterator::value_type& action_paired_distr) -> bool {
					return b && std::accumulate(action_paired_distr.second.cbegin(), action_paired_distr.second.cend(), false,
						[&](bool inner_b, const std::pair<std::string, rational_type>& next_state_prob) {
							return inner_b || prob_to_target_is_positive[next_state_prob.first];
						});
				}
			);
			if (update)
				has_changes = true;
			pair.second = update;
		}
	}
	bool found_error{ false };
	for (const auto& pair : prob_to_target_is_positive) {
		if (pair.second == false) {
			if constexpr (WRITE_LOG) standard_logger()->error(std::string("Found a state from which you cannot reach a target:   ") + pair.first);
			found_error = true;
		}
	}
	return !found_error;
}

class modification {
public:

	virtual rational_type threshold() const = 0;

	virtual rational_type func(const rational_type& arg) const = 0;

};

class crinkle : public modification {

	rational_type ratio;
	rational_type t;

	static const rational_type& max(const rational_type& l, const rational_type& r) {
		return l > r ? l : r;
	}
	static const rational_type& min(const rational_type& l, const rational_type& r) {
		return l < r ? l : r;
	}
public:
	crinkle(const rational_type& ratio, const rational_type& t) : ratio(ratio), t(t) {}

	virtual rational_type threshold() const override {
		return t;
	}

	virtual rational_type func(const rational_type& arg) const override {
		auto RATIONAL_ZERO{ rational_type(0) };
		if (arg < t) {
			return ratio * arg + min(t, RATIONAL_ZERO) * (ratio - rational_type(1));
		}
		else {
			return arg + max(RATIONAL_ZERO, t) * (ratio - rational_type(1));
		}
	}

};

class quadratic : public modification {

	rational_type a;
	rational_type t;

	static const rational_type& max(const rational_type& l, const rational_type& r) {
		return l > r ? l : r;
	}
	static const rational_type& min(const rational_type& l, const rational_type& r) {
		return l < r ? l : r;
	}
public:
	quadratic(const rational_type& a, const rational_type& t) : a(a), t(t) {}

	virtual rational_type threshold() const override {
		return t;
	}

	virtual rational_type func(const rational_type& arg) const override {
		auto RATIONAL_ZERO{ rational_type(0) };
		if (arg < t) {
			return arg - a * (t - arg) * (t - arg) + max(RATIONAL_ZERO, t) * a * t;
		}
		else {
			return arg + max(RATIONAL_ZERO, t) * a * t;
		}
	}

};

class identity : public modification {

	rational_type t;

public:
	identity(const rational_type& t) : t(t) {}

	virtual rational_type threshold() const override {
		return t;
	}

	virtual rational_type func(const rational_type& arg) const override {
		return arg;
	}

};

auto count_combinations(const big_int_type& decision_layers, const big_int_type& remaining_units_to_distribute)->big_int_type {
	if (decision_layers == 1) return big_int_type(1);

	big_int_type accum = 0;
	for (big_int_type r = 0; r < remaining_units_to_distribute + 1; r += 1) {
		accum += count_combinations(decision_layers - 1, r);
	}
	return accum;
}


std::pair<mdp, bool> generate_mdp(std::size_t count_states, std::size_t count_target_states, std::size_t count_actions, const rational_type& probability_unit, big_int_type& resolve_nondeterminism, big_int_type min_reward, big_int_type past_max_reward) {
	mdp m;
	// we only need two actions without loss of generality

	if (!(count_target_states < count_states) || !(count_target_states > 0) || (count_states < 1)) {
		throw std::runtime_error("count states"); // ### change error
	}

	//standard_logger()->debug(resolve_nondeterminism.str());

	const auto reziproke_prob = rational_type(1) / probability_unit;
	if ((reziproke_prob).denominator() != 1) {
		throw 1; // ### change error
	}

	for (std::size_t i = 0; i < count_states; ++i) {
		m.states.insert("s" + std::to_string(i));
	}

	const auto get_state_name = [](std::size_t i) { return "s" + std::to_string(i); };
	const auto get_action_name = [](std::size_t i) { return "alpha" + std::to_string(i); };

	m.initial = "s0";
	for (std::size_t i = count_states - count_target_states; i < count_states; ++i) {
		m.targets.insert(get_state_name(i));
	}

	for (std::size_t i = 0; i < count_actions; ++i) {
		m.actions.insert(get_action_name(i));
	}

	std::size_t first_non_target_state = 0;
	std::size_t behind_last_non_target_state = count_states - count_target_states;

	big_int_type combinations = count_combinations(count_states, reziproke_prob.numerator());

	// probability distributions...
	for (std::size_t state = first_non_target_state; state != behind_last_non_target_state; ++state) {
		for (std::size_t action = 0; action < count_actions; ++action) {

			big_int_type decision_maker = resolve_nondeterminism % combinations;
			resolve_nondeterminism /= combinations; // remove the information from resolve_nondeterminism

			rational_type remaining_probability = rational_type(1);
			for (std::size_t next_state = 0; next_state < count_states; ++next_state) {
				if (1 == count_states - next_state) { // no decision left (one possibility available only)
					m.probabilities[get_state_name(state)][get_action_name(action)][get_state_name(next_state)] = remaining_probability;
				}
				else {
					big_int_type choose_prob_max = (remaining_probability / probability_unit).numerator(); // decide for one value of 0 ... choose_prob_max

					for (big_int_type choose = 0; choose <= choose_prob_max; ++choose) {
						const auto sub_cases_for_choosing_so = count_combinations(count_states - next_state - 1, choose_prob_max - choose);

						if (decision_maker < sub_cases_for_choosing_so) { // choose this way!     we take probability_unit * choose
							// let us choose the following probability.
							rational_type p = probability_unit * choose;

							m.probabilities[get_state_name(state)][get_action_name(action)][get_state_name(next_state)] = p;
							remaining_probability -= p;
							break;
						}
						decision_maker -= sub_cases_for_choosing_so;
						if (choose == choose_prob_max) { // we are already at maximal possible chosen probability but we do not select this one.
							// if here: error : decision_maker was too big. Miscalculation of number of combinations somewhere
							throw 123; // ### improve error messages!
						}
					}
				}
			}
		}
	}

	big_int_type possible_rewards = past_max_reward - min_reward;

	// rewards ...
	for (std::size_t state = first_non_target_state; state != behind_last_non_target_state; ++state) {
		for (std::size_t action = 0; action < count_actions; ++action) {

			big_int_type decision_maker = resolve_nondeterminism % possible_rewards;
			resolve_nondeterminism /= possible_rewards; // remove the information from resolve_nondeterminism

			m.rewards[get_state_name(state)][get_action_name(action)] = rational_type(min_reward + decision_maker);

		}
	}

	return std::make_pair(m, resolve_nondeterminism > 0);
}

std::tuple<rational_type, std::vector <rational_type>, std::vector<scheduler_container>> check_all_exponential_schedulers_for_hVar(const mdp& m, const mdp& first_unfolded_with_normal_rewwards, const rational_type& lambda, const rational_type& cut_level, const std::vector<std::string>& ordered_variables) {

	scheduler_container cont;

	rational_type best_seen_result;
	std::vector <rational_type> mu_for_best_seen_result;
	std::vector<scheduler_container> best_seen_scheduler;
	bool first_result_found{ false };
	std::mutex access_optimal_values;

	//std::list<std::thread> threads;
	std::list<std::future<void>> the_futures;

	cont.init(first_unfolded_with_normal_rewwards);

	auto size_message = std::string("number of schedulers to be tested:   ") + cont.number_of_schedulers().numerator().str();
	standard_logger()->trace(size_message);
	do {
		//const scheduler_container& cont_const_ref{ cont };

		auto evaluate_one_scheduler = [&first_unfolded_with_normal_rewwards, &ordered_variables, &lambda, &m, &cut_level, &first_result_found, &best_seen_result, &mu_for_best_seen_result, &best_seen_scheduler, &access_optimal_values](const scheduler_container cont_const_ref) { //  

			// to be filled in...
			linear_systems::matrix mat;
			linear_systems::rational_vector rew;
			linear_systems::id_vector unresolved;
			linear_systems::id_vector resolved;

			// create matrix
			create_matrix(first_unfolded_with_normal_rewwards, ordered_variables, cont_const_ref, mat, rew, unresolved, resolved);


			// solve matrix
			solve_linear_system_dependency_order_optimized(mat, rew, unresolved, resolved);

			// we have mu for classical problem so far
			linear_systems::rational_vector current_solution = rew;

			std::size_t index_of_initial_state = std::find(ordered_variables.cbegin(), ordered_variables.cend(), first_unfolded_with_normal_rewwards.initial) - ordered_variables.cbegin(); // initial state should be the first one, so == 0
			if (index_of_initial_state != 0) {
				standard_logger()->error("Internal error: exp-sched-index-of-initial-state");
			}
			rational_type current_mu = current_solution[index_of_initial_state];

			const auto modify{
				[&](const rational_type& arg) {
					return arg - lambda * std::max(rational_type(0), current_mu) * std::max(rational_type(0), current_mu);
				}
			};

			// now modify the rewards in m, so we can calculat ethe actual mu - \lambda hVar.

			mdp unfolded_cut_with_modified_rewards;

			std::vector<std::string> modified_ordered_variables;

			scheduler_container cont2 = cont_const_ref;

			unfolded_cut_with_modified_rewards = modified_stupid_unfold(m, cut_level, modified_ordered_variables, modify, cont2);
			{
				// to be filled in...
				linear_systems::matrix mat2;
				linear_systems::rational_vector rew2;
				linear_systems::id_vector unresolved2;
				linear_systems::id_vector resolved2;

				// create matrix
				create_matrix(unfolded_cut_with_modified_rewards, modified_ordered_variables, cont2, mat2, rew2, unresolved2, resolved2);


				// solve matrix
				solve_linear_system_dependency_order_optimized(mat2, rew2, unresolved2, resolved2);

				// we have mu for classical problem so far
				linear_systems::rational_vector current_solution_with_hVar = rew2;

				std::size_t index_of_initial_state2 = std::find(modified_ordered_variables.cbegin(), modified_ordered_variables.cend(), unfolded_cut_with_modified_rewards.initial) - modified_ordered_variables.cbegin(); // initial state should be the first one, so == 0
				rational_type current_mu_minus_hVar = current_solution_with_hVar[index_of_initial_state2];

				const auto lock_access = std::lock_guard<std::mutex>(access_optimal_values);

				if (!first_result_found || current_mu_minus_hVar > best_seen_result) {
					first_result_found = true;
					best_seen_result = current_mu_minus_hVar;
					mu_for_best_seen_result.clear();
					best_seen_scheduler.clear();
				}
				if (current_mu_minus_hVar == best_seen_result) {
					mu_for_best_seen_result.push_back(current_mu);
					best_seen_scheduler.push_back(cont_const_ref);

				}
			}
		};

		if (!(the_futures.size() < feature_toggle::COUNT_THREADS)) {
			while (true) {
				for (auto iter = the_futures.begin(); iter != the_futures.end(); ++iter) {
					if (iter->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
						iter->wait();
						the_futures.erase(iter); // iterator invalidation!
						goto continue_6248974735;
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}
	continue_6248974735:

		//threads.push_back(std::thread(evaluate_one_scheduler, cont));
		the_futures.emplace_back(std::async(std::launch::async, evaluate_one_scheduler, cont));

	} while (cont.operator++());

	for (auto& f : the_futures) {
		f.wait();
	}

	return std::make_tuple(best_seen_result, mu_for_best_seen_result, best_seen_scheduler);
}

std::size_t number_of_steps_in_mdp_to_reach_goal_with_at_least(const mdp& m, rational_type probability) {
	std::map<std::string, std::vector<rational_type>> states_to_min_probabilities; // state s |-> [p0, p1, p2, p3, p4, ... ]
		// p_i is the minimum probability to reach a goal state t for the first time after  i steps or even earlier.

	//auto accumulated_probability_to_reach_goal_within_i_steps_starting_from_initial_state{ rational_type(0) };

	for (const auto& s : m.states) {
		states_to_min_probabilities[s].push_back(0); // for all non-target states we are in a target state after ZERO steps with at least probability 0. (always asking for the scheduler that lets us loop without reaching goal as long as possible)
	}
	for (const auto& t : m.targets) { // overwrite the initial value for target states
		states_to_min_probabilities[t].back() = rational_type(1); // for all target states we are in a target state after ZERO steps with excatly probability 1.
	}
	//accumulated_probability_to_reach_goal_within_i_steps_starting_from_initial_state += states_to_min_probabilities[m.initial][0];

	std::size_t last_filled_index{ 0 }; // 0... already filled. loop will increment at first step
	while (states_to_min_probabilities[m.initial].back() < probability) {

		for (auto& pair : states_to_min_probabilities) {
			if (pair.second[0] == rational_type(1)) {// is target state
				pair.second.push_back(rational_type(1)); // already reaching goal earlier (with zero steps)
			}
			else { // non-target state
				rational_type min_probability{ 1 };
				auto& distributions{ m.probabilities.at(pair.first) };
				if (distributions.empty()) {
					min_probability = 0;
				}
				for (const auto& action_paired_distr : distributions) { // look for the worst action, worst == reach goal within x steps with least probability
					// check for  the probability to reach goal in at least one step, but with [0... x-1] more steps...
					min_probability = std::min(
						min_probability,
						std::accumulate(action_paired_distr.second.cbegin(), action_paired_distr.second.cend(), rational_type(0),
							[&](const rational_type& acc, const std::pair<std::string, rational_type>& state_probability_pair) {
								return acc + state_probability_pair.second * states_to_min_probabilities[state_probability_pair.first][last_filled_index];
								// for each consecutive state add probability to enter this state times probability to reach goal in the remaining steps.
							}) // iterate all next_states
					);
				}
				pair.second.push_back(min_probability);
			}
		}
		++last_filled_index;
	}
	return last_filled_index;
}

rational_type maximal_weight_after_n_steps(const mdp& m, std::size_t n_steps) {
	std::map<std::string, rational_type> state_to_maximum_reward_when_here;

	state_to_maximum_reward_when_here[m.initial] = rational_type(0);

	for (std::size_t i = 0; i < n_steps; ++i) {
		std::map<std::string, rational_type> next_values{ state_to_maximum_reward_when_here };
		for (const auto& state_with_value : state_to_maximum_reward_when_here) {
			for (const auto& action_paired_distr : m.probabilities.at(state_with_value.first)) {
				for (const auto& next_state_paired_probability : action_paired_distr.second) {
					auto find_iter = next_values.find(next_state_paired_probability.first);
					const auto update_value{ state_with_value.second + m.rewards.at(state_with_value.first).at(action_paired_distr.first) };
					if (find_iter == next_values.cend()) {
						next_values[next_state_paired_probability.first] = update_value;
					}
					else {
						next_values[next_state_paired_probability.first] = std::max(next_values[next_state_paired_probability.first], update_value);
					}
				}
			}
		}
		state_to_maximum_reward_when_here = next_values;
	}
	return std::accumulate(state_to_maximum_reward_when_here.cbegin(), state_to_maximum_reward_when_here.cend(), rational_type(0),
		[](const rational_type& acc, const decltype(state_to_maximum_reward_when_here)::iterator::value_type& pair) {
			return std::max(acc, pair.second);
		});
}


int run_starting_from_merged_json(const nlohmann::json& merged_json) { // do-check!, ready but enhance the different cases -> common steps before might be inappropriate for other options
	standard_logger()->info("Checking for validness of MDP (json -> MDP)...");

	mdp m;

	/*
		BUILD UP MDP
	*/
	try {
		check_valid_mdp_and_load_mdp_from_json(merged_json, m);
	}
	catch (mdp_sanity& e) {
		standard_logger()->error(e.what());
		const std::size_t error_code{ 5 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		return error_code;
	}

	standard_logger()->info("Successfully build up MDP from json!");

	/*
		CHECK TASK
	*/
	try {
		check_task_okay(merged_json);
	}
	catch (const json_task_error& e) {
		standard_logger()->error(e.what());
		const std::size_t error_code{ 8 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		return error_code;
	}

	/*
		CHECK CONFIG
	*/
	const bool task_checks_no_unreachable_states{
		merged_json[keywords::task][keywords::checks::checks][keywords::checks::no_unreachable_states]
	};
	const bool task_checks_reaching_target_with_probability_1{
		merged_json[keywords::task][keywords::checks::checks][keywords::checks::reaching_target_with_probability_1]
	};
	const bool task_checks_no_negative_cycles{
		merged_json[keywords::task][keywords::checks::checks][keywords::checks::no_negative_cycles]
	};
	const bool task_checks_ignore_negative_cycles_on_target_states{
		merged_json[keywords::task][keywords::checks::checks][keywords::checks::ignore_negative_cycles_on_target_states]
	};

	if (!task_checks_ignore_negative_cycles_on_target_states) {
		standard_logger()->warn("check for negative cycles on target states not available");
	}

	standard_logger()->info("Check for certain MDP properties...");

	// check:no unreachable states / remove unreachable states and their transitions / rewards....
	// also removes 0-probability transitions!!!
	try {
		remove_unreachable_states(m, task_checks_no_unreachable_states);
	}
	catch (const found_unreachable_state& e) {
		standard_logger()->error(e.what());
		const std::size_t error_code{ 9 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		return error_code;
	}

	if (!task_checks_reaching_target_with_probability_1) {
		standard_logger()->warn("Skipping reachable target check!");
	}
	else {
		if (!check_reaching_target_is_guaranteed(m)) {
			const std::size_t error_code{ 10 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
	}

	std::map<std::string, rational_type> delta_max;
	try {
		delta_max = calc_delta_max_state_wise(m, task_checks_ignore_negative_cycles_on_target_states, task_checks_no_negative_cycles);
	}
	catch (const found_negative_loop& e) {
		standard_logger()->error(e.what());
		const std::size_t error_code{ 6 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		return error_code;
	}
	catch (const calc_delta_max_error& e) {
		standard_logger()->error(e.what());
		const std::size_t error_code{ 7 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		return error_code;
	}

	auto& calc_json{ merged_json.at(keywords::task).at(keywords::calc) };
	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::classic.data()) { // classical SSP-Problem

		std::vector<std::string> ordered_variables;
		std::copy(m.states.cbegin(), m.states.cend(), std::back_inserter(ordered_variables));

		optimize_scheduler(m, ordered_variables);
		goto before_return;
	}

	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::crinkle.data()) { // crinkle Problem

		rational_type t;
		rational_type r;
		try {
			json_task_error::check("mode_crinkle_has_t", calc_json.contains("t"));
			json_task_error::check("mode_crinkle_has_t_string", calc_json.at("t").is_string());
			t = string_to_rational_type(calc_json.at("t").get<std::string>());
			json_task_error::check("mode_crinkle_has_r", calc_json.contains("r"));
			json_task_error::check("mode_crinkle_has_r_string", calc_json.at("r").is_string());
			r = string_to_rational_type(calc_json.at("r").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 11 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		const auto c{ crinkle(r, t) };

		mdp n;
		std::vector<std::string> ordered_variables;
		standard_logger()->info("Unfolding MDP...");
		n = unfold(m, c, delta_max, ordered_variables);

		standard_logger()->trace(mdp_to_json(n).dump(3));

		optimize_scheduler(n, ordered_variables);
		goto before_return;
	}

	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::quadratic.data()) { // quadratically penalized Problem

		rational_type t;
		rational_type a;
		try {
			json_task_error::check("mode_quadratic_has_t", calc_json.contains("t"));
			json_task_error::check("mode_quadratic_has_t_string", calc_json.at("t").is_string());
			t = string_to_rational_type(calc_json.at("t").get<std::string>());
			json_task_error::check("mode_quadratic_has_a", calc_json.contains("a"));
			json_task_error::check("mode_quadratic_has_a_string", calc_json.at("a").is_string());
			a = string_to_rational_type(calc_json.at("a").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 11 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		const auto c{ quadratic(a, t) };

		mdp n;
		std::vector<std::string> ordered_variables;
		standard_logger()->info("Unfolding MDP...");
		n = unfold(m, c, delta_max, ordered_variables);

		optimize_scheduler(n, ordered_variables);
		goto before_return;
	}

	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::hVar_approach.data()) { // approach for hVar

		rational_type n; // maximum reward steps 
		rational_type seconds; // seconds: if one approximation step takes longer then time, it will be the last one.
		rational_type lambda; // lambda factor.

		try {
			json_task_error::check("mode_vVar_approach_has_n", calc_json.contains("n"));
			json_task_error::check("mode_vVar_approach_has_n_string", calc_json.at("n").is_string());
			n = string_to_rational_type(calc_json.at("n").get<std::string>());
			json_task_error::check("mode_vVar_approach_has_seconds", calc_json.contains("seconds"));
			json_task_error::check("mode_vVar_approach_has_seconds_string", calc_json.at("seconds").is_string());
			seconds = string_to_rational_type(calc_json.at("seconds").get<std::string>());
			json_task_error::check("mode_vVar_approach_has_lambda", calc_json.contains("lambda"));
			json_task_error::check("mode_vVar_approach_has_lambda_string", calc_json.at("lambda").is_string());
			lambda = string_to_rational_type(calc_json.at("lambda").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 11 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		standard_logger()->info("ready reading params!");

		rational_type cut_level = std::min(rational_type(0), n);

		std::vector<
			std::tuple<
			rational_type, // cut level
			std::tuple<
			rational_type, // hVar optimal value
			std::vector <rational_type>, // mu of optimal schedulers
			std::vector<scheduler_container> // optimal schedulers
			>,
			std::vector<
			std::pair<
			rational_type, // stabilisation distance
			std::map<
			std::string, // original state name
			std::size_t // index of chosen action
			> // cut end scheduler
			>
			> //
			>
		> cut_level_to_optimal_solutions;

		// use increasing cut_level until n
		while (!(cut_level > n))
		{
			standard_logger()->trace(std::string("run on cut_level:   ") + cut_level.numerator().str() + "/" + cut_level.denominator().str());

			auto timestamp_before_calculting = std::chrono::steady_clock::now();

			// to be unfolded without any changed step rewards
			mdp stupid_unfolded_mdp;

			std::vector<std::string> ordered_variables;
			std::map<std::string, // augmented state
				std::pair<std::string, rational_type> // original state, reward accum.
			> augmented_state_to_pair; // just to quickly get original state and accum reward out of a augmented state.

			stupid_unfolded_mdp = stupid_unfold(m, cut_level, ordered_variables, augmented_state_to_pair); // unfolding without any reward changes
			standard_logger()->trace("Done: stupid_unfold");

			auto tup = check_all_exponential_schedulers_for_hVar(m, stupid_unfolded_mdp, lambda, cut_level, ordered_variables); // tries all schedulers and returns the ones leading to maximum expected mu-hVar.
			standard_logger()->trace("Done: exponential scheduler check");

			cut_level_to_optimal_solutions.push_back(std::make_tuple(cut_level, std::move(tup), std::vector<std::pair<rational_type, std::map<std::string, std::size_t>>>()));

			auto& optimal_scheds_vector{ std::get<2>(std::get<1>(cut_level_to_optimal_solutions.back())) };
			auto& optimal_cut_end_schedulers_and_stabilization_distance{ std::get<2>(cut_level_to_optimal_solutions.back()) };

			const std::size_t number_of_optimal_scheds = optimal_scheds_vector.size();

			for (std::size_t i = 0; i < number_of_optimal_scheds; ++i) { // iterate all optimal schedulers...

				std::map<std::string, // original state name
					std::size_t // index of chosen action
				> cut_end_scheduler; // to extract the scheduler of the self-catching sub mdp at cut_level...

				std::map<std::string, std::string> cut_end_state_to_action_name;

				std::vector<std::map<std::string, std::pair<std::string, rational_type>>::iterator> cut_end_states; // iterators to the cut_end_states

				for (auto iter = augmented_state_to_pair.begin(); iter != augmented_state_to_pair.end(); ++iter) {
					if (iter->second.second == cut_level) {
						cut_end_states.push_back(iter);
						cut_end_scheduler[iter->second.first] = optimal_scheds_vector[i].sched[iter->first];
						auto& alll_actions_ath_this_state = optimal_scheds_vector[i].available_actions_per_state[iter->first];
						cut_end_state_to_action_name[iter->second.first] = alll_actions_ath_this_state.empty() /* trap state */ ? "--NONE--" : alll_actions_ath_this_state[cut_end_scheduler[iter->second.first]]; // action name
					}
				}

				// check for stabilizing distance...

				rational_type stabilization_distance{ 0 };
				bool abort = false;
				while (!abort && stabilization_distance <= cut_level) {
					rational_type check_stab_distance = stabilization_distance + rational_type{ 1 }; ///#### alllow another distance t364698234764325847

					for (auto iter = augmented_state_to_pair.begin(); iter != augmented_state_to_pair.end(); ++iter) {
						if (iter->second.second >= check_stab_distance && iter->second.second < stabilization_distance) {
							// check if the states in this range are stabilized with cut end scheduler:

							if (
								optimal_scheds_vector[i].sched[iter->first] != cut_end_scheduler[iter->second.first] // if scheduler decides not the stabilized way
								) {
								abort = true;
							}
						}
					}

					if (!abort) {
						stabilization_distance = check_stab_distance;
					}
				}

				// @here we have calculated the distance of stabilization...
				optimal_cut_end_schedulers_and_stabilization_distance.emplace_back(stabilization_distance, cut_end_scheduler);

				auto message1 = std::string("cut_level:   ") + cut_level.numerator().str() + "/" + cut_level.denominator().str() + "\n";
				auto message2 = std::string("stabilisation distance:   ") + optimal_cut_end_schedulers_and_stabilization_distance.back().first.numerator().str() + "/" + optimal_cut_end_schedulers_and_stabilization_distance.back().first.denominator().str() + "\n";
				auto message3 = std::string("end component scheduler:\n") + nlohmann::json(cut_end_state_to_action_name).dump(3);
				auto message4 = std::string("+++++++++++++++++++++++\n");

				//std::string message3 = "cut_end_scheduler: ...";

				standard_logger()->info(message1 + message2 + message3 + message4);

			}
			std::string message3 = "\n count optimal schedulers: " + std::to_string(number_of_optimal_scheds);
			standard_logger()->info(message3);

			standard_logger()->info("-------------------------------------------------------------------------------------");
			auto timestamp_after_calculting = std::chrono::steady_clock::now();
			if (timestamp_after_calculting - timestamp_before_calculting > std::chrono::seconds(seconds.numerator().convert_to<unsigned long long>())) {
				standard_logger()->warn("exiting because of timeout");
				goto continue_82757928765;
			}

			cut_level += rational_type(1); // ##### introduce step variable t364698234764325847
		}
	continue_82757928765:
		goto before_return;
	}
	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::hVar_approach_percentage_cut.data()) { // percentage cut solving

		//## do all the things here
		rational_type probability_of_reaching_goal;
		//rational_type a;
		try {
			json_task_error::check("mode_hVar_approach_percentage_cut_has_prob", calc_json.contains("prob"));
			json_task_error::check("mode_hVar_approach_percentage_has_prob_string", calc_json.at("prob").is_string());
			probability_of_reaching_goal = string_to_rational_type(calc_json.at("prob").get<std::string>());
			//json_task_error::check("mode_quadratic_has_a", calc_json.contains("a"));
			//json_task_error::check("mode_quadratic_has_a_string", calc_json.at("a").is_string());
			//a = string_to_rational_type(calc_json.at("a").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 13 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}

		std::size_t steps_needed = number_of_steps_in_mdp_to_reach_goal_with_at_least(m, probability_of_reaching_goal);

		standard_logger()->info(std::string("Minimum number of steps needed to satisfy probability constraint:  ") + std::to_string(steps_needed));

		//rational_type n; // maximum reward steps 
		//rational_type seconds; // seconds: if one approximation step takes longer then time, it will be the last one.
		rational_type lambda; // lambda factor.

		try {
			//json_task_error::check("mode_vVar_approach_has_n", calc_json.contains("n"));
			//json_task_error::check("mode_vVar_approach_has_n_string", calc_json.at("n").is_string());
			//n = string_to_rational_type(calc_json.at("n").get<std::string>());
			//json_task_error::check("mode_vVar_approach_has_seconds", calc_json.contains("seconds"));
			//json_task_error::check("mode_vVar_approach_has_seconds_string", calc_json.at("seconds").is_string());
			//seconds = string_to_rational_type(calc_json.at("seconds").get<std::string>());
			json_task_error::check("mode_vVar_approach_has_lambda", calc_json.contains("lambda"));
			json_task_error::check("mode_vVar_approach_has_lambda_string", calc_json.at("lambda").is_string());
			lambda = string_to_rational_type(calc_json.at("lambda").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 11 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		standard_logger()->info("ready reading params!");

		rational_type weight_threshold{ maximal_weight_after_n_steps(m, steps_needed) };




		rational_type cut_level = weight_threshold;


		// use increasing cut_level until n

		standard_logger()->trace(std::string("run on cut_level:   ") + cut_level.numerator().str() + "/" + cut_level.denominator().str());

		// to be unfolded without any changed step rewards
		mdp stupid_unfolded_mdp;

		std::vector<std::string> ordered_variables;
		std::map<std::string, // augmented state
			std::pair<std::string, rational_type> // original state, reward accum.
		> augmented_state_to_pair; // just to quickly get original state and accum reward out of a augmented state.

		stupid_unfolded_mdp = stupid_unfold(m, cut_level, ordered_variables, augmented_state_to_pair); // unfolding without any reward changes
		standard_logger()->trace("Done: stupid_unfold");

		// best seen penalized expectation, seen classical expectations, seen matching schedulers
		std::tuple<rational_type, std::vector <rational_type>, std::vector<scheduler_container>> tup =
			check_all_exponential_schedulers_for_hVar(m, stupid_unfolded_mdp, lambda, cut_level, ordered_variables); // tries all schedulers and returns the ones leading to maximum expected mu-hVar.
		standard_logger()->trace("Done: exponential scheduler check");

		//optimal_solutions = std::make_tuple(cut_level, std::move(tup), std::vector<std::pair<rational_type, std::map<std::string, std::size_t>>>());

		auto& optimal_scheds_vector{ std::get<2>(tup) };
		auto optimal_cut_end_schedulers_and_stabilization_distance{ std::vector<std::pair<rational_type, std::map<std::string, std::size_t>>>() };

		const std::size_t number_of_optimal_scheds = optimal_scheds_vector.size();

		standard_logger()->info(std::string("Number of optimal schedulers found:   ") + std::to_string(number_of_optimal_scheds));

		for (std::size_t i = 0; i < number_of_optimal_scheds; ++i) { // iterate all optimal schedulers...

			standard_logger()->info(std::string("Presenting optimal scheduler  #") + std::to_string(i));

			std::map<std::string, // original state name
				std::size_t // index of chosen action
			> cut_end_scheduler; // to extract the scheduler of the self-catching sub mdp at cut_level...

			std::map<std::string, std::string> cut_end_state_to_action_name;

			std::vector<std::map<std::string, std::pair<std::string, rational_type>>::iterator> cut_end_states; // iterators to the cut_end_states inside augmented_state_to_pair

			for (auto iter = augmented_state_to_pair.begin(); iter != augmented_state_to_pair.end(); ++iter) {
				if (iter->second.second == cut_level) { // if it is a state of the cut component
					cut_end_states.push_back(iter);
					cut_end_scheduler[iter->second.first] = optimal_scheds_vector[i].sched[iter->first]; // original state name  |-> scheduler decision using iterator inside scheduler container
					auto& all_actions_at_this_state = optimal_scheds_vector[i].available_actions_per_state[iter->first];
					cut_end_state_to_action_name[iter->second.first] = all_actions_at_this_state.empty() // trap state
						? "--NONE--" :
						all_actions_at_this_state[cut_end_scheduler[iter->second.first]]; // action name
				}
			}
			standard_logger()->info("The following scheduler decisions are optimal for the cut component:");
			for (const auto& pair : cut_end_state_to_action_name) {
				standard_logger()->info(pair.first + " :   " + pair.second);
			}

			std::map<std::string, std::string> reward_based_scheduler_map;
			for (auto iter = augmented_state_to_pair.begin(); iter != augmented_state_to_pair.end(); ++iter) {

				auto& all_actions_at_this_state = optimal_scheds_vector[i].available_actions_per_state[iter->first];

				reward_based_scheduler_map[iter->first] = all_actions_at_this_state.empty() // trap state
					? "--NONE--" :
					all_actions_at_this_state[optimal_scheds_vector[i].sched[iter->first]]; // action name
			}

			standard_logger()->info("The following scheduler decisions are optimal for the approximation scenario:");
			standard_logger()->info(nlohmann::json(reward_based_scheduler_map).dump(3));


			// check for stabilizing distance...

			rational_type stabilization_distance{ 0 };
			bool abort = false;
			while (!abort && stabilization_distance <= cut_level) {
				rational_type check_stab_distance = stabilization_distance + rational_type{ 1 }; ///#### alllow another distance t364698234764325847

				for (auto iter = augmented_state_to_pair.begin(); iter != augmented_state_to_pair.end(); ++iter) {
					if (iter->second.second >= check_stab_distance && iter->second.second < stabilization_distance) {
						// check if the states in this range are stabilized with cut end scheduler:

						if (
							optimal_scheds_vector[i].sched[iter->first] != cut_end_scheduler[iter->second.first] // if scheduler decides not the stabilized way
							) {
							abort = true;
						}
					}
				}

				if (!abort) {
					stabilization_distance = check_stab_distance;
				}
			}

			// @here we have calculated the distance of stabilization...
			//optimal_cut_end_schedulers_and_stabilization_distance.emplace_back(stabilization_distance, cut_end_scheduler);

			auto message1 = std::string("cut_level:   ") + cut_level.numerator().str() + "/" + cut_level.denominator().str() + "\n";
			auto message2 = std::string("stabilisation distance:   ") + stabilization_distance.numerator().str() + "/" + stabilization_distance.denominator().str() + "\n";
			auto message3 = std::string("cut component scheduler:\n") + nlohmann::json(cut_end_state_to_action_name).dump(3);
			auto message4 = std::string("+++++++++++++++++++++++\n");

			//std::string message3 = "cut_end_scheduler: ...";

			standard_logger()->info(message1 + message2 + message3 + message4);

		}
		std::string message3 = "\n count optimal schedulers: " + std::to_string(number_of_optimal_scheds);
		standard_logger()->info(message3);

		standard_logger()->info("-------------------------------------------------------------------------------------");

		goto before_return;
	}


	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::generatePerformance.data()) {
		standard_logger()->info("Start generating performance data");
		rational_type t;
		rational_type r;
		try {
			json_task_error::check("mode_generatePerformance_has_t", calc_json.contains("t"));
			json_task_error::check("mode_generatePerformance_has_t_string", calc_json.at("t").is_string());
			t = string_to_rational_type(calc_json.at("t").get<std::string>());
			json_task_error::check("mode_generatePerformance_has_r", calc_json.contains("r"));
			json_task_error::check("mode_generatePerformance_has_r_string", calc_json.at("r").is_string());
			r = string_to_rational_type(calc_json.at("r").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 11 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		const auto c{ crinkle(r, t) };


		std::size_t max_states{ 0 };
		std::size_t count_actions{ 0 };
		rational_type probability_unit{ 0 };

		try {
			json_task_error::check("mode_generatePerformance_has_max_states", calc_json.contains("max_states"));
			json_task_error::check("mode_generatePerformance_has_max_states_string", calc_json.at("max_states").is_string());
			max_states = std::stoull(calc_json.at("max_states").get<std::string>());

			json_task_error::check("mode_generatePerformance_has_count_actions", calc_json.contains("count_actions"));
			json_task_error::check("mode_generatePerformance_has_count_actions_string", calc_json.at("count_actions").is_string());
			count_actions = std::stoull(calc_json.at("count_actions").get<std::string>());

			json_task_error::check("mode_generatePerformance_has_probability_unit", calc_json.contains("probability_unit"));
			json_task_error::check("mode_generatePerformance_has_probability_unit_string", calc_json.at("probability_unit").is_string());
			probability_unit = string_to_rational_type(calc_json.at("probability_unit").get<std::string>());
		}
		catch (const json_task_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 8 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}
		catch (rational_parse_error& e) {
			standard_logger()->error(e.what());
			const std::size_t error_code{ 11 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			return error_code;
		}

		standard_logger()->debug(probability_unit.numerator().str());
		standard_logger()->debug(probability_unit.denominator().str());

		// ready with reading input

		std::vector<std::vector<std::chrono::nanoseconds>> mdp_size_to_all_time_durations;

		std::random_device rand;
		for (std::size_t count_states = 2; count_states <= max_states; ++count_states) {
			big_int_type increment{ 1 };
			bool next_mdp = true;
			big_int_type ndet_resolver{ 0 };
			big_int_type min_reward{ -1 };
			big_int_type past_max_reward{ 2 };

			while (next_mdp) {

				// Choose a random mean between 1 and 6
				std::default_random_engine e1(rand());
				std::uniform_int_distribution<int> uniform_dist(0, 9);
				int increment_random = uniform_dist(e1);

				auto resolver = ndet_resolver; // copy
				std::pair<mdp, bool> mdp_and_finished = generate_mdp(count_states, 1, count_actions, probability_unit, resolver, min_reward, past_max_reward);

				//next_mdp = !mdp_and_finished.second;

				ndet_resolver *= 10; // +1
				ndet_resolver += increment_random; // +1
				//increment = increment + (increment / 2);
				//increment += 3;

				if (!next_mdp) {
					continue;
				}

				nlohmann::json json_mdp = mdp_to_json(mdp_and_finished.first);

				//standard_logger()->debug(json_mdp.dump(3));

				mdp recreated_mdp;
				/*
					BUILD UP MDP
				*/
				try {
					check_valid_mdp_and_load_mdp_from_json(json_mdp, recreated_mdp);
				}
				catch (mdp_sanity&) { //invalid mdp skipped
					continue;
				}

				// check:no unreachable states / remove unreachable states and their transitions / rewards....
				// also removes 0-probability transitions!!!
				try {
					remove_unreachable_states<false>(recreated_mdp, true);
				}
				catch (const found_unreachable_state&) {
					continue;
				}
				if (!check_reaching_target_is_guaranteed<false>(recreated_mdp)) {
					continue;
				}


				std::map<std::string, rational_type> generate_delta_max;

				std::chrono::steady_clock::time_point time_stamp_before = std::chrono::steady_clock::now();
				try {
					generate_delta_max = calc_delta_max_state_wise<false>(recreated_mdp, true, true);
				}
				catch (const found_negative_loop&) {
					continue;
				}
				catch (const calc_delta_max_error&) {
					continue;
				}
				// check mdp valid

				//calculate the crinkle here



				mdp n;
				std::vector<std::string> ordered_variables;

				n = unfold<decltype(c), false>(m, c, generate_delta_max, ordered_variables);

				//standard_logger()->trace(mdp_to_json(n).dump(3));

				optimize_scheduler<false>(n, ordered_variables);
				std::chrono::steady_clock::time_point time_stamp_after = std::chrono::steady_clock::now();

				if (next_mdp) { // if not finished
					while (!(mdp_size_to_all_time_durations.size() > count_states))
						mdp_size_to_all_time_durations.emplace_back();
					auto time_delta = time_stamp_after - time_stamp_before;
					mdp_size_to_all_time_durations[count_states].push_back(time_delta);
					// add measure data
					if (mdp_size_to_all_time_durations[count_states].size() > 20) {
						next_mdp = false;
					}
				}
			}

			//standard_logger()->info(std::string("For #states: ") + std::to_string(count_states) + "     tried potential MDPs:   " + ndet_resolver.convert_to<std::string>());
			standard_logger()->info(std::string("Finished for #states: ") + std::to_string(count_states));
			increment = ndet_resolver;
		}


		for (std::size_t num_states = 2; num_states < mdp_size_to_all_time_durations.size(); ++num_states) {
			const auto& all_measures{ mdp_size_to_all_time_durations[num_states] };
			if (all_measures.empty())
				continue;
			std::chrono::nanoseconds average_case{ 0 };
			std::chrono::nanoseconds worst_case{ 0 };
			for (const auto& measure : all_measures) {
				if (measure > worst_case)
					worst_case = measure;

				average_case += measure;
			}
			average_case /= all_measures.size();

			standard_logger()->info(std::string("size= ") + std::to_string(num_states) + "   average= " + std::to_string(average_case.count()) + "   worst= " + std::to_string(worst_case.count()) + "   count= " + std::to_string(all_measures.size()));
		}
		goto before_return;
	}


	// unknown task calc mode -> throw error
	{
		const std::size_t error_code{ 12 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		standard_logger()->error(std::string("json: task.calc.mode   ==   \"") + calc_json.at(keywords::mode).get<std::string>() + "\"");
		return error_code;
	}
	// create unfold-mdp
	//#### will break if in m state names use underscore

	//#### calc all optimal scheduers!!!

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

	// json...
before_return:
	standard_logger()->info(application_errors::application_error_messages[0]); // no error
	return 0;
}

int load_jsons_and_run(const std::vector<std::string>& arguments) { // ready
	std::vector<nlohmann::json> jsons;

	// open files and parse as json
	for (std::size_t i{ 0 }; i < arguments.size(); ++i) {
		try {
			jsons.push_back(load_json<true>(arguments[i]));
			standard_logger()->trace(std::string("file  ") + std::to_string(i) + jsons.back().dump(3));
		}
		catch (const nlohmann::json&) {
			const std::size_t error_code{ 3 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			standard_logger()->error(std::string("file:") + arguments[i]);
			return error_code;
		}
	}

	nlohmann::json merged_json;

	// merge all json files
	try {
		merged_json = merge_json_objects(jsons);
	}
	catch (const json_logic_error& e) {
		standard_logger()->error(e.what());
		const std::size_t error_code{ 4 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		return error_code;
	}

	return run_starting_from_merged_json(merged_json);
}


int main(int argc, char* argv[])// ready
{
	init_logger();

	if (argc < 1) {
		(void)argv;
		// There should be >= 1 argument: the path to the executable
		const std::size_t error_code{ 1 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		standard_logger()->error(std::to_string(argc));
		return error_code;
	}

	if (argc == 1) {
		if constexpr (feature_toggle::RUN_ON_ZERO_ARGUMENTS) {
			const auto example_path = std::string("../../src/test.json");
			const auto b1_path = std::string("../../res/B-1.json");
			const auto task_classic = std::string("../../res/task_classic.json");
			const auto task_crinkle_t3_r10 = std::string("../../res/task_crinkle_t3_r10.json");
			const auto performance = std::string("../../res/performance_run.json");

			auto all_jsons = std::vector<std::string>{ performance }; // { b1_path, task_crinkle_t3_r10 };
			return load_jsons_and_run(all_jsons);
		}
		else {
			const std::size_t error_code{ 2 };
			standard_logger()->error(application_errors::application_error_messages[error_code].data());
			standard_logger()->info(argv[0]);
			return error_code;
		}
	}

	std::vector<std::string> arguments;
	standard_logger()->info("Called mdp-transformer using the following arguments:");
	standard_logger()->info(std::to_string(0) + "   :   " + argv[0]);
	for (int i = 1; i < argc; ++i) {
		arguments.emplace_back(argv[i]);
		standard_logger()->info(std::to_string(i) + "   :   " + arguments.back());
	}

	return load_jsons_and_run(arguments);
}
