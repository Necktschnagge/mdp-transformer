#include "logger.h"
#include "utility.h"
#include "linear_system.h"
#include "mdp_ops.h"
#include "feature_toggle.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

#include <nlohmann/json.hpp>


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
	using scheduler = std::map<std::string, std::size_t>; // stationary scheduler: maps to each state the index of the action chosen in the state 

	std::map<std::string, std::vector<std::string>> available_actions_per_state;
	scheduler sched;

};

std::size_t get_index(const std::vector<std::string>& str_vec, const std::string& s) {
	for (std::size_t i{ 0 }; i < str_vec.size(); ++i) {
		if (str_vec[i] == s) {
			return i;
		}
	}
	throw std::logic_error("String not contained in string vector.");
}

void optimize_scheduler(mdp& m, const std::vector<std::string>& ordered_variables) { // do-check!
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


class application_errors {
public:
	static constexpr std::size_t count_error_codes{ 13 };
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
		std::string_view("Unknown calc mode. Don't know what to do") // 12
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
				standard_logger()->error("Found unreachable states:  ");
				standard_logger()->error(std::string(".....") + state);
			}
			else {
				standard_logger()->warn("Found unreachable states:  ");
				standard_logger()->warn(std::string(".....") + state);
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
			standard_logger()->error(std::string("Found a state from which you cannot reach a target:   ") + pair.first);
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
	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::classic.data()) {

		std::vector<std::string> ordered_variables;
		std::copy(m.states.cbegin(), m.states.cend(), std::back_inserter(ordered_variables));

		optimize_scheduler(m, ordered_variables);
		goto before_return;
	}

	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::crinkle.data()) {

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

		optimize_scheduler(n, ordered_variables);
		goto before_return;
	}

	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::quadratic.data()) {

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

	if (calc_json.at(keywords::mode).get<std::string>() == keywords::value::vVar_approach.data()) {

		rational_type n; // maximum reward steps 
		rational_type seconds; // seconds: if one approximation step takes longer then time, it will be the last one.

		try {
			json_task_error::check("mode_vVar_approach_has_n", calc_json.contains("n"));
			json_task_error::check("mode_vVar_approach_has_n_string", calc_json.at("n").is_string());
			n = string_to_rational_type(calc_json.at("n").get<std::string>());
			json_task_error::check("mode_vVar_approach_has_seconds", calc_json.contains("seconds"));
			json_task_error::check("mode_vVar_approach_has_seconds_string", calc_json.at("seconds").is_string());
			seconds = string_to_rational_type(calc_json.at("seconds").get<std::string>());
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

		rational_type cut_level = std::min(rational_type(0), n);

		while (!cut_level > n)
		{
			mdp stupid_unfolded_mdp;

			std::vector<std::string> ordered_variables;
			stupid_unfolded_mdp = stupid_unfold(m, cut_level, ordered_variables);

				// unfold cut without changed rewards -> unfolded_mdp

				// list all (exponential many) schedulers... -> vec_schedulers

				// for x : vec__scheudlers:
					// solve for expect values... mu
					// modify unfolded_mdp for using poena quadratica (mu)
					// solve for expect values .. result for this scheduler...
		}

/*
		const auto c{identity(t)};

		mdp n;
		std::vector<std::string> ordered_variables;
		standard_logger()->info("Unfolding MDP (stupid mode)...");
		n = unfold(m, c, delta_max, ordered_variables);

		optimize_scheduler(n, ordered_variables);
*/
		goto before_return;
	}


	// unknown task calc mode -> throw error
	const std::size_t error_code{ 12 };
	standard_logger()->error(application_errors::application_error_messages[error_code].data());
	standard_logger()->error(std::string("json: task.calc.mode   ==   \"") + calc_json.at(keywords::mode).get<std::string>() + "\"");
	return error_code;


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
			const auto example_path = std::string("../../src/example-mdp.json");
			return load_jsons_and_run(std::vector<std::string>{example_path});
		}
		const std::size_t error_code{ 2 };
		standard_logger()->error(application_errors::application_error_messages[error_code].data());
		standard_logger()->info(argv[0]);
		return error_code;
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
