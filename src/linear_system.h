#pragma once

#include "custom_types.h"

namespace feature_toggle {
	constexpr bool LINEAR_SYSTEMS_DEBUG_OUTPUT{ true }; //### addd some debug level output
	constexpr bool LINEAR_SYSTEMS_DEBUG_CHECKS{ true };
}

namespace linear_systems {
	using var_id = std::size_t;
	using id_vector = std::vector<var_id>;
	using rational_vector = std::vector<rational_type>;
	using matrix_entry = std::pair<std::size_t, rational_type>;
	using matrix_line = std::vector<matrix_entry>;
	using matrix = std::vector<matrix_line>;
}

void print_mat(const linear_systems::matrix& mat, const linear_systems::rational_vector& r) {
	for (std::size_t i = 0; i < mat.size(); ++i) {
		standard_logger()->trace(std::to_string(i) + ":");
		for (const auto& pair : mat[i]) {
			standard_logger()->trace(std::string("     next:  [") + std::to_string(pair.first) + "] :   " + pair.second.numerator().str() + "/" + pair.second.denominator().str());
		}
		standard_logger()->trace(std::string("     rew:  ") + r[i].numerator().str() + "/" + r[i].denominator().str());
	}
}


class linear_system_error : public std::logic_error {
public:
	template<class ...T>
	linear_system_error(const T&... args) : std::logic_error(args...) {}
};

class resolved_line_of_wrong_size : public linear_system_error {
	std::string make_what_message(const std::string& where_thrown, linear_systems::var_id line, std::size_t actual_size) {
		std::string message{ "A lines was marked resolved and is supposed to have line size 1. The actual size however is " };
		message += std::to_string(actual_size);
		message += ". The line id is " + std::to_string(line) + ".";
		message += "The error was thrown here:   " + where_thrown;
		return message;
	}
public:
	resolved_line_of_wrong_size(const std::string& where_thrown, linear_systems::var_id line, std::size_t actual_size) :
		linear_system_error(make_what_message(where_thrown, line, actual_size)) {}
};

class ill_formed_new_resolved_line : public linear_system_error {
	std::string make_what_message(const std::string& where_thrown, linear_systems::var_id line, std::size_t actual_variable) {
		std::string message{ "A lines was marked resolved but the firt entry is not the diagonal variable. The actual variable however is " };
		message += std::to_string(actual_variable);
		message += ". The line id is " + std::to_string(line) + ".";
		message += "The error was thrown here:   " + where_thrown;
		return message;
	}
public:
	ill_formed_new_resolved_line(const std::string& where_thrown, linear_systems::var_id line, std::size_t actual_variable) :
		linear_system_error(make_what_message(where_thrown, line, actual_variable)) {}
};

class unexpected_zero_coefficient : public linear_system_error {
	std::string make_what_message(const std::string& where_thrown, linear_systems::var_id line, std::size_t variable_id) {
		std::string message{ "A coefficient should not be 0 but it is.   Line:  " };
		message += std::to_string(line);
		message += "   variable_id:  " + std::to_string(variable_id);
		message += "The error was thrown here:   " + where_thrown;
		return message;
	}
public:
	unexpected_zero_coefficient(const std::string& where_thrown, linear_systems::var_id line, std::size_t variable_id) :
		linear_system_error(make_what_message(where_thrown, line, variable_id)) {}
};

class unable_to_move_from_unresolved_to_resolved : public linear_system_error {
	std::string make_what_message(const std::string& where_thrown, linear_systems::var_id the_variable) {
		std::string message{ "Cannot move the variable >>" };

		message += std::to_string(the_variable) + "<<.";
		message += "The variable is not contained in the unresolved ones. ";
		message += "The error was thrown here:   " + where_thrown;
		return message;
	}
public:
	unable_to_move_from_unresolved_to_resolved(const std::string& where_thrown, linear_systems::var_id the_variable) :
		linear_system_error(make_what_message(where_thrown, the_variable)) {}
};

/*
	@param where_is_x_in_unresolved_variables must be valid iterator for unresolved_variables, but no end iterator.
	@param x_id must be the value at where where_is_x_in_unresolved_variables is pointing to
*/
inline void inline_move_resolved_line(
	linear_systems::id_vector& unresolved_variables,
	linear_systems::id_vector& resolved_variables,
	linear_systems::id_vector::const_iterator where_is_x_in_unresolved_variables,
	linear_systems::var_id x_id
) {
	unresolved_variables.erase(where_is_x_in_unresolved_variables);
	resolved_variables.push_back(x_id);
}

/*
	@param where_is_x_in_unresolved_variables must be valid iterator for unresolved_variables, but no end iterator.
*/
inline void inline_move_resolved_line(
	linear_systems::id_vector& unresolved_variables,
	linear_systems::id_vector& resolved_variables,
	linear_systems::id_vector::const_iterator where_is_x_in_unresolved_variables
) {
	const linear_systems::var_id x_id{ *where_is_x_in_unresolved_variables }; // check for valid iterator? or add it as call-constraint
	unresolved_variables.erase(where_is_x_in_unresolved_variables);
	resolved_variables.push_back(x_id);
}

inline void inline_move_resolved_line(
	linear_systems::id_vector& unresolved_variables,
	linear_systems::id_vector& resolved_variables,
	linear_systems::var_id x_id
) {
	auto where_is_x_in_unresolved_variables = std::find(unresolved_variables.cbegin(), unresolved_variables.cend(), x_id);
	if constexpr (feature_toggle::LINEAR_SYSTEMS_DEBUG_CHECKS) {
		if (where_is_x_in_unresolved_variables == unresolved_variables.cend()) {
			throw unable_to_move_from_unresolved_to_resolved("Move a single line from resolved to unresolved", x_id);
		}
	}
	unresolved_variables.erase(where_is_x_in_unresolved_variables);
	resolved_variables.push_back(x_id);
}

inline bool compare_id_rational_pair(const linear_systems::matrix_entry& l, const linear_systems::matrix_entry& r) {
	return l.first < r.first;
}

inline void inline_normalize_resolved_line(
	linear_systems::matrix& P_table,
	linear_systems::rational_vector& rew_vector,
	linear_systems::var_id id_resolved
) {
	if constexpr (feature_toggle::LINEAR_SYSTEMS_DEBUG_CHECKS) {
		if (P_table[id_resolved].size() != 1) {
			throw resolved_line_of_wrong_size("Normalizing a resolved line", id_resolved, P_table[id_resolved].size());
		}
		if (P_table[id_resolved][0].first != id_resolved) {
			throw ill_formed_new_resolved_line("Normalizing a resolved line", id_resolved, P_table[id_resolved][0].first);
		}
		if (P_table[id_resolved][0].second == rational_type(0)) {
			throw unexpected_zero_coefficient("Normalizing a resolved line", id_resolved, id_resolved);
		} // means we have ambiguous solutions...
	}
	rew_vector[id_resolved] /= P_table[id_resolved][0].second;
	P_table[id_resolved][0].second = 1;
}


/*
	see requirements in the checks of the first lines!
	@after x_j in line_i will have an entry with 0-coefficient, others might have a 0-coefficient too!
*/
inline void resolve_x_j_in_line_i_using_line_j(
	linear_systems::matrix& P_table, // lines are sorted and kept sorted
	linear_systems::rational_vector& r,
	linear_systems::var_id line_i,
	linear_systems::var_id line_j,
	linear_systems::matrix_line::iterator iter_on_x_j_inside_line_i
) {
	using namespace linear_systems;

	const auto iter_on_x_j_inside_line_j = std::lower_bound(P_table[line_j].begin(), P_table[line_j].end(), std::make_pair(line_j, rational_type()), compare_id_rational_pair);

	if (feature_toggle::LINEAR_SYSTEMS_DEBUG_CHECKS) {
		if (iter_on_x_j_inside_line_j == P_table[line_j].end()) {
			throw linear_system_error("Should apply resolve_x_j_in_line_i_using_line_j but x_j in line_j does not exist");
		}
		if (iter_on_x_j_inside_line_j->second == rational_type(0)) {
			throw linear_system_error("Should apply resolve_x_j_in_line_i_using_line_j but x_j in line_j has zero coefficient");
		}
		// remark both are forbidden per desgin of probability matrices.
	}
	const rational_type equation_multiply_factor{ iter_on_x_j_inside_line_i->second / iter_on_x_j_inside_line_j->second };
	// resolve the variable "line_j" in equation "line_i"
	// P_table[line_i] -= equation_multiply_factor * "line_j" // -> coefficient of line_j in line_i will be zero.
	matrix_line::iterator iter{ P_table[line_i].begin() };
	matrix_line::iterator jter{ P_table[line_j].begin() };
	r[line_i] -= r[line_j] * equation_multiply_factor;

	matrix_line the_new_line_i;
	while (true) {
		if (iter == P_table[line_i].end()) {
			for (; jter != P_table[line_j].end(); ++jter) {
				the_new_line_i.push_back(*jter);
				the_new_line_i.back().second *= rational_type(-1) * equation_multiply_factor;
			}
			P_table[line_i] = std::move(the_new_line_i);
			return;
		}
		if (jter == P_table[line_j].end()) {
			for (; iter != P_table[line_i].end(); ++iter) {
				the_new_line_i.push_back(*iter);
			}
			P_table[line_i] = std::move(the_new_line_i);
			return;
		}
		if (iter->first < jter->first) {
			// only line_i has an entry for this variable..
			the_new_line_i.push_back(*iter);
			++iter;
			continue;
		}
		if (iter->first > jter->first) {
			// only "line_j" has an entry for this variable
			the_new_line_i.push_back(*jter);
			the_new_line_i.back().second *= rational_type(-1) * equation_multiply_factor;
			++jter;
			continue;
		}
		if (iter->first == jter->first) {
			the_new_line_i.push_back(*iter);
			the_new_line_i.back().second -= equation_multiply_factor * jter->second;
			++iter;
			++jter;
			continue;
		}
	}
}

inline void apply_resolved_variables_on_unresolved_ones(
	linear_systems::matrix& P, // lines are ordered, order is kept
	linear_systems::rational_vector& r,
	linear_systems::id_vector& unresolved, // they have an external order. it should be kept.
	linear_systems::id_vector& resolved,
	linear_systems::id_vector& done// is not required to be ordered.
) {
rerun_apply_resolved_variables:

	using namespace linear_systems;
	id_vector unresolved_to_resolved; // for lines that just became resolved

	// apply resolved lines to all unresolved lines:
	for (const auto& resolved_id : resolved) {
		for (const auto& unresolved_line : unresolved) {
			auto found = std::lower_bound(
				P[unresolved_line].cbegin(),
				P[unresolved_line].cend(),
				std::make_pair(resolved_id, rational_type(0)),
				compare_id_rational_pair);
			if (found != P[unresolved_line].cend() && found->first == resolved_id) {
				r[unresolved_line] -= r[resolved_id] * found->second; // does not matter if the coefficient was zero.
				P[unresolved_line].erase(found);
				if (P[unresolved_line].size() < 2) {
					// the line is resolved;
					unresolved_to_resolved.push_back(unresolved_line);
				}
			}
		}
	}

	if (!unresolved_to_resolved.empty()) {

		// normalize the new resolved lines, do checks if enables
		for (const auto& movee : unresolved_to_resolved) {
			if constexpr (feature_toggle::LINEAR_SYSTEMS_DEBUG_CHECKS) {
				if (P[movee].size() != 1) {
					throw resolved_line_of_wrong_size(
						"Moving newly resolved lines inside apply_resolved_variables_on_unresolved_ones",
						movee,
						P[movee].size());
				}
				if (P[movee][0].first != movee) {
					throw ill_formed_new_resolved_line(
						"Moving newly resolved lines inside apply_resolved_variables_on_unresolved_ones",
						movee,
						P[movee][0].first);
				}
			}
			inline_normalize_resolved_line(P, r, movee);
		}

		// remove from unresolved, put into resolved:
		std::copy(resolved.cbegin(), resolved.cend(), std::back_inserter(done));
		resolved = std::move(unresolved_to_resolved);
		std::sort(resolved.begin(), resolved.end());
		auto new_end = std::copy_if(
			unresolved.cbegin(),
			unresolved.cend(),
			unresolved.begin(),
			[&resolved](const std::size_t& var_id) -> bool {
				const auto x{ std::lower_bound(resolved.cbegin(), resolved.cend(), var_id) };
				return /* val not in resolved */  !(x != resolved.cend() && (*x == var_id));
			});
		unresolved.erase(new_end, unresolved.cend());

		goto rerun_apply_resolved_variables; // since we got new resolved equation whe should apply them to the rest.
	}

	std::copy(resolved.cbegin(), resolved.cend(), std::back_inserter(done));
	resolved.clear();
}

/*
	@param P must be a quadratic matrix for solving Px = r, P is a std::vector of its matrix lines.
	@param r
	@param unresolved must contain all variable ids where there lines are not trivial like p * x_i = r_i.
			Its order is crucial for the performance of solving.
			As far as possible a variable x that depends on y should appear prior to y in this vector.
	@param resolved should be disjoint with unresolved. Together both vectors need to cover all variables. Lines must be in trivial form p * x_i = r_i.
*/
inline void solve_linear_system_dependency_order_optimized(
	linear_systems::matrix P,
	linear_systems::rational_vector& r,
	linear_systems::id_vector unresolved, // they have an external order. it should be kept.
	linear_systems::id_vector resolved // is not required to be ordered.
) {
	using namespace linear_systems;

	rational_vector& rew_vector{ r };
	id_vector done; // not sorted
	// node s   |->   {(s1', r1) (s2',r2) (s3',r3), (self,-1)} "=  r_alpha"


	// normailze resolved lines (the caller does not need to satisfy this condition):
	for (const auto& resolved_id : resolved) {
		inline_normalize_resolved_line(P, r, resolved_id);
	}

	// sort all lines of P (the caller does not need to satisfy this condition):
	for (std::size_t i{ 0 }; i < P.size(); ++i) {
		std::sort(P[i].begin(), P[i].end(), compare_id_rational_pair);
	}

	/*
		inner constraints:
			* The order of unresolved has to be kept. The last elements have priority for resolving.
			* All resolved lines are normalized
			* lines inside P are sorted
	*/


rerun__solve_linear_system_dependency_order_optimized:

	apply_resolved_variables_on_unresolved_ones(P, r, unresolved, resolved, done);

	/*
		Here we do not have any resolved equations ($a_{m,n} * x_i = r_i$) anymore that we can apply to the unresolved equations.
		All coefficients of resolved variables in unresolved equations are zero.
		The corresponding pairs in the sparse matrix are erased.
	*/
	if (!unresolved.empty()) { // else we are done
		/*
			Based on an ordering of the variables, we choose the one where we most likely expect
			that we can resolve this variable by only using a few equations of the whole system.
		*/
		const std::size_t high_prio_select{ unresolved.back() };

		/*
			We will attempt to resolve the variable "high_prio_select".
			In order to achieve this goal, we will select other variables' equations one by one
			to be able to get rid of non-zero coefficients inside the equation of "high_prio_select".
			We try to choose as few as needed to resolve "high_prio_select".
			We depend on these equtions for solving so we call them "dependents".
			However if once we see that we suddenly resolved a different variable we will stop
			resolving "high_prio_select" and jump to the beginning of our strategy.
			Stop resolving "high_prio_select" does not mean that have have done some work without effect.
			Next time we enter this section we will per design select the same "high_prio_select" again until it becomes resolved.

			The dependent equations, to be more precise say >their id< will be push_back()ed into the resolve_stack:
		*/
		std::vector<std::size_t> resolve_stack;
		/*
			However each time we push a new equation to the resolve_stack,
			it must have been reduced by all the previous elements of resolve_stack.
			So the new equation's coefficients of variables which where previously pushed back to "resolve_stack" are zero.
		*/

		while (true) // while true do some operations on the equations until finally jumping out.
		{
		begin_of_inner_while:
			// check high_prio_select for already being resolved...
			if (P[high_prio_select].size() < 2) {
				// high_prio_select already resolved!

				inline_normalize_resolved_line(P, rew_vector, high_prio_select);
				inline_move_resolved_line(unresolved, resolved, unresolved.cend() - 1);
				goto rerun__solve_linear_system_dependency_order_optimized; // jump to first part of our strategy.
			}

			// select a line that high_prio_select depends on...
			size_t select_next_dependent_line;
			matrix_line::iterator iter_of_next_dependent_inside_high_prio_select;
			for (auto iter = P[high_prio_select].begin(); true; ++iter) {
				if (iter == P[high_prio_select].end())
					// we should only reach this, if we erased elements (~7 lines below) so we cannot find something to resolve.
					// high_prio_select is supposed to be resolved here...
					//#### it is a point of possible inifinite loop here.... we should check here for being resolved, otherwise throw error, not solvable... (?), but of course
					//#### per Design of probability LGS this case does not happen
					goto begin_of_inner_while;
				if (iter->first == high_prio_select) {
					continue; // found diagonal entry, we cannot select this.
				}
				// we found some non-diagonal entry
				if (iter->second == rational_type(0)) {
					P[high_prio_select].erase(iter); // invalidates iterators -> need to jumpo out!!!!
					goto begin_of_inner_while;
				}
				// we found some non-diagonal, non-zero entry, so choose this one
				select_next_dependent_line = iter->first;
				iter_of_next_dependent_inside_high_prio_select = iter;
				break;
			}

			// per design we do not choose a line already contained in stack (these lines variables are already resolved inside line high_prio_select)
			if constexpr (feature_toggle::LINEAR_SYSTEMS_DEBUG_CHECKS) {
				if (std::find(resolve_stack.cbegin(), resolve_stack.cend(), select_next_dependent_line) != resolve_stack.cend()) {
					throw linear_system_error("Algorithm Design error: We choose a dependent line that was already chosen some time before.");
				}
			}

			for (const auto& stack_line : resolve_stack) {

			solve_linear_system_dependency_order_optimized___repeat_apply_previous_stack_line:
				const auto x_stack_line_in_select_next_dependent_line = std::lower_bound(
					P[select_next_dependent_line].begin(),
					P[select_next_dependent_line].end(),
					std::make_pair(stack_line, rational_type(0)),
					compare_id_rational_pair);

				if (x_stack_line_in_select_next_dependent_line != P[select_next_dependent_line].end() && x_stack_line_in_select_next_dependent_line->first == stack_line) { // if line is a variable in select_next_dependent_line
					if (x_stack_line_in_select_next_dependent_line->second == rational_type(0)) { // if there is some 0-entry, remove it.
						P[select_next_dependent_line].erase(x_stack_line_in_select_next_dependent_line); // invalidates iterators on P_table[select_next_dependent_line]!
						if (P[select_next_dependent_line].size() < 2) {
							// select_next_dependent_line became resolved.
							inline_normalize_resolved_line(P, rew_vector, select_next_dependent_line);
							inline_move_resolved_line(unresolved, resolved, select_next_dependent_line);
							goto rerun__solve_linear_system_dependency_order_optimized;
						}
						goto solve_linear_system_dependency_order_optimized___repeat_apply_previous_stack_line; // need to jump because of invalidates oterators
					}
					resolve_x_j_in_line_i_using_line_j(
						P,
						r,
						select_next_dependent_line,
						stack_line,
						x_stack_line_in_select_next_dependent_line);
				}
			}

			if (P[select_next_dependent_line].size() < 2) {
				inline_normalize_resolved_line(P, rew_vector, select_next_dependent_line);
				inline_move_resolved_line(unresolved, resolved, select_next_dependent_line);
				goto rerun__solve_linear_system_dependency_order_optimized;
			}

			// resolve high_prio_select using select_next_dependent_line:
			resolve_x_j_in_line_i_using_line_j(
				P,
				r,
				high_prio_select,
				select_next_dependent_line,
				iter_of_next_dependent_inside_high_prio_select);
			// REMARK: Check "high_prio_select resolved?" is done at the top of next while(true) iteration

			resolve_stack.push_back(select_next_dependent_line);
		} // while (true)
	}

}
