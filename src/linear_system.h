#pragma once

#include "custom_types.h"

namespace feature_toggle {
	constexpr bool LINEAR_SYSTEMS_DEBUG_OUTPUT{ true };
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

#if false
template <class _Corpus>
class Mat {
public:
	using area = std::vector<std::vector<_Corpus>>;
	area m;

	Mat(std::size_t m, std::size_t n) : m(area(std::vector<_Corpus>(_Corpus(0), n), m)) {
	}
};

template<class _Corpus>
Mat<_Corpus> enhanced_solve_linear_system(Mat<_Corpus> A, Mat<_Corpus> b, Mat<bool> target) {
	/*
	Ax = b
	A: m x n
	b: n x 1
	x: m x 1
	ready: m x 1
	*/

	// check:
	A.area.size() != 0;
	A.area[0].size() == b.area.size();
	ready.area.size() == A.area.size();

	Mat<bool> ready = target;

}

#endif

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
	// move resolved line into resolved...
	auto where_is_x_in_unresolved_variables = std::find(unresolved_variables.cbegin(), unresolved_variables.cend(), x_id);//### check this whole line! 
	// unresolved darf nicht sorted sein, check for find complies with unsorted container best##########
	unresolved_variables.erase(where_is_x_in_unresolved_variables); // what in case of end iterator! throw an error!
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
	// normalize the resolved line:
	//### check resolved line has zero entries.. .empty()
	//### check resolved line has entry, but it is zero.
	// then throw anbiguous error
	rew_vector[id_resolved] /= P_table[id_resolved][0].second;
	P_table[id_resolved][0].second = 1;
}


inline void resolve_x_j_in_line_i_using_line_j(
	linear_systems::matrix& P_table,
	linear_systems::var_id line_i,
	linear_systems::var_id line_j,
	linear_systems::matrix_line::iterator iter_on_x_j_inside_line_i
) {

	const auto iter_on_x_j_inside_line_j = std::lower_bound(P_table[line_j].begin(), P_table[line_j].end(), std::make_pair(line_j, rational_type()), compare_id_rational_pair);

	if (iter_on_x_j_inside_line_j == P_table[line_j].end()) {
		throw 1; // thow error #################
	}
	if (iter_on_x_j_inside_line_j->second == rational_type(0)) {
		throw 2; // throw error ##############
	}
	const rational_type equation_multiply_factor{ iter_on_x_j_inside_line_i->second / iter_on_x_j_inside_line_j->second };
	// resolve the variable "line_j" in equation "select_next_dependent_line"
	// P_table[select_next_dependent_line] -= a * "line_j" // -> coefficient of x_stack_line in select_next_dependent_line will be zero.
	auto iter{ P_table[line_i].begin() };
	auto jter{ P_table[line_j].begin() };
	std::vector<std::pair<std::size_t, rational_type>> the_new_line_i;
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
			// only select_next_dependent_line has an entry for this variable..
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
	linear_systems::matrix& P,
	linear_systems::rational_vector& r,
	linear_systems::id_vector& unresolved, // they have an external order. it should be kept.
	linear_systems::id_vector& resolved,
	linear_systems::id_vector& done// is not required to be ordered.
) {
	using namespace linear_systems;
	id_vector move_from_unresolved_to_resolved; // for lines that just became resolved

rerun_apply_resolved_variables:
	// apply resolved lines to all unresolved lines:
	for (const auto& resolved_id : resolved) {
		for (const auto& unresolved_line : unresolved) {
			auto found = std::lower_bound(
				P[unresolved_line].cbegin(),
				P[unresolved_line].cend(),
				std::make_pair(resolved_id, rational_type(0)),
				compare_id_rational_pair);
			if (found != P[unresolved_line].cend()) {
				r[unresolved_line] -= r[resolved_id] * found->second;
				P[unresolved_line].erase(found);
				if (P[unresolved_line].size() < 2) {
					// the line is resolved;
					move_from_unresolved_to_resolved.push_back(unresolved_line);
				}
			}
		}
	}

	if (!move_from_unresolved_to_resolved.empty()) {

		// normalize the new resolved lines
		for (const auto& movee : move_from_unresolved_to_resolved) {
			// check: 
			if (P[movee][1].first != movee) {
				throw 0;
			}
			// use normalize_resolved_line!!!#######
			r[movee] /= P[movee][1].second; // #### impossibly a zero... (only if broken)
			P[movee][1].second = rational_type(1);
		}

		// remove from unresolved, put into resolved:
		std::copy(resolved.cbegin(), resolved.cend(), std::back_inserter(done));
		resolved = std::move(move_from_unresolved_to_resolved);
		std::sort(resolved.begin(), resolved.end());
		auto new_end = std::copy_if(
			unresolved.cbegin(),
			unresolved.cend(),
			unresolved.begin(),
			[&resolved](const std::size_t& val) -> bool {
				const auto x{ std::lower_bound(resolved.cbegin(), resolved.cend(), val) };
				return /* val not in resolved */  !(x != resolved.cend() && (*x == val)); // is this extra == needed for cheking if value is present?
			});
		unresolved.erase(new_end, unresolved.cend());

		goto rerun_apply_resolved_variables;
	}
	resolved.clear();
}

/*
	@param P must be a quadratic matrix for solving Px = r, P is a std::vector of its matrix lines.
	@param r
	@param unresolved must contain all variable ids where there lines are not trivial like p * x_i = r_i.
			Its order is crucial for the performance of solving.
			As far as possible a variable x that depends on y should appear prior to y in this vector.
	@param resolved should be disjoint with unresolved. Together both vectors need to cover all variables.
*/
inline void solve_linear_system_dependency_order_optimized(
	linear_systems::matrix P,
	linear_systems::rational_vector r,
	linear_systems::id_vector unresolved, // they have an external order. it should be kept.
	linear_systems::id_vector resolved // is not required to be ordered.
) {
	using namespace linear_systems;

	matrix& P_table{ P }; // must be sorted.
	rational_vector& rew_vector{ r };
	id_vector done; // not sorted
	// node s   |->   {(s1', r1) (s2',r2) (s3',r3), (self,-1)} "=  r_alpha"

	/* inner constraints:
		* The order of unresolved has to be kept. The last elements have priority for resolving.
	*/

alternate_solve_linear_system___rerun:
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
			if (P_table[high_prio_select].size() < 2) {
				// high_prio_select already resolved!

				inline_normalize_resolved_line(P_table, rew_vector, high_prio_select);
				inline_move_resolved_line(unresolved, resolved, unresolved.cend() - 1);
				goto alternate_solve_linear_system___rerun; // jump to first part of our strategy.
			}

			// select a line that high_prio_select depends on...
			size_t select_next_dependent_line;
			matrix_line::iterator iter_of_next_dependent_inside_high_prio_select;
			for (auto iter = P_table[high_prio_select].begin(); true; ++iter) {
				if (iter != P_table[high_prio_select].end())
					goto begin_of_inner_while;
				if (iter->first == high_prio_select) {
					continue; // found diagonal entry
				}
				if (iter->second == rational_type(0)) {
					P_table[high_prio_select].erase(iter); // invalidates iterators!!!!
					goto begin_of_inner_while;
				}
				select_next_dependent_line = iter->first;
				iter_of_next_dependent_inside_high_prio_select = iter;
				break;
			}

			// per design we do not choose a line already contained in stack (check this in a debug mode...)

			//const std::size_t select_next_dependent_line{ iter->first };

			for (const auto& stack_line : resolve_stack) {
				std::sort(P_table[select_next_dependent_line].begin(), P_table[select_next_dependent_line].end(), compare_id_rational_pair);
				std::sort(P_table[stack_line].begin(), P_table[stack_line].end(), compare_id_rational_pair); // question: optimization: can we sort it when it is pushed into resolve_stack and keep it sorted? ######

			alternate_solve_linear_system___repeat_apply_previous_stack_line:
				const auto x_stack_line_in_select_next_dependent_line = std::lower_bound(
					P_table[select_next_dependent_line].begin(),
					P_table[select_next_dependent_line].end(),
					std::make_pair(stack_line, rational_type(0)),
					compare_id_rational_pair);
				// if line is a variable in select_next_dependent_line
				if (x_stack_line_in_select_next_dependent_line != P_table[select_next_dependent_line].end() && x_stack_line_in_select_next_dependent_line->first == stack_line) {
					if (x_stack_line_in_select_next_dependent_line->second == rational_type(0)) { // if there is some 0-entry, remove it.
						P_table[select_next_dependent_line].erase(x_stack_line_in_select_next_dependent_line); // invalidates iterators on P_table[select_next_dependent_line]!
						if (P_table[select_next_dependent_line].size() < 2) {
							// select_next_dependent_line became resolved.
							inline_normalize_resolved_line(P_table, rew_vector, select_next_dependent_line);
							//##### can we assume that all resolved lines are always normalized? then we might avoid some addiitonal division operations.
							inline_move_resolved_line(unresolved, resolved, select_next_dependent_line); // ####overload the function that it can find the correct iterator itself by searching id inside the unresolved vector...
							goto alternate_solve_linear_system___rerun;
						}
						goto alternate_solve_linear_system___repeat_apply_previous_stack_line; // need to jump because of invalidates oterators
					}
					resolve_x_j_in_line_i_using_line_j(
						P_table,
						select_next_dependent_line,
						stack_line,
						x_stack_line_in_select_next_dependent_line);
				}
			} // for (const auto& stack_line : resolve_stack)
			// check if select_next_dependent_line got resolved -> rerun #############################
			if (P_table[select_next_dependent_line].size() < 2) {
				inline_normalize_resolved_line(P_table, rew_vector, select_next_dependent_line);
				inline_move_resolved_line(unresolved, resolved, select_next_dependent_line);
				goto alternate_solve_linear_system___rerun;
			}

			// resolve high_prio_select using select_next_dependent_line:
			resolve_x_j_in_line_i_using_line_j(
				P_table,
				high_prio_select,
				select_next_dependent_line,
				iter_of_next_dependent_inside_high_prio_select);

			resolve_stack.push_back(select_next_dependent_line);
		}
	}


	/*
	move s into done(= solved and applied into all other equations...)
		if not yet(not resolved empty) {
			solve the highest prio line using only lines which are dependet.
		}

	for (s in targets + already solved) {
		for (for t in not- yet - solved) {
			resolve s in the line of t.
				if t resolved - move t to resolved ones.
		}
		move s into done(= solved and applied into all other equations...)
	}
	if not yet(not resolved empty) {
		solve the highest prio line using only lines which are dependet.
	}
	*/

}
