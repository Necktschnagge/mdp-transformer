#pragma once

#include "custom_types.h"

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

inline void inline_move_resolved_line(
	std::vector<std::size_t>& unresolved,
	std::vector<std::size_t>& resolved,
	std::vector<std::size_t>::const_iterator iterator_inside_vector_unresolved,
	std::size_t var_eq_id
) {
	// move resolved line into resolved...
	unresolved.erase(iterator_inside_vector_unresolved);
	resolved.push_back(var_eq_id);
}


inline void inline_move_resolved_line(
	std::vector<std::size_t>& unresolved,
	std::vector<std::size_t>& resolved,
	std::vector<std::size_t>::const_iterator iterator_inside_vector_unresolved
) {
	const std::size_t var_eq_id{ *iterator_inside_vector_unresolved }; // check for valid iterator? or add it as call-constraint
	// move resolved line into resolved...
	unresolved.erase(iterator_inside_vector_unresolved);
	resolved.push_back(var_eq_id);
}

inline void inline_move_resolved_line(
	std::vector<std::size_t>& unresolved,
	std::vector<std::size_t>& resolved,
	std::size_t var_eq_id
) {
	// move resolved line into resolved...
	auto iterator_inside_vector_unresolved = std::find(unresolved.cbegin(), unresolved.cend(), var_eq_id);//### check this whole line! 
	// unresolved darf nicht sorted sein, check for find complies with unsorted container best##########
	unresolved.erase(iterator_inside_vector_unresolved); // what in case of end iterator! throw an error!
	resolved.push_back(var_eq_id);
}

inline auto compare_id_rational_pair(const std::pair<std::size_t, rational_type>& l, const std::pair<std::size_t, rational_type>& r) -> bool {
	return l.first < r.first;
}

inline void inline_normalize_resolved_line(
	std::vector<std::vector<std::pair<std::size_t, rational_type>>>& P_table,
	std::vector<rational_type>& rew_vector,
	std::size_t id_resolved
) {
	// normalize the resolved line:
	//### check resolved line has zero entries.. .empty()
	//### check resolved line has entry, but it is zero.
	// then throw anbiguous error
	rew_vector[id_resolved] /= P_table[id_resolved][0].second;
	P_table[id_resolved][0].second = 1;
}


inline void resolve_x_j_in_line_i_using_line_j(
	std::vector<std::vector<std::pair<std::size_t, rational_type>>>& P_table,
	std::size_t line_i,
	std::size_t line_j,
	std::vector<std::pair<std::size_t, rational_type>>::iterator iter_on_x_j_inside_line_i
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

template <class _Corpus>
inline Mat<_Corpus> alternate_solve_linear_system() {
	std::vector<std::vector<std::pair<std::size_t, rational_type>>> P_table; // must be sorted.
	std::vector<rational_type> rew_vector;

	std::vector<std::size_t> unresolved; // they have an external order. it should be kept.
	std::vector<std::size_t> resolved; // is not required to be ordered.
	std::vector<std::size_t> done; // not sorted
	std::vector<std::size_t> move_from_unresolved_to_resolved; // for lines that just became resolved

	static const auto move_resolved_line{
		[&](std::vector<std::size_t>::const_iterator iterator_inside_vector_unresolved, std::size_t var_eq_id = *iter) {
			// move resolved line into resolved...
			unresolved.erase(iterator_inside_vector_unresolved);
			resolved.push_back(var_eq_id);
		}
	};

	// node s   |->   {(s1', r1) (s2',r2) (s3',r3), (self,-1)} "=  r_alpha"

alternate_solve_linear_system___rerun:
	// apply resolved lines to all unresolved lines:
	for (const auto& resolved_id : resolved) {
		for (const auto& unresolved_line : unresolved) {
			auto found = std::lower_bound(
				P_table[unresolved_line].cbegin(),
				P_table[unresolved_line].cend(),
				resolved_id,
				compare_entry_pair);
			if (found != P_table[unresolved_line].cend()) {
				rew_vector[unresolved_line] -= rew_vector[resolved_id] * found->second;
				P_table[unresolved_line].erase(found);
				if (P_table[unresolved_line].size() < 2) {
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
			if (P_table[movee][1].first != movee) {
				throw 0;
			}
			// use normalize_resolved_line!!!#######
			rew_vector[movee] /= P_table[movee][1].second; // #### impossibly a zero... (only if broken)
			P_table[movee][1].second = rational_type(1);
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

		goto alternate_solve_linear_system___rerun;
	}
	resolved.clear();

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
			// check high_prio_select for already being resolved...
			if (P_table[high_prio_select].size() < 2) {
				// high_prio_select already resolved!

				inline_normalize_resolved_line(P_table, rew_vector, high_prio_select);
				inline_move_resolved_line(unresolved, resolved, unresolved.cend() - 1);
				goto alternate_solve_linear_system___rerun; // jump to first part of our strategy.
			}

			// select a line that high_prio_select depends on...
			const decltype(P_table[high_prio_select])::const_iterator select_next_dependent_line_it{ // we don't need that iterator.#####
				[&]() {
					for (auto iter = P_table[high_prio_select].cbegin(); iter != P_table[high_prio_select].cend(); ++iter) {
						if (iter->first == high_prio_select) {
							continue; // found diagonal entry
						}
						if (iter->second == rational_type(0)) {
							P_table[high_prio_select].erase(iter); // invalidates iterators!!!!
							continue;
						}
						return iter;
					}
				}()
			};
			const std::size_t select_next_dependent_line{ iter->first };

			for (const auto& stack_line : resolve_stack) {
				std::sort(P_table[select_next_dependent_line].begin(), P_table[select_next_dependent_line].end(), compare_entry_pair; );
				std::sort(P_table[stack_line].begin(), P_table[stack_line].end(), compare_entry_pair); // question: optimization: can we sort it when it is pushed into resolve_stack and keep it sorted? ######

			alternate_solve_linear_system___repeat_apply_previous_stack_line:
				const auto x_stack_line_in_select_next_dependent_line = std::lower_bound(
					P_table[select_next_dependent_line].begin(),
					P_table[select_next_dependent_line].end(),
					stack_line,
					compare_entry_pair);
				// if line is a variable in select_next_dependent_line
				if (x_stack_line_in_select_next_dependent_line != P_table[select_next_dependent_line].end() && x_stack_line_in_select_next_dependent_line->first == stack_line) {
					if (x_stack_line_in_select_next_dependent_line->second == rational_type(0)) { // if there is some 0-entry, remove it.
						P_table[select_next_dependent_line].erase(x_stack_line_in_select_next_dependent_line); // invalidates iterators on P_table[select_next_dependent_line]!
						if (P_table[select_next_dependent_line].size() < 2) {
							// select_next_dependent_line became resolved.
							normalize_resolved_line(select_next_dependent_line);
							//##### can we assume that all resolved lines are always normalized? then we might avoid some addiitonal division operations.
							inline_move_resolved_line(select_next_dependent_line); // ####overload the function that it can find the correct iterator itself by searching id inside the unresolved vector...
							goto alternate_solve_linear_system___rerun;
						}
						goto alternate_solve_linear_system___repeat_apply_previous_stack_line; // need to jump because of invalidates oterators
					}
					resolve_x_j_in_line_i_using_line_j(
						P_table,
						select_next_dependent_line,
						stack_line,
						x_stack_line_in_select_next_dependent_line);
#if false
					const auto stack_line_diagonal_element = std::lower_bound(
						P_table[stack_line].begin(),
						P_table[stack_line].end(),
						stack_line,
						compare_id_rational_pair);
					if (stack_line_diagonal_element == P_table[stack_line].end()) {
						throw 1; // thow error #################
					}
					if (stack_line_diagonal_element->second == rational_type(0)) {
						throw 2; // throw error ##############
					}
					const rational_type equation_multiply_factor{ x_stack_line_in_select_next_dependent_line->second / stack_line_diagonal_element->second };
					// resolve the variable "stack_line" in equation "select_next_dependent_line"
					// P_table[select_next_dependent_line] -= a * "stack_line" // -> coefficient of x_stack_line in select_next_dependent_line will be zero.
					auto iter{ P_table[select_next_dependent_line].begin() };
					auto jter{ P_table[stack_line].begin() };
					std::vector<std::pair<std::size_t, rational_type>> reduced_next_dependent_line;
					while (true) {
						if (iter == P_table[select_next_dependent_line].end()) {
							for (; jter != P_table[stack_line].end(); ++jter) {
								reduced_next_dependent_line.push_back(*jter);
								reduced_next_dependent_line.back().second *= rational_type(-1) * equation_multiply_factor;
							}
							goto while_end;
						}
						if (jter == P_table[stack_line].end()) {
							for (; iter != P_table[select_next_dependent_line].end()) {
								reduced_next_dependent_line.push_back(*iter)
							}
							goto while_end;
						}
						if (iter->first < jter->first) {
							// only select_next_dependent_line has an entry for this variable..
							reduced_next_dependent_line.push_back(*iter);
							++iter;
							continue;
						}
						if (iter->first > jter->first) {
							// only "stack_line" has an entry for this variable
							reduced_next_dependent_line.push_back(*jter);
							reduced_next_dependent_line.back().second *= rational_type(-1) * equation_multiply_factor;
							++jter;
							continue;
						}
						if (iter->first == jter->first) {
							reduced_next_dependent_line.push_back(*iter);
							reduced_next_dependent_line.back().second -= equation_multiply_factor * jter->second;
							++iter;
							++jter;
							continue;
						}
					}
				while_end:
					(void)iter;
#endif
				}
			} // for (const auto& stack_line : resolve_stack)
			// check if select_next_dependent_line got resolved -> rerun #############################
			if (P_table[select_next_dependent_line].size() < 2) {
				inline_normalize_resolved_line(P_table, rew_vector, select_next_dependent_line);
			}

			// resolve high_prio_select using select_next_dependent_line

			resolve_stack.push_back(select_next_dependent_line);
		}

		// look for dependet line -> a---------------
		// apply all lines in resolve_stack front to back into a
		// check if a is resolved -> then break and goto rerun!
		// 
		// resolve_stack.push_back(a);
		// resolve a in high_prio_select
		// check high_prio_select resolved? -> goto rerun! ------> comes at begin of new loop run...
		// goto look for next dependet

	}


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


}
