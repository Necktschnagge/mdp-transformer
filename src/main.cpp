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
