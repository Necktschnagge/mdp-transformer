#pragma once

#include "logger.h"
#include "custom_types.h"
#include "const_strings.h"


#include <nlohmann/json.hpp>

#include <set>
#include <map>
#include <string>


template <class IteratorA, class IteratorB>
inline bool sets_disjoint(IteratorA a_begin, IteratorA a_end, IteratorB b_begin, IteratorB b_end) {
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

bool check_valid_mdp(const nlohmann::json& input, mdp& fill_in);

