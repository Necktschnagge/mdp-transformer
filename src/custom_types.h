#pragma once


#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

#include <set>
#include <map>

using rational_type = boost::rational<boost::multiprecision::cpp_int>;


class rational_parse_error : public std::logic_error {

public:

	template <class T>
	rational_parse_error(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw mdp_sanity(message);
	}
};

class calc_delta_max_error : public std::runtime_error {

public:

	template <class T>
	calc_delta_max_error(const T& arg) : std::runtime_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw calc_delta_max_error(message);
	}
};

class found_negative_loop : public std::logic_error {

public:

	template <class T>
	found_negative_loop(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw found_negative_loop(message);
	}
};

class found_unreachable_state : public std::logic_error {
public:
	template <class T>
	found_unreachable_state(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw found_unreachable_state(message);
	}
};

class mdp {
public:
	std::set<std::string> states;
	std::set<std::string> actions;
	std::map<std::string, std::map<std::string, std::map<std::string, rational_type>>> probabilities;
	std::string initial;
	std::map<std::string, std::map<std::string, rational_type>> rewards;
	std::set<std::string> targets;


	rational_type min_reward() const {
		std::vector<rational_type> values;
		for (const auto& state_paired_rewards : rewards) {
			for (const auto& action_paired_reward : state_paired_rewards.second) {
				values.push_back(action_paired_reward.second);
			}
		}
		if (values.empty())
			return rational_type(0);
		rational_type min{ values.front() };
		for (const auto& v : values) {
			if (v < min)
				min = v;
		}
		return min;
	}

	rational_type negative_loop_delta_threshold() const {
		return rational_type(states.size()) * min_reward();
	}

};

class further_expand_record {
public:
	std::string augmented_state_name;
	std::string original_state_name;
	rational_type accumulated_reward;

	further_expand_record(const std::string& original_state_name, rational_type accumulated_reward, const std::string& augmented_state_name) :
		augmented_state_name(augmented_state_name),
		original_state_name(original_state_name),
		accumulated_reward(accumulated_reward)
	{}

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

class json_task_error : public std::logic_error {

public:

	template <class T>
	json_task_error(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw json_task_error(message);
	}
};


rational_type string_to_rational_type(const std::string& s);

