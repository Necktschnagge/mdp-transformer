#pragma once


#include <boost/multiprecision/cpp_int.hpp>
#include <boost/rational.hpp>

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
	calc_delta_max_error(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw calc_delta_max_error(message);
	}
};

class found_negative_loop : public std::runtime_error {

public:

	template <class T>
	found_negative_loop(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw found_negative_loop(message);
	}
};

rational_type string_to_rational_type(const std::string& s);

