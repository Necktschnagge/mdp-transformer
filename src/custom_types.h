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

rational_type string_to_rational_type(const std::string& s);

