#include "custom_types.h"

rational_type string_to_rational_type(const std::string& s) { // do-check!
	if (s.empty())
		throw rational_parse_error("empty");
	if (s.front() == ' ')
		return string_to_rational_type(s.substr(1));
	if (s.back() == ' ')
		return string_to_rational_type(s.substr(0, s.size() - 1));
	if (s.front() == '(' && s.back() == ')')
		return string_to_rational_type(s.substr(1, s.size() - 2));
	std::string copy{ s };
	int64_t level{ 0 };
	for (auto iter = copy.begin(); iter != copy.end(); ++iter) {

		if (*iter == ')')
			--level;
		if (level < 0)
			throw rational_parse_error("too many ')'");
		if (level > 0)
			*iter = 'x';
		if (*iter == '(')
			++level;
	}
	for (auto iter = copy.rbegin(); iter != copy.rend(); ++iter) {
		if (*iter == '+') {
			return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) + string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
		}
		if (*iter == '-') {
			if (s.size() - 1 - (iter - copy.rbegin()) == 0) { // leading "-"
				return rational_type::int_type(0) - string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
			}
			else {
				return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) - string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
			}
		}
	}
	for (auto iter = copy.rbegin(); iter != copy.rend(); ++iter) {
		if (*iter == '*') {
			return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) * string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
		}
		if (*iter == '/') {
			return string_to_rational_type(s.substr(0, s.size() - 1 - (iter - copy.rbegin()))) / string_to_rational_type(s.substr(s.size() - (iter - copy.rbegin())));
		}
	}
	rational_type::int_type value;
	try {
		value = rational_type::int_type(s);
	}
	catch (...) {
		throw rational_parse_error(std::string("Cannot convert \"") + s + "\" into integer");
	}
	return rational_type(value);
}
