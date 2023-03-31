#pragma once
#include "logger.h"

#include <nlohmann/json.hpp>

#include <string>
#include <set>
#include <stdexcept>
#include <fstream>

namespace {
	auto& utility_logger = standard_logger;
}

inline bool set_contains(const std::set<std::string>& set, const std::string& s) {
	return set.find(s) != set.cend();
}

template<bool _THROW_EXCEPTION = false>
inline nlohmann::json load_json(const std::string& file_name, const nlohmann::json& default_json = nlohmann::json::object()) {
	std::ifstream ofile;
	utility_logger()->debug("Try to open a file...");
	ofile.open(file_name);
	if (ofile.good()) { // check if opened a file
		ofile >> std::noskipws;
		try {
			return nlohmann::json::parse(std::istream_iterator<std::ifstream::char_type>(ofile), std::istream_iterator<std::ifstream::char_type>());
		}
		catch (...) {
			utility_logger()->warn(std::string("Could not parse opened file named:   ").append(file_name));
		}
		ofile.close();
	}
	else {
		utility_logger()->warn(std::string("Could not open a file named:   ").append(file_name));
	}
	utility_logger()->info("Will return a default_json as specified.");
	if constexpr (_THROW_EXCEPTION) throw default_json;
	return default_json;
}

inline void save_json(const std::string& file_name, const nlohmann::json& json, int indent = 1) {
	std::ofstream file;
	file.open(file_name);
	file << std::setw(indent) << json << std::endl;
}

inline nlohmann::json fold_json_object_array_into_value_set(const nlohmann::json& array_of_objects, const std::string& property_of_interest, const std::string& ERROR_VALUE = "ERROR") {
	std::set<std::string> accumulated;
	if (!array_of_objects.is_array()) {
		standard_logger()->error("Cannot fold given json. It is not an array!");
	}
	else {
		std::transform(
			array_of_objects.cbegin(),
			array_of_objects.cend(),
			std::inserter(accumulated, accumulated.cbegin()),
			[&property_of_interest, &ERROR_VALUE](const nlohmann::json& obj) {
				try {
					return obj.at(property_of_interest).get<std::string>();
				}
				catch (...) {
					return ERROR_VALUE;
				}
			}
		);
	}
	nlohmann::json result = nlohmann::json::array();
	std::copy(accumulated.cbegin(), accumulated.cend(), std::back_inserter(result));
	return result;
}

class json_logic_error : public std::logic_error {

public:

	template <class T>
	json_logic_error(const T& arg) : std::logic_error(arg) {}

	template <class T>
	static void check(const T& message, bool check_result) {
		if (!check_result) throw json_logic_error(message);
	}
};

inline nlohmann::json merge_json_objects(const std::vector<nlohmann::json>& jsons) { // ready
	nlohmann::json merged = nlohmann::json::object();

	for (const auto& j : jsons) {
		json_logic_error::check("Found a json file which is not a json object", j.is_object());
		for (auto iter = j.cbegin(); iter != j.cend(); ++iter) {
			json_logic_error::check(std::string("Found a json key twice:   ") + iter.key(), !merged.contains(iter.key()));
			merged[iter.key()] = iter.value();
		}
	}

	return merged;
}
