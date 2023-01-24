#include "logger.h"

#include "utility.h"

#include <nlohmann/json.hpp>


namespace feature_toogle {

	
}



int main(int argc, char* argv[])
{
	init_logger();

	if (argc > 1) {
		//on server
		(void)argv;
	}


	// json...
	nlohmann::json summary = load_json("../../src/example-mdp.json");

	standard_logger()->trace(summary.dump(3));

	standard_logger()->info("     DONE     ");
	return 0;
}
