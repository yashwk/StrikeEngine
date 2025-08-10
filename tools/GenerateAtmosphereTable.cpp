#include "strikeengine/atmosphere/AtmosphereModel.hpp"
#include <fstream>
#include <iostream>
#include <vector>

int main () {
	using namespace StrikeEngine;

	const std::string layersFilepath = "data/config/atmosphere_layers.json";
	const std::string tableFilepath  = "data/atmosphere_table.bin";

	try {
		std::cout << "Loading atmosphere layer definitions from: " << layersFilepath << std::endl;
		const auto layers = loadAtmosphereLayers( layersFilepath );
		std::cout << "Layer definitions loaded successfully." << std::endl;

		std::ofstream out( tableFilepath , std::ios::binary );
		if (!out) {
			std::cerr << "Error: Failed to open file for writing: " << tableFilepath << std::endl;
			return 1;
		}

		std::cout << "Generating atmosphere lookup table..." << std::endl;
		for (int i = 0 ; i <= 86000 ; ++i) {
			AtmosphereProperties props = calculateAtmosphere( i , layers );
			out.write( reinterpret_cast < const char * >(&props) , sizeof(props) );
		}

		std::cout << "Binary table generated successfully at: " << tableFilepath << std::endl;
	}
	catch (const std::exception &e) {
		std::cerr << "An error occurred: " << e.what() << std::endl;
		return 1;
	}

	return 0;
}
