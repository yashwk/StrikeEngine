#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include <fstream>
#include <stdexcept>

namespace StrikeEngine {
	bool AtmosphereManager::loadTable ( const std::string &filepath ) {
		std::ifstream file( filepath , std::ios::binary );
		if (!file) { return false; }

		_table.clear();
		AtmosphereProperties entry{};
		while (file.read( reinterpret_cast < char * >(&entry) , sizeof(entry) )) { _table.push_back( entry ); }

		return !_table.empty();
	}

	bool AtmosphereManager::isLoaded () const { return !_table.empty(); }

	AtmosphereProperties AtmosphereManager::getProperties ( double altitude ) const {
		if (!isLoaded()) { throw std::runtime_error( "AtmosphereManager error: Table not loaded." ); }

		if (altitude <= _table.front().altitude) { return _table.front(); }
		if (altitude >= _table.back().altitude) { return _table.back(); }

		const auto lowerIndex   = static_cast < size_t >(altitude);
		const size_t upperIndex = lowerIndex + 1;

		if (upperIndex >= _table.size()) { return _table.back(); }

		const double fraction = altitude - static_cast < double >(lowerIndex);

		const auto &low  = _table[lowerIndex];
		const auto &high = _table[upperIndex];

		AtmosphereProperties interpolated{};
		interpolated.altitude     = altitude;
		interpolated.temperature  = low.temperature + fraction * (high.temperature - low.temperature);
		interpolated.pressure     = low.pressure + fraction * (high.pressure - low.pressure);
		interpolated.density      = low.density + fraction * (high.density - low.density);
		interpolated.speedOfSound = low.speedOfSound + fraction * (high.speedOfSound - low.speedOfSound);

		return interpolated;
	}
} // namespace StrikeEngine
