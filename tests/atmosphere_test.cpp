#include "strikeengine/atmosphere/AtmosphereManager.hpp"
#include "strikeengine/atmosphere/AtmosphereModel.hpp"
#include <iostream>
#include <chrono>
#include <cassert>
#include <vector>

// Benchmarks the performance of the table lookup method using the AtmosphereManager.
void benchmark_lookup(const StrikeEngine::AtmosphereManager& manager) {
    std::cout << "--- Running Lookup Benchmark ---" << std::endl;
    constexpr int steps = 86000;
    const auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i <= steps; ++i) {
        // Volatile to prevent the compiler from optimizing the loop away.
        volatile StrikeEngine::AtmosphereProperties props = manager.getProperties(static_cast<double>(i));
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration<double>(end - start).count();
    std::cout << "Lookup Time: " << duration << "s for " << steps + 1 << " steps" << std::endl;
}

// Benchmarks the performance of the direct calculation method.
void benchmark_calculation() {
    std::cout << "--- Running Calculation Benchmark ---" << std::endl;
    constexpr int steps = 86000;

    // The calculation now depends on the layer data, which we must load first.
    const auto layers = StrikeEngine::loadAtmosphereLayers("data/config/atmosphere_layers.json");

    const auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i <= steps; ++i) {
        volatile StrikeEngine::AtmosphereProperties props = StrikeEngine::calculateAtmosphere(static_cast<double>(i), layers);
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration<double>(end - start).count();
    std::cout << "Calculation Time: " << duration << "s for " << steps + 1 << " steps" << std::endl;
}

int main() {
    StrikeEngine::AtmosphereManager atmosphereManager;
    const std::string tablePath = "data/atmosphere_table.bin";

    std::cout << "Loading atmosphere table from: " << tablePath << std::endl;
    if (!atmosphereManager.loadTable(tablePath)) {
        std::cerr << "TEST FAILED: Could not load atmosphere table." << std::endl;
        std::cerr << "Please run the GenerateAtmosphereTable tool first." << std::endl;
        return 1;
    }
    assert(atmosphereManager.isLoaded());
    std::cout << "Table loaded successfully." << std::endl;

    benchmark_lookup(atmosphereManager);
    benchmark_calculation();

    std::cout << "\nAtmosphere benchmarks completed successfully." << std::endl;
    return 0;
}
