#include <strikeengine/kernel/backend/BackendFactory.hpp>

#include <map>

namespace StrikeEngine::Kernel {

    namespace {

        std::map<int, PhysicsBackendFactory>& registry() {
            static std::map<int, PhysicsBackendFactory> factories;
            return factories;
        }

    } // namespace

    void registerPhysicsBackendFactory(int backendType, PhysicsBackendFactory factory) {
        registry()[backendType] = std::move(factory);
    }

    PhysicsBackendFactory getPhysicsBackendFactory(int backendType) {
        const auto& factories = registry();
        const auto it = factories.find(backendType);
        return it == factories.end() ? PhysicsBackendFactory{} : it->second;
    }

} // namespace StrikeEngine::Kernel
