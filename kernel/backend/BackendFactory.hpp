#pragma once

#include <functional>
#include <memory>

namespace StrikeEngine::Kernel {

    class PhysicsBackend;

    using PhysicsBackendFactory = std::function<std::unique_ptr<PhysicsBackend>()>;

    /**
     * @brief Registry of physics-backend factories.
     *
     * The core library ships `BackendType::CPU`. Optional backends (e.g. the
     * Vulkan compute backend in the companion `strikeengine_vulkan` library)
     * self-register through static initialization when linked in. Requesting a
     * backend that is not linked produces a clear runtime error instead of a
     * link failure.
     *
     * Keys use `static_cast<int>(BackendType)` so the registry stays
     * independent of the SimulationKernel header.
     */
    void registerPhysicsBackendFactory(int backendType, PhysicsBackendFactory factory);

    /**
     * @brief Returns the registered factory for a backend, or nullptr.
     */
    PhysicsBackendFactory getPhysicsBackendFactory(int backendType);

} // namespace StrikeEngine::Kernel
