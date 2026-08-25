#pragma once

#include <strikeengine/kernel/backend/PhysicsBackend.hpp>
#include <strikeengine/kernel/backend/vulkan/VulkanContext.hpp>
#include <memory>
#include <vector>

namespace StrikeEngine::Kernel {

    class VulkanBackend : public PhysicsBackend {
    public:
        VulkanContext context;

        VulkanBackend();
        ~VulkanBackend() override;

        void initialize(
            PhysicsBlock& physics,
            ControlBlock& control
        ) override;

        void step(
            PhysicsBlock& physics,
            ControlBlock& control,
            double currentTime,
            double dt
        ) override;

        void shutdown() override;

    private:
        // Vulkan Resources
        vk::CommandPool commandPool;
        vk::CommandBuffer commandBuffer;

        // Buffers
        vk::Buffer physicsBuffer;
        vk::DeviceMemory physicsMemory;

        vk::Buffer controlBuffer;
        vk::DeviceMemory controlMemory;

        // Pipeline
        vk::DescriptorSetLayout descriptorSetLayout;
        vk::PipelineLayout pipelineLayout;
        vk::Pipeline computePipeline;
        vk::DescriptorPool descriptorPool;
        vk::DescriptorSet descriptorSet;

        void createBuffers(size_t maxSize);
        void createPipeline();
        void createDescriptorSets();
        void createCommandPool();

        // Memory helpers
        uint32_t findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties);
    };

} // namespace StrikeEngine::Kernel
