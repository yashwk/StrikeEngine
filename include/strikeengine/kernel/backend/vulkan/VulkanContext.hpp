#pragma once

#include <vulkan/vulkan.hpp>
#include <memory>
#include <vector>

namespace StrikeEngine::Kernel {

    class VulkanContext {
    public:
        VulkanContext();
        ~VulkanContext();

        // Prevent copying
        VulkanContext(const VulkanContext&) = delete;
        VulkanContext& operator=(const VulkanContext&) = delete;

        vk::Instance getInstance() const { return instance; }
        vk::PhysicalDevice getPhysicalDevice() const { return physicalDevice; }
        vk::Device getDevice() const { return device; }
        vk::Queue getComputeQueue() const { return computeQueue; }
        uint32_t getComputeQueueFamilyIndex() const { return computeQueueFamilyIndex; }

    private:
        void createInstance();
        void pickPhysicalDevice();
        void createLogicalDevice();

        vk::Instance instance;
        vk::PhysicalDevice physicalDevice;
        vk::Device device;
        vk::Queue computeQueue;
        uint32_t computeQueueFamilyIndex = -1;

#ifndef NDEBUG
        bool enableValidationLayers = true;
        vk::DebugUtilsMessengerEXT debugMessenger;
        void setupDebugMessenger();
#else
        bool enableValidationLayers = false;
#endif
    };

} // namespace StrikeEngine::Kernel
