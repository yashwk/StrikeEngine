#include <strikeengine/kernel/backend/vulkan/VulkanContext.hpp>
#include <iostream>
#include <stdexcept>

namespace StrikeEngine::Kernel {

#ifndef NDEBUG
    VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageType,
        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
        void* pUserData) {
        
        if (messageSeverity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
            std::cerr << "Vulkan Validation Layer: " << pCallbackData->pMessage << std::endl;
        }
        return VK_FALSE;
    }
#endif

    VulkanContext::VulkanContext() {
        createInstance();
#ifndef NDEBUG
        setupDebugMessenger();
#endif
        pickPhysicalDevice();
        createLogicalDevice();
    }

    VulkanContext::~VulkanContext() {
        if (device) {
            device.destroy();
        }
#ifndef NDEBUG
        if (enableValidationLayers && instance) {
            // EXT debug-utils entry points are not loaded by default;
            // resolve via vkGetInstanceProcAddr and call the C API directly.
            auto destroyFn = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
                instance.getProcAddr("vkDestroyDebugUtilsMessengerEXT"));
            if (destroyFn) {
                destroyFn(instance, static_cast<VkDebugUtilsMessengerEXT>(debugMessenger), nullptr);
            }
        }
#endif
        if (instance) {
            instance.destroy();
        }
    }

    void VulkanContext::createInstance() {
        vk::ApplicationInfo appInfo(
            "StrikeEngine", VK_MAKE_VERSION(1, 0, 0),
            "No Engine", VK_MAKE_VERSION(1, 0, 0),
            VK_API_VERSION_1_3
        );

        std::vector<const char*> extensions;
#ifndef NDEBUG
        if (enableValidationLayers) {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }
#endif

        std::vector<const char*> validationLayers;
#ifndef NDEBUG
        if (enableValidationLayers) {
            validationLayers.push_back("VK_LAYER_KHRONOS_validation");
        }
#endif

        vk::InstanceCreateInfo createInfo(
            {}, &appInfo,
            static_cast<uint32_t>(validationLayers.size()), validationLayers.data(),
            static_cast<uint32_t>(extensions.size()), extensions.data()
        );

        instance = vk::createInstance(createInfo);
    }

#ifndef NDEBUG
    void VulkanContext::setupDebugMessenger() {
        if (!enableValidationLayers) return;

        vk::DebugUtilsMessengerCreateInfoEXT createInfo(
            {},
            vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose | vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning | vk::DebugUtilsMessageSeverityFlagBitsEXT::eError,
            vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral | vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation | vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance,
            debugCallback
        );

        auto createFn = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            instance.getProcAddr("vkCreateDebugUtilsMessengerEXT"));
        if (!createFn) {
            throw std::runtime_error("Vulkan: vkCreateDebugUtilsMessengerEXT not available");
        }
        VkDebugUtilsMessengerEXT messenger = VK_NULL_HANDLE;
        if (createFn(instance, &static_cast<const VkDebugUtilsMessengerCreateInfoEXT&>(createInfo),
                     nullptr, &messenger) != VK_SUCCESS) {
            throw std::runtime_error("Vulkan: failed to create debug messenger");
        }
        debugMessenger = vk::DebugUtilsMessengerEXT(messenger);
    }
#endif

    void VulkanContext::pickPhysicalDevice() {
        std::vector<vk::PhysicalDevice> devices = instance.enumeratePhysicalDevices();
        if (devices.empty()) {
            throw std::runtime_error("Failed to find GPUs with Vulkan support!");
        }

        // Simply pick the first one that supports compute
        for (const auto& dev : devices) {
            std::vector<vk::QueueFamilyProperties> queueFamilies = dev.getQueueFamilyProperties();
            
            uint32_t i = 0;
            for (const auto& queueFamily : queueFamilies) {
                if (queueFamily.queueFlags & vk::QueueFlagBits::eCompute) {
                    physicalDevice = dev;
                    computeQueueFamilyIndex = i;
                    return; // Picked successfully
                }
                i++;
            }
        }

        throw std::runtime_error("Failed to find a suitable GPU with a Compute Queue!");
    }

    void VulkanContext::createLogicalDevice() {
        float queuePriority = 1.0f;
        vk::DeviceQueueCreateInfo queueCreateInfo(
            {}, computeQueueFamilyIndex, 1, &queuePriority
        );

        vk::PhysicalDeviceFeatures deviceFeatures{};

        vk::DeviceCreateInfo createInfo(
            {}, 1, &queueCreateInfo,
            0, nullptr,
            0, nullptr,
            &deviceFeatures
        );

        device = physicalDevice.createDevice(createInfo);

        computeQueue = device.getQueue(computeQueueFamilyIndex, 0);
    }

} // namespace StrikeEngine::Kernel
