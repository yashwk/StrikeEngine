#include "VulkanBackend.hpp"
#include "../BackendFactory.hpp"
#include "../../SimulationKernel.hpp"
#include "../../data/ControlBlock.hpp"
#include "../../data/PhysicsBlock.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstring>

namespace StrikeEngine::Kernel {

    const size_t MAX_ENTITIES = 1000000;
    const size_t PHYSICS_BUFFER_SIZE = 23 * MAX_ENTITIES * sizeof(double) + MAX_ENTITIES * sizeof(uint32_t);
    const size_t CONTROL_BUFFER_SIZE = 4 * MAX_ENTITIES * sizeof(double);

    VulkanBackend::VulkanBackend() {
        std::cout << "Initializing Vulkan Backend...\n";
        createCommandPool();
    }

    VulkanBackend::~VulkanBackend() {
        shutdown();
    }

    void VulkanBackend::initialize(PhysicsBlock& physics, ControlBlock& control) {
        createBuffers(MAX_ENTITIES);
        createDescriptorSets();
        createPipeline();
    }

    void VulkanBackend::shutdown() {
        auto device = context.getDevice();
        if (device) {
            if (physicsBuffer) device.destroyBuffer(physicsBuffer);
            if (physicsMemory) device.freeMemory(physicsMemory);
            
            if (controlBuffer) device.destroyBuffer(controlBuffer);
            if (controlMemory) device.freeMemory(controlMemory);

            if (descriptorPool) device.destroyDescriptorPool(descriptorPool);
            if (descriptorSetLayout) device.destroyDescriptorSetLayout(descriptorSetLayout);
            if (pipelineLayout) device.destroyPipelineLayout(pipelineLayout);
            if (computePipeline) device.destroyPipeline(computePipeline);
            
            if (commandPool) device.destroyCommandPool(commandPool);
        }
    }

    void VulkanBackend::createCommandPool() {
        vk::CommandPoolCreateInfo poolInfo(
            vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
            context.getComputeQueueFamilyIndex()
        );
        commandPool = context.getDevice().createCommandPool(poolInfo);
        
        vk::CommandBufferAllocateInfo allocInfo(
            commandPool,
            vk::CommandBufferLevel::ePrimary,
            1
        );
        commandBuffer = context.getDevice().allocateCommandBuffers(allocInfo)[0];
    }

    uint32_t VulkanBackend::findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties) {
        vk::PhysicalDeviceMemoryProperties memProperties = context.getPhysicalDevice().getMemoryProperties();
        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }
        throw std::runtime_error("Failed to find suitable memory type!");
    }

    void VulkanBackend::createBuffers(size_t maxSize) {
        auto device = context.getDevice();

        // Physics Buffer (Host Visible + Coherent for easy mapping)
        vk::BufferCreateInfo physicsInfo({}, PHYSICS_BUFFER_SIZE, vk::BufferUsageFlagBits::eStorageBuffer);
        physicsBuffer = device.createBuffer(physicsInfo);
        vk::MemoryRequirements memReq = device.getBufferMemoryRequirements(physicsBuffer);
        vk::MemoryAllocateInfo allocInfo(memReq.size, findMemoryType(memReq.memoryTypeBits, vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent));
        physicsMemory = device.allocateMemory(allocInfo);
        device.bindBufferMemory(physicsBuffer, physicsMemory, 0);

        // Control Buffer
        vk::BufferCreateInfo controlInfo({}, CONTROL_BUFFER_SIZE, vk::BufferUsageFlagBits::eStorageBuffer);
        controlBuffer = device.createBuffer(controlInfo);
        memReq = device.getBufferMemoryRequirements(controlBuffer);
        allocInfo = vk::MemoryAllocateInfo(memReq.size, findMemoryType(memReq.memoryTypeBits, vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent));
        controlMemory = device.allocateMemory(allocInfo);
        device.bindBufferMemory(controlBuffer, controlMemory, 0);
    }

    void VulkanBackend::createDescriptorSets() {
        auto device = context.getDevice();

        // Descriptor Set Layout
        std::vector<vk::DescriptorSetLayoutBinding> bindings = {
            {0, vk::DescriptorType::eStorageBuffer, 1, vk::ShaderStageFlagBits::eCompute},
            {1, vk::DescriptorType::eStorageBuffer, 1, vk::ShaderStageFlagBits::eCompute}
        };
        vk::DescriptorSetLayoutCreateInfo layoutInfo({}, static_cast<uint32_t>(bindings.size()), bindings.data());
        descriptorSetLayout = device.createDescriptorSetLayout(layoutInfo);

        // Pool
        std::vector<vk::DescriptorPoolSize> poolSizes = {
            {vk::DescriptorType::eStorageBuffer, 2}
        };
        vk::DescriptorPoolCreateInfo poolInfo({}, 1, static_cast<uint32_t>(poolSizes.size()), poolSizes.data());
        descriptorPool = device.createDescriptorPool(poolInfo);

        // Set
        vk::DescriptorSetAllocateInfo allocInfo(descriptorPool, 1, &descriptorSetLayout);
        descriptorSet = device.allocateDescriptorSets(allocInfo)[0];

        // Update
        vk::DescriptorBufferInfo physicsBufInfo(physicsBuffer, 0, VK_WHOLE_SIZE);
        vk::DescriptorBufferInfo controlBufInfo(controlBuffer, 0, VK_WHOLE_SIZE);

        std::vector<vk::WriteDescriptorSet> writes = {
            {descriptorSet, 0, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr, &physicsBufInfo, nullptr},
            {descriptorSet, 1, 0, 1, vk::DescriptorType::eStorageBuffer, nullptr, &controlBufInfo, nullptr}
        };
        device.updateDescriptorSets(static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }

    void VulkanBackend::createPipeline() {
        auto device = context.getDevice();

        // Read Shader
        std::ifstream file("physics_step.spv", std::ios::ate | std::ios::binary);
        if (!file.is_open()) throw std::runtime_error("Failed to open physics_step.spv");
        size_t fileSize = (size_t)file.tellg();
        std::vector<char> buffer(fileSize);
        file.seekg(0);
        file.read(buffer.data(), fileSize);
        file.close();

        vk::ShaderModuleCreateInfo createInfo({}, buffer.size(), reinterpret_cast<const uint32_t*>(buffer.data()));
        vk::ShaderModule shaderModule = device.createShaderModule(createInfo);

        vk::PushConstantRange pcRange(vk::ShaderStageFlagBits::eCompute, 0, sizeof(double) * 2 + sizeof(uint32_t));

        vk::PipelineLayoutCreateInfo pipelineLayoutInfo({}, 1, &descriptorSetLayout, 1, &pcRange);
        pipelineLayout = device.createPipelineLayout(pipelineLayoutInfo);

        vk::PipelineShaderStageCreateInfo shaderStageInfo({}, vk::ShaderStageFlagBits::eCompute, shaderModule, "main");
        vk::ComputePipelineCreateInfo computeInfo({}, shaderStageInfo, pipelineLayout);
        
        auto result = device.createComputePipeline(nullptr, computeInfo);
        if (result.result != vk::Result::eSuccess) throw std::runtime_error("Failed to create compute pipeline");
        computePipeline = result.value;

        device.destroyShaderModule(shaderModule);
    }

    // Helper macro to copy SoA vectors
    #define COPY_TO_GPU(buffer, vec, index) \
        std::memcpy((uint8_t*)buffer + (index) * MAX_ENTITIES * sizeof(double), vec.data(), count * sizeof(double))
    
    #define COPY_FROM_GPU(buffer, vec, index) \
        std::memcpy(vec.data(), (uint8_t*)buffer + (index) * MAX_ENTITIES * sizeof(double), count * sizeof(double))

    void VulkanBackend::step(
        PhysicsBlock& physics,
        ControlBlock& control,
        double currentTime,
        double dt) 
    {
        auto device = context.getDevice();
        size_t count = physics.size;
        if (count == 0) return;

        // 1. Map and copy CPU SoA to GPU SSBO
        void* mappedPhysics = device.mapMemory(physicsMemory, 0, VK_WHOLE_SIZE);
        
        COPY_TO_GPU(mappedPhysics, physics.px, 0);
        COPY_TO_GPU(mappedPhysics, physics.py, 1);
        COPY_TO_GPU(mappedPhysics, physics.pz, 2);
        
        COPY_TO_GPU(mappedPhysics, physics.vx, 3);
        COPY_TO_GPU(mappedPhysics, physics.vy, 4);
        COPY_TO_GPU(mappedPhysics, physics.vz, 5);

        COPY_TO_GPU(mappedPhysics, physics.qw, 6);
        COPY_TO_GPU(mappedPhysics, physics.qx, 7);
        COPY_TO_GPU(mappedPhysics, physics.qy, 8);
        COPY_TO_GPU(mappedPhysics, physics.qz, 9);

        COPY_TO_GPU(mappedPhysics, physics.wx, 10);
        COPY_TO_GPU(mappedPhysics, physics.wy, 11);
        COPY_TO_GPU(mappedPhysics, physics.wz, 12);

        COPY_TO_GPU(mappedPhysics, physics.ax, 13);
        COPY_TO_GPU(mappedPhysics, physics.ay, 14);
        COPY_TO_GPU(mappedPhysics, physics.az, 15);

        COPY_TO_GPU(mappedPhysics, physics.alphax, 16);
        COPY_TO_GPU(mappedPhysics, physics.alphay, 17);
        COPY_TO_GPU(mappedPhysics, physics.alphaz, 18);

        COPY_TO_GPU(mappedPhysics, physics.Ixx, 19);
        COPY_TO_GPU(mappedPhysics, physics.Iyy, 20);
        COPY_TO_GPU(mappedPhysics, physics.Izz, 21);
        
        COPY_TO_GPU(mappedPhysics, physics.mass, 22);

        // active array requires special care (vector<bool> to uint32_t array)
        uint32_t* activePtr = (uint32_t*)((uint8_t*)mappedPhysics + 23 * MAX_ENTITIES * sizeof(double));
        for (size_t i = 0; i < count; ++i) {
            activePtr[i] = physics.active[i] ? 1 : 0;
        }

        device.unmapMemory(physicsMemory);

        // Map Control Data
        void* mappedControl = device.mapMemory(controlMemory, 0, VK_WHOLE_SIZE);
        COPY_TO_GPU(mappedControl, control.thrustCommand, 0);
        COPY_TO_GPU(mappedControl, control.pitchCommand, 1);
        COPY_TO_GPU(mappedControl, control.yawCommand, 2);
        COPY_TO_GPU(mappedControl, control.rollCommand, 3);
        device.unmapMemory(controlMemory);

        // 2. Dispatch Compute
        commandBuffer.reset();
        commandBuffer.begin(vk::CommandBufferBeginInfo());
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, computePipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
        
        // Push Constants struct
        struct {
            double dt;
            double currentTime;
            uint32_t entityCount;
        } pushData = { dt, currentTime, (uint32_t)count };

        commandBuffer.pushConstants(pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pushData), &pushData);
        
        uint32_t groupCount = (uint32_t)((count + 255) / 256);
        commandBuffer.dispatch(groupCount, 1, 1);
        commandBuffer.end();

        vk::SubmitInfo submitInfo(0, nullptr, nullptr, 1, &commandBuffer, 0, nullptr);
        auto queue = context.getComputeQueue();
        (void)queue.submit(1, &submitInfo, nullptr);
        queue.waitIdle();

        // 3. Read back integrated positions/velocities/quaternions/forces
        mappedPhysics = device.mapMemory(physicsMemory, 0, VK_WHOLE_SIZE);
        
        COPY_FROM_GPU(mappedPhysics, physics.px, 0);
        COPY_FROM_GPU(mappedPhysics, physics.py, 1);
        COPY_FROM_GPU(mappedPhysics, physics.pz, 2);

        COPY_FROM_GPU(mappedPhysics, physics.vx, 3);
        COPY_FROM_GPU(mappedPhysics, physics.vy, 4);
        COPY_FROM_GPU(mappedPhysics, physics.vz, 5);

        COPY_FROM_GPU(mappedPhysics, physics.qw, 6);
        COPY_FROM_GPU(mappedPhysics, physics.qx, 7);
        COPY_FROM_GPU(mappedPhysics, physics.qy, 8);
        COPY_FROM_GPU(mappedPhysics, physics.qz, 9);

        COPY_FROM_GPU(mappedPhysics, physics.wx, 10);
        COPY_FROM_GPU(mappedPhysics, physics.wy, 11);
        COPY_FROM_GPU(mappedPhysics, physics.wz, 12);

        COPY_FROM_GPU(mappedPhysics, physics.ax, 13);
        COPY_FROM_GPU(mappedPhysics, physics.ay, 14);
        COPY_FROM_GPU(mappedPhysics, physics.az, 15);

        device.unmapMemory(physicsMemory);
    }

    namespace {

        // Self-registration: linking strikeengine_vulkan into a program makes
        // BackendType::Vulkan available to SimulationKernel via the registry.
        struct VulkanBackendRegistrar {
            VulkanBackendRegistrar() {
                registerPhysicsBackendFactory(
                    static_cast<int>(BackendType::Vulkan),
                    []() { return std::make_unique<VulkanBackend>(); });
            }
        };
        VulkanBackendRegistrar vulkanBackendRegistrar;

    } // namespace

} // namespace StrikeEngine::Kernel
