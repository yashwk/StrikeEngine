#pragma once

#include <string>
#include <vector>
#include <memory>
#include <glm/vec3.hpp> // Assuming GLM is available, or use struct
#include <imgui.h>
#include <nlohmann/json.hpp>

// Simple Transform Struct
struct Transform {
	float position[3] = {0.0f, 0.0f, 0.0f};
	float rotation[3] = {0.0f, 0.0f, 0.0f};
	float scale[3]    = {1.0f, 1.0f, 1.0f};
};

enum class PartType {
	Root,
	NoseCone,
	BodyTube,
	FinSet,
	Motor
};

class RocketPart {
public:
	RocketPart(const std::string& name, PartType type);
	virtual ~RocketPart() = default;

	// The Hierarchy
	std::string name;
	PartType type;
	Transform transform;

	RocketPart* parent = nullptr;
	std::vector<std::shared_ptr<RocketPart>> children;

	// Core Logic
	void addChild(std::shared_ptr<RocketPart> child);
	void removeChild(RocketPart* child);

	// Virtual Inspect: Each part knows how to draw its own Inspector UI
	virtual void onInspect();

	// Serialization
	virtual nlohmann::json serialize() const;
	virtual void deserialize(const nlohmann::json& j);
	
	// Factory
	static std::shared_ptr<RocketPart> createFromType(PartType type);
};

// --- Derived Parts ---

class NoseCone : public RocketPart {
public:
	NoseCone() : RocketPart("Nose Cone", PartType::NoseCone) {}

	float length = 0.5f;
	float baseDiameter = 0.15f;
	int shapeType = 0; // 0=Ogive, 1=Conical

	void onInspect() override;
	nlohmann::json serialize() const override;
	void deserialize(const nlohmann::json& j) override;
};

class BodyTube : public RocketPart {
public:
	BodyTube() : RocketPart("Body Tube", PartType::BodyTube) {}

	float length = 1.0f;
	float diameter = 0.15f;
	float wallThickness = 0.002f;
	bool motorMount = false;

	void onInspect() override;
	nlohmann::json serialize() const override;
	void deserialize(const nlohmann::json& j) override;
};

class FinSet : public RocketPart {
public:
	FinSet() : RocketPart("Fin Set", PartType::FinSet) {}

	int finCount = 3;
	float rootChord = 0.2f;
	float tipChord = 0.1f;
	float span = 0.15f;
	float sweepAngle = 20.0f;

	void onInspect() override;
	nlohmann::json serialize() const override;
	void deserialize(const nlohmann::json& j) override;
};

class Motor : public RocketPart {
public:
	Motor() : RocketPart("Motor", PartType::Motor) {}

	float propellantMass = 1.0f;
	float dryMass = 0.5f;
	float burnTime = 5.0f;
	float averageThrust = 1000.0f;
	float specificImpulse = 250.0f;

	void onInspect() override;
	nlohmann::json serialize() const override;
	void deserialize(const nlohmann::json& j) override;
};

// --- The Project Manager ---
class Project {
public:
	Project();

	std::shared_ptr<RocketPart> root;
	RocketPart* selectedPart = nullptr;

	void select(RocketPart* part);
	
	bool saveToFile(const std::string& filepath);
	bool loadFromFile(const std::string& filepath);
};