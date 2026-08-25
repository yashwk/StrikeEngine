#include "VehicleModel.hpp"
#include <algorithm>
#include <iostream>
#include <fstream>

using json = nlohmann::json;

RocketPart::RocketPart(const std::string& name, PartType type) : name(name), type(type) {}

void RocketPart::addChild(std::shared_ptr<RocketPart> child) {
    child->parent = this;
    children.push_back(child);
}

void RocketPart::removeChild(RocketPart* child) {
    children.erase(
        std::remove_if(children.begin(), children.end(), 
            [child](const std::shared_ptr<RocketPart>& p) { return p.get() == child; }),
        children.end()
    );
}

void RocketPart::onInspect() {
    ImGui::TextDisabled("Base Properties");
    ImGui::InputText("Name", &name[0], name.capacity() + 1); // Simple string edit
    
    if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragFloat3("Position", transform.position, 0.01f);
        ImGui::DragFloat3("Rotation", transform.rotation, 0.1f);
        ImGui::DragFloat3("Scale", transform.scale, 0.01f);
    }
}

json RocketPart::serialize() const {
    json j;
    j["name"] = name;
    j["type"] = static_cast<int>(type);
    j["transform"] = {
        {"position", {transform.position[0], transform.position[1], transform.position[2]}},
        {"rotation", {transform.rotation[0], transform.rotation[1], transform.rotation[2]}},
        {"scale", {transform.scale[0], transform.scale[1], transform.scale[2]}}
    };
    
    json childrenJson = json::array();
    for (const auto& child : children) {
        childrenJson.push_back(child->serialize());
    }
    j["children"] = childrenJson;
    return j;
}

void RocketPart::deserialize(const json& j) {
    // Name and type should be read before calling this, but we update name here
    if (j.contains("name")) name = j["name"];
    if (j.contains("transform")) {
        auto tr = j["transform"];
        if (tr.contains("position")) {
            transform.position[0] = tr["position"][0];
            transform.position[1] = tr["position"][1];
            transform.position[2] = tr["position"][2];
        }
        if (tr.contains("rotation")) {
            transform.rotation[0] = tr["rotation"][0];
            transform.rotation[1] = tr["rotation"][1];
            transform.rotation[2] = tr["rotation"][2];
        }
        if (tr.contains("scale")) {
            transform.scale[0] = tr["scale"][0];
            transform.scale[1] = tr["scale"][1];
            transform.scale[2] = tr["scale"][2];
        }
    }
}

std::shared_ptr<RocketPart> RocketPart::createFromType(PartType type) {
    switch (type) {
        case PartType::Root: return std::make_shared<RocketPart>("Root", PartType::Root);
        case PartType::NoseCone: return std::make_shared<NoseCone>();
        case PartType::BodyTube: return std::make_shared<BodyTube>();
        case PartType::FinSet: return std::make_shared<FinSet>();
        case PartType::Motor: return std::make_shared<Motor>();
        default: return nullptr;
    }
}

// --- Derived Implementations ---

void NoseCone::onInspect() {
    RocketPart::onInspect(); // Draw base transform
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Nose Cone Geometry");
    
    ImGui::Combo("Shape", &shapeType, "Ogive\0Conical\0Ellipsoid\0");
    ImGui::DragFloat("Length (m)", &length, 0.01f, 0.01f, 5.0f);
    ImGui::DragFloat("Base Dia (m)", &baseDiameter, 0.001f, 0.01f, 2.0f);
}

json NoseCone::serialize() const {
    json j = RocketPart::serialize();
    j["length"] = length;
    j["baseDiameter"] = baseDiameter;
    j["shapeType"] = shapeType;
    return j;
}

void NoseCone::deserialize(const json& j) {
    RocketPart::deserialize(j);
    if (j.contains("length")) length = j["length"];
    if (j.contains("baseDiameter")) baseDiameter = j["baseDiameter"];
    if (j.contains("shapeType")) shapeType = j["shapeType"];
}


void BodyTube::onInspect() {
    RocketPart::onInspect();
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Body Tube Geometry");
    
    ImGui::DragFloat("Length (m)", &length, 0.01f, 0.01f, 10.0f);
    ImGui::DragFloat("Diameter (m)", &diameter, 0.001f, 0.01f, 2.0f);
    ImGui::DragFloat("Thickness (m)", &wallThickness, 0.0001f, 0.001f, 0.1f);
    ImGui::Checkbox("Has Motor Mount", &motorMount);
}

json BodyTube::serialize() const {
    json j = RocketPart::serialize();
    j["length"] = length;
    j["diameter"] = diameter;
    j["wallThickness"] = wallThickness;
    j["motorMount"] = motorMount;
    return j;
}

void BodyTube::deserialize(const json& j) {
    RocketPart::deserialize(j);
    if (j.contains("length")) length = j["length"];
    if (j.contains("diameter")) diameter = j["diameter"];
    if (j.contains("wallThickness")) wallThickness = j["wallThickness"];
    if (j.contains("motorMount")) motorMount = j["motorMount"];
}


void FinSet::onInspect() {
    RocketPart::onInspect();
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Fin Geometry");
    
    ImGui::SliderInt("Fin Count", &finCount, 2, 8);
    ImGui::DragFloat("Root Chord (m)", &rootChord, 0.001f);
    ImGui::DragFloat("Tip Chord (m)", &tipChord, 0.001f);
    ImGui::DragFloat("Span (m)", &span, 0.001f);
    ImGui::DragFloat("Sweep (deg)", &sweepAngle, 0.1f);
}

json FinSet::serialize() const {
    json j = RocketPart::serialize();
    j["finCount"] = finCount;
    j["rootChord"] = rootChord;
    j["tipChord"] = tipChord;
    j["span"] = span;
    j["sweepAngle"] = sweepAngle;
    return j;
}

void FinSet::deserialize(const json& j) {
    RocketPart::deserialize(j);
    if (j.contains("finCount")) finCount = j["finCount"];
    if (j.contains("rootChord")) rootChord = j["rootChord"];
    if (j.contains("tipChord")) tipChord = j["tipChord"];
    if (j.contains("span")) span = j["span"];
    if (j.contains("sweepAngle")) sweepAngle = j["sweepAngle"];
}


void Motor::onInspect() {
    RocketPart::onInspect();
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Motor Properties");
    
    ImGui::DragFloat("Propellant Mass (kg)", &propellantMass, 0.01f, 0.0f, 1000.0f);
    ImGui::DragFloat("Dry Mass (kg)", &dryMass, 0.01f, 0.0f, 1000.0f);
    ImGui::DragFloat("Burn Time (s)", &burnTime, 0.1f, 0.01f, 100.0f);
    ImGui::DragFloat("Avg Thrust (N)", &averageThrust, 10.0f, 0.0f, 100000.0f);
    ImGui::DragFloat("Isp (s)", &specificImpulse, 1.0f, 0.0f, 500.0f);
}

json Motor::serialize() const {
    json j = RocketPart::serialize();
    j["propellantMass"] = propellantMass;
    j["dryMass"] = dryMass;
    j["burnTime"] = burnTime;
    j["averageThrust"] = averageThrust;
    j["specificImpulse"] = specificImpulse;
    return j;
}

void Motor::deserialize(const json& j) {
    RocketPart::deserialize(j);
    if (j.contains("propellantMass")) propellantMass = j["propellantMass"];
    if (j.contains("dryMass")) dryMass = j["dryMass"];
    if (j.contains("burnTime")) burnTime = j["burnTime"];
    if (j.contains("averageThrust")) averageThrust = j["averageThrust"];
    if (j.contains("specificImpulse")) specificImpulse = j["specificImpulse"];
}


// --- Project ---

Project::Project() {
    // Create default rocket
    root = std::make_shared<RocketPart>("Rocket Root", PartType::Root);
    
    auto mainTube = std::make_shared<BodyTube>();
    mainTube->name = "Main Fuselage";
    root->addChild(mainTube);
    
    auto nose = std::make_shared<NoseCone>();
    mainTube->addChild(nose); // Nose attached to fuselage
    
    selectedPart = root.get();
}

void Project::select(RocketPart* part) {
    selectedPart = part;
}

bool Project::saveToFile(const std::string& filepath) {
    if (!root) return false;
    
    json j;
    j["version"] = 1;
    j["root"] = root->serialize();
    
    std::ofstream file(filepath);
    if (file.is_open()) {
        file << j.dump(4);
        return true;
    }
    return false;
}

bool Project::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;
    
    json j;
    file >> j;
    
    if (j.contains("root")) {
        auto rootJson = j["root"];
        int typeInt = rootJson.value("type", 0);
        
        // Helper recursive function to deserialize
        std::function<std::shared_ptr<RocketPart>(const json&)> parseNode = [&](const json& nodeJson) -> std::shared_ptr<RocketPart> {
            int t = nodeJson.value("type", 0);
            auto part = RocketPart::createFromType(static_cast<PartType>(t));
            if (part) {
                part->deserialize(nodeJson);
                if (nodeJson.contains("children") && nodeJson["children"].is_array()) {
                    for (const auto& childJson : nodeJson["children"]) {
                        auto childPart = parseNode(childJson);
                        if (childPart) {
                            part->addChild(childPart);
                        }
                    }
                }
            }
            return part;
        };
        
        auto newRoot = parseNode(rootJson);
        if (newRoot) {
            root = newRoot;
            selectedPart = root.get();
            return true;
        }
    }
    return false;
}