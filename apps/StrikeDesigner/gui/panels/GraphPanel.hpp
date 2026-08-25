#pragma once
class Project; // Forward declaration
class GraphPanel {
public:
	static void draw(Project& project, bool* p_open = nullptr);
};