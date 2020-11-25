#pragma once

#include <UGM/UGM.h>

struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	std::vector<Ubpa::pointf2> lpoints;
	std::vector<Ubpa::vecf2> tangents;
	std::vector<Ubpa::pointf2> tPoints;
	std::vector<float> delta;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	int method{4};
	int selected = -1; 
	int tselected = -1;
	int continuity = 3;
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };
	bool ChangeControlPoint{ false };
};

#include "details/CanvasData_AutoRefl.inl"
