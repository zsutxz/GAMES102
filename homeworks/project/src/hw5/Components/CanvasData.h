#pragma once

#include <UGM/UGM.h>

struct CanvasData {
	std::vector<Ubpa::pointf2> points;

	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ false };
	bool opt_enable_context_menu{ true };

	bool importData{ false };
	bool exportData{ false };

	bool isEnd{ false };

};

#include "details/CanvasData_AutoRefl.inl"
