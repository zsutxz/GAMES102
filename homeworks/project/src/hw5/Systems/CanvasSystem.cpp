#include "CanvasSystem.h"

#include "../Components/CanvasData.h"
#include "../Subdivision/Subdivision.h"

#include <_deps/imgui/imgui.h>
#include "../ImGuiFileBrowser.h"
#include "spdlog/spdlog.h"

#include <fstream>


using namespace Ubpa;

constexpr auto MAX_PLOT_NUM_POINTS = 10000;

imgui_addons::ImGuiFileBrowser file_dialog;

bool chaikin = true;
bool cubic = false;
bool quad_ = false;
bool closed = false;
int step_num = 3;
int alpha = 12;
bool originPoints = true;


void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	spdlog::set_pattern("[%H:%M:%S] %v");
	//spdlog::set_pattern("%+"); // back to default format
	
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();

		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			if (ImGui::CollapsingHeader("File")) {
				if (ImGui::MenuItem("Import data", "Ctrl+O")) { data->importData = true; }
				if (ImGui::MenuItem("Export data", "Ctrl+S")) { data->exportData = true; }
			}
			//ImGui::SameLine();
			if (ImGui::CollapsingHeader("Help")) {
				ImGui::Text("ABOUT THIS DEMO:");
				ImGui::BulletText("GAMES-102");
				ImGui::Separator();

				ImGui::Text("USER GUIDE:");
				ImGui::BulletText("Mouse Left: click to add points.");
				ImGui::BulletText("Mouse Right: drag to scroll, click for context menu.");
				ImGui::BulletText("Ctrl+O: Import Data");
				ImGui::BulletText("Ctrl+S: Export Data");
				ImGui::Separator();

			}

			ImGui::Checkbox("Enable grid", &data->opt_enable_grid); ImGui::SameLine(200);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);

			ImGui::Separator();

			ImGui::BeginChild("order_als_id", ImVec2(200, 200));
			ImGui::Checkbox("origin", &originPoints);
			ImGui::Checkbox("Chaikin", &chaikin);
			ImGui::Checkbox("cubic", &cubic);
			ImGui::Checkbox("closed", &closed);
			ImGui::InputInt("step_num", &step_num);
			step_num = step_num < 0 ? 0 : step_num;
			step_num = step_num > 10 ? 10 : step_num;
			ImGui::Checkbox("quad", &quad_);
			ImGui::SliderInt("alpha", &alpha, 1, 32, "alpha = 1/%d");
			ImGui::EndChild(); ImGui::SameLine(250);

			ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
			ImVec2 canvas_sz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
			if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
			if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
			ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

			// Draw border and background color
			ImGuiIO& io = ImGui::GetIO();
			ImDrawList* draw_list = ImGui::GetWindowDrawList();
			draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
			draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

			// This will catch our interactions
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

			// add a point
			if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) { 
				data->isEnd = true;
			}	// 双击结束
			if (is_hovered && !data->isEnd && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
				data->points.push_back(mouse_pos_in_canvas);
				spdlog::info("Point added at: {}, {}", data->points.back()[0], data->points.back()[1]);
			}

			if (data->importData || (io.KeyCtrl && ImGui::IsKeyPressed(79))) {
				ImGui::OpenPopup("Import Data");
				data->importData = false;
			}
			if (data->exportData || (io.KeyCtrl && ImGui::IsKeyPressed(83))) {
				ImGui::OpenPopup("Export Data");
				data->exportData = false;
			}


			if (file_dialog.showFileDialog("Import Data", imgui_addons::ImGuiFileBrowser::DialogMode::OPEN, ImVec2(700, 310), ".txt,.xy,.*")) {
				data->points.clear();
				std::ifstream in(file_dialog.selected_path);
				float x, y;
				while (in >> x >> y) {
					data->points.push_back(ImVec2(x, y));
				}
				spdlog::info(file_dialog.selected_path);
			}

			if (file_dialog.showFileDialog("Export Data", imgui_addons::ImGuiFileBrowser::DialogMode::SAVE, ImVec2(700, 310), ".txt,.xy,.*")) {
				std::ofstream out(file_dialog.selected_path);

				for (int n = 0; n < data->points.size(); ++n)
					out << data->points[n][0] << "\t" << data->points[n][1] << std::endl;

				spdlog::info(file_dialog.selected_path);
			}


			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan)) {
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			if (!data->points.size()) {
				data->isEnd = false;
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context")) {
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) {
					data->points.resize(data->points.size() - 1);
				}
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) {
					data->points.clear();
				}
				if (ImGui::MenuItem("Add...", NULL, false)) {
					data->isEnd = false;
				}
				ImGui::EndPopup();
			}

			// Draw grid + all lines in the canvas
			draw_list->PushClipRect(canvas_p0, canvas_p1, true);
			if (data->opt_enable_grid) {
				const float GRID_STEP = 64.0f;
				for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
				for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
			}

			// 画数据点
			if (true) {
				ImVec2 ps[MAX_PLOT_NUM_POINTS];
				for (int n = 0; n < data->points.size(); ++n) {
					draw_list->AddCircleFilled(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 4, IM_COL32(255, 255, 255, 255));
					ps[n] = ImVec2(data->points[n] + origin);
				}
				if (originPoints) {
					draw_list->AddPolyline(ps, data->points.size(), IM_COL32(0, 255, 0, 255), false, 2.0f);
				}
			}

			if (closed && data->points.size()>2) {
				draw_list->AddLine(data->points[0] + origin, data->points.back() + origin, IM_COL32(0, 255, 0, 255), 2.0f);
			}


			if (data->points.size() > 3) {
				// 计算曲线
				if (chaikin) {
					ImVec2 subdiv_ps_chaikin[MAX_PLOT_NUM_POINTS];
					std::vector<Ubpa::pointf2> subdivP_chaikin = data->points;
					for (int st = 0; st < step_num; ++st) {
						subdivP_chaikin = Subdivision::Chaikin_subdivision(&subdivP_chaikin, closed);
					}
					for (int n = 0; n < subdivP_chaikin.size(); ++n) {
						subdiv_ps_chaikin[n] = ImVec2(subdivP_chaikin[n] + origin);
					}
					draw_list->AddPolyline(subdiv_ps_chaikin, subdivP_chaikin.size(), IM_COL32(0, 255, 255, 255), closed, 1.0f);
					draw_list->AddText(ImVec2(canvas_p1.x - 120, canvas_p1.y - 20 - (cubic + quad_) * 20), IM_COL32(255, 255, 255, 255), "Chaikin");
					draw_list->AddLine(ImVec2(canvas_p1.x - 175, canvas_p1.y - 13 - (cubic + quad_) * 20), ImVec2(canvas_p1.x - 125, canvas_p1.y - 13 - (cubic + quad_) * 20), IM_COL32(0, 255, 255, 255), 2.0f);
				}
				if (cubic) {
					ImVec2 subdiv_ps_cubic[MAX_PLOT_NUM_POINTS];
					std::vector<Ubpa::pointf2> subdivP_cubic = data->points;
					for (int st = 0; st < step_num; ++st) {
						subdivP_cubic = Subdivision::cubic_subdivision(&subdivP_cubic, closed);
					}
					for (int n = 0; n < subdivP_cubic.size(); ++n) {
						subdiv_ps_cubic[n] = ImVec2(subdivP_cubic[n] + origin);
					}
					draw_list->AddPolyline(subdiv_ps_cubic, subdivP_cubic.size(), IM_COL32(255, 0, 255, 255), closed, 1.0f);
					draw_list->AddText(ImVec2(canvas_p1.x - 120, canvas_p1.y - 20 - (quad_) * 20), IM_COL32(255, 255, 255, 255), "cubic");
					draw_list->AddLine(ImVec2(canvas_p1.x - 175, canvas_p1.y - 13 - (quad_) * 20), ImVec2(canvas_p1.x - 125, canvas_p1.y - 13 - (quad_) * 20), IM_COL32(255, 0, 255, 255), 2.0f);
				}
				if (quad_)
				{
					ImVec2 subdiv_ps_quad[MAX_PLOT_NUM_POINTS];
					std::vector<Ubpa::pointf2> subdivP_quad = data->points;
					for (int st = 0; st < step_num; ++st) {
						subdivP_quad = Subdivision::quad_subdivision(&subdivP_quad, closed, 1.0f / alpha);
					}
					for (int n = 0; n < subdivP_quad.size(); ++n) {
						subdiv_ps_quad[n] = ImVec2(subdivP_quad[n] + origin);
					}
					draw_list->AddPolyline(subdiv_ps_quad, subdivP_quad.size(), IM_COL32(255, 255, 0, 255), closed, 1.0f);
					draw_list->AddText(ImVec2(canvas_p1.x - 120, canvas_p1.y - 20 ), IM_COL32(255, 255, 255, 255), "quad");
					draw_list->AddLine(ImVec2(canvas_p1.x - 175, canvas_p1.y - 13), ImVec2(canvas_p1.x - 125, canvas_p1.y - 13), IM_COL32(255, 255, 0, 255), 2.0f);
				}
			}
			draw_list->PopClipRect();
		}
		ImGui::End();
	});
}

