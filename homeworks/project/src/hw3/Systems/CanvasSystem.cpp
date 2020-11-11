#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>
#include "../Eigen/Core"
#include "../Eigen/Dense"

using namespace Ubpa;

#define M_PI 3.1415926f

float PAI(const std::vector<Ubpa::pointf2>& P, int j, float x)
{
	int n = P.size();
	float numerator = 1;
	float denominator = 1;
	float ret_v = 1.0f;
	for (int k = 0; k < n; ++k)
	{
		if (j == k)
		{
			continue;
		}

		numerator *= (x - P[k][0]);
		denominator *= (P[j][0] - P[k][0]);
	}
	return numerator / denominator;
}

float Polynomial(const std::vector<Ubpa::pointf2>& P, float x)
{
	int n = P.size();
	float sum = 0;
	for (int j = 0; j < n; ++j)
	{
		sum += P[j][1] * PAI(P, j, x);
	}
	return sum;
}
ImVec2 lagrangeInterpolation(float t, const std::vector<float>& parameterization, const std::vector<Ubpa::pointf2>& points) {
	int n = points.size();
	for (size_t i = 0; i < n; i++)
	{
		if (t == parameterization[i]) return ImVec2(points[i][0], points[i][1]);
	}

	float x = 0, y = 0;
	for (size_t j = 0; j < n; j++)
	{
		float l = 1.0;
		for (size_t i = 0; i < n; i++) {
			if (j == i) continue;
			l = l * (t - parameterization[i]) / (parameterization[j] - parameterization[i]);
		}

		x = x + points[j][0] * l;
		y = y + points[j][1] * l;
	}

	return ImVec2(x, y);
}

//第一种方法
void UniformParameterization(CanvasData* data, const ImVec2 &origin)
{
	std::vector<Ubpa::pointf2> temp_pX;
	std::vector<Ubpa::pointf2> temp_pY;

	std::vector<float> temp_sample_nums;
	for (int i = 0;i < data->points.size();i++)
	{
		//size个数，从0-1均匀分布
		float den_num = data->points.size() - 1;
		Ubpa::pointf2 temp_v = Ubpa::pointf2(i * 1.0f / den_num, data->points[i][0]);
		temp_pX.push_back(temp_v);

		temp_v = Ubpa::pointf2(i * 1.0f / den_num, data->points[i][1]);
		temp_pY.push_back(temp_v);
		temp_sample_nums.push_back(i * 1.0f / den_num);
	}

	Eigen::VectorXf temp_draw_nums = Eigen::VectorXf::LinSpaced(1000, 0, 1);

	for (size_t i = 0; i < temp_draw_nums.size(); ++i)
	{
		ImVec2 point = ImVec2(Polynomial(temp_pX, temp_draw_nums[i]), Polynomial(temp_pY, temp_draw_nums[i]));

		data->LagrangeResults.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
	}
}

//第二种方法
void ChordalParameterization(CanvasData* data, const ImVec2& origin)
{
	std::vector<Ubpa::pointf2> temp_pX;
	std::vector<Ubpa::pointf2> temp_pY;

	std::vector<float> temp_sample_nums;

	int num = data->points.size();
	if (num == 1 || num == 2)
	{
		temp_sample_nums.push_back(1.0f);
		if (num == 2)
		{
			temp_sample_nums.push_back(1.0f);
		}
	}
	else
	{
		temp_sample_nums.push_back(0.0f);
		for (int i = 0; i < num - 1; i++)
		{
			float dx = data->points[i + 1][0] - data->points[i][0];
			float dy = data->points[i + 1][1] - data->points[i][1];
			float chord = dx * dx + dy * dy;//sqrt(dx * dx + dy * dy);
			temp_sample_nums.push_back(temp_sample_nums[i] + chord);
		}
	}

	for (int i = 0;i < data->points.size();i++)
	{
		//size个数
		float den_num = temp_sample_nums[data->points.size()-1];
		Ubpa::pointf2 temp_v = Ubpa::pointf2(1.0f*temp_sample_nums[i], data->points[i][0]);
		temp_pX.push_back(temp_v);

		temp_v = Ubpa::pointf2(1.0f * temp_sample_nums[i], data->points[i][1]);
		temp_pY.push_back(temp_v);
	}

	Eigen::VectorXf temp_draw_nums = Eigen::VectorXf::LinSpaced(1000, 0, temp_sample_nums[num - 1]);

	for (size_t i = 0; i < temp_draw_nums.size(); ++i)
	{
		ImVec2 point = ImVec2(Polynomial(temp_pX, temp_draw_nums[i]), Polynomial(temp_pY, temp_draw_nums[i]));

		data->GaussResults.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
	}
}

//第三种
void CentripetalParameterization(CanvasData* data, const ImVec2& origin)
{
	std::vector<float> temp_sample_nums;

	int num = data->points.size();
	if (num == 1 || num == 2)
	{
		temp_sample_nums.push_back(1.0f);
		if (num == 2)
		{
			temp_sample_nums.push_back(1.0f);
		}
	}
	else
	{
		temp_sample_nums.push_back(0.0f);
		for (size_t i = 0; i < num - 1; i++)
		{
			float dx = data->points[i + 1][0] - data->points[i][0];
			float dy = data->points[i + 1][1] - data->points[i][1];
			float chord = sqrt(dx * dx + dy * dy);
			temp_sample_nums.push_back(temp_sample_nums[i] + chord);
		}
	}

	Eigen::VectorXf T = Eigen::VectorXf::LinSpaced(1000, 0, temp_sample_nums[num-1]);
	for (size_t i = 0; i < T.size(); i++)
	{
		ImVec2 point = lagrangeInterpolation(T[i], temp_sample_nums, data->points);
		data->LeastSquaresResults.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
	}
}

float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points) {
	float dx, dy;
	// d_prev
	dx = points[i][0] - points[i - 1][0];
	dy = points[i][1] - points[i - 1][1];
	float d_prev = sqrt(dx * dx + dy * dy);

	// d_next
	dx = points[i + 1][0] - points[i][0];
	dy = points[i + 1][1] - points[i][1];
	float d_next = sqrt(dx * dx + dy * dy);

	// l2
	dx = points[i + 1][0] - points[i - 1][0];
	dy = points[i + 1][1] - points[i - 1][1];
	float l2 = dx * dx + dy * dy;

	float alpha = M_PI - acos((d_prev * d_prev + d_next * d_next - l2 / (2 * d_next * d_prev)));

	return alpha;
}

//第四种
void FoleyParameterization(CanvasData* data, const ImVec2& origin)
{
	std::vector<float> temp_sample_nums;

	int num = data->points.size();

	if (num == 1 || num == 2)
	{
		temp_sample_nums.push_back(1.0f);
		if (num == 2)
		{
			temp_sample_nums.push_back(1.0f);
		}
		return;
	}

	temp_sample_nums.push_back(0.0f);
	float d_prev = 0, d_next = 0;
	for (size_t i = 0; i < num - 1; i++)
	{
		float dx = data->points[i + 1][0] - data->points[i][0];
		float dy = data->points[i + 1][1] - data->points[i][1];
		float chord = sqrt(dx * dx + dy * dy);

		if (i == num - 2) d_next = 0;
		else {
			float dx = data->points[i + 2][0] - data->points[i + 1][0];
			float dy = data->points[i + 2][1] - data->points[i + 1][1];

			d_next = sqrt(dx * dx + dy * dy);
		}

		float factor = 1.0;
		if (i == 0)
		{
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, data->points));
			factor = 1 + 1.5 * (theta_next * d_next) / (chord + d_next);
		}
		else if (i == num - 2) {
			float theta = fminf(M_PI / 2, _getAlpha(i, data->points));
			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord);
		}
		else {
			float theta = fminf(M_PI / 2, _getAlpha(i, data->points));
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, data->points));

			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord) + 1.5 * (theta_next * d_next) / (chord + d_next);
		}

		temp_sample_nums.push_back(temp_sample_nums[i] + chord * factor);

		d_prev = chord;
	}
	for (int i = 0;i < num;i++)
	{
		temp_sample_nums[i] = temp_sample_nums[i] / temp_sample_nums[num - 1];
	}

	Eigen::VectorXf T = Eigen::VectorXf::LinSpaced(100, 0, temp_sample_nums[num - 1]);
	for (size_t i = 0; i < T.size(); i++)
	{
		ImVec2 point = lagrangeInterpolation(T[i], temp_sample_nums, data->points);
		data->RidgeRegressionResults.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
	}
}

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: click to add points,\nMouse Right: drag to scroll, click for context menu.");

			ImGui::Checkbox("Lagrange", &data->opt_lagrange);
			ImGui::Checkbox("Gauss", &data->opt_gauss);

			ImGui::Checkbox("LeastSquares", &data->opt_least_squares);
			ImGui::SameLine(200);
			ImGui::InputInt("m", &data->LeastSquaresM);

			ImGui::Checkbox("RidgetRegression", &data->opt_ridge_regression);
			ImGui::SameLine(200);
			ImGui::InputFloat("lamda", &data->RidgeRegressionLamda, 0.01, 1, 3);

			// Using InvisibleButton() as a convenience 1) it will advance the layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
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

			// Add first and second point
			if (is_hovered /*&& !data->adding_line*/ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				data->points.push_back(mouse_pos_in_canvas);
				float Xmin = 999999;
				float Xmax = -99999;
				for (size_t i = 0; i < data->points.size(); ++i)
				{
					if (data->points[i][0] < Xmin)
					{
						Xmin = data->points[i][0];
					}
					if (data->points[i][0] > Xmax)
					{
						Xmax = data->points[i][0];
					}
				}
				data->LagrangeResults.clear();
				data->GaussResults.clear();
				data->LeastSquaresResults.clear();
				data->RidgeRegressionResults.clear();

				if (data->points.size() > 1)
				{
					UniformParameterization(data, origin);
					ChordalParameterization(data, origin);
					CentripetalParameterization(data, origin);
					FoleyParameterization(data, origin);
				}

				//for (int x = Xmin - 1; x < Xmax + 2; ++x)
				//{
				//	data->LagrangeResults.push_back(ImVec2(origin.x + x, origin.y + Polynomial(data->points, x)));
				//	data->GaussResults.push_back(ImVec2(origin.x + x, origin.y + Gauss(data->points, x)));
				//	data->LeastSquaresResults.push_back(ImVec2(origin.x + x, origin.y + LeastSquares(data->points, x, data->LeastSquaresM)));
				//	data->RidgeRegressionResults.push_back(ImVec2(origin.x + x, origin.y + RidgetRegression(data->points, x, data->RidgeRegressionLamda, data->LeastSquaresM)));
				//}
			}

			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context"))
			{
				/*if (data->adding_line)
					data->points.resize(data->points.size() - 2);
				data->adding_line = false;*/
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { data->points.resize(data->points.size() - 1); }
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { data->points.clear(); }
				ImGui::EndPopup();
			}

			// Draw grid + all lines in the canvas
			draw_list->PushClipRect(canvas_p0, canvas_p1, true);
			if (data->opt_enable_grid)
			{
				const float GRID_STEP = 64.0f;
				for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
				for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
			}
			for (int n = 0; n < data->points.size(); n++)
			{
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 4.0f, IM_COL32(255, 255, 0, 255));
			}

			if (data->opt_lagrange)
			{
				draw_list->AddPolyline(data->LagrangeResults.data(), data->LagrangeResults.size(), IM_COL32(64, 128, 255, 255), false, 1.0f);
			}
			if (data->opt_gauss)
			{
				draw_list->AddPolyline(data->GaussResults.data(), data->GaussResults.size(), IM_COL32(128, 255, 255, 255), false, 1.0f);
			}
			if (data->opt_least_squares)
			{
				draw_list->AddPolyline(data->LeastSquaresResults.data(), data->LeastSquaresResults.size(), IM_COL32(255, 128, 128, 255), false, 1.0f);
			}
			if (data->opt_ridge_regression)
			{
				draw_list->AddPolyline(data->RidgeRegressionResults.data(), data->RidgeRegressionResults.size(), IM_COL32(255, 64, 64, 255), false, 1.0f);
			}

			draw_list->PopClipRect();
		}

		ImGui::End();
		});
}
