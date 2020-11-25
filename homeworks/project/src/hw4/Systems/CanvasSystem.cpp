#include "CanvasSystem.h"
#include "../Eigen/Dense"
#include "../Eigen/Sparse"
#include "../Eigen/SparseLU"
#include "../Components/CanvasData.h"
#include <algorithm>
#include <cmath>

#include <_deps/imgui/imgui.h>

using namespace Ubpa;
float ratio = 15.0f;
typedef Eigen::Triplet<float> float_tri;
bool cmp(Ubpa::pointf2 a, Ubpa::pointf2 b)
{
	return a[0] <= b[0];
}
bool selecting(ImVec2 center, ImVec2 mousePos,float r)
{
	bool flag = true;
	if (center[0] - r > mousePos[0])
		flag = false;
	if (center[1] - r > mousePos[1])
		flag = false;
	if (center[0] + r < mousePos[0])
		flag = false;
	if (center[1] + r < mousePos[1])
		flag = false;
	return flag;
}
//参数化方法
std::vector<float> UniformP(std::vector<Ubpa::pointf2>& point);
std::vector<float> ChordalP(std::vector<Ubpa::pointf2>& point);
std::vector<float> CentripetalP(std::vector<Ubpa::pointf2>& point);
std::vector<float> FoleyP(std::vector<Ubpa::pointf2>& point);

//计算样条曲线参数
void CacuSpline(CanvasData* data);
//Hermite插值
std::vector<Ubpa::pointf2>Hermite(CanvasData* data);

float nest(std::vector<Ubpa::pointf2>& point, Eigen::VectorXf& c, float x)
{
	int d = point.size();
	float ret = c[0];
	float accu = 1.0;
	for (int i = 1; i < d; ++i)
	{
		accu *= (x - point[i - 1][0]);
		ret += c[i] * accu;
	}
	return ret;

}

//int selected = -1;  //选择的非切线控制点
//int tselected = -1; //选择的切线控制点
//int continuity = 3; //默认改变切线时C1连续 2: G1 1: G0

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::SliderInt("Patameterization methods", &data->method, 1, 4);
			ImGui::Checkbox("Check to enable point change and disaple point add", &data->ChangeControlPoint);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.\n Change continuity in the context menu ");

			// Typically you would use a BeginChild()/EndChild() pair to benefit from a clipping region + own scrolling.
			// Here we demonstrate that this can be replaced by simple offsetting + custom drawing + PushClipRect/PopClipRect() calls.
			// To use a child window instead we could use, e.g:
			//      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));      // Disable padding
			//      ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(50, 50, 50, 255));  // Set a background color
			//      ImGui::BeginChild("canvas", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoMove);
			//      ImGui::PopStyleColor();
			//      ImGui::PopStyleVar();
			//      [...]
			//      ImGui::EndChild();

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
			if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				int selected = -1;
				ImVec2 mPos = mouse_pos_in_canvas;
				//检测是否选择某个点
				for (size_t i = 0; i < data->points.size(); ++i)
				{
					if (selecting(data->points[i], mPos, 5.0f) == true)
					{
						selected = i;
						data->selected = i;
						data->tselected = -1;
						break;
					}
				}
				//若没有选择点
				if (selected == -1)
				{
					//若是更改控制点模式选择了一个点并且该帧没有选中点
					if (data->ChangeControlPoint && data->selected != -1)
					{
						//若选择了左边的切线控制点
						if (selecting(data->tPoints[0], mPos, 5.0f))
						{
							data->tselected = 0;
						}
						//选择了右边的切线控制点
						else if (selecting(data->tPoints[1], mPos, 5.0f))
						{
							data->tselected = 1;
						}
						//之前选择了非切线控制点
						else if(data->tselected == -1)
						{
							//更改该控制点
							data->points[data->selected] = mPos;
							CacuSpline(data);
						}
						//之前选择了切线控制点
						else
						{
							pointf2 pos(mPos[0], mPos[1]);
							float tx, ty;//新斜率
							//左侧切点
							if (data->tselected == 0)
							{
								tx = data->points[data->selected][0] - pos[0];
								ty = data->points[data->selected][1] - pos[1];
								tx *= ratio;
								ty *= ratio;
							}
							//右侧
							else if (data->tselected == 1)
							{
								tx = pos[0] - data->points[data->selected][0];
								ty = pos[1] - data->points[data->selected][1];
								tx *= ratio;
								ty *= ratio;
							}
							if (data->selected == 0)
							{
								data->tangents[0][0] = tx;
								data->tangents[0][1] = ty;
							}
							else if (data->selected == data->points.size() - 1)
							{
								data->tangents[2 * data->selected - 1][0] = tx;
								data->tangents[2 * data->selected - 1][1] = ty;
							}
							else
							{
								switch (data->continuity)
								{
								case 3:
									//C1 
									if (data->selected != 0)
									{
										data->tangents[2 * data->selected - 1][0] = tx;
										data->tangents[2 * data->selected - 1][1] = ty;
									}
									if (data->selected != data->points.size() - 1)
									{
										data->tangents[2 * data->selected][0] = tx;
										data->tangents[2 * data->selected][1] = ty;
									}
									break;
								case 2:
									//G1

									if (data->tselected == 0)
									{
										//计算右侧切线长度
										float length = data->tangents[2 * data->selected].norm();
										//更新左侧斜率
										data->tangents[2 * data->selected - 1][0] = tx;
										data->tangents[2 * data->selected - 1][1] = ty;
										//获取左侧切线反向单位向量
										vecf2 tv(tx, ty);
										tv = tv.normalize();
										tv *= length;
										//更新右侧
										data->tangents[2 * data->selected][0] = tv[0];
										data->tangents[2 * data->selected][1] = tv[1];
										
									}
									else
									{
										//计算左侧切线长度
										float length = data->tangents[2 * data->selected - 1].norm();
										//更新右侧斜率
										data->tangents[2 * data->selected][0] = tx;
										data->tangents[2 * data->selected ][1] = ty;
										//获取右侧切线单位向量
										vecf2 tv(tx, ty);
										tv = tv.normalize();
										tv *= length;
										data->tangents[2 * data->selected - 1][0] = tv[0];
										data->tangents[2 * data->selected - 1][1] = tv[1];
									}
									break;
								case 1:
									//G0
									if (data->tselected == 0)
									{
										data->tangents[2 * data->selected - 1][0] = tx;
										data->tangents[2 * data->selected - 1][1] = ty;
									}
									else
									{
										data->tangents[2 * data->selected][0] = tx;
										data->tangents[2 * data->selected][1] = ty;
									}
									break;
								default:
									break;
								}
							}
									
							//TODO :G1 G0
						}
					}
					else if (!data->ChangeControlPoint) //不在移动控制点模式
					{
						if (data->selected != -1)  //之前选择了控制点 则取消之
						{
							data->selected = -1;
							data->tselected = -1;
						}
						else //未选择点则插入新点
						{
							data->points.push_back(mPos);

							switch (data->method)
							{
							case 1: data->delta = UniformP(data->points); break;
							case 2: data->delta = ChordalP(data->points); break;
							case 3: data->delta = CentripetalP(data->points); break;
							case 4: data->delta = FoleyP(data->points); break;
							}
							CacuSpline(data);
						}	
					}
				}
			}

			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan))
			{
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context"))
			{
				if (data->adding_line)
					data->points.resize(data->points.size() - 2);
				data->adding_line = false;
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() >= 0)) { data->points.clear(); 
				data->lpoints.clear();
				data->delta.clear();
				data->selected = -1;
				data->tselected = -1;
				data->tPoints.clear();
				}
				if (ImGui::MenuItem("Keep C1", NULL, false))
				{
					data->continuity = 3;
				}
				if (ImGui::MenuItem("Keep G1", NULL, false))
				{
					data->continuity = 2;
				}
				if (ImGui::MenuItem("Keep G0", NULL, false))
				{
					data->continuity = 1;
				}
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
			for (int n = 0; n < data->points.size(); n += 1)
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 5.0f, IM_COL32(255, 255, 0, 255), 0);
			if (data->selected != -1)
			{
				data->tPoints.clear();
				int s = data->selected;
				//普通点的选择框
					draw_list->AddRect(ImVec2(origin.x + data->points[data->selected][0] - 5.0f, origin.y + data->points[data->selected][1] - 5.0f),
						ImVec2(origin.x + data->points[data->selected][0] + 5.0f, origin.y + data->points[data->selected][1] + 5.0f),
						IM_COL32(255, 255, 255, 255));
					//切线与切线控制点绘制
					if (s != 0)
					{
						pointf2 tp1(data->points[s][0] - data->tangents[2 * s - 1][0] / ratio,
							data->points[s][1] - data->tangents[2 * s - 1][1] / ratio);
						data->tPoints.push_back(tp1);
						draw_list->AddLine(ImVec2(origin.x + data->points[s][0], origin.y + data->points[s][1]),
							ImVec2(origin.x + tp1[0], origin.y + tp1[1]), IM_COL32(255, 255, 255, 255), 2.0f);

						draw_list->AddCircleFilled(ImVec2(origin.x + tp1[0], origin.y + tp1[1]), 5.0f, IM_COL32(0, 0, 255, 255), 0);
						if (data->tselected == 0)
						{
							draw_list->AddRect(ImVec2(origin.x + tp1[0] - 5.0f, origin.y + tp1[1] - 5.0f),
								ImVec2(origin.x + tp1[0] + 5.0f, origin.y + tp1[1] + 5.0f),
								IM_COL32(255, 255, 255, 255));
						}
					}
					else
						data->tPoints.push_back(pointf2(0.0f, 0.0f));
					if (s != data->points.size() - 1)
					{
						pointf2 tp1(data->points[s][0] + data->tangents[2 * s][0] / ratio,
							data->points[s][1] + data->tangents[2 * s][1] / ratio);
						data->tPoints.push_back(tp1);
						draw_list->AddLine(ImVec2(origin.x + data->points[s][0], origin.y + data->points[s][1]),
							ImVec2(origin.x + tp1[0], origin.y + tp1[1]), IM_COL32(255, 255, 255, 255), 2.0f);
						draw_list->AddCircleFilled(ImVec2(origin.x + tp1[0], origin.y + tp1[1]), 5.0f, IM_COL32(0, 0, 255, 255), 0);
						if (data->tselected == 1)
						{
							draw_list->AddRect(ImVec2(origin.x + tp1[0] - 5.0f, origin.y + tp1[1] - 5.0f),
								ImVec2(origin.x + tp1[0] + 5.0f, origin.y + tp1[1] + 5.0f),
								IM_COL32(255, 255, 255, 255));
						}
					}
					else
						data->tPoints.push_back(pointf2(0.0f, 0.0f));
			}

			// draw lines
			data->lpoints = Hermite(data);
			for (int n = 0; n < data->lpoints.size(); n += 2)
			{
				draw_list->AddLine(ImVec2(origin.x + data->lpoints[n][0], origin.y + data->lpoints[n][1]), ImVec2(origin.x + data->lpoints[n + 1][0], origin.y + data->lpoints[n + 1][1]), IM_COL32(255, 0, 0, 255), 2.0f);
			}
		
			draw_list->PopClipRect();
		}	

		ImGui::End();
	});
}

std::vector<float>UniformP(std::vector<Ubpa::pointf2>& point)
{
	int d = point.size();
	float tdelta = 1.0 / (d - 1);
	std::vector<Ubpa::pointf2> ret;
	std::vector<float> delta;
	for (int i = 0; i < d - 1; ++i)
		delta.push_back(tdelta);
	return delta;
}

std::vector<float> ChordalP(std::vector<Ubpa::pointf2>& point)
{
	int d = point.size();
	std::vector<float> delta;
	float arc_length = 0.0;
	for (int i = 1; i < d; ++i)
	{
		arc_length += (point[i] - point[i - 1]).norm();
	}
	for (int i = 1; i < d; ++i)
	{
		float tl = (point[i] - point[i - 1]).norm();
		tl /= arc_length;
		delta.push_back(tl);
	}
	return delta;
}

std::vector<float> CentripetalP(std::vector<Ubpa::pointf2>& point)
{
	int d = point.size();
	std::vector<float> delta;
	float arc_length = 0.0;
	for (int i = 1; i < d; ++i)
	{
		arc_length += sqrt((point[i] - point[i - 1]).norm());

	}
	for (int i = 1; i < d; ++i)
	{
		float tl = sqrt((point[i] - point[i - 1]).norm());
		tl /= arc_length;
		delta.push_back(tl);
	}
	return delta;
}

std::vector<float> FoleyP(std::vector<Ubpa::pointf2>& point)
{
	int d = point.size();
	std::vector<float> delta;
	if (d < 3)
		return delta;
	std::vector<float> dis;
	std::vector<float> theta;
	float delta_sum = 0.0;
	theta.push_back(-1);
	for (int i = 1; i < d; ++i)
	{
		if (i != d - 1)
		{
			Ubpa::vecf2 a = point[i - 1] - point[i];
			Ubpa::vecf2 b = point[i + 1] - point[i];
			float ttheta = acos(a.dot(b) / (a.norm() * b.norm()));

			theta.push_back(std::min(PI<float> -ttheta, PI<float> / 2.0f));
		}
		dis.push_back((point[i] - point[i - 1]).norm());
	}
	float ttemp = dis[0] * (1 + 3 * theta[1] * dis[1] / (2 * (dis[0] + dis[1])));
	delta.push_back(ttemp);
	delta_sum += ttemp;
	for (int i = 1; i < d - 2; ++i)
	{
		ttemp = dis[i] * (1 + 3 * theta[i] * dis[i - 1] / (2 * (dis[i - 1] + dis[i]) + 3 * theta[i + 1] * dis[i + 1] / (2 * (dis[i] + dis[i + 1]))));
		delta.push_back(ttemp);
		delta_sum += ttemp;
	}
	ttemp = dis[d - 2] * (1 + 3 * theta[d - 2] * dis[d - 3] / (2 * (dis[d - 3] + dis[d - 2])));
	delta_sum += ttemp;
	delta.push_back(ttemp);

	for (int i = 0; i < delta.size(); ++i)
	{
		delta[i] /= delta_sum;
	}
	return delta;
}


void CacuSpline(CanvasData* data)
{
	std::vector<Ubpa::pointf2> ret;
	//start here
	data->tangents.clear();
	int d = data->points.size();
	if (d < 3)
		return;
	Eigen::SparseMatrix<float> A(d - 2, d - 2);
	Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int> >solver;
	//Eigen::MatrixXf A(d, dim + 1);
	Eigen::VectorXf b(d - 2, 1);
	Eigen::VectorXf by(d - 2, 1);
	//存储三元数的vector
	std::vector<float_tri> tv;
	float t = 0.0;
	for (int i = 0; i < d - 2; ++i)
	{
		float h_pre = data->delta[i];
		float h_now = data->delta[i + 1];
		float u = 2 * (h_pre + h_now);
		if (i != 0)
		{
			float_tri temp(i, i - 1, h_pre);
			tv.push_back(temp);
		}
		if (i != d - 3)
		{
			float_tri temp(i, i + 1, h_now);
			tv.push_back(temp);
		}
		float_tri temp(i, i, u);
		tv.push_back(temp);
		//handle x first
		float b_pre = 6 * (data->points[i + 1][0] - data->points[i][0]) / h_pre;
		float b_now = 6 * (data->points[i + 2][0] - data->points[i + 1][0]) / h_now;
		float v = b_now - b_pre;
		b(i) = v;
		//then handle y
		b_pre = 6 * (data->points[i + 1][1] - data->points[i][1]) / h_pre;
		b_now = 6 * (data->points[i + 2][1] - data->points[i + 1][1]) / h_now;
		v = b_now - b_pre;
		by(i) = v;

	}
	//解三对角方程
	A.setFromTriplets(tv.begin(), tv.end());
	solver.analyzePattern(A);
	solver.factorize(A);
	Eigen::VectorXf M = solver.solve(b);
	std::vector<float> Mx, My;
	Mx.push_back(0);
	for (int i = 0; i < M.size(); ++i)
	{
		Mx.push_back(M[i]);
	}
	Mx.push_back(0);
	M = solver.solve(by);
	My.push_back(0);
	for (int i = 0; i < M.size(); ++i)
	{
		My.push_back(M[i]);
	}
	My.push_back(0);
	Ubpa::pointf2 pre;
	float accu = 0.0;
	bool first = 1;
	for (int i = 0; i < d - 1; ++i)
	{
		float st = accu;
		float ed = accu + data->delta[i];
		accu = ed;
		float x = data->points[i][0];
		float x1 = data->points[i + 1][0];
		float y = data->points[i][1];
		float y1 = data->points[i + 1][1];
		float h = data->delta[i];
		float f0_hat_x = -h / 3 * Mx[i] - h / 6 * Mx[i + 1] - x / h + x1 / h;
		float f1_hat_x = h / 6 * Mx[i] + h / 3 * Mx[i + 1] - x / h + x1 / h;
		float f0_hat_y = -h / 3 * My[i] - h / 6 * My[i + 1] - y / h + y1 / h;
		float f1_hat_y = h / 6 * My[i] + h / 3 * My[i + 1] - y / h + y1 / h;
		float hneg = -data->delta[i];
		vecf2 temp(f0_hat_x, f0_hat_y);
		data->tangents.push_back(temp);
		temp[0] = f1_hat_x;
		temp[1] = f1_hat_y;
		data->tangents.push_back(temp);
		
	}
}

std::vector<Ubpa::pointf2>Hermite(CanvasData* data)
{
	std::vector<Ubpa::pointf2> ret;
	if (data->points.size() < 3)
		return ret;
	int d = data->points.size();
	float accu = 0.0;
	Ubpa::pointf2 pre;
	bool first = true;
	for (int i = 0; i < d - 1; ++i)
	{
		float st = accu;
		float ed = accu + data->delta[i];
		accu = ed;
		float x = data->points[i][0];
		float x1 = data->points[i + 1][0];
		float y = data->points[i][1];
		float y1 = data->points[i + 1][1];
		float h = data->delta[i];
		float f0_hat_x = data->tangents[2 * i][0];
		float f1_hat_x = data->tangents[2 * i + 1][0];
		float f0_hat_y = data->tangents[2 * i][1];
		float f1_hat_y = data->tangents[2 * i + 1][1];
		float hneg = -data->delta[i];
		for (float t = st; t < ed; t += 0.01)
		{
			Ubpa::pointf2 tp;
			float h0 = (1 + 2 * (t - st) / h) * pow((t - ed) / hneg, 2);
			float h1 = (1 + 2 * (t - ed) / hneg) * pow((t - st) / h, 2);
			float h0_hat = (t - st) * pow((t - ed) / hneg, 2);
			float h1_hat = (t - ed) * pow((t - st) / h, 2);
			tp[0] = h0 * x + h1 * x1 + h0_hat * f0_hat_x + h1_hat * f1_hat_x;
			tp[1] = h0 * y + h1 * y1 + h0_hat * f0_hat_y + h1_hat * f1_hat_y;
			/*tp[0] = Mx[i] * pow((ed - t), 3) / (6 * h) + Mx[i + 1] * pow((t - st), 3) / (6 * h) +
				(x1 / h - Mx[i + 1] * h / 6.0) * (t - st) + (x / h - Mx[i] * h / 6.0) * (ed - t);
			tp[1] = My[i] * pow((ed - t), 3) / (6 * h) + My[i + 1] * pow((t - st), 3) / (6 * h) +
				(y1 / h - My[i + 1] * h / 6.0) * (t - st) + (y / h - My[i] * h / 6.0) * (ed - t);*/
			if (first)
			{
				first = false;
			}
			else
			{
				ret.push_back(pre);
				ret.push_back(tp);
			}
			pre = tp;

		}
	}
	return ret;
}
