#pragma once
#include <UGM/UGM.h>

//#include <Eigen/Dense>

//#include "spdlog/spdlog.h"  // 调试使用
/**********************************************************************************
/// @file       Subdivision.h
/// @author     Qingjun Chang
/// @date       2020.11.21
/// @brief      
/// @details    
**********************************************************************************/

#define EPSILON 1E-15

// 重载加法
Ubpa::pointf2 operator+(const Ubpa::pointf2& a, const Ubpa::pointf2& b) {
	Ubpa::pointf2 pf2;
	pf2[0] = a[0] + b[0];
	pf2[1] = a[1] + b[1];
	return pf2;
}

// 重载减法
Ubpa::pointf2 operator-(const Ubpa::pointf2& a, const Ubpa::pointf2& b) {
	Ubpa::pointf2 pf2;
	pf2[0] = a[0] - b[0];
	pf2[1] = a[1] - b[1];
	return pf2;
}

// 重载乘法
Ubpa::pointf2 operator*(const Ubpa::pointf2& a, const float b) {
	Ubpa::pointf2 pf2;
	pf2[0] = a[0] * b;
	pf2[1] = a[1] * b;
	return pf2;
}

// 重载除法
Ubpa::pointf2 operator/(const Ubpa::pointf2& a, const float b) {
	Ubpa::pointf2 pf2;
	pf2[0] = a[0] / b;
	pf2[1] = a[1] / b;
	return pf2;
}

namespace Subdivision {
	/*
	/// @brief      Chaikin细分曲线
	/// @details    
	/// @param[in]  : 
	/// @return     
	/// @attention  
	*/
	std::vector<Ubpa::pointf2> Chaikin_subdivision(std::vector<Ubpa::pointf2>* p, bool close) {
		std::vector<Ubpa::pointf2> newP;
		newP.clear();
		int n = p->size();
		if (close) {
			for (int i = 0; i < p->size(); ++i) {
				newP.push_back(p->at((i - 1 + n) % n) * 0.25f + p->at(i) * 0.75f);
				newP.push_back(p->at(i) * 0.75f + p->at((i + 1) % n) * 0.25f);
			}
		}
		if (!close) {
			newP.push_back(p->at(0) * 0.75f + p->at(1) * 0.25f);
			for (int i = 1; i < p->size() - 1; ++i) {
				newP.push_back(p->at((i - 1 + n) % n) * 0.25f + p->at(i) * 0.75f);
				newP.push_back(p->at(i) * 0.75f + p->at((i + 1) % n) * 0.25f);
			}
			newP.push_back(p->at((n - 1 - 1 + n) % n) * 0.25f + p->at(n - 1) * 0.75f);
		}
		return newP;
	}

	/*
	/// @brief      三次B-spline细分曲线
	/// @details    
	/// @param[in]  : 
	/// @return     
	/// @attention  
	*/
	std::vector<Ubpa::pointf2> cubic_subdivision(std::vector<Ubpa::pointf2>* p, bool close) {
		std::vector<Ubpa::pointf2> newP;
		newP.clear();
		int n = p->size();
		if (close) {
			for (int i = 0; i < p->size(); ++i) {
				newP.push_back(p->at((i - 1 + n) % n) * 0.125f + p->at(i) * 0.75f + p->at((i + 1) % n) * 0.125f);
				newP.push_back(p->at(i) * 0.5f + p->at((i + 1) % n) * 0.5f);
			}
		}
		if (!close) {
			newP.push_back(p->at(0) * 0.5f + p->at(1) * 0.5f);
			for (int i = 1; i < p->size() - 1; ++i) {
				newP.push_back(p->at((i - 1 + n) % n) * 0.125f + p->at(i) * 0.75f + p->at((i + 1) % n) * 0.125f);
				newP.push_back(p->at(i) * 0.5f + p->at((i + 1) % n) * 0.5f);
			}
		}
		return newP;
	}

	/*
	/// @brief      四点插值型细分曲线
	/// @details    
	/// @param[in]  : 
	/// @return     
	/// @attention  
	*/
	std::vector<Ubpa::pointf2> quad_subdivision(std::vector<Ubpa::pointf2>* p, bool close,float alpha = 0.075) {
		std::vector<Ubpa::pointf2> newP;
		newP.clear();
		int n = p->size();
		if (close) {
			for (int i = 0; i < p->size(); ++i) {
				newP.push_back(p->at(i));
				newP.push_back((p->at(i) + p->at((i + 1) % n)) / 2.0f + ((p->at(i) + p->at((i + 1) % n)) / 2.0f - (p->at((i - 1 + n) % n) + p->at((i + 2) % n)) / 2.0f) * alpha);
			}
		}
		if (!close) {
			for (int i = 0; i < p->size() - 1; ++i) {
				newP.push_back(p->at(i));
				newP.push_back((p->at(i) + p->at((i + 1) % n)) / 2.0f + ((p->at(i) + p->at((i + 1) % n)) / 2.0f - (p->at((i - 1 + n) % n) + p->at((i + 2) % n)) / 2.0f) * alpha);
			}
			
		}
		return newP;
	}
}
