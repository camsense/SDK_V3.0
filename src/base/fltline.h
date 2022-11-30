/* Copyright camsense Co.Ltd, 2022 all rights reserved
 * Decription: filter line dot
 * Author: cenjinbin
 * Create: 2022-11-2
*/
#ifndef FLTLINE_H
#define FLTLINE_H
#include "ployfit.h"
#include <unordered_map>
#include <vector>
#include <tuple>
#include <utility>
#include <iostream>
#include <functional>
using namespace std;
using namespace hcmath;
#define  CV_PI                3.1415926535897932384626433832795
#define  ORG_REGION           85.0
#define  LINE_DIST            6.0
#define  DIST_THRESH          48
#define  DELETE_INVALID_CLOUDPOINT 0

class PloyFitFun
{
public:
	//射线判断函数体
	class FunLine {
		double m_k;
		double m_d;
	public:
		FunLine(double k, double d) : m_k(k), m_d(d) {}
		bool operator ()(const tsPointCloud& a) {
			double angle = a.dAngle * CV_PI / 180;
			double x = a.u16Dist * cos(angle);
			double y = a.u16Dist * sin(angle);
			double distk = (m_k * x - y + m_d) / sqrt(m_k * m_k + 1);
			return fabs(distk) < LINE_DIST;
		}
	};
	//点判断函数体
	template<typename T>
	class FunDot {
		T& m_tag;
	public:
		FunDot(T& tag) : m_tag(tag) {}
		bool operator ()(const tsPointCloud& a) {
			bool bfind = false;
			for (auto& it : m_tag) {
				if (a.dAngle == it.dAngle && a.u16Dist == it.u16Dist) {
					bfind = true;
				}
			}
			return bfind;
		}
	};
	//删除点云操作
	class DelPC
	{
	public:
		
		template<typename Fun>
		static void Process(vector<tsPointCloud>& lstpc, Fun& fun)
		{
			auto delpos = remove_if(lstpc.begin(), lstpc.end(), fun);
			lstpc.erase(delpos, lstpc.end());
		}
	};
	//把点云设置无效操作
	class InvalidPC
	{
	public:
		
		template<typename Fun>
		static void Process(vector<tsPointCloud>& lstpc, Fun& fun)
		{
			for (auto& it : lstpc) {
				if (fun(it)) {
					it.bValid = false;
				}
			}
		}
	};
	//实现删除或者设无效操作
	template<typename T, typename Fun>
	static void delPcImp(vector<tsPointCloud>& lstpc,
		Fun& fun) {
		T::Process<Fun>(lstpc, fun);
	}
	//调用删除或者无效接口
	template<typename Fun>
	static void delPc(vector<tsPointCloud>& lstpc, Fun& fun) {
#if DELETE_INVALID_CLOUDPOINT
		delPcImp<DelPC, Fun>(lstpc, fun);
#else
		delPcImp<InvalidPC, Fun>(lstpc, fun);
#endif
	}

	
	//过滤函数
	static void filter(vector<tsPointCloud>& lstPointCloud)
	{
		//射线过滤
		fltline(lstPointCloud);
		//单点过滤
		fltdot1(lstPointCloud);
		//双点过滤
		fltdot2(lstPointCloud);
	}

	//射线过滤
	static bool fltline(vector<tsPointCloud>& lstPC)
	{
		//拷贝副本，删除无效点
		auto lstPointCloud = lstPC;
		delInvalid(lstPointCloud);

		//判空
		if (lstPointCloud.empty()) {
			return false;
		}
		//根据角度排序
		sort(lstPointCloud.begin(), lstPointCloud.end(), [](tsPointCloud &a,
			tsPointCloud& b) {
			return a.dAngle < b.dAngle;
		});
		
		//计算相邻两点间隔距离
		vector<unsigned short> adjacent_dist(1, 0);
		for (auto it = lstPointCloud.begin(); next(it) != lstPointCloud.end(); ++it) {
			adjacent_dist.push_back(abs(it->u16Dist - next(it)->u16Dist));
		}
		//根据间隔分段
		vector<list<tsPointCloud>> segmentList;
		const size_t bufSize = adjacent_dist.size();
		list<tsPointCloud> ad_list(1, lstPointCloud.front());
		for (size_t i = 1; i < bufSize; ++i) {
			if (adjacent_dist[i] < DIST_THRESH) {
				ad_list.push_back(lstPointCloud[i]);
			}
			else {
				segmentList.push_back(ad_list);
				ad_list.clear();
				ad_list.push_back(lstPointCloud[i]);
			}
		}
		if (!ad_list.empty()) {
			segmentList.push_back(ad_list);
		}
		//每段小于等于3个点的段删除
		auto delpos = remove_if(segmentList.begin(), segmentList.end(), [](decltype(segmentList)::value_type& a) {
			return a.size() <= 3;
		});
		segmentList.erase(delpos, segmentList.end());

		

		//最小线性拟合
		for (auto &segit : segmentList) {
			vector<double> x;
			vector<double> y;
			for (auto& it : segit) {
				auto ret = culPoint(it);
				x.push_back(ret.first);
				y.push_back(ret.second);
			}
			Fit fit;
			fit.linearFit(x, y);
			double k = fit.getSlope();
			double d = fit.getIntercept();
			double doorg = (d) / sqrt(k * k + 1);
			//经过ORG_REGION阈值半径小圈，都判定为射线
			if (fabs(doorg) < ORG_REGION) {		

				FunLine funline(k, d);
				delPc<FunLine>(lstPC, funline);

			}
		}
		
	}
	//单点过滤
	static void fltdot1(vector<tsPointCloud>& lstPC)
	{
		//拷贝副本，删除无效点
		auto lstPointCloud = lstPC;
		//删除副本无效点
		delInvalid(lstPointCloud);

		//如果点云只有一个点，删除
		if (lstPointCloud.size() < 2) {
			lstPointCloud.clear();
			return;
		}
		//按角度排序
		sort(lstPointCloud.begin(), lstPointCloud.end(), [](tsPointCloud& a,
			tsPointCloud& b) {
			return a.dAngle < b.dAngle;
		});

		
		//只有两个点，超过阈值删除
		if (lstPointCloud.size() == 2) {
			double dist = culPointDist(lstPointCloud[0], lstPointCloud[1]);
			if (dist > DIST_THRESH) {
				lstPointCloud.clear();
			}
			return;
		}

		//判断pt2左右间隔距离是否大于阈值
		auto culInterval3P = [=](tsPointCloud& pt1, tsPointCloud& pt2, tsPointCloud& pt3) {
			double l = culPointDist(pt1, pt2);
			double r = culPointDist(pt2, pt3);
			return l > DIST_THRESH && r > DIST_THRESH;
		};
		
		//判断以第一个点为中心，与最后一个点，第二个点间隔是否大于阈值
		vector<tsPointCloud> singlePt;
		if (culInterval3P(lstPointCloud.back(), lstPointCloud.front(), lstPointCloud[1])) {
			singlePt.push_back(lstPointCloud.front());
		}
		//判断其他点是否左右间隔大于阈值
		auto it = next(lstPointCloud.begin());
		for (; next(it) != lstPointCloud.end(); ++it) {
			if (culInterval3P(*prev(it), *it, *next(it))) {
				singlePt.push_back(*it);
			}
		}
		//判断最后一个点与左右是否大于阈值
		if (culInterval3P(lstPointCloud[lstPointCloud.size() - 2], lstPointCloud.back(), lstPointCloud.front())) {
			singlePt.push_back(lstPointCloud.back());
		}

		FunDot<vector<tsPointCloud>> fundot(singlePt);
		delPc<FunDot<vector<tsPointCloud>>>(lstPC, fundot); //删除单点或者设置单点无效
		
	}

	//删除双点
	static void fltdot2(vector<tsPointCloud>& lstPC)
	{
		//拷贝副本
		auto lstPointCloud = lstPC;
		//删除副本无效点
		delInvalid(lstPointCloud);

		if (lstPointCloud.empty()) {
			return ;
		}
		//按角度排序
		sort(lstPointCloud.begin(), lstPointCloud.end(), [](tsPointCloud &a,
			tsPointCloud& b) {
			return a.dAngle < b.dAngle;
		});

		//计算间隔距离
		vector<unsigned short> adjacent_dist(1, 0);
		for (auto it = lstPointCloud.begin(); next(it) != lstPointCloud.end(); ++it) {
			adjacent_dist.push_back(culPointDist(*it, *next(it)));
		}
		//大于阈值分段
		vector<list<tsPointCloud>> segmentList;
		const size_t bufSize = adjacent_dist.size();
		list<tsPointCloud> ad_list(1, lstPointCloud.front());
		for (size_t i = 1; i < bufSize; ++i) {
			if (adjacent_dist[i] < DIST_THRESH) {
				ad_list.push_back(lstPointCloud[i]);
			}
			else {
				segmentList.push_back(ad_list);
				ad_list.clear();
				ad_list.push_back(lstPointCloud[i]);
			}
		}
		if (!ad_list.empty()) {
			segmentList.push_back(ad_list);
		}

		//删除长度不为2的段
		auto delpos = remove_if(segmentList.begin(), segmentList.end(), [](decltype(segmentList)::value_type& a) {
			return a.size() != 2;
		});
		segmentList.erase(delpos, segmentList.end());

		//拼接满足条件段
		list<tsPointCloud> mergeList;
		for (auto& it : segmentList) {
			mergeList.splice(mergeList.end(), it);
		}

		FunDot<list<tsPointCloud>> fundot(mergeList);
		delPc<FunDot<list<tsPointCloud>>>(lstPC, fundot); //删除双点或者设置无效点
	}

private:
	static void delInvalid(vector<tsPointCloud>& lstPointCloud)
	{
		auto delpos = remove_if(lstPointCloud.begin(), lstPointCloud.end(), [](tsPointCloud& a) {
			return a.u16Dist < 1 || !a.bValid;
		});
		lstPointCloud.erase(delpos, lstPointCloud.end());
	}
	static pair<double, double> culPoint(tsPointCloud& pc)
	{
		double angle_rad_cur = pc.dAngle * CV_PI / 180;
		double x = pc.u16Dist * cos(angle_rad_cur);
		double y = pc.u16Dist * sin(angle_rad_cur);
		pair<double, double> pt = { x, y };
		return pt;
	};

	static double culPointDist (tsPointCloud& p1, tsPointCloud& p2)
	{
		auto pt1 = culPoint(p1);
		auto pt2 = culPoint(p2);
		double dist = sqrt((pt1.first - pt2.first) * (pt1.first - pt2.first) +
			(pt1.second - pt2.second) * (pt1.second - pt2.second));
		return dist;
	};
};

template<typename T>
class Filter
{
public:
	Filter() = default;
    
	void operator()(vector<tsPointCloud>& lstPointCloud)
	{
		T::filter(lstPointCloud);
	}
};



#endif // FLTLINE_H
