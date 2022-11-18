
#ifndef ROBOCALC_FUNCTIONS_H_
#define ROBOCALC_FUNCTIONS_H_

#include "DataTypes.h"
#include <vector>
#include <set>

namespace robocalc
{
	namespace functions
	{
		 scalarMul(double s,  v);
		 targetForces( current,  target, double targetTotalForce);
		double pow(double x, int i);
		 jointPosition( xs, int i);
		 scalarMul(double s,  v);
		double sqrt(double x);
		double norm( v);
		double norm( v);
		double dist( v1,  v2);
		double dist( v1,  v2);
		 targetForces( current,  target, double targetTotalForce);
		double sqrt(double x);
		double pow(double x, int i);
		 jointPosition( xs, int i);
		
		template<typename T>
		std::set<T> set_union(std::set<T> s1, std::set<T> s2)
		{
			std::set<T> ret;
			ret.insert(s1.begin(), s1.end());
			ret.insert(s2.begin(), s2.end());
			return ret;
		}
		
		template<typename T>
		unsigned int size(std::set<T> s)
		{
			return s.size();
		}
	}
}

#endif
