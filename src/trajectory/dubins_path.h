// Copyright (c) 2008-2014, Zetian Zhang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DUBINS_H
#define DUBINS_H

#include <vector>

// Error codes
#define EDUBOK        (0)   // No error
#define EDUBCOCONFIGS (1)   // Colocated configurations
#define EDUBPARAM     (2)   // Path parameterisitation error
#define EDUBBADRHO    (3)   // the rho value is invalid
#define EDUBNOPATH    (4)   // no connection between configurations with this word

typedef struct{
	// the initial configuration
	std::vector<double> qi = std::vector<double>(3);     
	// the lengths of the three segments
	std::vector<double> param = std::vector<double>(3);
	// model forward velocity / model angular velocity
	double rho;     
	// path type. one of LSL, LSR, ...     
    int type;           
}DubinsPathInfo;

namespace DubinsPath{
	typedef struct{
		// a sequence of configurations(x, y, theta) along the path
		std::vector<std::vector<double>> traj_point_wise;
		// length map of the sequence of configurations
		std::vector<double> traj_len_map;
		// the length of the steering path
		double traj_length;
	}PathData;

	/**
	 * Calculate the length of an initialised path
	 *
	 * @param q0 - the initial configuration
	 * @param q1 - the end configuration
	 * @param min_radius - minimum turning radius
	 */
	double GetDubinsPathLength(std::vector<double> q0, std::vector<double> q1, double min_radius);

	/**
	 * Calculate a sequence of configurations from initial to end
	 *
	 * @param q0 - the initial configuration
	 * @param q1 - the end configuration
	 * @param min_radius - minimum turning radius
	 * @param step - a length measure, where 0 < step < length of the path
	 */
	PathData GetDubinsPathPointWise(std::vector<double> q0, std::vector<double> q1, double min_radius, double step);
};

#endif // DUBINS_H


