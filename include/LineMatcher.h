/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include "gridStructure.h"
#include "Frame.h"
#include "MapPoint.h"
#include "MapLine.h"

using namespace line_descriptor;

namespace ORB_SLAM3 {

class Frame;
class MapPoint;
class MapLine;

typedef std::pair<int, int> point_2d;
typedef std::pair<point_2d, point_2d> line_2d;

inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
    return (a.first*b.first + a.second*b.second);
}

inline void normalize(std::pair<double, double> &v) {
    double magnitude = std::sqrt(dot(v, v));

    v.first /= magnitude;
    v.second /= magnitude;
}

int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

int match(const std::vector<MapLine*> &mvpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12);

int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

int distance(const cv::Mat &a, const cv::Mat &b);

//Points
int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12);

//Lines
int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

class Linematcher
    {
        public:
        Linematcher(float nnratio=0.7, bool checkOri=true);
        int SearchDouble(Frame &InitialFrame, Frame &CurrentFrame, vector<int> &LineMatches);

        public:

        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;

        protected:
        void FrameBFMatch(cv::Mat ldesc1, cv::Mat ldesc2, vector<int>& LineMatches, float TH);
        void lineDescriptorMAD(vector<vector<DMatch>> line_matches, double &nn_mad, double &nn12_mad) const;
    
        float mfNNratio;
        bool mbCheckOrientation;
    };
} // namesapce ORB_SLAM2
