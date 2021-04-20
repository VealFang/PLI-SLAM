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

#include "LineMatcher.h"

//STL
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "gridStructure.h"

#include "Config.h"

#define PI 3.1415926

namespace ORB_SLAM3 {

    const int Linematcher::TH_HIGH = 80;
    const int Linematcher::TH_LOW = 50;
    const int Linematcher::HISTO_LENGTH = 30;

    Linematcher::Linematcher(float nnratio, bool checkOri):mfNNratio(nnratio), mbCheckOrientation(checkOri)
    {
    }

    void Linematcher::FrameBFMatch(cv::Mat ldesc1, cv::Mat ldesc2, vector<int>& LineMatches, float TH)
    {
        LineMatches = vector<int>(ldesc1.rows,-1);
        
        vector<vector<DMatch>> lmatches;

        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        double nn_dist_th, nn12_dist_th;
        lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);

        nn12_dist_th = nn12_dist_th*0.5;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());

        for(int i=0; i<lmatches.size(); i++)
        {
            int qdx = lmatches[i][0].queryIdx;
            int tdx = lmatches[i][0].trainIdx;

            double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
            if(dist_12>nn12_dist_th && lmatches[i][0].distance < TH && lmatches[i][0].distance < mfNNratio * lmatches[i][1].distance)
                LineMatches[qdx] = tdx;
        }
    }

    void Linematcher::lineDescriptorMAD(vector<vector<DMatch>> line_matches, double &nn_mad, double &nn12_mad) const
    {
        vector<vector<DMatch>> matches_nn, matches_12;
        matches_nn = line_matches;
        matches_12 = line_matches;

        // estimate the NN's distance standard deviation
        double nn_dist_median;
        sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
        nn_dist_median = matches_nn[int(matches_nn.size()/2)][0].distance;

        for(unsigned int i=0; i<matches_nn.size(); i++)
            matches_nn[i][0].distance = fabsf(matches_nn[i][0].distance - nn_dist_median);
        sort(matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
        nn_mad = 1.4826 * matches_nn[int(matches_nn.size()/2)][0].distance;

        // estimate the NN's 12 distance standard deviation
        double nn12_dist_median;
        sort( matches_12.begin(), matches_12.end(), compare_descriptor_by_NN12_dist()); //NN12_dist?
        nn12_dist_median = matches_12[int(matches_12.size()/2)][1].distance - matches_12[int(matches_12.size()/2)][0].distance;
        for (unsigned int j=0; j<matches_12.size(); j++)
            matches_12[j][0].distance = fabsf( matches_12[j][1].distance - matches_12[j][0].distance - nn12_dist_median);
        sort(matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist());
        nn12_mad = 1.4826 * matches_12[int(matches_12.size()/2)][0].distance;
    }

    int Linematcher::SearchDouble(Frame &InitialFrame, Frame &CurrentFrame, vector<int> &LineMatches)
    {
        LineMatches = vector<int>(InitialFrame.N_l,-1);
        vector<int> tempMatches1, tempMatches2;
    
        Mat ldesc1, ldesc2;
        ldesc1 = InitialFrame.mDescriptors_Line;
        ldesc2 = CurrentFrame.mDescriptors_Line;
        if(ldesc1.rows == 0 || ldesc2.rows == 0)
            return 0;
        
        int nmatches = 0;

        thread thread12(&Linematcher::FrameBFMatch, this, ldesc1, ldesc2, std :: ref(tempMatches1), TH_LOW);
        thread thread21(&Linematcher::FrameBFMatch, this, ldesc2, ldesc1, std :: ref(tempMatches2), TH_LOW);
        thread12.join();
        thread21.join();

        for(int i = 0; i<tempMatches1.size(); i++)
        {
            int j= tempMatches1[i];
            if(j>=0){
                if(tempMatches2[j] != i){
                    tempMatches1[i] = -1;
                }else{
                    nmatches++;
                }
            }
        }

        LineMatches = tempMatches1;

        return nmatches;
    }

int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    std::vector<std::vector<cv::DMatch>> matches_;
    cv::Ptr<cv::BFMatcher> bfm = cv::BFMatcher::create(cv::NORM_HAMMING, false); // cross-check
    bfm->knnMatch(desc1, desc2, matches_, 2);

    if (desc1.rows != matches_.size())
        throw std::runtime_error("[matchNNR] Different size for matches and descriptors!");

    for (int idx = 0, nsize = desc1.rows; idx < nsize; ++idx) {
        if (matches_[idx][0].distance < matches_[idx][1].distance * nnr) {
            matches_12[idx] = matches_[idx][0].trainIdx;
            matches++;
        }
    }

    return matches;
}

int match(const std::vector<MapLine*> &vpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12)
{
    cv::Mat desc1;
    desc1.reserve(vpLocalMapLines.size());
    for(int i=0,z=vpLocalMapLines.size(); i<z; ++i)
        desc1.push_back(vpLocalMapLines[i]->GetDescriptor());
    cv::Mat desc2;
    CurrentFrame.mDescriptors_Line.copyTo(desc2);

    return matchNNR(desc1, desc2, nnr, matches_12);


    int matches;
    if (Config::bestLRMatches()) {
        std::vector<int> matches_21;
        if (Config::lrInParallel()) {
            auto match_12 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc1), std::cref(desc2), nnr, std::ref(matches_12));
            auto match_21 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc2), std::cref(desc1), nnr, std::ref(matches_21));
            matches = match_12.get();
            match_21.wait();
        } else {
            matches = matchNNR(desc1, desc2, nnr, matches_12);
            matchNNR(desc2, desc1, nnr, matches_21);
        }

        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }
    else matches = matchNNR(desc1, desc2, nnr, matches_12);

    return matches;
}

int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    if (Config::bestLRMatches()) {
        int matches;
        std::vector<int> matches_21;
        if (Config::lrInParallel()) {
            auto match_12 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc1), std::cref(desc2), nnr, std::ref(matches_12));
            auto match_21 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc2), std::cref(desc1), nnr, std::ref(matches_21));
            matches = match_12.get();
            match_21.wait();
        } else {
            matches = matchNNR(desc1, desc2, nnr, matches_12);
            matchNNR(desc2, desc1, nnr, matches_21);
        }

        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }

        return matches;
    } else
        return matchNNR(desc1, desc2, nnr, matches_12);
}

int distance(const cv::Mat &a, const cv::Mat &b) {

    // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;
    for(int i = 0; i < 8; i++, pa++, pb++) {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12) {

    if (points1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each point needs a corresponding descriptor!");

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    int best_d, best_d2, best_idx;
    std::vector<int> matches_21, distances;

    if (Config::bestLRMatches()) {
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0, nsize = points1.size(); i1 < nsize; ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const std::pair<int, int> &coords = points1[i1];
        cv::Mat desc = desc1.row(i1);

        std::unordered_set<int> candidates;
        grid.get(coords.first, coords.second, w, candidates);

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {
            if (i2 < 0 || i2 >= desc2.rows) continue;

            const int d = distance(desc, desc2.row(i2));

            if (Config::bestLRMatches()) {
                if (d < distances[i2]) {
                    distances[i2] = d;
                    matches_21[i2] = i1;
                } else continue;
            }

            if (d < best_d) {
                best_d2 = best_d;
                best_d = d;
                best_idx = i2;
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * Config::minRatio12P()) {
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (Config::bestLRMatches()) {
        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1,
              const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2,
              const GridWindow &w,
              std::vector<int> &matches_12) {

    if (lines1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each line needs a corresponding descriptor!");

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    int best_d, best_d2, best_idx;
    std::vector<int> matches_21, distances;

    if (Config::bestLRMatches()) {
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0, nsize = lines1.size(); i1 < nsize; ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const line_2d &coords = lines1[i1];
        cv::Mat desc = desc1.row(i1);

        const point_2d sp = coords.first;
        const point_2d ep = coords.second;

        std::pair<double, double> v = std::make_pair(ep.first - sp.first, ep.second - sp.second);
        normalize(v);

        std::unordered_set<int> candidates;
        grid.get(sp.first, sp.second, w, candidates);
        grid.get(ep.first, ep.second, w, candidates);

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {
            if (i2 < 0 || i2 >= desc2.rows) continue;

            if (std::abs(dot(v, directions2[i2])) < Config::lineSimTh())
                continue;

            const int d = distance(desc, desc2.row(i2));

            if (Config::bestLRMatches()) {
                if (d < distances[i2]) {
                    distances[i2] = d;
                    matches_21[i2] = i1;
                } else continue;
            }

            if (d < best_d) {
                best_d2 = best_d;
                best_d = d;
                best_idx = i2;
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * Config::minRatio12L()) {
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (Config::bestLRMatches()) {
        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

} //namesapce ORB_SLAM2
