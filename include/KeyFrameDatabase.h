/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KDTree.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include<mutex>


namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class Map;

class point_kdtree : public std::array<float, 2>
{
public:
    static const int DIM = 2;       // dimension of space (or "k" of k-d tree)

    point_kdtree() { (*this)[0]=0; (*this)[1]=0; r=0; }
    point_kdtree(float x, float y, float _r=0) { (*this)[0]=x; (*this)[1]=y; r=_r; }
    point_kdtree(const cv::KeyPoint& kp) : point_kdtree(kp.pt, kp.size) { }
    point_kdtree(const cv::Point2f& p, float _r=0) { (*this)[0]=p.x; (*this)[1]=p.y; r=_r; }

    float radius() { return r; }

private:
    float r;
};

class KeyFrameDatabase
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mvBackupInvertedFileId;
    }

public:

    KeyFrameDatabase(const ORBVocabulary &voc);
    KeyFrameDatabase(const ORBVocabulary &voc, const LineVocabulary &voc_l);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();
   void clearMap(Map* pMap);

   // Loop Detection(DEPRECATED)
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);
   std::vector<KeyFrame*> DetectLoopCandidatesWithLine(KeyFrame* pKF, float minScore);

   // Loop and Merge Detection
   void DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand);
   void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
   void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);
   std::vector<KeyFrame*> DetectRelocalizationCandidatesWithLine(Frame* F, Map* pMap);

   void PreSave();
   void PostLoad(map<long unsigned int, KeyFrame*> mpKFid);
   void SetORBVocabulary(ORBVocabulary* pORBVoc);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;
  const LineVocabulary* mpVoc_l;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // For save relation without pointer, this is necessary for save/load function
  std::vector<list<long unsigned int> > mvBackupInvertedFileId;
  std::vector<list<KeyFrame*> > mvInvertedFile_l;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
