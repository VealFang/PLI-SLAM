#ifndef MAPLINE_H
#define MAPLINE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>
#include <map>

#include <eigen3/Eigen/Core>
using namespace Eigen;

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapLine
{
public:
    MapLine();

    MapLine(Vector6d &Pos, KeyFrame *pRefKF, Map *pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, Map* pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, KeyFrame* pRefKF, Map* pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP);
    void SetWorldPos(const Vector6d &Pos);
    Vector6d GetWorldPos();

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,int idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLine* pML);    
    MapLine* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }
    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    Map* GetMap();

    void UpdateAverageDir();
    void UpdateMap(Map* pMap);

    //void UpdateNormalAndDepth();

    KeyFrame* SetReferenceKeyFrame(KeyFrame* RFKF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    static std::mutex mGlobalMutex;

     // Position in absolute coordinates
     Eigen::Vector3d mWorldPos_sP;
     Eigen::Vector3d mWorldPos_eP;

     // Tracking
     bool mbTrackInView;
     float mTrackProjsX;
     float mTrackProjsY;
     float mTrackProjeX;
     float mTrackProjeY;
     double mnTrackangle;
     long unsigned int mnTrackReferenceForFrame;
     long unsigned int mnLastFrameSeen;

     bool mbTrackInViewLine, mbTrackInViewRLine; //new

     // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopLineForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

public:
    // Position in absolute coordinates
    Vector6d mWorldPos;
    Vector3d mStart3D;
    Vector3d mEnd3D;
    Vector3d mWorldVector;
    Vector3d mNormalVector;  //MapPoint中，指的是该MapPoint的平均观测方向，这里指的是观测特征线段的方向

protected:    

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapLine* mpReplaced;

     Map* mpMap;

     std::mutex mMutexMap; //new
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPLINE_H
