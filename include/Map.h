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


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h" 
#include "InitKeyFrame.h"
#include "SystemSetting.h"
#include "Converter.h"

#include <set>
#include <list> // new
#include <vector> // new
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class MapLine; 
class KeyFrame;
class Atlas;
class KeyFrameDatabase;
class SystemSetting;
class InitKeyFrame;
class GeometricCamera; 

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;

        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnLastLoopKFid;
    }

public:
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);
    void EraseKeyFrame(KeyFrame* pKF);
    //---MapPoint---
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    //---MapLine---
    void AddMapLine(MapLine* pML);
    void EraseMapLine(MapLine* pML);
    void SetReferenceMapLines(const std::vector<MapLine*> &vpMLs);

    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    //---MapPoint---
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    //---MapLine---
    std::vector<MapLine*> GetAllMapLines();
    std::vector<MapLine*> GetReferenceMapLines();

    long unsigned int MapPointsInMap();
    long unsigned int MapLinesInMap(); //new
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void RotateMap(const cv::Mat &R);
    void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel=false, const cv::Mat t=cv::Mat::zeros(cv::Size(1,3),CV_32F));

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, LineVocabulary* pLineVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    std::mutex mMutexLineCreation; //new

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

protected:

    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<MapPoint*> mspMapPoints_each;
    std::set<MapLine*> mspMapLines;
    std::set<KeyFrame*> mspKeyFrames;

    std::map<unsigned long int, MapPoint*> mmpnIdx2MapPoints;
	std::map<unsigned long int, MapLine*> mmpnIdx2MapLines;
    std::map<unsigned long int, unsigned long int>mNewId2OldId;

    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<MapLine*> mvpBackupMapLines;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    std::vector<MapLine*> mvpReferenceMapLines;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    std::mutex mMutexMap;

    void SaveMapPoint(ofstream& f, MapPoint* mp);
    MapPoint* LoadMapPoint(ifstream& f);
    void SaveMapLine(ofstream& f, MapLine* ml);
    MapLine* LoadMapLine(ifstream& f);
    void SaveKeyFrame(ofstream& f, KeyFrame* kf);
    KeyFrame* LoadKeyFrame(ifstream& f, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB);
    KeyFrame* LoadKeyFrame( ifstream &f, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB, Eigen::Isometry3d transforms, int &MapPointCount, int &KeyFrameCount, int &FrameCount, int &FrameNow);
    void GetMapPointsIdx();
};

} //namespace ORB_SLAM3

#endif // MAP_H
