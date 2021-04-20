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


#include "Map.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),mnLastLoopKFid(initKFid), mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddMapLine(MapLine* pML)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLines.insert(pML); 
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseMapLine(MapLine *pML)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLines.erase(pML);
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::SetReferenceMapLines(const vector<MapLine *> &vpMLs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapLines = vpMLs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapLine*> Map::GetAllMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapLine*>(mspMapLines.begin(),mspMapLines.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::MapLinesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapLines.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

vector<MapLine*> Map::GetReferenceMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapLines;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

//--------------NEW-------------
void Map::SaveMapPoint(ofstream& f, MapPoint* mp)
{
	f.write((char*)&mp->mnId, sizeof(mp->mnId));
	f.write((char*)&mp->mnFirstKFid, sizeof(mp->mnFirstKFid));
	f.write((char*)&mp->mnFirstFrame, sizeof(mp->mnFirstFrame));
	f.write((char*)&mp->nObs, sizeof(mp->nObs));

    cv::Mat mpWorldPos = mp->GetWorldPos();
    f.write((char*)& mpWorldPos.at<float>(0), sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(1), sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(2), sizeof(float));
}

MapPoint* Map::LoadMapPoint(ifstream& f)
{
    long unsigned int id;
    long int nFirstKFid, nFirstFrame;
    int obs;
    f.read((char*)&id, sizeof(long unsigned int));
    f.read((char*)&nFirstKFid, sizeof(long int));
    f.read((char*)&nFirstFrame, sizeof(long int));
    f.read((char*)&obs, sizeof(int));

    cv::Mat Position(3,1,CV_32F);
    f.read((char*)&Position.at<float>(0), sizeof(float));
    f.read((char*)&Position.at<float>(1), sizeof(float));
    f.read((char*)&Position.at<float>(2), sizeof(float));

    //init a mappoint and set id & position
    MapPoint *mp = new MapPoint(Position, this);
    mp->mnId = id;
    mp->mnFirstKFid = nFirstKFid;
    mp->mnFirstFrame = nFirstFrame;
    mp->nObs = obs;

    return mp;
}

void Map::SaveMapLine(ofstream& f, MapLine* ml)
{
	f.write((char*)&ml->mnId, sizeof(ml->mnId));
	f.write((char*)&ml->mnFirstKFid, sizeof(ml->mnFirstKFid));
	f.write((char*)&ml->mnFirstFrame, sizeof(ml->mnFirstFrame));
	f.write((char*)&ml->nObs, sizeof(ml->nObs));

    Vector6d mpWorldPos = ml->GetWorldPos();
    f.write((char*)& mpWorldPos(0), sizeof(double));
    f.write((char*)& mpWorldPos(1), sizeof(double));
    f.write((char*)& mpWorldPos(2), sizeof(double));
    f.write((char*)& mpWorldPos(3), sizeof(double));
    f.write((char*)& mpWorldPos(4), sizeof(double));
    f.write((char*)& mpWorldPos(5), sizeof(double));
}

MapLine* Map::LoadMapLine(ifstream& f)
{
    long unsigned int id;
    long int nFirstKFid, nFirstFrame;
    int obs;
    f.read((char*)&id, sizeof(long unsigned int));
    f.read((char*)&nFirstKFid, sizeof(long int));
    f.read((char*)&nFirstFrame, sizeof(long int));
    f.read((char*)&obs, sizeof(int));

    Vector3d sp, ep;
    f.read((char*)&sp(0), sizeof(double));
    f.read((char*)&sp(1), sizeof(double));
    f.read((char*)&sp(2), sizeof(double));
    f.read((char*)&ep(0), sizeof(double));
    f.read((char*)&ep(1), sizeof(double));
    f.read((char*)&ep(2), sizeof(double));

    //init a mapline and set id & position
    MapLine *ml = new MapLine(sp, ep, this);
    ml->mnId = id;
    ml->mnFirstKFid = nFirstKFid;
    ml->mnFirstFrame = nFirstFrame;
    ml->nObs = obs;

    return ml;
}

void Map::SaveKeyFrame(ofstream& f, KeyFrame* kf)
{
    f.write((char*)&kf->mnFrameId, sizeof(kf->mnFrameId));
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));

    cv::Mat Tcw = kf->GetPose();
    std::vector<float> Quat = Converter::toQuaternion(Tcw);

    for(int i = 0; i < 3; ++i)
        f.write((char*)&Tcw.at<float>(i,3), sizeof(float));

    for(int i = 0; i < 4; i++)
        f.write((char*)&Quat[i], sizeof(float));

    // write KeyPoints&MapPoints
    f.write((char*)&kf->N, sizeof(kf->N));
    for(int i = 0; i < kf->N; i++)
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.size, sizeof(kp.size));
        f.write((char*)&kp.angle, sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave, sizeof(kp.octave));

        float uRight = kf->mvuRight[i];
        f.write((char*)&uRight, sizeof(uRight));
        float depth = kf->mvDepth[i];
        f.write((char*)&depth, sizeof(depth));

        for(int j = 0;j < kf->mDescriptors.cols; ++j)
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j),sizeof(char));

        unsigned long int mnIdx;
        MapPoint* mp = kf->GetMapPoint(i);
        if(mp == NULL)
            mnIdx = ULONG_MAX;
        else
            mnIdx = mp->mnId;

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }

#ifdef HasLine
    // write KeyLines&MapLines
    f.write((char*)&kf->N_l, sizeof(kf->N_l));
    for(int i = 0; i < kf->N_l; i++)
    {
        cv::line_descriptor::KeyLine kl = kf->mvKeys_Line[i];
        f.write((char*)&kl.angle, sizeof(kl.angle));
        f.write((char*)&kl.class_id, sizeof(kl.class_id));
        f.write((char*)&kl.octave, sizeof(kl.octave));
        f.write((char*)&kl.pt.x, sizeof(kl.pt.x));
        f.write((char*)&kl.pt.y, sizeof(kl.pt.y));
        f.write((char*)&kl.response, sizeof(kl.response));
        f.write((char*)&kl.size, sizeof(kl.size));
        f.write((char*)&kl.startPointX, sizeof(kl.startPointX));
        f.write((char*)&kl.startPointY, sizeof(kl.startPointY));
        f.write((char*)&kl.endPointX, sizeof(kl.endPointX));
        f.write((char*)&kl.endPointY, sizeof(kl.endPointY));
        f.write((char*)&kl.sPointInOctaveX, sizeof(kl.sPointInOctaveX));
        f.write((char*)&kl.sPointInOctaveY, sizeof(kl.sPointInOctaveY));
        f.write((char*)&kl.ePointInOctaveX, sizeof(kl.ePointInOctaveX));
        f.write((char*)&kl.ePointInOctaveY, sizeof(kl.ePointInOctaveY));
        f.write((char*)&kl.lineLength, sizeof(kl.lineLength));
        f.write((char*)&kl.numOfPixels, sizeof(kl.numOfPixels));

        pair<float,float> disparity_l = kf->mvDisparity_l[i];
        f.write((char*)&disparity_l.first, sizeof(disparity_l.first));
        f.write((char*)&disparity_l.second, sizeof(disparity_l.second));

        Vector3d le_l = kf->mvle_l[i];
        f.write((char*)&le_l(0), sizeof(double));
        f.write((char*)&le_l(1), sizeof(double));
        f.write((char*)&le_l(2), sizeof(double));

        for(int j = 0;j < kf->mDescriptors_l.cols; ++j)
            f.write((char*)&kf->mDescriptors_l.at<unsigned char>(i,j),sizeof(char));

        unsigned long int mnIdx;
        MapLine* ml = kf->GetMapLine(i);
        if(ml == NULL)
            mnIdx = ULONG_MAX;
        else
            mnIdx = ml->mnId;

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }
#endif
}

KeyFrame* Map::LoadKeyFrame(ifstream& f, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB)
{
    //init keyframe
    InitKeyFrame initkf(*mySystemSetting);
    //read keyframe id & timestamp
    f.read((char*)&initkf.mnFrameId, sizeof(initkf.mnFrameId));
    f.read((char*)&initkf.nId, sizeof(initkf.nId));
    f.read((char*)&initkf.TimeStamp, sizeof(double));

    //read keyframe pos&ori
    cv::Mat Tcw(4,4,CV_32F);
    for( int i = 0; i < 4; ++i)
        for( int j = 0; j < 4; ++j)
            Tcw.at<float>(i,j) = 0;

    for( int i = 0; i < 3; ++i)
       f.read((char*)&Tcw.at<float>(i,3), sizeof(float));
    Tcw.at<float>(3,3) = 1;
    cv::Mat Qcw(1, 4, CV_32F);
    for( int i = 0; i < 4; ++i)
       f.read((char*)&Qcw.at<float>(0,i), sizeof(float));

    Converter::RmatOfQuat(Tcw, Qcw);

    std::vector<MapPoint*> vpMapPoints;
    std::vector<MapLine*> vpMapLines;

    f.read((char*)&initkf.N, sizeof(initkf.N));
    vpMapPoints = vector<MapPoint*>(initkf.N, static_cast<MapPoint*>(NULL));
    initkf.vKps.reserve(initkf.N);
    initkf.Descriptors.create(initkf.N, 32, CV_8UC1);

    //read this keyframe keypoint & descriptor
    for(int i = 0; i < initkf.N; ++i )
    {
        cv::KeyPoint kp;
        f.read((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.read((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.read((char*)&kp.size, sizeof(kp.size));
        f.read((char*)&kp.angle, sizeof(kp.angle));
        f.read((char*)&kp.response, sizeof(kp.response));
        f.read((char*)&kp.octave, sizeof(kp.octave));
        initkf.vKps.push_back(kp);

        float uRight;
        f.read((char*)&uRight, sizeof(uRight));
        initkf.vRight.push_back(uRight);

        float depth;
        f.read((char*)&depth, sizeof(depth));
        initkf.vDepth.push_back(depth);

        for( int j = 0; j < 32; ++j)
            f.read((char*)&initkf.Descriptors.at<unsigned char>(i,j), sizeof(char));

        //read this keypoint's relationship with mappoint
        unsigned long int mpidx;
        f.read((char*)&mpidx, sizeof(mpidx));

        //read all mappoints & find this keyframe mappoint
        if( mpidx == ULONG_MAX )
            vpMapPoints[i] = NULL;
        else
            vpMapPoints[i] = mmpnIdx2MapPoints[mpidx];
    }

    initkf.UndistortKeyPoints();
    initkf.AssignFeaturesToGrid();

#ifdef HasLine
    //read keyline number
    f.read((char*)&initkf.N_l, sizeof(initkf.N_l));
    vpMapLines = vector<MapLine*>(initkf.N_l, static_cast<MapLine*>(NULL));
    initkf.vKeys_Line.reserve(initkf.N_l);
    initkf.Descriptors_Line.create(initkf.N_l, 32, CV_8UC1);

    //read this keyframe keypoint & descriptor
    for(int i = 0; i < initkf.N_l; ++i )
    {
        cv::line_descriptor::KeyLine kl;
        f.read((char*)&kl.angle, sizeof(kl.angle));
        f.read((char*)&kl.class_id, sizeof(kl.class_id));
        f.read((char*)&kl.octave, sizeof(kl.octave));
        f.read((char*)&kl.pt.x, sizeof(kl.pt.x));
        f.read((char*)&kl.pt.y, sizeof(kl.pt.y));
        f.read((char*)&kl.response, sizeof(kl.response));
        f.read((char*)&kl.size, sizeof(kl.size));
        f.read((char*)&kl.startPointX, sizeof(kl.startPointX));
        f.read((char*)&kl.startPointY, sizeof(kl.startPointY));
        f.read((char*)&kl.endPointX, sizeof(kl.endPointX));
        f.read((char*)&kl.endPointY, sizeof(kl.endPointY));
        f.read((char*)&kl.sPointInOctaveX, sizeof(kl.sPointInOctaveX));
        f.read((char*)&kl.sPointInOctaveY, sizeof(kl.sPointInOctaveY));
        f.read((char*)&kl.ePointInOctaveX, sizeof(kl.ePointInOctaveX));
        f.read((char*)&kl.ePointInOctaveY, sizeof(kl.ePointInOctaveY));
        f.read((char*)&kl.lineLength, sizeof(kl.lineLength));
        f.read((char*)&kl.numOfPixels, sizeof(kl.numOfPixels));        
        initkf.vKeys_Line.push_back(kl);

        pair<float,float> disparity_l;
        f.read((char*)&disparity_l.first, sizeof(disparity_l.first));
        f.read((char*)&disparity_l.second, sizeof(disparity_l.second));
        initkf.vDisparity_l.push_back(disparity_l);

        Vector3d le_l;
        f.read((char*)&le_l(0), sizeof(double));
        f.read((char*)&le_l(1), sizeof(double));
        f.read((char*)&le_l(2), sizeof(double));
        initkf.vle_l.push_back(le_l);

        for( int j = 0; j < 32; ++j)
            f.read((char*)&initkf.Descriptors_Line.at<unsigned char>(i,j), sizeof(char));

        //read this keypoint's relationship with mappoint
        unsigned long int mpidx;
        f.read((char*)&mpidx, sizeof(mpidx));

        //read all mappoints & find this keyframe mappoint
        if( mpidx == ULONG_MAX )
            vpMapLines[i] = NULL;
        else
            vpMapLines[i] = mmpnIdx2MapLines[mpidx];
    }
    initkf.UndistortKeyLines();
#endif

    //use initkf to initialize
    KeyFrame* kf = new KeyFrame(initkf, this, KFDB, vpMapPoints, vpMapLines);
    kf->mnId = initkf.nId;
    kf->SetPose(Tcw);
    kf->ComputeBoW();
    for( int i = 0; i < initkf.N; ++i)
    {
       if( vpMapPoints[i] )
       {
           vpMapPoints[i]->AddObservation(kf,i);
           if( !vpMapPoints[i]->GetReferenceKeyFrame())
               vpMapPoints[i]->SetReferenceKeyFrame(kf);
       }
    }
#ifdef HasLine
    for( int i = 0; i < initkf.N_l; ++i)
    {
       if( vpMapLines[i] )
       {
           vpMapLines[i]->AddObservation(kf,i);
           if( !vpMapLines[i]->GetReferenceKeyFrame())
               vpMapLines[i]->SetReferenceKeyFrame(kf);
       }
    }
#endif
    if( KFDB != NULL )
        KFDB->add(kf);

    return kf;
}
//? Map::Load&Save

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<MapLine*>::iterator sit=mspMapLines.begin(), send=mspMapLines.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mnLastLoopKFid = 0;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}

void Map::RotateMap(const cv::Mat &R)
{
    unique_lock<mutex> lock(mMutexMap);

    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    KeyFrame* pKFini = mvpKeyFrameOrigins[0];
    cv::Mat Twc_0 = pKFini->GetPoseInverse();
    cv::Mat Txc_0 = Txw*Twc_0;
    cv::Mat Txb_0 = Txc_0*pKFini->mImuCalib.Tcb;
    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);
    Tyx.rowRange(0,3).col(3) = -Txb_0.rowRange(0,3).col(3);
    cv::Mat Tyw = Tyx*Txw;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy);
        cv::Mat Vw = pKF->GetVelocity();
        pKF->SetVelocity(Ryw*Vw);
    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(Ryw*pMP->GetWorldPos()+tyw);
        pMP->UpdateNormalAndDepth();
    }
}

void Map::ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel, const cv::Mat t)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    cv::Mat Txw = cv::Mat::eye(4,4,CV_32F);
    R.copyTo(Txw.rowRange(0,3).colRange(0,3));

    cv::Mat Tyx = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Tyw = Tyx*Txw;
    Tyw.rowRange(0,3).col(3) = Tyw.rowRange(0,3).col(3)+t;
    cv::Mat Ryw = Tyw.rowRange(0,3).colRange(0,3);
    cv::Mat tyw = Tyw.rowRange(0,3).col(3);

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        cv::Mat Twc = pKF->GetPoseInverse();
        Twc.rowRange(0,3).col(3)*=s;
        cv::Mat Tyc = Tyw*Twc;
        cv::Mat Tcy = cv::Mat::eye(4,4,CV_32F);
        Tcy.rowRange(0,3).colRange(0,3) = Tyc.rowRange(0,3).colRange(0,3).t();
        Tcy.rowRange(0,3).col(3) = -Tcy.rowRange(0,3).colRange(0,3)*Tyc.rowRange(0,3).col(3);
        pKF->SetPose(Tcy);
        cv::Mat Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s*Ryw*pMP->GetWorldPos()+tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::PrintEssentialGraph()
{
    //Print the essential graph
    vector<KeyFrame*> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFrame* pFirstKF;
    for(KeyFrame* pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    cout << "KF: " << pFirstKF->mnId << endl;
    set<KeyFrame*> spChilds = pFirstKF->GetChilds();
    vector<KeyFrame*> vpChilds;
    vector<string> vstrHeader;
    for(KeyFrame* pKFi : spChilds){
        vstrHeader.push_back("--");
        vpChilds.push_back(pKFi);
    }
    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        string strHeader = vstrHeader[i];
        KeyFrame* pKFi = vpChilds[i];

        cout << strHeader << "KF: " << pKFi->mnId << endl;

        set<KeyFrame*> spKFiChilds = pKFi->GetChilds();
        for(KeyFrame* pKFj : spKFiChilds)
        {
            vpChilds.push_back(pKFj);
            vstrHeader.push_back(strHeader+"--");
        }
    }
    if (count == (mspKeyFrames.size()+10))
        cout << "CYCLE!!"    << endl;

    cout << "------------------" << endl << "End of the essential graph" << endl;
}

bool Map::CheckEssentialGraph(){
    vector<KeyFrame*> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFrame* pFirstKF;
    for(KeyFrame* pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    cout << "Checking if the first KF has parent" << endl;
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    set<KeyFrame*> spChilds = pFirstKF->GetChilds();
    vector<KeyFrame*> vpChilds;
    vpChilds.reserve(mspKeyFrames.size());
    for(KeyFrame* pKFi : spChilds)
        vpChilds.push_back(pKFi);

    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        KeyFrame* pKFi = vpChilds[i];
        set<KeyFrame*> spKFiChilds = pKFi->GetChilds();
        for(KeyFrame* pKFj : spKFiChilds)
            vpChilds.push_back(pKFj);
    }

    cout << "count/tot" << count << "/" << mspKeyFrames.size() << endl;
    if (count != (mspKeyFrames.size()-1))
        return false;
    else
        return true;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder)
{
    string path_imgs = "./" + name_folder + "/";
    for(KeyFrame* pKFi : lpLocalWindowKFs)
    {
        //cout << "KF " << pKFi->mnId << endl;
        cv::Mat img_i = cv::imread(pKFi->mNameFile, CV_LOAD_IMAGE_UNCHANGED);
        //cout << "Image -> " << img_i.cols << ", " << img_i.rows << endl;
        cv::cvtColor(img_i, img_i, CV_GRAY2BGR);
        //cout << "Change of color in the image " << endl;

        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        int num_points = 0;
        for(int j=0; j<vpMPs.size(); ++j)
        {
            MapPoint* pMPij = vpMPs[j];
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            cv::KeyPoint point_img = pKFi->mvKeysUn[j];
            cv::Point2f reproj_p;
            float u, v;
            bool bIsInImage = pKFi->ProjectPointUnDistort(pMPij, reproj_p, u, v);
            if(bIsInImage){
                //cout << "Reproj in the image" << endl;
                cv::circle(img_i, point_img.pt, 1/*point_img.octave*/, cv::Scalar(0, 255, 0));
                cv::line(img_i, point_img.pt, reproj_p, cv::Scalar(0, 0, 255));
                num_points++;
            }
            else
            {
                //cout << "Reproj out of the image" << endl;
                cv::circle(img_i, point_img.pt, point_img.octave, cv::Scalar(0, 0, 255));
            }

        }
        //cout << "Image painted" << endl;
        string filename_img = path_imgs +  "KF" + to_string(mpCurrentKF->mnId) + "_" + to_string(pKFi->mnId) +  name + "points" + to_string(num_points) + ".png";
        cv::imwrite(filename_img, img_i);
    }

}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, size_t> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, size_t>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this)
            {
                pMPi->EraseObservation(it->first); //We need to find where the KF is set as Bad but the observation is not removed
            }

        }
    }
    cout << "  Bad MapPoints removed" << endl;

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }

    mvpBackupMapPoints.clear();
    // Backup of set container to vector
    //std::copy(mspMapPoints.begin(), mspMapPoints.end(), std::back_inserter(mvpBackupMapPoints));
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Pre-save of mappoint " << pMPi->mnId << endl;
        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }
    cout << "  MapPoints back up done!!" << endl;

    mvpBackupKeyFrames.clear();
    //std::copy(mspKeyFrames.begin(), mspKeyFrames.end(), std::back_inserter(mvpBackupKeyFrames));
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }
    cout << "  KeyFrames back up done!!" << endl;

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, LineVocabulary* pLineVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupMapLines.begin(), mvpBackupMapLines.end(), std::inserter(mspMapLines, mspMapLines.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int,MapLine*> mpMapLineId;
    for(MapLine* pMLi : mspMapLines)
    {
        pMLi->UpdateMap(this);
        mpMapLineId[pMLi->mnId] = pMLi;
    }

    //map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetLineVocabulary(pLineVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }
    cout << "Number of KF: " << mspKeyFrames.size() << endl;
    cout << "Number of MP: " << mspMapPoints.size() << endl;
    cout << "Number of ML: " << mspMapLines.size() << endl;

    // References reconstruction between different instances
    for(MapPoint* pMPi : mspMapPoints)
    {
        //cout << "Post-Load of mappoint " << pMPi->mnId << endl;
        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }
    cout << "End to rebuild MapPoint references" << endl;

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }

    cout << "End to rebuild KeyFrame references" << endl;

    mvpBackupMapPoints.clear();
    mvpBackupMapLines.clear();
}


} //namespace ORB_SLAM3
