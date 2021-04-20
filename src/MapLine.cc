#include "MapLine.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int MapLine::nNextId=0;

mutex MapLine::mGlobalMutex;

MapLine::MapLine(Vector6d &Pos, KeyFrame *pRefKF, Map *pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopLineForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos = Pos;
    mWorldVector = Pos.head(3) - Pos.tail(3);
    mWorldVector.normalize();

    mNormalVector << 0, 0, 0;

    //new
    mbTrackInViewRLine = false;
    mbTrackInViewLine = false;

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, Map* pMap):
    mnFirstKFid(-1), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    // mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, KeyFrame* pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0),
    mpRefKF(pRefKF), mnVisible(1), mnFound(1),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    // mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP,  Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;

    pFrame->mDescriptors_Line.row(idxF).copyTo(mDescriptor);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

void MapLine::SetWorldPos(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
}

void MapLine::SetWorldPos(const Vector6d &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos = Pos;
        mWorldVector = Pos.head(3) - Pos.tail(3);
        mWorldVector.normalize();
    }

Vector6d MapLine::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    Vector6d sep;
    sep.head(3) = mWorldPos_sP;
    sep.tail(3) = mWorldPos_eP;
    return sep;
}


KeyFrame* MapLine::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapLine::AddObservation(KeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);

    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    nObs+=2;
}

void MapLine::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            nObs-=2;
            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapLine::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapLine::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapLine::SetBadFlag()
{
    map<KeyFrame*, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*, size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapLineMatch(mit->second);
        }

    mpMap->EraseMapLine(this);
}

MapLine* MapLine::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapLine::Replace(MapLine* pML)
{
    if(pML->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*, size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pML;
    }

    for(map<KeyFrame*, size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pML->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapLineMatch(mit->second, pML);
            pML->AddObservation(pKF,mit->second);
        } 
        else
        {
            pKF->EraseMapLineMatch(mit->second);
        }
    }
    pML->IncreaseFound(nfound);
    pML->IncreaseVisible(nvisible);
    pML->ComputeDistinctiveDescriptors();

    mpMap->EraseMapLine(this);        
}

bool MapLine::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapLine::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapLine::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapLine::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapLine::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*, size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        
        if(!pKF->isBad())
        {
            vDescriptors.push_back(pKF->mDescriptors_l.row(mit->second));
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t NL = vDescriptors.size();

    //float Distances[N][N]
    std::vector<std::vector<float>> Distances;
    Distances.resize(NL, vector<float>(NL, 0));
    for(size_t i=0; i<NL; i++)
    {
        Distances[i][i]=0;
        for(size_t j=0; j<NL; j++)
        {
            int distij = norm(vDescriptors[i], vDescriptors[j], NORM_HAMMING);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0; i<NL; i++)
    {
        vector<int> vDists(Distances[i].begin(), Distances[i].end());
        sort(vDists.begin(), vDists.end());

        //获得中值
        int median = vDists[0.5*(NL-1)];

        //寻找最小的中值
        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapLine::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

/*void MapLine::UpdateAverageDir()
    {
        map<KeyFrame*, tuple<int,int>> observations;
        KeyFrame* pRefKF;
        Vector6d Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if(mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos;
        }

        if(observations.empty())
            return;

        Vector3d normal(0, 0, 0);
        int n=0;
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            Mat Owi = pKF->GetCameraCenter();
            Vector3d OWi(Owi.at<float>(0), Owi.at<float>(1), Owi.at<float>(2));
            Vector3d middlePos = 0.5*(mWorldPos.head(3)+mWorldPos.tail(3));
            Vector3d normali = middlePos - OWi;
            normal = normal + normali/normali.norm();
            n++;
        }

        cv::Mat SP = (Mat_<float>(3,1) << Pos(0), Pos(1), Pos(2));
        cv::Mat EP = (Mat_<float>(3,1) << Pos(3), Pos(4), Pos(5));
        cv::Mat MP = 0.5*(SP+EP);

        cv::Mat CM = MP - pRefKF->GetCameraCenter();
        const float dist = cv::norm(CM);

        tuple<int ,int> indexes = observations[pRefKF];
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        int level;
        if(pRefKF -> NleftLine == -1){
            level = pRefKF->mvKeylines[leftIndex].octave;
        }
        else if(leftIndex != -1){
            level = pRefKF -> mvKeylines[leftIndex].octave;
        }
        else{
            level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
        }
        //const int level = pRefKF->mvKeylines[observations[pRefKF]].octave;
        const float levelScaleFactor = pRefKF->mvScaleFactorsLine[level];
        const int nLevels = pRefKF->mnScaleLevelsLine;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist*levelScaleFactor;
            mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactorsLine[nLevels-1];
            mNormalVector = normal/n;
        }
    }*/

int MapLine::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

Map* MapLine::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

void MapLine::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

bool MapLine::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

KeyFrame* MapLine::SetReferenceKeyFrame(KeyFrame* RFKF)
{
    return mpRefKF = RFKF;
}

} //namespace ORB_SLAM
