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


#ifndef FRAME_H
#define FRAME_H

//#define SAVE_TIMES

#include <vector>
#include <fstream>

#include "MapPoint.h"
#include "MapLine.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "ImuTypes.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "LineExtractor.h"

#include <mutex>
#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>

#include "Config.h"
#include "LineMatcher.h"
#include "gridStructure.h"

#include <eigen3/Eigen/Core>
using namespace Eigen;

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class MapLine;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;

class Frame
{
    typedef unsigned int WordId;
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
        Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight, ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef,
        const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor, Lineextractor* Lineextractor, 
        ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, 
        GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, Lineextractor* Lineextractor, ORBVocabulary* voc,  LineVocabulary* voc_l,
        GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());


    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

    // 自添加 extract line feature
    void ExtractORBandLine(int flag, const cv::Mat &im, const int x0, const int x1);
    void ExtractLine(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose. (Imu pose is not modified!)
    void SetPose(cv::Mat Tcw);
    void GetPose(cv::Mat &Tcw);

    // 自添加 Set Information of prevoius frame for track optimization
    void SetprevInformation(cv::Mat _Tcw, Matrix6d _DT_cov, double _err_norm);

    // Set IMU velocity
    void SetVelocity(const cv::Mat &Vwb);

    // Set IMU pose and velocity (implicitly changes camera pose)
    void SetImuPoseVelocity(const cv::Mat &Rwb, const cv::Mat &twb, const cv::Mat &Vwb);


    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    cv::Mat GetImuPosition();
    cv::Mat GetImuRotation();
    cv::Mat GetImuPose();

    void SetNewBias(const IMU::Bias &b);

    // 自添加 Check if a MapPoint/MapLine is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    bool isInFrustum_l(MapLine* pML, float viewingCosLimit);

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    cv::Mat inRefCoordinates(cv::Mat pCw);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1, const bool bRight = false) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();
    // 自添加 Search a match for each line in the left image to a line in the right image
    void ComputeStereoMatches_Lines(bool initial = true);

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

    // backproject point[u,v] into 3D world coordinate
    Eigen::Vector3d backProjection( const double &u, const double &v, const double &disp );
    // backproject 3D point(camera coodinate) into [u,v] image coordinate
    Eigen::Vector2d projection( const Eigen::Vector3d &P );
    // overlap between obs lineSegment and proj lineSegment
    double lineSegmentOverlap(Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj);

    ConstraintPoseImu* mpcpi;

    bool imuIsPreintegrated();
    void setIntegrated();

    cv::Mat mRwc;
    cv::Mat mOw;
public:

    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;
    LineVocabulary* mpLinevocabulary;

    //自添加 Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    Lineextractor* mpLineextractorLeft, *mpLineextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;
    //自添加 Number of KeyLines.
    int N_l;
    int N_p;

    // Vector of keypoints (original for visualization) and undistorted无畸变的 (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;
    // 自添加 Vector of keylines，特征线的描述子
    std::vector<cv::line_descriptor::KeyLine> mvKeys_Line, mvKeysRight_Line;
    std::vector<cv::line_descriptor::KeyLine> mvKeysUn_Line;
    std::vector<Eigen::Vector3d> mvKeyLineFunctions;    //特征线段所在直线的系数

    // Corresponding stereo coordinate and depth for each keypoint.
    std::vector<MapPoint*> mvpMapPoints;
    std::vector<MapLine*> mvpMapLines; //new

    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // for stereo PL
    std::vector<pair<float,float>> mvDisparity_l;
    std::vector<Vector3d> mvle_l;

    std::vector<Vector3d> mv3DpointInPrevFrame;
    std::vector<pair<Vector3d,Vector3d>> mv3DlineInPrevFrame;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;
    // line
    DBoW2::BowVector mBowVec_l;
    DBoW2::FeatureVector mFeatVec_l;
    // bag of word pairs
    map<WordId,list<WordId>> mwordPairs;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;
    // Line descriptor, each row associated to a keyline.
    cv::Mat mDescriptors_Line, mDescriptorsRight_Line;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    int mnCloseMPs;
    //自添加 和KeyPoint类似，标识特征线段是否属于外线
    std::vector<bool> mvbOutlier_Line;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];


    // Camera pose.
    cv::Mat mTcw;
    cv::Mat mTcw_prev;

    // IMU linear velocity
    cv::Mat mVw;

    cv::Mat mPredRwb, mPredtwb, mPredVwb;
    IMU::Bias mPredBias;

    // IMU bias
    IMU::Bias mImuBias;

    // Imu calibration
    IMU::Calib mImuCalib;

    // Imu preintegration from last keyframe
    IMU::Preintegrated* mpImuPreintegrated;
    KeyFrame* mpLastKeyFrame;

    // Pointer to previous frame
    Frame* mpPrevFrame;
    IMU::Preintegrated* mpImuPreintegratedFrame;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    // grid cell
    double inv_width, inv_height;

    Matrix4d DT;
    Matrix6d DT_cov;
    Vector6d DT_cov_eig;
    double   err_norm;

    int  n_inliers, n_inliers_pt, n_inliers_ls;

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

    string mNameFile;

    int mnDataset;

    double mTimeStereoMatch;
    double mTimeORB_Ext;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();
    // Undistort keylines
    void UndistortKeyLines();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints&keylines to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // PLI
    double lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj);
    void filterLineSegmentDisparity(Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e);

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    //cv::Mat mRwc;
    //cv::Mat mOw;
    //==mtwc

    bool mbImuPreintegrated;

    std::mutex *mpMutexImu;

public:
    GeometricCamera* mpCamera, *mpCamera2;

    //Number of KeyPoints extracted in the left and right images
    int Nleft, Nright;
    int NleftLine, NrightLine; // new
    //Number of Non Lapping Keypoints
    int monoLeft, monoRight;
    int monoLeftLine, monoRightLine; //new

    //For stereo matching
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    //For stereo fisheye matching
    static cv::BFMatcher BFmatcher;

    //Triangulated stereo observations using as reference the left camera. These are
    //computed during ComputeStereoFishEyeMatches
    std::vector<cv::Mat> mvStereo3Dpoints;

    //Grid for the right image
    std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    cv::Mat mTlr, mRlr, mtlr, mTrl;

    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
        ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, cv::Mat& Tlr,
        Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    //Stereo fisheye
    void ComputeStereoFishEyeMatches();

    bool isInFrustumChecks(MapPoint* pMP, float viewingCosLimit, bool bRight = false);

    cv::Mat UnprojectStereoFishEye(const int &i);

    cv::Mat imgLeft, imgRight;
    //cv::Mat mask; //new

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (Nleft != -1) ? Nleft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i] && !mvbOutlier[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
    }
};

}// namespace ORB_SLAM

#endif // FRAME_H
