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
#include <string>
#include <iostream>
#include<opencv2/core/core.hpp>

class Config
{

public:

    Config();
    ~Config();

    static void loadFromFile( const std::string &config_file );
    static Config& getInstance();

    static bool&    isRelocalizationMode()    { return getInstance().bIsRelocalizationMode; }
    static std::string&  pathLoadMap()      { return getInstance().path_LoadMap; }
    static std::string&  pathSaveMap()      { return getInstance().path_SaveMap; }

    static int&     imgWidth()  { return getInstance().img_width; }
    static int&     imgHeight()       { return getInstance().img_height; }
    static void     setImgWidthHeight(int w, int h)  { getInstance().img_width = w; getInstance().img_height = h;}

    // Keyframe selection parameters (for SLAM, if any)
    static double&  minEntropyRatio()   { return getInstance().min_entropy_ratio; }
    static double&  maxKFTDist()        { return getInstance().max_kf_t_dist; }
    static double&  maxKFRDist()        { return getInstance().max_kf_r_dist; }

    // flags
    static bool&    hasPoints()         { return getInstance().has_points; }
    static bool&    hasLines()          { return getInstance().has_lines; }
    static bool&    useFLDLines()       { return getInstance().use_fld_lines; }
    static bool&    lrInParallel()      { return getInstance().lr_in_parallel; }
    static bool&    plInParallel()      { return getInstance().pl_in_parallel; }
    static bool&    bestLRMatches()     { return getInstance().best_lr_matches; }
    static bool&    adaptativeFAST()    { return getInstance().adaptative_fast; }
    static bool&    useMotionModel()    { return getInstance().use_motion_model; }

    // points detection and matching
    static int&     matchingStrategy()  { return getInstance().matching_strategy; }
    static int&     matchingSWs()       { return getInstance().matching_s_ws; }
    static int&     matchingF2FWs()     { return getInstance().matching_f2f_ws; }

    static int&     orbNFeatures()      { return getInstance().orb_nfeatures; }
    static double&  orbScaleFactor()    { return getInstance().orb_scale_factor; }
    static int&     orbNLevels()        { return getInstance().orb_nlevels; }
    static int&     orbEdgeTh()         { return getInstance().orb_edge_th; }
    static int&     orbWtaK()           { return getInstance().orb_wta_k; }
    static int&     orbScore()          { return getInstance().orb_score; }
    static int&     orbPatchSize()      { return getInstance().orb_patch_size; }
    static int&     orbFastTh()         { return getInstance().orb_fast_th; }
    static int&     fastMinTh()         { return getInstance().fast_min_th; }
    static int&     fastMaxTh()         { return getInstance().fast_max_th; }
    static int&     fastIncTh()         { return getInstance().fast_inc_th; }
    static int&     fastFeatTh()        { return getInstance().fast_feat_th; }
    static double&  fastErrTh()         { return getInstance().fast_err_th; }
    static double&  maxDistEpip()       { return getInstance().max_dist_epip; }
    static double&  minDisp()           { return getInstance().min_disp; }
    static double&  minRatio12P()       { return getInstance().min_ratio_12_p; }

    static double&  rgbdMinDepth()      { return getInstance().rgbd_min_depth; }
    static double&  rgbdMaxDepth()      { return getInstance().rgbd_max_depth; }

    // lines detection and matching
    static int&     lsdNFeatures()      { return getInstance().lsd_nfeatures; }
    static int&     lsdRefine()         { return getInstance().lsd_refine; }
    static double&  lsdScale()          { return getInstance().lsd_scale; }
    static double&  lsdSigmaScale()     { return getInstance().lsd_sigma_scale; }
    static double&  lsdQuant()          { return getInstance().lsd_quant; }
    static double&  lsdAngTh()          { return getInstance().lsd_ang_th; }
    static double&  lsdLogEps()         { return getInstance().lsd_log_eps; }
    static double&  lsdDensityTh()      { return getInstance().lsd_density_th; }
    static int&     lsdNBins()          { return getInstance().lsd_n_bins; }
    static double&  lineHorizTh()       { return getInstance().line_horiz_th; }
    static double&  minLineLength()     { return getInstance().min_line_length; }
    static double&  minRatio12L()       { return getInstance().min_ratio_12_l; }
    static double&  stereoOverlapTh()   { return getInstance().stereo_overlap_th; }
    static double&  f2fOverlapTh()      { return getInstance().f2f_overlap_th; }
    static double&  lineSimTh()         { return getInstance().line_sim_th; }
    static double&  lsMinDispRatio()    { return getInstance().ls_min_disp_ratio; }

    // optimization
    static double&  homogTh()           { return getInstance().homog_th; }
    static int&     minFeatures()       { return getInstance().min_features; }
    static int&     maxIters()          { return getInstance().max_iters; }
    static int&     maxItersRef()       { return getInstance().max_iters_ref; }
    static double&  minError()          { return getInstance().min_error; }
    static double&  minErrorChange()    { return getInstance().min_error_change; }
    static double&  inlierK()           { return getInstance().inlier_k; }


    // SLAM parameters
    static double&  maxKFEpipP()        { return getInstance().max_kf_epip_p; }
    static double&  maxKFEpipL()        { return getInstance().max_kf_epip_l; }
    static int&     minLMEssGraph()     { return getInstance().min_lm_ess_graph; }
    static int&     minLMCovGraph()     { return getInstance().min_lm_cov_graph; }
    static int&     minKFLocalMap()     { return getInstance().min_kf_local_map; }
    static double&  maxLM3DErr()        { return getInstance().max_lm_3d_err; }
    static double&  maxLMDirErr()       { return getInstance().max_lm_dir_err; }
    static double&  lambdaLbaLM()       { return getInstance().lambda_lba_lm; }
    static double&  lambdaLbaK()        { return getInstance().lambda_lba_k; }
    static int&     maxItersLba()       { return getInstance().max_iters_lba; }
    static int&     minLMObs()          { return getInstance().min_lm_obs; }
    static double&  maxCommonFtsKF()    { return getInstance().max_common_fts_kf; }
    static double&  maxDirLineError()   { return getInstance().max_dir_line_error; }
    static double&  maxPointLineError() { return getInstance().max_point_line_error; }
    static double&  maxPointPointError(){ return getInstance().max_point_point_error; }
    static std::string&  dbowVocP()     { return getInstance().vocabulary_p; }
    static std::string&  dbowVocL()     { return getInstance().vocabulary_l; }
    static double&  lcRes()             { return getInstance().lc_res; }
    static double&  lcUnc()             { return getInstance().lc_unc; }
    static double&  lcInl()             { return getInstance().lc_inl; }
    static double&  lcTrs()             { return getInstance().lc_trs; }
    static double&  lcRot()             { return getInstance().lc_rot; }
    static double&  lcMat()             { return getInstance().lc_mat; }
    static int&     maxItersPGO()       { return getInstance().max_iters_pgo; }
    static int&     lcKFDist()          { return getInstance().lc_kf_dist; }
    static int&     lcKFMaxDist()       { return getInstance().lc_kf_max_dist; }
    static int&     lcNKFClosest()      { return getInstance().lc_nkf_closest; }
    static double&  lcInlierRatio()     { return getInstance().lc_inlier_ratio; }
    static int&     minPointMatches()   { return getInstance().min_pt_matches; }
    static int&     minLineMatches()    { return getInstance().min_ls_matches; }
    static double&  kfInlierRatio()     { return getInstance().kf_inlier_ratio; }
    static bool&    fastMatching()      { return getInstance().fast_matching; }
    static bool&    hasRefinement()     { return getInstance().has_refinement; }
    static bool&    multithreadSLAM()   { return getInstance().mutithread_slam; }

    bool bIsRelocalizationMode;
    std::string path_LoadMap, path_SaveMap;

    int img_width;
    int img_height;

    // SLAM parameters (keyframe selection)
    double min_entropy_ratio;
    double max_kf_t_dist;
    double max_kf_r_dist;

    // flags
    bool has_points;
    bool has_lines;
    bool lr_in_parallel;
    bool pl_in_parallel;
    bool best_lr_matches;
    bool adaptative_fast;
    bool use_fld_lines;
    bool use_motion_model;

    // points detection and matching
    int matching_strategy;
    int matching_s_ws;
    int matching_f2f_ws;

    int    orb_nfeatures;
    double orb_scale_factor;
    int    orb_nlevels;
    int    orb_edge_th;
    int    orb_wta_k;
    int    orb_score;
    int    orb_patch_size;
    int    orb_fast_th;

    int    fast_min_th;
    int    fast_max_th;
    int    fast_inc_th;
    int    fast_feat_th;
    double fast_err_th;

    double max_dist_epip;
    double min_disp;
    double min_ratio_12_p;
    double stereo_overlap_th;
    double f2f_overlap_th;
    double line_sim_th;

    double rgbd_min_depth;
    double rgbd_max_depth;

    // lines detection and matching
    int    lsd_nfeatures;
    int    lsd_refine;
    double lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    double line_horiz_th;
    double min_line_length;
    double min_ratio_12_l;
    double ls_min_disp_ratio;

    // optimization
    double homog_th;
    int    min_features;
    int    max_iters;
    int    max_iters_ref;
    double min_error;
    double min_error_change;
    double inlier_k;


    // SLAM parameters
    int    max_kf_num_frames;
    double min_kf_t_dist;
    double min_kf_r_dist;
    int    min_kf_n_feats;
    double max_kf_epip_p;
    double max_kf_epip_l;
    int    min_lm_cov_graph;
    int    min_lm_ess_graph;
    int    min_kf_local_map;
    double max_lm_3d_err;
    double max_lm_dir_err;
    double max_dir_line_error;
    double max_point_line_error;
    double max_point_point_error;
    double lambda_lba_lm;
    double lambda_lba_k;
    int    max_iters_lba;
    int    min_lm_obs;
    double max_common_fts_kf;
    std::string vocabulary_p, vocabulary_l;
    double lc_res;
    double lc_unc;
    double lc_inl;
    double lc_trs;
    double lc_rot;
    double lc_mat;
    int    max_iters_pgo;
    int    lc_kf_dist;
    int    lc_kf_max_dist;
    int    lc_nkf_closest;
    double lc_inlier_ratio;
    int    min_pt_matches;
    int    min_ls_matches;
    double kf_inlier_ratio;
    bool   fast_matching;
    bool   has_refinement;
    bool   mutithread_slam;

private:
    //template<typename T> inline T loadSafe(const cv::FileStorage &fSettings, std::string param, T default_value = T());
};

