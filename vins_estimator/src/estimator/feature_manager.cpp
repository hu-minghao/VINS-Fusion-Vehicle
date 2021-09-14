/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"
#include <mutex>
#include <stdio.h>

//ʹ��rosNodeTest.cpp�е�wheel_odom_buf����
queue<nav_msgs::Odometry::ConstPtr> wheel_odom_buf;
std::mutex m_wheel_odom;

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    //printf("MIN_CONTAIN_FRAME is %d", MIN_CONTAIN_FRAME);

    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= MIN_CONTAIN_FRAME)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        //������4֡�г��֣��ż��������
        if (it_per_id.used_num < MIN_CONTAIN_FRAME)
        {
            //printf("used_num is %d,pass! \n",it_per_id.used_num);
            continue;
        }
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{

    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < MIN_CONTAIN_FRAME)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    //����row(n),ȡ��n�У�col(n),��n��
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    //���ǲ���
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

//void FeatureManager::quaternion2Matrix3d()

/*
bool FeatureManager::solvePoseByWheel_1(Eigen::Matrix3d &Rcam, Eigen::Vector3d &Pcam,
    double point3d_time,double point2d_time) 
{
    
    
    nav_msgs::Odometry::ConstPtr temp_point3d_odom;
    nav_msgs::Odometry::ConstPtr temp_point2d_odom;
    
    if (wheel_odom_buf.empty()) {
        printf("wheel_odom_buf is empty! \n");
        return false;
    }
    else if (wheel_odom_buf.front()->header.stamp.toSec() > point3d_time) {
        printf("no wheel_odom data is before point3d_time!  \n");
        return false;
    }
    else if (wheel_odom_buf.back()->header.stamp.toSec() < point2d_time) {
        printf("all wheel_odom data is before point2d_time!  \n");
        return false;
    }
    else {
        printf("have good wheel_odom data for point3d and point2d time!  \n");
        //������
        m_wheel_odom.lock();

        temp_point3d_odom = wheel_odom_buf.front();
        wheel_odom_buf.pop();
        while (wheel_odom_buf.front()->header.stamp.toSec() < point3d_time) {
            temp_point3d_odom = wheel_odom_buf.front();
            wheel_odom_buf.pop();
        }

        //ȡpoint_3dλ���е���ת
        Eigen::Quaterniond point3d_q;
        point3d_q.x() = temp_point3d_odom->pose.pose.orientation.x;
        point3d_q.y() = temp_point3d_odom->pose.pose.orientation.y;
        point3d_q.z() = temp_point3d_odom->pose.pose.orientation.z;
        point3d_q.w() = temp_point3d_odom->pose.pose.orientation.w;

        Eigen::Matrix3d point3d_R = point3d_q.toRotationMatrix();

        temp_point2d_odom = wheel_odom_buf.front();
        wheel_odom_buf.pop();
        while (wheel_odom_buf.front()->header.stamp.toSec() < point2d_time) {
            temp_point2d_odom = wheel_odom_buf.front();
            wheel_odom_buf.pop();
        }
        //����
        m_wheel_odom.unlock();
        
        //ȡpoint_2dλ���е���ת
        Eigen::Quaterniond point2d_q;
        point2d_q.x() = temp_point2d_odom->pose.pose.orientation.x;
        point2d_q.y() = temp_point2d_odom->pose.pose.orientation.y;
        point2d_q.z() = temp_point2d_odom->pose.pose.orientation.z;
        point2d_q.w() = temp_point2d_odom->pose.pose.orientation.w;

        Eigen::Matrix3d point2d_R = point2d_q.toRotationMatrix();
        
        //���������ת
        Rcam = point3d_R.transpose() * point2d_R;

        //�������λ��
        Pcam << temp_point2d_odom->pose.pose.position.x - temp_point3d_odom->pose.pose.position.x,
            temp_point2d_odom->pose.pose.position.y - temp_point3d_odom->pose.pose.position.y,
            temp_point2d_odom->pose.pose.position.z - temp_point3d_odom->pose.pose.position.z;

        //�������
        printf("point3d_time is %f,point2d_time is %f,R is \n", point3d_time, point2d_time);
        std::cout << Rcam << std::endl;
       
        for (auto p = begin(Rcam); p != end(Rcam); ++p) {   //��begin()��end()������ֹ��Ŀ��Ʊ���
            for (auto q = begin(*p); q != end(*p); ++q) {
                cout << *q << ' ';
            }
            cout << endl;
        }
         

        printf("Pcam is %f,%f,%f \n", Pcam[0], Pcam[1], Pcam[2]);
        return true;
    }
}
*/

void FeatureManager::linear_insert(Eigen::Quaterniond &Qodom, Eigen::Vector3d& Podom,const double sync_time,nav_msgs::Odometry::ConstPtr
    &front_data, nav_msgs::Odometry::ConstPtr &back_data) {

    double front_scale = (back_data->header.stamp.toSec() - sync_time) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    double back_scale = (sync_time - front_data->header.stamp.toSec()) / (back_data->header.stamp.toSec() - front_data->header.stamp.toSec());
    //���Բ�ֵλ��
    //ֻ��ֵx,y����ת��z,w
    Podom << front_data->pose.pose.position.x * front_scale + back_data->pose.pose.position.x * back_scale,
        front_data->pose.pose.position.y* front_scale + back_data->pose.pose.position.y * back_scale,
        //front_data->pose.pose.position.z* front_scale + back_data->pose.pose.position.z * back_scale;
        0;
    //���Բ�ֵ��ת
    //Qodom.x() = front_data->pose.pose.orientation.x*front_scale+back_data->pose.pose.orientation.x*back_scale;
    //Qodom.y() = front_data->pose.pose.orientation.y * front_scale + back_data->pose.pose.orientation.y * back_scale;
    Qodom.x() = 0.0;
    Qodom.y() = 0.0;
    Qodom.z() = front_data->pose.pose.orientation.z * front_scale + back_data->pose.pose.orientation.z * back_scale;
    Qodom.w() = front_data->pose.pose.orientation.w * front_scale + back_data->pose.pose.orientation.w * back_scale;
    Qodom.normalize();
}

void FeatureManager::getOdomData(Eigen::Quaterniond &Q,Eigen::Vector3d &P, nav_msgs::Odometry::ConstPtr &odomData) {
    //ȡ��ת
    
    Q.x() = -odomData->pose.pose.orientation.x;
    Q.y() = -odomData->pose.pose.orientation.y;
    Q.z() = odomData->pose.pose.orientation.z;
    Q.w() = odomData->pose.pose.orientation.w;
    
    //ȡλ��
    P << odomData->pose.pose.position.x, odomData->pose.pose.position.y, odomData->pose.pose.position.z;
}

/*
void FeatureManager::transformOdomTCam(Eigen::Matrix3d& Rcam, Eigen::Vector3d& Pcam) {
    Rcam *= Rwc;
    Pcam += Twc;
}
*/

bool FeatureManager::getPoseByWheelOdom(Eigen::Vector3d& Pcam,Eigen::Matrix3d& Rcam,const double curTime) {
    
    if (wheel_odom_buf.empty()) {
        printf("odom data has not push into buf yet! \n");
        return false;
    }

    nav_msgs::Odometry::ConstPtr odomFront;
    nav_msgs::Odometry::ConstPtr odomBack;
    Eigen::Quaterniond odomQ;
    Eigen::Vector3d odomP;

    //wheel��cam��ת����

    Eigen::Matrix3d wheel2Cam;
    Eigen::Matrix<double,1,3> Ttemp;
    wheel2Cam << 0, 0, 1, -1, 0, 0, 0, -1, 0;


    if (curTime < wheel_odom_buf.front()->header.stamp.toSec() || wheel_odom_buf.back()->header.stamp.toSec() < curTime) {
        printf("curtime odom data not push into wheel buf! \n ");
        return false;
    }
    else{
        //��ʱ��
        TicToc use_time;

        m_wheel_odom.lock();
        while (wheel_odom_buf.front()->header.stamp.toSec() <= curTime) {
            if (wheel_odom_buf.front()->header.stamp.toSec() == curTime) {
                getOdomData(odomQ,odomP, wheel_odom_buf.front());
                //printf("get odom curtime data: q %f %f %f %f \n",odomQ.x(), odomQ.y(), odomQ.z(), odomQ.w());
                cout << odomP << endl;
                //�������ϵ��world�µ�λ��
                odomQ.normalize();
                Rcam = odomQ.toRotationMatrix();
                //Ttemp=Twc.transpose()* Rcam.transpose();
                Pcam = odomP + Rcam*Twc; 
                //�������ϵ����ת
                Rcam *= wheel2Cam;//7-8
                //��base_linkת����camera

                //transformOdomTCam(Rcam,Pcam);

                //Rcam = odomQ.toRotationMatrix();
                wheel_odom_buf.pop();
                m_wheel_odom.unlock();
                return true;
            }
            odomFront = wheel_odom_buf.front();
            wheel_odom_buf.pop();
        }
        odomBack = wheel_odom_buf.front();
        m_wheel_odom.unlock();
        linear_insert(odomQ, odomP, curTime, odomFront, odomBack);
        //printf("get odom curtime data: q %f %f %f %f \n", odomQ.x(), odomQ.y(), odomQ.z(), odomQ.w());
        //cout << odomP << endl;
        //�������ϵ��world�µ�λ��
        Rcam = odomQ.toRotationMatrix();
        //Ttemp = Twc.transpose() * Rcam;
        //Ttemp = Twc.transpose() * Rcam.transpose();
        Pcam = odomP + Rcam*Twc;
        //Pcam = odomP + Rcam * Twc;
        //�������ϵ����ת
        Rcam *= wheel2Cam;//7-8

        //Rcam = odomQ.toRotationMatrix();
        //��ʱ
        //printf("get pose data from wheel odom cost %f \n",use_time.toc());
        return true;
    }
}

bool FeatureManager::getRelativePoseByWheel(Eigen::Matrix3d& Rcam, Eigen::Vector3d& Pcam,
    const double prevTime, const double curTime)
{

    if (wheel_odom_buf.empty()) {
        printf("wheel_odom_buf is empty! \n");
        return false;
    }

    nav_msgs::Odometry::ConstPtr tempPrevOdom_front;
    nav_msgs::Odometry::ConstPtr tempPrevOdom_back;
    nav_msgs::Odometry::ConstPtr tempCurOdom_front;
    nav_msgs::Odometry::ConstPtr tempCurOdom_back;
    bool curTimeodomFlag = false;


    //��Ҫ��õ�����
    Eigen::Quaterniond prevTimeQ, curTimeQ;
    Eigen::Vector3d prevTimeP, curTimeP;

    //���ڵ��ڣ�����curTime��odom���ݣ�curTime��odom���ݣ����Ա�prevTime�õ�
    //���ڶ��У����ͱյ������Ƿ�ȡ��pop()
    if (curTime <= wheel_odom_buf.back()->header.stamp.toSec()) {

        if (wheel_odom_buf.front()->header.stamp.toSec() <= prevTime) {
            //���������������
            m_wheel_odom.lock();

            //ʹ��at()��Ҫ���б߽��飬��0��ʼ
            //��α�֤����ʹ�ò�Խ��
            //�������
            //����,ʹ�ü򵥶��оͿ�����
            //�����ȣ����ò�ֵ
            if (wheel_odom_buf.front()->header.stamp.toSec() == prevTime) {
                //ֱ�Ӹ�ֵ
                getOdomData(prevTimeQ, prevTimeP, wheel_odom_buf.front());
                wheel_odom_buf.pop();
            }
            else {
                while (wheel_odom_buf.front()->header.stamp.toSec() < prevTime) {
                    //printf("pop messages before prevTime in wheel_odom_buf!\n");
                    //ȡprevtime�Ĳ�ֵǰ����
                    tempPrevOdom_front = wheel_odom_buf.front();
                    wheel_odom_buf.pop();
                }

                //prevtimeȡ���ݺ�pop��һ�㣬��ֹodom�����������µ�prevtime��curtimeȡ��ͬһʱ������
                tempPrevOdom_back = wheel_odom_buf.front();
                //���в�ֵ
                linear_insert(prevTimeQ, prevTimeP, prevTime, tempPrevOdom_front, tempPrevOdom_back);
            }

            while (wheel_odom_buf.front()->header.stamp.toSec() <= curTime) {
                if (wheel_odom_buf.front()->header.stamp.toSec() == curTime) {
                    //ֱ�Ӹ�ֵ
                    getOdomData(curTimeQ, curTimeP, wheel_odom_buf.front());
                    curTimeodomFlag = true;
                    break;
                }
                else {
                    tempCurOdom_front = wheel_odom_buf.front();
                    wheel_odom_buf.pop();
                }
            }
            //ȡ��һ�����ڵ���curtime������
            tempCurOdom_back = wheel_odom_buf.front();
            if (!curTimeodomFlag) {
                linear_insert(curTimeQ, curTimeP, curTime, tempCurOdom_front, tempCurOdom_back);
            }
            //��pop�����һ��prevtime����ȡ��ǰһ��ʱ�����curtime������
            //wheel_odom_buf.pop(); 
            //���ݲ�����ϣ�����
            m_wheel_odom.unlock();


            //��ֵ���prevTimeλ���е���ת��ƽ��
            Eigen::Matrix3d preR = prevTimeQ.toRotationMatrix();
            Eigen::Matrix3d curR = curTimeQ.toRotationMatrix();

            //���������ת
            Rcam = preR.transpose() * curR;

            //�������λ��
            Pcam << curTimeP - prevTimeP;

            //�������
            //printf("preTime is %f,curTime is %f,R is \n", prevTime, curTime);
            //std::cout << Rcam << std::endl;
            /*
            for (auto p = begin(Rcam); p != end(Rcam); ++p) {   //��begin()��end()������ֹ��Ŀ��Ʊ���
                for (auto q = begin(*p); q != end(*p); ++q) {
                    cout << *q << ' ';
                }
                cout << endl;
            }
            */

            //printf("Pcam is %f,%f,%f \n", Pcam[0], Pcam[1], Pcam[2]);
            return true;
        }
        else {
            //printf("prevTime odom data has not in buf!\n");
            return false;
        }
    }
    else {
        //printf("curTime odom data has not push in buf yet!\n");
        return false;
    }

}


void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], const double curtime)
{

    //printf("enter initFramePoseByPnP! \n");
    if(frameCnt > 0)
    {
        //����ȡ�����ע�͵�
        /*
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];
                    //ʱ���Ϊstart_frame��index��ʱ���
                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        //ע�ͽ���
        */
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        //����ϵ������ת���ԣ�ric imu(body����ϵ�£�cam����ת��̬)���ù�ʽΪ�ֲ�����תȫ�����깫ʽ
        
        //camԭ������ϵ��ת�����ֲ�����תȫ�����깫ʽ����Ͳ�������ת�����ˣ�
        
        //RCam = Rs[frameCnt - 1] * ric[0];
        //PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        //�˴������ʽ��̼ƻ�ȡ�߼�������odometry���ݣ��޸İ���Ҫע��
        //double point3d_time = Headers[frameCnt-1];
        //double point2d_time = Headers[frameCnt];
        //end

        //printf("point3d_time is %f,point2d_time is %f \n", point3d_time, point2d_time);
        //if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        //if (solvePoseByWheel(RCam, PCam, point3d_time, point2d_time))
        if(getPoseByWheelOdom(PCam,RCam,curtime))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
        else {
            printf("solvePoseByWheel failed! \n");
        }
    }
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    //printf("triangulate point! \n");
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;

        if((STEREO || DEPTH) && it_per_id.feature_per_frame[0].is_stereo) {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            //�������ϵԭ������������ϵ�µ����꣨�������һ�α��۲⵽��֡�ڻ��������е�λ�ã�
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            //ת����ϵ������ϵ����Ϊ�෴��ϵ��
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            //�������α任����
            //t1 ps:body��������λ�ã�tic[1]��ȣ�����������ͷ���imu�����λ�ơ�t1����Ȼ���������ͷ����ϵ����������ϵ��λ��
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];//����ͷ����������ϵ����ת
            
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            //vector.head(n)��ȡǰn��Ԫ�أ�point��x,y,Ϊ��һ�������ϵĵ����꣬zΪ�����
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();

            if (depth > 0 && depth < 7) {
                it_per_id.estimated_depth = depth;
            } else
                it_per_id.estimated_depth = INIT_DEPTH;
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1)
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            continue;
        }

        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < MIN_CONTAIN_FRAME)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
            it_per_id.estimated_depth = INIT_DEPTH;

    }

}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}
