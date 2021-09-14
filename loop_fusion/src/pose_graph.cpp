/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "pose_graph.h"
#define rgbd_in_world 0
//posegraph与全局轨迹相关



PoseGraph::PoseGraph()
{
    //定义相机可视化的外形
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);

    //初始化回环序列号和保存历史漂移数据的w_t_vio，w_r_vio和当前漂移的t_drift,t_drift
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();

    //2021-8-12 hmh
    t_drift_baselink = Eigen::Vector3d(0, 0, 0);
    r_drift_baselink = Eigen::Matrix3d::Identity();
    yaw_r_drift_baselink = Eigen::Matrix3d::Identity();


    global_index = 0;
    //不同sequence_cnt对应不同的漂移，产生新的sequence时会将漂移重置
    //pose_graph坐标，以base_sequence（导入的地图数据）或者sequence=为基准坐标
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;
    use_imu = 0;

    //设置点云对象
    octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>( 0.01 );
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    save_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
    octree->defineBoundingBox(-10000, -1000, -10000, 10000, 10000, 10000);
}

PoseGraph::~PoseGraph()
{
    t_optimization.detach();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 100);//1000
    pub_base_path = n.advertise<nav_msgs::Path>("base_path", 100);//1000
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 100);//1000
    for (int i = 1; i < 10; i++)
        pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 100);//1000
    //pub_octomap = n.advertise<octomap_msgs::Octomap>("octomap", 1000, true);
    pub_octree = n.advertise<sensor_msgs::PointCloud2>("octree", 100);//1000
    pub_loop_id = n.advertise<std_msgs::String>("Loop_id",100);//1000

    //2021-8-12 发布基于loop坐标的base_link
    //pub_loop_base_link_path = n.advertise<nav_msgs::Path>("loop_base_link_path",10);//1000
    //pub_base_link_path = n.advertise<nav_msgs::Path>("base_link_path", 10);//1000

}

void PoseGraph::setIMUFlag(bool _use_imu)
{
    use_imu = _use_imu;
    if(!use_imu)//origin use_imu 2021-8-18 hmh 
    {
        printf("VIO input, perfrom 4 DoF (x, y, z, yaw) pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    }
    else
    {
        printf("VO input, perfrom 6 DoF pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize6DoF, this);
    }

}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    //printf("enter addkeyFrame \n");
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    //如果当前帧sequence不等于sequence_cnt,即两者不匹配，初始化地图与里程计之间的漂移
    //使sequence_cnt序号自加一
    if (sequence_cnt != cur_kf->sequence)
    {
        printf("sequence_cnt is not equal to cur frame's sequence \n");
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    //2021-8-16 hmh 加载地图回环前的位姿
    Matrix3d R_odom_by_vio = vio_R_cur * Rbaselink_camera.transpose();
    Vector3d P_odom_by_vio = vio_P_cur - R_odom_by_vio * Tbaselink_camera;
    //cout << "before add w_r_vio and w_t_vio, vio_P_cur is " << vio_P_cur.transpose() << endl;
    cout << "w_t_vio: " << w_t_vio.transpose() << " " << "w_r_vio: " << w_r_vio << endl;
    //w_r_vio w_t_vio vio 里程计在世界坐标系中的漂移
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    if (flag_detect_loop)
    {
        //printf("detect loop is true \n");
        //printf("detect loop one.\n");
        TicToc tmp_t;
        loop_index = detectLoop(cur_kf, cur_kf->index);
        //printf("detect loop end. \n");
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
        //printf("addKeyFrameIntoVoc \n");
    }

    //添加定位模式，可控制只与加载姿态图的帧回环或者与所有帧回环
    if (AGV_MODEL == "location") {
        cout << "AGV_MODEL : " << AGV_MODEL << endl;
        if (loop_index != -1 && loop_index <= load_pose_graph_size)
        {
            // printf("loop_index is %d and not equal -1, enter loop check \n",loop_index);
            cout << "load pose graph size is " << load_pose_graph_size << " " << "loop_index is " << loop_index << endl;
            KeyFrame* old_kf = getKeyFrame(loop_index);
            //回环检测的策略，看了大概是算一个相对位姿
            if (cur_kf->findConnection(old_kf))
            {
                printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
                //printf("Loop判断第二分支");
                //为什么要把回环的id定在最早发生回环的地方？
                if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                    earliest_loop_index = loop_index;
                //发布回环消息 2021-6-8 huminghao

                std_msgs::String loop_msg;
                std::stringstream ss;

                ss << cur_kf->index << " detect loop with " << loop_index;

                loop_msg.data = ss.str();

                ROS_INFO("%s", loop_msg.data.c_str());

                pub_loop_id.publish(loop_msg);

                //回环消息发布结束

                Vector3d w_P_old, w_P_cur, vio_P_cur;
                Matrix3d w_R_old, w_R_cur, vio_R_cur;
                old_kf->getVioPose(w_P_old, w_R_old);
                cur_kf->getVioPose(vio_P_cur, vio_R_cur);

                Vector3d relative_t;
                Quaterniond relative_q;
                //计算当前帧与回环发生帧的相对位姿
                //w_X_old,老的一帧是检测到的关键帧,世界坐标系下的位姿
                //w_X_cur，当前帧，经过回环检测后计算出的当前坐标系下的位姿
                //vio_X_cur，当前帧在里程计下的位姿
                relative_t = cur_kf->getLoopRelativeT();
                relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
                w_P_cur = w_R_old * relative_t + w_P_old;
                w_R_cur = w_R_old * relative_q;
                double shift_yaw;
                Matrix3d shift_r;
                Vector3d shift_t;
                //计算漂移的地方
                if (!use_imu)//origin use_imu 2021-8-18 hmh 
                {
                    //使用imu，只计算偏航角的角度漂移，所以是四自由度
                    printf("four degree of freedom drift \n");
                    shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
                    shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
                }
                else {
                    printf("six degree of freedom drift \n");
                    //否则，计算六个自由度的漂移，其中角度漂移需要计算三个方向的
                    shift_r = w_R_cur * vio_R_cur.transpose();
                }

                //与w_R_cur 坐标系对齐，计算对齐后的位移差
                //w_R_cur * vio_R_cur.transpose()=w_r-vio_r,最后旋转量是两个坐标系间的差值
                shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
                if (!use_imu) {
                    printf("four degree of freedom drift,set z drift 0 \n");
                    shift_t(2) = 0;
                    t_drift(2) = 0;
                }
                // shift vio pose of whole sequence to the world frame
                if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
                {
                    //加载了地图的情况，与地图保存的姿态图回环了
                    cout << "enter update vio pose, sequence_loop[cur_kf->sequence] is " << sequence_loop[cur_kf->sequence] << endl;
                    w_r_vio = shift_r;
                    w_t_vio = shift_t;
                    cout << "w_t_vio=shift_t is : " << w_t_vio.transpose() << endl;
                    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                    vio_R_cur = w_r_vio * vio_R_cur;
                    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                    //cur_kf->updatePose(vio_P_cur, vio_R_cur);//2021-8-16 hmh
                    //更新vio路径

                    list<KeyFrame*>::iterator it = keyframelist.begin();
                    for (; it != keyframelist.end(); it++)
                    {
                        if ((*it)->sequence == cur_kf->sequence)
                        {
                            Vector3d vio_P_cur;
                            Matrix3d vio_R_cur;
                            (*it)->getVioPose(vio_P_cur, vio_R_cur);
                            vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                            vio_R_cur = w_r_vio * vio_R_cur;
                            (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                            //(*it)->updatePose(vio_P_cur, vio_R_cur);//2021-8-16 hmh
                        }
                    }
                    sequence_loop[cur_kf->sequence] = 1;
                }
                m_optimize_buf.lock();
                optimize_buf.push(cur_kf->index);
                m_optimize_buf.unlock();
            }
        }
    }
    else {
    if (loop_index != -1)
    {
        // printf("loop_index is %d and not equal -1, enter loop check \n",loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        //回环检测的策略，看了大概是算一个相对位姿
        if (cur_kf->findConnection(old_kf))
        {
            printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
            //printf("Loop判断第二分支");
            //为什么要把回环的id定在最早发生回环的地方？
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            //发布回环消息 2021-6-8 huminghao

            std_msgs::String loop_msg;
            std::stringstream ss;

            ss << cur_kf->index << " detect loop with " << loop_index;

            loop_msg.data = ss.str();

            ROS_INFO("%s", loop_msg.data.c_str());

            pub_loop_id.publish(loop_msg);

            //回环消息发布结束

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            //计算当前帧与回环发生帧的相对位姿
            //w_X_old,老的一帧是检测到的关键帧,世界坐标系下的位姿
            //w_X_cur，当前帧，经过回环检测后计算出的当前坐标系下的位姿
            //vio_X_cur，当前帧在里程计下的位姿
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            //计算漂移的地方
            if (!use_imu)//origin use_imu 2021-8-18 hmh 
            {
                //使用imu，只计算偏航角的角度漂移，所以是四自由度
                printf("four degree of freedom drift \n");
                shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            }
            else {
                printf("six degree of freedom drift \n");
                //否则，计算六个自由度的漂移，其中角度漂移需要计算三个方向的
                shift_r = w_R_cur * vio_R_cur.transpose();
            }

            //与w_R_cur 坐标系对齐，计算对齐后的位移差
            //w_R_cur * vio_R_cur.transpose()=w_r-vio_r,最后旋转量是两个坐标系间的差值
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
            if (!use_imu) {
                printf("four degree of freedom drift,set z drift 0 \n");
                shift_t(2) = 0;
                t_drift(2) = 0;
            }
            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {
                //加载了地图的情况，与地图保存的姿态图回环了
                cout << "enter update vio pose, sequence_loop[cur_kf->sequence] is " << sequence_loop[cur_kf->sequence] << endl;
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                cout << "w_t_vio=shift_t is : " << w_t_vio.transpose() << endl;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio * vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                //cur_kf->updatePose(vio_P_cur, vio_R_cur);//2021-8-16 hmh
                //更新vio路径

                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)
                {
                    if ((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio * vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                        //(*it)->updatePose(vio_P_cur, vio_R_cur);//2021-8-16 hmh
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    }
	m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    //2021-8-12 hmh
    //计算baselink漂移
    //get baselink tf
    /* 2021-8-16 hmh
    Matrix3d R_odom_by_vio = R * Rbaselink_camera.transpose();
    Vector3d P_odom_by_vio = P - R_odom_by_vio * Tbaselink_camera;
    */
    //能正常定位到world的小车坐标
    Matrix3d R_odom_loop = r_drift * R * Rbaselink_camera.transpose();
    Vector3d P_odom_loop = r_drift * P + t_drift-R_odom_loop* Tbaselink_camera;

    //loop camera
    //Matrix3d R_cam_loop = r_drift * R;
    //Vector3d P_cam_loop = r_drift * P + t_drift;

    r_drift_baselink = R_odom_loop * R_odom_by_vio.transpose();
    t_drift_baselink = P_odom_loop - r_drift_baselink * P_odom_by_vio;

    double yaw_shift_baselink;
    yaw_shift_baselink = Utility::R2ypr(r_drift_baselink).x();
    yaw_r_drift_baselink = Utility::ypr2R(Vector3d(yaw_shift_baselink, 0, 0));
   
    cout << " ********  t_dritf:          " << t_drift.transpose() << endl;
    cout << " ********  t_dritf_baselink: " << t_drift_baselink.transpose() << endl;
    cout << " ********  w_t_vio:          " << w_t_vio.transpose() << endl;

    //cout << " ******** " << "\n" << " r_dritf: \n" << r_drift << "\n" << endl;
    //cout << " r_dritf_baselink: \n" << r_drift_baselink << "\n" << " ******** " << endl;
    
    //world与odom之间漂移计算结束
    
    //之前发布baselink轨迹的部分，对程序功能没有影响，为节省计算资源就不计算了
    /*
    //回环baselink坐标
    geometry_msgs::PoseStamped loop_base_link_pose_stamped;
    Quaterniond Q_loop_baselink{ R_odom_loop };
    loop_base_link_pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    loop_base_link_pose_stamped.header.frame_id = "world";
    loop_base_link_pose_stamped.pose.position.x = P_odom_loop.x();
    loop_base_link_pose_stamped.pose.position.y = P_odom_loop.y();
    loop_base_link_pose_stamped.pose.position.z = P_odom_loop.z();
    loop_base_link_pose_stamped.pose.orientation.x = Q_loop_baselink.x();
    loop_base_link_pose_stamped.pose.orientation.y = Q_loop_baselink.y();
    loop_base_link_pose_stamped.pose.orientation.z = Q_loop_baselink.z();
    loop_base_link_pose_stamped.pose.orientation.w = Q_loop_baselink.w();
    loop_base_link_path.poses.push_back(loop_base_link_pose_stamped);
    loop_base_link_path.header = loop_base_link_pose_stamped.header;
    
    //未回环baselink坐标
    geometry_msgs::PoseStamped base_link_pose_stamped;
    Quaterniond Q_baselink{ R_odom_by_vio };
    base_link_pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    base_link_pose_stamped.header.frame_id = "odom";
    base_link_pose_stamped.pose.position.x = P_odom_by_vio.x();
    base_link_pose_stamped.pose.position.y = P_odom_by_vio.y();
    base_link_pose_stamped.pose.position.z = P_odom_by_vio.z();
    base_link_pose_stamped.pose.orientation.x = Q_baselink.x();
    base_link_pose_stamped.pose.orientation.y = Q_baselink.y();
    base_link_pose_stamped.pose.orientation.z = Q_baselink.z();
    base_link_pose_stamped.pose.orientation.w = Q_baselink.w();
    base_link_path.poses.push_back(base_link_pose_stamped);
    base_link_path.header = base_link_pose_stamped.header;
    //end
    */
    P = r_drift * P + t_drift;
    R = r_drift * R;

    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::PoseStamped rgbd_stamped;
    
    //设置camera消息头
    rgbd_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    rgbd_stamped.header.frame_id = "camera";
    //2021-7-19
    
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    m_octree.lock();
    //生成点云
    //printf("生成点云");
    sensor_msgs::PointCloud2 tmp_pcl;
    int pcl_count_temp = 0;
    

    if(!cur_kf->point_rgbd.empty())
    {
        //printf("点云生成下第一判断分支");
        for(auto &point_rgbd : cur_kf->point_rgbd)
        {
            Eigen::Vector3d point(point_rgbd[0], point_rgbd[1], point_rgbd[2]);
            //就输出为camera下点云
#if rgbd_in_world
            Eigen::Vector3d pointWorld = R * (qic * point + tic) + P;
#else
            Eigen::Vector3d pointWorld = qic * point;
#endif

            if (pointWorld.z() > 3.5 || pointWorld.z() < -0.5)
                continue;
            pcl::PointXYZRGB searchPoint;
            searchPoint.x = pointWorld.x();
            searchPoint.y = pointWorld.y();
            searchPoint.z = pointWorld.z();
            searchPoint.r = point_rgbd[3];
            searchPoint.g = point_rgbd[4];
            searchPoint.b = point_rgbd[5];
            //cout <<"x:"<< searchPoint.x <<" y:"<< searchPoint.y <<" z:"<< searchPoint.z << endl;
            /*
			//2021-8-9 hmh,针对x86工控机修改
            double min_x, min_y, min_z, max_x, max_y, max_z;
            octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
            bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);
            //if (isInBox) cout << "searchPoint In Box \n" << endl;
            if(isInBox&&octree->getVoxelDensityAtPoint(searchPoint) < 1)*/
			if(octree->getVoxelDensityAtPoint(searchPoint) < 1)
            {
                //printf("点云生成下第三判断分支");
                cur_kf->point_rgbd[pcl_count_temp] = point_rgbd;
                octree->addPointToCloud(searchPoint, cloud);
                // Uncomment this to get pointcloud
                //点云交给八叉树，这里就不用单独存点云了
                //save_cloud->points.push_back(searchPoint);
                ++pcl_count_temp;
            }
        }
        cur_kf->point_rgbd.resize(pcl_count_temp);
        //printf("生成点云ros消息");
        pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);
    }
    

    m_octree.unlock();

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(0);
        loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        loop_path_file.precision(5);
        loop_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        loop_path_file.close();
    }
    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            //printf("has loop \n");
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if(cur_kf->sequence > 0)
            {
                //printf("add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }
            
        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
#if rgbd_in_world
    tmp_pcl.header = pose_stamped.header;
    //cur_kf keyframelist 为列表，里面装的是cur_kf
#else
    tmp_pcl.header = rgbd_stamped.header;
#endif
    //2021-8-10 hmh
    //定位模式，限制keyframelist规模
    /* 2021 8-13测试，直接pop keyframe会导致loop节点直接退出，还未查明原因
    if (AGV_MODEL =="location") {
        cout << "AGV_MODEL : " << AGV_MODEL << endl;
        printf("keyframe window size is %d \n",KEYFRAMEWINDOWSIZE);

        while (keyframelist.size()> KEYFRAMEWINDOWSIZE) {
            printf("pop old keyframe!");
            keyframelist.pop_front();
        }
        printf("keyframe window size end");
        //keyframelist.pop_front();
        keyframelist.push_back(cur_kf);
    }
    else {
        cout << "AGV_MODEL : " << AGV_MODEL << endl;
        keyframelist.push_back(cur_kf);
    }
    */
    /*
    if (AGV_MODEL == "location") {
        cout << "AGV_MODEL : " << AGV_MODEL << endl;
        printf("keyframe window size is %d \n", KEYFRAMEWINDOWSIZE);
    }
    else {
        cout << "AGV_MODEL : " << AGV_MODEL << endl;
        keyframelist.push_back(cur_kf);
    }
    */
    keyframelist.push_back(cur_kf);
    publish();
    //printf("publish pointcloud by addKeyFrame \n");
    pub_octree.publish(tmp_pcl);

    //发布点云后就对点云数据进行清理 7-78 hmh
    m_octree.lock();
    octree->deleteTree();
    cloud->clear();
    //save_cloud->clear();
    m_octree.unlock();
    //清除完毕，解锁


	m_keyframelist.unlock();
}


void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    //printf("index of cur_kf is %d \n",global_index);
    global_index++;
    int loop_index = -1;
    
    if (flag_detect_loop)
    {
        //printf("detect loop one.\n");
        loop_index = detectLoop(cur_kf, cur_kf->index);
        //printf("detect loop end. \n");
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
        //printf("detect loop one.\n");
    }
    if (loop_index != -1)
    {
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::PoseStamped rgbd_stamped;

    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";

    //设置camera消息头
    rgbd_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    rgbd_stamped.header.frame_id = "camera";
    //2021-7-19

    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    if(!cur_kf->point_rgbd.empty()){
        m_octree.lock();
        sensor_msgs::PointCloud2 tmp_pcl;

        for(auto &point_rgbd : cur_kf->point_rgbd){
            Eigen::Vector3d point(point_rgbd[0], point_rgbd[1], point_rgbd[2]);
#if rgbd_in_world
            Eigen::Vector3d pointWorld = R * (qic * point + tic) + P;
#else
            Eigen::Vector3d pointWorld = qic * point;
#endif
            if(pointWorld.z() > 3.5 || pointWorld.z() < -0.5)
                continue;
            pcl::PointXYZRGB searchPoint;
            searchPoint.x = pointWorld.x();
            searchPoint.y = pointWorld.y();
            searchPoint.z = pointWorld.z();
            searchPoint.r = point_rgbd[3];
            searchPoint.g = point_rgbd[4];
            searchPoint.b = point_rgbd[5];

            if(octree->getVoxelDensityAtPoint(searchPoint) < 1){
                octree->addPointToCloud(searchPoint, cloud);
                //save_cloud->points.push_back(searchPoint);
            }
        }
        pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);

#if rgbd_in_world
        tmp_pcl.header = pose_stamped.header;
        //cur_kf keyframelist 为列表，里面装的是cur_kf
#else
        tmp_pcl.header = rgbd_stamped.header;
#endif
        //printf("publish pointcloud by loadKeyFrame \n");
        pub_octree.publish(tmp_pcl);
        
        //发布点云后就对点云数据进行清理 7-78 hmh
        octree->deleteTree();
        cloud->clear();
        //save_cloud->clear();
        //清除完毕，解锁

        m_octree.unlock();
    }

    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    /*
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */
    keyframelist.push_back(cur_kf);
    //publish();
    m_keyframelist.unlock();
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    //printf("Get keyFrame\n");
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        //将图片进行缩放操作，缩放至（376，240）
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
/*
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                            t_array[i-j], 
                                            euler_array[i], 
                                            t_array[i]);
                  }
                }

                //add loop edge
                
                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                  t_array[connected_index], 
                                                                  euler_array[i], 
                                                                  t_array[i]);
                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            //计算漂移
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}


void PoseGraph::optimize6DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            double q_array[max_length][4];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //ptions.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i][0] = tmp_q.w();
                q_array[i][1] = tmp_q.x();
                q_array[i][2] = tmp_q.y();
                q_array[i][3] = tmp_q.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                    if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                    {
                        Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                        Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                        relative_t = q_i_j.inverse() * relative_t;
                        Quaterniond relative_q = q_i_j.inverse() * q_i;
                        ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                0.1, 0.01);
                        problem.AddResidualBlock(vo_function, NULL, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
                    }
                }

                //add loop edge
                
                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    Quaterniond relative_q;
                    relative_q = (*it)->getLoopRelativeQ();
                    ceres::CostFunction* loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            r_drift = cur_r * vio_r.transpose();
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}

void PoseGraph::updatePath()
{
    TicToc updatePath_time;
    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    //temp_keyframelist装rgbd点的
    //去除update时点云发布
    //vector< vector< Eigen::Matrix<float ,6, 1> > > tmp_keyframelist;
    queue< pair< Matrix3d, Vector3d> > tmp_RTlist;

    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();
    posegraph_visualization->reset();

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);

        tmp_RTlist.push(make_pair(R, P));

        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();

        //tmp_keyframelist.push_back((*it)->point_rgbd);
        

        if((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
        }

        if (SAVE_LOOP_PATH)
        {
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(0);
            loop_path_file << (*it)->time_stamp * 1e9 << ",";
            loop_path_file.precision(5);
            loop_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            loop_path_file.close();
        }
        //draw local connection
        if (SHOW_S_EDGE)
        {
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)  
            {  
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            } 
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {
                
                KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if((*it)->sequence > 0)
                {
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }

    }
    publish();
    m_keyframelist.unlock();
    //去除发布环节 7-29 hmh
    /*
    m_octree.lock();
    //some clean up
    octree->deleteTree();
    cloud->clear();
    save_cloud->clear();
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
    octree->defineBoundingBox(-10000, -10000, -10000, 10000, 10000, 10000);

    int update_count = 0;
    for (auto &point_rgbd_vect : tmp_keyframelist)
    {
        Vector3d P;
        Matrix3d R;
        R = tmp_RTlist.front().first;
        P = tmp_RTlist.front().second;
        for (auto &point_rgbd : point_rgbd_vect)
        {
            Vector3d point(point_rgbd[0] , point_rgbd[1], point_rgbd[2]);

#if rgbd_in_world
            Eigen::Vector3d pointWorld = R * (qic * point + tic) + P;
#else
            Eigen::Vector3d pointWorld = qic * point;
#endif           
            pcl::PointXYZRGB searchPoint;
            searchPoint.x = pointWorld.x();
            searchPoint.y = pointWorld.y();
            searchPoint.z = pointWorld.z();
            searchPoint.r = point_rgbd[3];
            searchPoint.g = point_rgbd[4];
            searchPoint.b = point_rgbd[5];
            ++update_count;
            if(octree->getVoxelDensityAtPoint(searchPoint) < 5){
                octree->addPointToCloud(searchPoint, cloud);
                save_cloud->points.push_back(searchPoint);
            }
        }
        tmp_RTlist.pop();
    }
    sensor_msgs::PointCloud2 tmp_pcl;
    pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);
#if rgbd_in_world
    tmp_pcl.header.stamp =  ros::Time::now();
    tmp_pcl.header.frame_id = "world";
#else
    tmp_pcl.header.stamp = ros::Time::now();
    tmp_pcl.header.frame_id = "camera";
#endif
    printf("publish pointcloud by updatePath \n");
    pub_octree.publish(tmp_pcl);
    m_octree.unlock();
    //ROS_INFO("Update done! Time cost: %f   total points: %d", t_update.toc(), update_count);
    */
    ROS_INFO("Update Path done!,Time cost: %f", updatePath_time.toc());
}


void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("打开文件");
    pFile = fopen (file_path.c_str(),"w");
    // fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;
        
        /*
        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d %d\n",(*it)->index, (*it)->time_stamp,
                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), 
                                    PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(), 
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), 
                                    (*it)->loop_index, 
                                    (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                                    (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                                    (int)(*it)->keypoints.size(), (int)(*it)->point_rgbd.size());
        */
        fprintf(pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d \n", (*it)->index, (*it)->time_stamp,
            VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
            PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
            VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),
            PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),
            (*it)->loop_index,
            (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
            (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
            (int)(*it)->keypoints.size());
        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, 
                                                     (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);

        /*
        std::string densepoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_densepoints.txt";
        FILE *densepoints_file;
        densepoints_file = fopen(densepoints_path.c_str(), "w");
        for(int i = 0; i < (int)(*it)->point_rgbd.size();i++)
        {
            fprintf(densepoints_file, "%f %f %f %f %f %f\n",(*it)->point_rgbd[i][0],(*it)->point_rgbd[i][1],(*it)->point_rgbd[i][2],
                    (*it)->point_rgbd[i][3],(*it)->point_rgbd[i][4],(*it)->point_rgbd[i][5]);
        }
        fclose(densepoints_file);
        */
    }
    fclose(pFile);

    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}
void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("load pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    //去除点云
    //int keypoints_num,densepoints_num;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    //末尾去除一个%d 2021-8-5 hmh
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp,
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &PG_Tx, &PG_Ty, &PG_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF)//&keypoints_num, &densepoints_num) != EOF)
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp, 
                                    VIO_Tx, VIO_Ty, VIO_Tz, 
                                    PG_Tx, PG_Ty, PG_Tz, 
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz, 
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz, 
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors   
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++){
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);
        //去除点云发布
        /*
        string densepoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_densepoints.txt";
        FILE *densepoints_file;
        densepoints_file = fopen(densepoints_path.c_str(), "r");
        vector<Eigen::Matrix<float ,6, 1>> points_rgbd;
        for (int i = 0; i < densepoints_num; i++){
            double p_x, p_y, p_z, p_r, p_g, p_b;
            Eigen::Matrix<float ,6, 1> point_rgbd;
            if(!fscanf(densepoints_file,"%lf %lf %lf %lf %lf %lf", &p_x, &p_y, &p_z, &p_r, &p_g, &p_b))
                printf(" fail to load pose graph \n");
            point_rgbd[0] = p_x;
            point_rgbd[1] = p_y;
            point_rgbd[2] = p_z;
            point_rgbd[3] = p_r;
            point_rgbd[4] = p_g;
            point_rgbd[5] = p_b;
            points_rgbd.push_back(point_rgbd);
        }
        fclose(densepoints_file);
        */
        //2021-8-5 hmh
        //KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, points_rgbd, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    load_pose_graph_size = index;
    printf("pose graph  size is %d", load_pose_graph_size);
    publish();
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        //if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            pub_pg_path.publish(path[i]);
            pub_path[i].publish(path[i]);
            posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }
    pub_base_path.publish(base_path);
    //pub_loop_base_link_path.publish(loop_base_link_path);
    //pub_base_link_path.publish(base_link_path);
    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}
