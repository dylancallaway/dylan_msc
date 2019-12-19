#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <dylan_msc/obj.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h> // Required for pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
// #include <Eigen/StdVector>

// function defs ------------------------------------------

// end function defs ---------------------------------------

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

PCLCloud::Ptr cloud(new PCLCloud), cloud_f(new PCLCloud);
PCLCloud::Ptr cloud_msg(new PCLCloud);

ros::Publisher seg_points_pub, rviz_marker_pub, cluster_info_pub;

float t0 = 0.0f;

uint64_t frame_index = 0;

Eigen::MatrixXf A(6, 6), Q(6, 6), u(6, 1), H(3, 6), R(3, 3);
Eigen::MatrixXf x_p(6, 1), x_m(3, 1), z(3, 1);
Eigen::MatrixXf P_p(6, 6), P_m(6, 6);

std::vector<Eigen::MatrixXf> x_ps, x_ms, x_ms_last, zs, P_ps, P_ms, zs_orig, x_ms_orig;

std::vector<pcl::PointXYZ> min_pts, max_pts;

// Function called for each frame received
void frame_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *cloud); // Convert PointCloud2 ROS msg to PCL PointCloud

    // ------------- plane filter ---------------------
    // Estimate planar model of floor and remove it
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // planar inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.01f); // maximum euclidean distance between points considered to be planar

    int nr_points = (int)cloud->points.size(); // get size of cloud in number of points
    // Filter out largest planar model until cloud is reduced to XX% (see code) of original cloud
    bool check_val = 1;
    uint8_t check_sum = 0;
    uint8_t filter_index = 0;
    while (filter_index < 10)
    {
        // std::cout << filter_index << "\n"
        //   << std::endl;
        filter_index += 1;
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // If no planar inliers exist
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        check_val = 1;
        static float delta_check = 0.25f;
        check_val = check_val && (coefficients->values[0] > -delta_check && coefficients->values[0] < delta_check);
        check_val = check_val && (coefficients->values[1] > -1.0f - delta_check && coefficients->values[1] < -1.0f + delta_check);
        check_val = check_val && (coefficients->values[2] > -delta_check && coefficients->values[2] < delta_check);
        // A  = 0
        // B = -1
        // C = 0
        // D can be anything

        if (!check_val)
        {
            continue;
        }

        // info about plane
        // std::cout << "Coeffs: " << coefficients->values[0] << " "
        //           << coefficients->values[1] << " "
        //           << coefficients->values[2] << " "
        //           << coefficients->values[3] << " "
        //           << inliers->indices.size() << " "
        //           << check_val << "\n"
        //           << std::endl;

        // check_sum += 1;
        // std::cout << check_sum << "\n"
        //           << std::endl;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        // extract.setNegative(false);

        // Get the points associated with the planar surface
        // extract.filter(*cloud_plane);
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points.\n"
        //   << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud = *cloud_f; // update cloud to be the plane filtered cloud (cloud_f)
    }
    // std::cout << "PointCloud after plane filtering has: " << cloud->points.size() << " data points." << std::endl;
    // -------------------- end plane filter -----------------------

    // ------------------- cluster segmentation -----------------------
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02f);
    ec.setMinClusterSize(250);
    ec.setMaxClusterSize(30000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    PCLCloud::Ptr cluster_sum(new PCLCloud);
    pcl::PointXYZ min_pt, max_pt, cent_pt;
    float t1 = float(std::clock()) / float(CLOCKS_PER_SEC);
    float dt = t1 - t0;
    int num_clusters = 0;
    static int last_num_clusters = 0;
    // std::cout << dt << "\n"
    //   << std::endl;

    zs.clear();
    min_pts.clear();
    max_pts.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PCLCloud::Ptr cloud_cluster(new PCLCloud);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::computeCentroid(*cloud_cluster, cent_pt);

		// if (cent_pt.y > 0.1f)
        // {
        // continue;
        // }

        z(0) = cent_pt.x;
        z(1) = cent_pt.y;
        z(2) = cent_pt.z;
        zs.push_back(z);

        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        min_pts.push_back(min_pt);
        max_pts.push_back(max_pt);



        *cluster_sum += *cloud_cluster;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        // std::cout << "Centroid of cluster: " << cent_pt << "\n"
        //   << std::endl;

        num_clusters += 1;
    }
    // ------------------- cluster segmentation -----------------------

    // publish raw measurements
    for (int i = 0; i < num_clusters; i++)
    {
    	// markers for rviz
        visualization_msgs::Marker cent_marker;
        cent_marker.pose.position.x = zs[i](0);
        cent_marker.pose.position.y = zs[i](1);
        cent_marker.pose.position.z = zs[i](2);
        cent_marker.type = visualization_msgs::Marker::SPHERE;
        cent_marker.header.stamp = ros::Time::now();
        cent_marker.header.frame_id = msg->header.frame_id;
        cent_marker.ns = "centroids";
        cent_marker.id = i;
        cent_marker.action = visualization_msgs::Marker::ADD;
        cent_marker.scale.x = 0.05;
        cent_marker.scale.y = 0.05;
        cent_marker.scale.z = 0.05;
        cent_marker.color.r = 0.0f;
        cent_marker.color.g = 1.0f;
        cent_marker.color.b = 0.0f;
        cent_marker.color.a = 1.0f;
        cent_marker.lifetime = ros::Duration(1.0f);
        rviz_marker_pub.publish(cent_marker);

        // centroids and bounding boxes for other processes
        dylan_msc::obj plot_obj;

        plot_obj.index = i;

        plot_obj.centroid.x = zs[i](0);
        plot_obj.centroid.y = zs[i](1);
        plot_obj.centroid.z = zs[i](2);

        plot_obj.min.x = min_pt.x;
        plot_obj.min.y = min_pt.y;
        plot_obj.min.z = min_pt.z;

        plot_obj.max.x = max_pt.x;
        plot_obj.max.y = max_pt.y;
        plot_obj.max.z = max_pt.z;

        cluster_info_pub.publish(plot_obj);
    }

    // kalman filter stuff
    // if (frame_index < 10)
    // {
    //     x_ms.clear();
    //     x_ps.clear();
    //     P_ms.clear();
    //     P_ps.clear();
    //     for (int i = 0; i < num_clusters; i++)
    //     {
    //         x_m(0) = zs[i](0);
    //         x_m(1) = zs[i](1);
    //         x_m(2) = zs[i](2);
    //         // x_m(3) = 0.0f;
    //         // x_m(4) = 0.0f;
    //         // x_m(5) = 0.0f;
    //         x_ms.push_back(x_m);
    //         x_ps.push_back(x_m);

    //         P_m << 0.1f * Eigen::MatrixXf::Identity(6, 6);
    //         P_ms.push_back(P_m);
    //         P_ps.push_back(P_m);
    //     }
    // }
    // else
    // {
    //     std::cout << "\nFrame: " << frame_index << "\n";

    //     zs_orig = zs; // store raw measurements
    //     x_ms_orig = x_ms;

    //     int m = zs.size(); // get matrix size
    //     int n = x_ms.size();

    //     // Generate e matrix
    //     Eigen::ArrayXXf e(m, n);    // euc distance matrix (rows are measurements, cols are estimates)
    //     for (int i = 0; i < m; i++) // for ith z (measurement)
    //     {
    //         for (int j = 0; j < n; j++) // for jth x_m (estimate)
    //         {
    //             e(i, j) = (zs_orig[i] - x_ms[j]).norm(); // TODO add H back here // euc distance between zs[i] and x_ms[j]
    //         }
    //     }

    //     std::cout << "e:\n"
    //               << e << "\n\n";

    //     if (m == n) // if no objects are added/removed
    //     {
    //         // colwise min
    //         Eigen::MatrixXf col_mins(1, n);
    //         col_mins = e.colwise().minCoeff();

    //         std::cout << "Col Mins: " << col_mins << "\n\n";

    //         // subtract mins from e
    //         Eigen::ArrayXXf e_star(m, n);
    //         for (int j = 0; j < n; j++) // for jth x_m (estimate)
    //         {
    //             e_star.col(j) = e.col(j) - col_mins(j);
    //         }

    //         std::cout << "e*:\n"
    //                   << e_star << "\n\n";

    //         // find zero indices
    //         int num_zeros = 0;
    //         Eigen::Array<bool, -1, -1> find_zeros = e_star == 0.0f;
    //         Eigen::Array<int, -1, -1> zero_inds_temp(m * n, 2);
    //         zero_inds_temp.setConstant(-1);
    //         for (int i = 0; i < m; i++) // for ith z (measurement)
    //         {
    //             for (int j = 0; j < n; j++) // for jth x_m (estimate)
    //             {
    //                 if (find_zeros(i, j) == 1)
    //                 {
    //                     zero_inds_temp(num_zeros, 0) = i;
    //                     zero_inds_temp(num_zeros, 1) = j;
    //                     num_zeros += 1;
    //                 }
    //             }
    //         }
    //         Eigen::Array<int, -1, -1> zero_inds = zero_inds_temp.topRows(num_zeros);

    //         std::cout << "Zero Inds:\n"
    //                   << zero_inds << "\n\n";

    //         // set est to meas it is closest to
    //         for (int j = 0; j < n; j++) // for jth x_m (estimate)
    //         {
    //             int i_star = zero_inds(j, 0);
    //             int j_star = zero_inds(j, 1);
    //             x_ms[i_star] = x_ms_orig[j_star];
    //         }
    //     }
    //     // kalman filter
    //     // -------------
    //     A << 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f,
    //         0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f,
    //         0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt,
    //         0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    //         0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    //         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    //     for (int j = 0; j < n; j++) // for jth x_m (estimate)
    //     {
    //         x_p = x_ps[j];
    //         x_m = x_ms[j];
    //         P_p = P_ps[j];
    //         P_m = P_ms[j];
    //         z = zs[j];

    //         // Prediction update
    //         x_p = A * x_m + u;
    //         P_p = A * P_m * A.transpose() + Q;
    //         // TODO use x_p for euc distance filter
    //         // look into noise matrix
    //         // look into diagonals on covariance matrix (maybe determinant)
    //         // look into x_p leaving FOV
            
    //         // drop x_ps that are predicted to leave FOV

    //         // turtlebot pwd: nvidia
    //         // laptop pwd: nsfnri

    //         // Measurement update
    //         P_m = (P_p.inverse() + H.transpose() * R.inverse() * H).inverse();
    //         x_m = x_p + P_m * H.transpose() * R.inverse() * (z - H * x_p);

    //         x_ps[j] = x_p;
    //         P_ps[j] = P_p;
    //         P_ms[j] = P_m;
    //         x_ms[j] = x_m;
    //         // -------------

    //         // std::cout << "Frame Index: " << frame_index << "\n"
    //         //           << "Cluster Index: " << i << "\n"
    //         //           << "z:\n"
    //         //           << z << "\n"
    //         //           << "x_m:\n"
    //         //           << x_m << "\n"
    //         //           << "P_m:\n"
    //         //           << P_m << "\n";
    //     }

    //     // kalman marker plotting
    //     for (int j = 0; j < n; j++) // for jth x_m (estimate)
    //     {
    //         visualization_msgs::Marker kal_marker;
    //         kal_marker.pose.position.x = x_ms[j](0);
    //         kal_marker.pose.position.y = x_ms[j](1);
    //         kal_marker.pose.position.z = x_ms[j](2);
    //         kal_marker.type = visualization_msgs::Marker::SPHERE;
    //         kal_marker.header.stamp = ros::Time::now();
    //         kal_marker.header.frame_id = msg->header.frame_id;
    //         kal_marker.ns = "centroids";
    //         kal_marker.id = j + 100;
    //         kal_marker.action = visualization_msgs::Marker::ADD;
    //         kal_marker.scale.x = 0.02;
    //         kal_marker.scale.y = 0.02;
    //         kal_marker.scale.z = 0.02;
    //         kal_marker.color.r = 0.0f;
    //         kal_marker.color.g = 0.0f;
    //         kal_marker.color.b = 1.0f;
    //         kal_marker.color.a = 1.0f;
    //         kal_marker.lifetime = ros::Duration(0.75f);
    //         rviz_marker_pub.publish(kal_marker);
    //     }
    // }
    // --------------------------------------------

    t0 = t1;
    last_num_clusters = num_clusters;

    // publish clusters for rviz
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cluster_sum, output);
    output.header = msg->header;
    seg_points_pub.publish(output);
    
    frame_index += 1;
    // std::cout << "\n";
}

int main(int argc, char **argv)
{
    // static matrices
    // u << 0.0f,
    //     0.0f,
    //     0.0f,
    //     0.0f,
    //     0.0f,
    //     0.0f;

    // Q << 0.025f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f,
    //     0.05f, 0.025f, 0.05f, 0.05f, 0.05f, 0.05f,
    //     0.05f, 0.05f, 0.025f, 0.05f, 0.05f, 0.05f,
    //     0.05f, 0.05f, 0.05f, 0.1f, 0.05f, 0.05f,
    //     0.05f, 0.05f, 0.05f, 0.05f, 0.1f, 0.05f,
    //     0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.1f;

    // R << 0.5f, 0.0f, 0.0f,
    //     0.0f, 0.5f, 0.0f,
    //     0.0f, 0.0f, 0.5f;

    // H << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    //     0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    //     0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;

    ros::init(argc, argv, "imageProc1");
    ros::NodeHandle node;

    // publishes cloud with segmented objects
    seg_points_pub = node.advertise<PCLCloud>("/seg_points", 2);

    // publishes segmented object centroid markers
    rviz_marker_pub = node.advertise<visualization_msgs::Marker>("/seg_markers", 5);
    
    // publishes segmented object information (index, centroid, bbox info)
    cluster_info_pub = node.advertise<dylan_msc::obj>("/obj_info", 5);

    // receives cropped images from turtelbot
    ros::Subscriber sub = node.subscribe("/cropped_points", 2, frame_cb);

    ros::spin();

    return 0;
}
