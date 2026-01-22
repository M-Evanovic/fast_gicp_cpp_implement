#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

template <typename Registration>
void align(Registration& reg,
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
            pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned) {
    auto t1 = std::chrono::high_resolution_clock::now();
    reg.setInputTarget(target);
    reg.setInputSource(source);
    reg.align(*aligned);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "align time cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " ms" << std::endl;
    double fitness_score = reg.getFitnessScore();
    std::cout << "fitness_score: " << fitness_score << std::endl;
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {
  int size = pc.points.size();
  pc_colored.clear();
  pcl::PointXYZRGB pt_tmp;
  for (int i = 0; i < size; ++i) {
    const auto &pt = pc.points[i];
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = pt.z;
    pt_tmp.r = color[0];
    pt_tmp.g = color[1];
    pt_tmp.b = color[2];
    pc_colored.points.emplace_back(pt_tmp);
  }
}

// gicp_align target_pcd source_pcd det_range(optional) voxel_size(optional)
// ./gicp_align ./data/map.pcd ./data/scan.pcd 20 0.5
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "usage: gicp_align target_pcd source_pcd" << std::endl;
        return 0;
    }

    // load data
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    auto t1 = std::chrono::high_resolution_clock::now();
    // target: map
    if (pcl::io::loadPCDFile(argv[1], *temp_target_cloud)) {
        std::cerr << "failed to open " << argv[1] << std::endl;
        return -1;
    }
    // source: scan
    if (pcl::io::loadPCDFile(argv[2], *source_cloud)) {
        std::cerr << "failed to open " << argv[2] << std::endl;
        return -1;
    }

    // extract submap
    double det_range = 100.0;
    if (argc > 3) {
        char *endptr;
        det_range = strtof(argv[3], &endptr);
        std::cout << "detection range: " << det_range << "m" << std::endl;
    }
    for (auto pt : temp_target_cloud->points) {
        if (pt.x < det_range && pt.x > -det_range &&
            pt.y < det_range && pt.y > -det_range && 
            pt.z < det_range && pt.z > -det_range) {
        // if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= det_range * det_range) {
            target_cloud->points.emplace_back(pt);
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "load pcd time cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " ms" << std::endl;

    // downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_down(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_down(new pcl::PointCloud<pcl::PointXYZ>());
    float down_voxel_size = 0.01f;
    if (argc > 4) {
        char *endptr;
        down_voxel_size = strtof(argv[4], &endptr);
        std::cout << "downsample voxel size: " << down_voxel_size << "m" << std::endl;
    }
    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(down_voxel_size, down_voxel_size, down_voxel_size);
    voxel_filter.setInputCloud(target_cloud);
    voxel_filter.filter(*target_cloud_down);
    voxel_filter.setInputCloud(source_cloud);
    voxel_filter.filter(*source_cloud_down);
    auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "downsample time cost: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
              << " ms" << std::endl;

    std::cout << std::endl;

    // align
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::cout << "--- pcl_gicp ---" << std::endl;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> pcl_gicp;
    align(pcl_gicp, target_cloud_down, source_cloud_down, result_cloud);
    std::cout << "----------------" << std::endl << std::endl;
    
    std::cout << "--- pcl_ndt ---" << std::endl;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> pcl_ndt;
    pcl_ndt.setResolution(1.0);
    align(pcl_ndt, target_cloud_down, source_cloud_down, result_cloud);
    std::cout << "---------------" << std::endl << std::endl;

    std::cout << "--- fast gicp single thread ---" << std::endl;
    fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ> fast_gicp_st;
    align(fast_gicp_st, target_cloud_down, source_cloud_down, result_cloud);
    std::cout << "-------------------------------" << std::endl << std::endl;

    std::cout << "--- fast gicp multi threads ---" << std::endl;
    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> fast_gicp_mt;
    // fast_gicp uses all the CPU cores by default
    // fast_gicp_mt.setNumThreads(8);
    align(fast_gicp_mt, target_cloud_down, source_cloud_down, result_cloud);
    std::cout << "-------------------------------" << std::endl << std::endl;

    fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> fast_vgicp;
    fast_vgicp.setResolution(2.0);
    std::cout << "--- fast vgicp single thread ---" << std::endl;
    fast_vgicp.setNumThreads(1);
    align(fast_vgicp, target_cloud_down, source_cloud_down, result_cloud);
    std::cout << "--------------------------------" << std::endl << std::endl;

    std::cout << "--- fast vgicp multi threads ---" << std::endl;
    fast_vgicp.setNumThreads(omp_get_max_threads());
    align(fast_vgicp, target_cloud_down, source_cloud_down, result_cloud);
    std::cout << "--------------------------------" << std::endl << std::endl;

    #ifdef USE_CUDA
    std::cout << "--- ndt cuda (P2D) ---" << std::endl;
    fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;
    ndt_cuda.setResolution(1.0);
    ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    align(ndt_cuda, target_cloud_down, source_cloud_down, result_cloud);

    std::cout << "--- ndt cuda (D2D) ---" << std::endl;
    ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    align(ndt_cuda, target_cloud_down, source_cloud_down, result_cloud);

    std::cout << "--- fast vgicp cuda (parallel_kdtree) ---" << std::endl;
    fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> fast_vgicp_cuda;
    fast_vgicp_cuda.setResolution(1.0);
    // fast_vgicp_cuda uses CPU-based parallel KDTree in covariance estimation by default
    // on a modern CPU, it is faster than GPU_BRUTEFORCE
    // fast_vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
    align(fast_vgicp_cuda, target_cloud_down, source_cloud_down, result_cloud);

    std::cout << "--- fast vgicp cuda (gpu_bruteforce) ---" << std::endl;
    // use GPU-based bruteforce nearest neighbor search for covariance estimation
    // this would be a good choice if your PC has a weak CPU and a strong GPU (e.g., NVIDIA Jetson)
    fast_vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);
    align(fast_vgicp_cuda, target_cloud_down, source_cloud_down, result_cloud);

    std::cout << "--- fast vgicp cuda (gpu_rbf_kernel) ---" << std::endl;
    // use RBF-kernel-based covariance estimation
    // extremely fast but maybe a bit inaccurate
    fast_vgicp_cuda.setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
    // kernel width (and distance threshold) need to be tuned
    fast_vgicp_cuda.setKernelWidth(0.5);
    align(fast_vgicp_cuda, target_cloud_down, source_cloud_down, result_cloud);
    #endif

    // visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    colorize(*target_cloud_down, *tgt_colored, {0, 0, 255});
    colorize(*source_cloud_down, *src_colored, {255, 0, 0});
    colorize(*result_cloud, *est_colored, {255, 255, 255});

    pcl::visualization::PCLVisualizer viewer1("Cloud Viewer");
    viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_blue");
    viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
    viewer1.addPointCloud<pcl::PointXYZRGB>(est_colored, "est_white");
    while (!viewer1.wasStopped()) {
        viewer1.spinOnce();
    }
}
