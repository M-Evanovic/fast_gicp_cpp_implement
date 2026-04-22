## Statement
Implement of pcl gicp, pcl ndt, fast_gicp single thread, fast_gicp multi threads, fast_vgicp single thread, fast_vgicp multi threads and their CUDA version.

## Installation
```
git clone https://github.com/M-Evanovic/fast_gicp_cpp.git  
cd path/to/fast_gicp_cpp  
```

## Preparation
Put your target cloud and source cloud to ```/data``` and then compile.
```
mkdir build && cd build  
cmake ..  
make  
```

## Start
```
./gicp_align target_pcd_path source_pcd_path det_range(optional) voxel_size(optional) max_correspondence_distance(optional)  
```
example
```
./gicp_align ./data/map.pcd ./data/scan.pcd 30 0.3 3
```

## Acknowledgments
Thank the authors of [fast-gicp](https://github.com/koide3/fast_gicp) for open-sourcing their outstanding works.
