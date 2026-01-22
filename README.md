
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
./gicp_align target_pcd_path source_pcd_path det_range(optional) voxel_size(optional)  
```
example
```
./gicp_align ./data/map.pcd ./data/scan.pcd 50 0.5
```

## Acknowledgments
Thank the authors of [fast-gicp](https://github.com/koide3/fast_gicp) for open-sourcing their outstanding works.
