# A scalar grid over X–Z with per-cell density in [0,1] or NaN when unknown.
std_msgs/Header header     # header.frame_id is the grid frame (e.g., base_link)
float32 cell_size
float32 origin_x           # x of cell (0,0) center or lower-left; see indexing note below
float32 origin_z           # z of cell (0,0)
float32 min_y              # vertical column start (for column-density mode)
float32 max_y              # vertical column end
uint32 width               # number of cells along +X
uint32 height              # number of cells along +Z

# Row-major: index = i*height + j, where i in [0,width), j in [0,height)
# i increases with +X, j increases with +Z. NaN means unknown/insufficient data.
float32[] data

# Optional diagnostics (parallel layout to data)
uint32[] point_counts         # raw points that contributed to the cell (after filtering)
uint16[] occupied_voxels      # number of occupied vertical voxels (column-density mode)
uint16   num_vertical_voxels  # vertical voxel count used for column-density mode, 0 if N/A
