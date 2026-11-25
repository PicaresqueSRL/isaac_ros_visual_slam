# Statistics of one input image of a frame, computed on a subsample of the pixels
# (parameter diagnostics_pixel_stride).

string name          # camera name of the profile (e.g. cam2_ir1), or "depth"
string topic
uint32 width
uint32 height

# Intensity images (8 or 16 bit; 16-bit values are scaled to 0-255): mean brightness and
# share of saturated (>= 250) and dark (<= 5) pixels, in percent. NaN for depth images.
float32 mean_intensity
float32 saturated_pct
float32 dark_pct

# Depth images: share of pixels with a valid (non-zero, finite) depth, in percent. NaN for
# intensity images.
float32 valid_depth_pct
