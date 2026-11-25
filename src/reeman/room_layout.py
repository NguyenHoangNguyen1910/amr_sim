"""Fixed obstacles added to the 8020 room and its navigation map."""

PARTITION = (0.34, 0.49, 0.57, 1)
RACK = (0.24, 0.39, 0.44, 1)
CRATE = (0.69, 0.47, 0.27, 1)

# The center-west area stays open for the saved route, charging dock and
# depth-camera test. The new aisles loop around the north and south islands.
ROOM_OBSTACLES = (
    ("west_long_partition", (0.18, 2.50, 1.02), (-3.70, 1.43, .51), PARTITION),
    ("west_return_partition", (1.80, 0.18, 1.02), (-2.89, 2.59, .51), PARTITION),
    ("west_inner_partition", (0.18, 1.48, 1.02), (-2.53, 0.90, .51), PARTITION),
    ("north_cross_partition", (1.82, 0.25, 0.92), (0.03, 2.51, .46), PARTITION),
    ("north_short_partition", (0.22, 0.77, 0.92), (0.84, 2.77, .46), PARTITION),
    ("east_long_rack", (0.51, 2.82, 1.16), (2.68, 1.22, .58), RACK),
    ("east_short_rack", (0.40, 1.71, 1.16), (4.40, 1.41, .58), RACK),
    ("south_west_rack", (2.48, 0.36, 0.84), (-3.18, -2.04, .42), RACK),
    ("south_west_crates", (1.31, 0.44, 0.58), (-2.17, -3.23, .29), CRATE),
    ("south_long_partition", (0.20, 2.19, 1.02), (1.02, -2.36, .51), PARTITION),
    ("south_return_partition", (1.60, 0.20, 1.02), (1.72, -3.36, .51), PARTITION),
    ("south_east_crates", (0.61, 0.54, 0.72), (3.71, -2.78, .36), CRATE),
)
