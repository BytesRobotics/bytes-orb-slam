//
// Created by michael on 4/13/20.
//

#ifndef BR_SLAM_MAP_H
#define BR_SLAM_MAP_H
// ROS
// OpenCV
// Other


class Map{
public:
    /**
     * This creates a new map object, maps are individually finite based on a storage capacity with # of keyframes
     * Other parameters available such as
     * @param map_size this is the maximum number of frames that a map can contain, the tracking node will be responsible
     * for removing extraneous keyframes and routinely running the cleanup routines
     */
    Map(int map_size);

    /// Main debug tool for observing maps
    void publish_pointcloud();

private:

};

#endif //BR_SLAM_MAP_H
