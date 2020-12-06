/* Copyright (C) 2019 DroneOA Group - All Rights Reserved
 * This file is part of DroneOA_ROS.
 *
 * DroneOA_ROS is free software: you can redistribute it and/or 
 * modify it under the terms of the GNU Affero General Public License
 * as published by the Free Software Foundation.
 *
 * DroneOA_ROS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with DroneOA_ROS. 
 * If not, see <https://www.gnu.org/licenses/>.
 *
 * Written by Bohan Shi, August 2019
 */

#include <droneoa_ros/OAC/OAAlgFGM.hpp>
#include <droneoa_ros/OAC/OAC.hpp>

namespace OAC {

class Gap {
 private:
    // TODO(yxu): change how center angle is calculated, currently using FGM-basic
    double angle_l;  // Angle for the left side of the gap in degrees
    double angle_r;  // Angle for the right side of the gap in degrees

    double distance;  // Average distance to this obstacle in meters

    double size;  // Size of gap in degrees

 public:
    Gap(double angle_l, double angle_r, double distance, double size) :
        angle_l(angle_l), angle_r(angle_r), distance(distance), size(size) {}

    double getSize() const {
        return size;
    }

    double getGapCenter() const {
        return (angle_l + angle_r) / 2;
    }
};

class gapComparator {
 public:
    int operator()(const Gap &g1, const Gap &g2) {
        return g1.getSize() < g2.getSize();
    }
};

class Gap_array{
 private:
    std::priority_queue<Gap, std::vector<Gap>, gapComparator> gaps;

 public:
    const double UNOBSTRUCTED_DISTANCE = 5.0f;  // defines what the algorithm considers "gap"

    // Assuming lidar data is 180 degrees. 0 degree start from left side.
    explicit Gap_array(std::vector<double> lidar_data) {
        int gap_size = 0;
        double distance_sum = 0;
        for (unsigned i = 0; i < lidar_data.size(); i++) {
            // Treating void spaces with more than 5m of unobstructed distance as gap
            if (lidar_data.at(i) > UNOBSTRUCTED_DISTANCE) {
                gap_size++;
                distance_sum += lidar_data.at(i);
            } else if (gap_size > 0) {
                gaps.push(Gap(i - gap_size, i, distance_sum/gap_size, gap_size));
                gap_size = 0;
                distance_sum = 0;
            }

            // process the last degree
            if (i == lidar_data.size()-1 && gap_size > 0) {
                gaps.push(Gap(i - gap_size, i, distance_sum/gap_size, gap_size));
            }
        }
    }

    Gap getMaxGap() {
        if (gaps.size() > 0) {
            return gaps.top();
        }

        return Gap(0, 0, 0, 0);
    }
};

OAAlgFGM::OAAlgFGM(CNC::CNCInterface *cnc, Lidar::LidarGeneric *lidar) : BaseAlg(cnc) {
    init(lidar);
}

void OAAlgFGM::init(Lidar::LidarGeneric *lidar) {
    mpLidar = lidar;
}

OAAlgFGM::~OAAlgFGM() {
    CMDQueue_.clear();
}

bool OAAlgFGM::collect() {
    // Collect data directly from interface as required
    //! @todo WIP
    if (!mpCNC) {
        ROS_ERROR("[OAAlgFGM] Missing CNC pointer !!!");
        return false;
    }
    if (!mpLidar) {
        ROS_ERROR("[OAAlgFGM] Missing LIDAR pointer !!!");
        return false;
    }
    //! @todo remain false until implemented
    return false;
}

bool OAAlgFGM::plan() {
    CMDQueue_.clear();
    DATAQueue_.clear();
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_ALG_NAME, SYS_Algs_STR[SYS_Algs::ALG_FGM]));
    DATAQueue_.push_back(Command::DataLine(Command::DATA_QUEUE_TYPES::DATA_CONFIDENCE, std::to_string(0.0f)));
    //! @todo remain false until implemented
    return false;
}

}  // namespace OAC
