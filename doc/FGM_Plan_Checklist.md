# Purposed Algorithm Pipeline

[@DanielXu Note to Self Review]

## Class Structure

- Create skeleton Class structure
  - Class interaction: Use composition for class interaction. Probably use inheritance for different levels of pure algorithmic classes. i.e. My algo class -> base AFP class
  - Public interface/fields [WIP]
  - Runner thread: Planning to use runner thread. Will see how it goes

## Stages

### Collect & Evaluation

- Process lidar data:
  - Sector data v. raw data: Sector data would be prefered. But this would depends on the precision/range we can get from lidar raw data and the accuracy needed from the algorithm
  - Speed correction(data inaccuracies caused by speed of drone): Yes. Speed correction should compensate the time difference between when distance is measured and when data is relayed to this algorithm. There are two types of correction that I can think of: 1) When our lidar are doing scanning, 1 cycle of scan from 0 degree to 360 degree would give `speed_of_drone/scanning_frequency` of error. 2) Latency of lidar data, `speed_of_drone*latency` is the error, a simple translation should be able to fix this
  - Orientation correction(data inaccuracies caused by angular speed of drone): ^same as above (I'm not expecting high level of accuracy, use models as simple as possible to do data correction)
  - Unit for data: Using metic standard (i.e. m) would be helpful
  - Data cleansing: (Not needed if getting sector data from lidar interface.)
  - Intermediate data storage
- CNC Data (e.g. vehicle speed, altitude): Yes. These may not be needed for initial implementaion, but further development with more sophisticated algorithm would need these.
- Preconditions location, speed, **lidar scan data**

### Planning

- Pure algorithmic stage
  - Major steps: Multiple steps from basic algorithm to algorithms that fits our need better (i.e. improved algorithm)
    - Basic form: <https://www.sciencedirect.com/science/article/abs/pii/S0921889012000838>
    - An improved FGM can be found at <https://www.computer.org/csdl/proceedings-article/irc/2017/07926547/12OmNAXxXdZ.> Specifically, this method provides angular and linear velocity in addition to heading angle given by FGM
    - Another improved FGM is presented at <https://ieeexplore.ieee.org/abstract/document/8014220.> This method eliminates two drawbacks from the original FGM: extension of the path which sometimes happens unnecessarily, and small differences between the gap sizes
  - Possible parallel development: (e.g. maybe fake out some data using unittest) Probably no
  - Convert algo. results to instruction for drone
    - Possible PID utility
    - Perhaps using gain/weight?
    - Algorithm specific
- Generate Command Queue:
  - Commands required: (e.g. Something simple like: goto relative position, change altitude. Something complex like: wait until arrive at) (go to relative/absolute position, change altitude, heading angle)
  - Possible delay commands: The only use case I can think of for now to use Delay command would be when flying directly to target position(when there're no obstacles)
- Generate Data Queue:
  - Rules for confidence: closest obstacle distance(physical restriction of lidar), speed(timing), margin of error for GAP
  - Additional data for OAC: (e.g. in the future: display a msg on debugging overlay) Probably in future
- Feed results back to OAController

### Execution

- Handled by OAC
- Do you need to monitor the execution? Not needed for proof of conception
