# Purposed Algorithm Pipeline

[@DanielXu Note to Self Review]

## Class Structure

- Create skeleton Class structure
  - How classes interacts? Use composition for class interaction. Probably use inheritance for different levels of pure algorithmic classes. i.e. My algo class -> base AFP class
  - Determine public interface/fields
  - Need runner thread? Planning to use runner thread. Will see how it goes
gi
## Stages

### Collect & Evaluation

- Process lidar data:
  - Want sector data (e.g. per 1 degree) OR raw data? Sector data. But this would depends on the precision/range we can get from lidar raw data and the accuracy needed from the algorithm
  - Need speed correction? Yes. Should be easy
  - Need orientation correction? ^same as above
  - Unified unit for data? I would suggest using metic standard (i.e. m)
  - data cleansing (Not needed if getting sector data from lidar interface.)
  - intermediate data storage
- Do you need CNC data? (e.g. vehicle speed, altitude) Yes
- What's the precondition for this algorithm to run? location, speed, lidar scan data

### Planning

- Pure algorithmic stage
  - Add summaries of major steps here. Multiple steps from basic algorithm to algorithms that fits our need better (i.e. improved algorithm)
    - Basic form: <https://www.sciencedirect.com/science/article/abs/pii/S0921889012000838>
    - An improved FGM can be found at <https://www.computer.org/csdl/proceedings-article/irc/2017/07926547/12OmNAXxXdZ.> Specifically, this method provides angular and linear velocity in addition to heading angle given by FGM 
    - Another improved FGM is presented at <https://ieeexplore.ieee.org/abstract/document/8014220.> This method eliminates two drawbacks from the original FGM: extension of the path which sometimes happens unnecessarily, and small differences between the gap sizes
  - Can those steps be developed at the same time? (e.g. maybe fake out some data using unittest) Probably no
  - Covert algo. results to instruction for drone
    - Possible PID utility
    - Perhaps using gain/weight?
    - Algorithm specific
- Generate Command Queue:
  - List commands you need (e.g. Something simple like: goto relative position, change altitude. Something complex like: wait until arrive at) (go to relative/absolute position, change altitude, heading angle)
  - Will planned command queue have Delay commands? The only use case I can think of for now to use Delay command would be when flying directly to target position(when there're no obstacles)
- Generate Data Queue:
  - Rules for confidence (closest obstacle distance(physical restriction of lidar), speed(timing), margin of error for GAP)
  - Any additional data you want OAC to know? (e.g. in the future: display a msg on debugging overlay) Probably in future
- Feed results back to OAController

### Execution

- Handled by OAC
- Do you need to monitor the execution? Not needed for proof of conception
