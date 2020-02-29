# Purposed Algorithm Pipeline:

[@shibohan Note to Self Review]

## Class Structure:
- Create skeleton Class structure
  - How classes interacts? Use composition. Probably inheritance for pure algorithmic class. i.e. My algo class -> base AFP class 
  - [Add Your Decision @DanielXu] Determine public interface/fields
  - Need runner thread? Planning to use runner thread. Will see how it goes

## Stages:
### Collect & Evaluation
- Process lidar data:
  - Want sector data (e.g. per 1 degree) OR raw data? Sector data [Would the algo require more info than this?]
  - Need speed correction? Yes. Should be easy
  - Need orientation correction? ^same as above
  - Unified unit for data? I would suggest using metic standard (i.e. m)
  - data cleansing (Not needed if getting sector data from lidar interface ?)
  - intermediate data storage
- Do you need CNC data? (e.g. vehicle speed, altitude) Yes
- What's the precondition for this algorithm to run? location, speed, lidar scan data

### Planning
- Pure algorithmic stage
  - Add summaries of major steps here. Multiple steps from basic algorithm to algorithms that fits our need better (i.e. improved algorithm)
  - Can those steps be developed at the same time? (e.g. maybe fake out some data using unittest) Probably no
  - Covert algo. results to instruction for drone
    - Possible PID utility
    - Perhaps using gain/weight?
    - Algorithm specific
- Generate Command Queue:
  - List commands you need (e.g. Something simple like: goto relative position, change altitude. Something complex like: wait until arrive at) (go to relative/absolute position, change altitude, heading angle?)
  - Will planned command queue have Delay commands? The only use case for Delay command would be helpful in this algorithm is fly directly to target position(when there're no obstacles) ?
- Generate Data Queue:
  - Rules for confidence (closest obstacle distance(physical restriction of lidar), speed(timing), margin of error for GAP)
  - Any additional data you want OAC to know? (e.g. in the future: display a msg on debugging overlay) Probably in future
- Feed results back to OAController

### Execution
- Handled by OAC
- Do you need to monitor the execution? Not needed for proof of conception
