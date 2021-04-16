include the following:

o Darie Roman 1002902736
  Sebastian Gri 1003407985
  Yaakob Bendayan 1004206995
  Migjun Yang 1003317859
  Rohan Ramdoss 1003219233

o Run the commands mentioned in the contest3 instructions doc, in the 
order in which they appear. 

o Our training code is in emotionTraining.py

o Changes to existing code:
1. getter in Explore class to return robot pose from costmap_client_ (used for determining if robot has moved)
2. function in Explore class for clearing blacklist if there are still frontiers, but they are all blacklisted (sometimes a doorway to unexplored room gets blacklisted)
3. added param to allow move_base to travel through unknown areas (for when frontier centroid falls outside of known map)

o Other navigation improvements:
1. Move in small circle if robot pose change is below threshold for certain amount of time
2. If the scenario above happens 3 times consecutively, move forward in a straight line (to solve issue where circular frontier around robot has centroid close to robot pose)

