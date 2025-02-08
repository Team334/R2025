# Purpose of wristevator pathfinder
In desmos you make the configuration space then for any edges that went through the obstacle space, you remember those
and append them to the end of the csv file. The csv file contains all verticies and their coordinates in the configuration space, along
with the appended edges at the end. Run the python script, and enter the path you want to pathfind and find the shortest path for. The
shortest path is found, and displayed on the weighted graph.

This is needed to find the best intermediate setpoint for certain wristevator movements. The shortest path can be saved as a map in robot code.
