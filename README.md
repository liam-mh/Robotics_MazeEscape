# Robotics_MazeEscape
Robotics 55-608216-AF-20245   
Assessment One

C1022456  
Liam Hammond

## Task 
Use the VEX VR robot platform to perform a simulated mouse escaping a random maze.
- Escape maze
- Find the quickest route
- Map the maze
- Return back to home

## Solution
### Algorithm
I have chosen to use the Trémaux's maze-solving algorithm to achieve the tasks at hand. I have chosen this method as it enables a thorough search of the maze with a visual mapping aid.
### Maze mapping
My solution consists of virutally mapping the maze in a 2D array. The robot carries out a full 360 scan of each cell it is in the record where the walls are. This is then paired with virtual markers that have been dropped, to determine a direction to travel.

## Demonstration
A video to showcase the code base and how it works. As well as examples of the robot completing all tasks in random mazes.

[Robot maze-solving demonstration video]()

## Resources Used
- [Maze-solving algortihms wikipedia](https://en.wikipedia.org/wiki/Maze-solving_algorithm)
- [Trémaux's algorithm demonstration video](https://www.youtube.com/watch?v=RjWSlz-aEr8)
