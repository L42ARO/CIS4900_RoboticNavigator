## Acomplishments
- Created documentation for the robot hardware
- Read through GraphSLAM paper and github resources
- Created hypothetical system design

## Overview
So throughout week 3 and 4 by helping to build the robot I started to create some documentation on the different sensors and components it can be found [here](https://github.com/L42ARO/CIS4900_RoboticNavigator/wiki/Robot-Hardware-Documentation), it has served me as reference for the limits of the hardware I will be using.
Throughout week 5 I mostly focused on getting down a system design, however I felt I needed to understand GraphSLAM, my chosen SLAM approach, before I went ahead and designed a potential system, therefore I read through [the original Stanford paper on GraphSLAM](http://robots.stanford.edu/papers/thrun.graphslam.pdf) and it had some math and concepts which were hard to grasp therefore it took me a lot of time to grasp, and I had to take some very thorough notes and reference some videos and Github repositories to understand better the concepts, but I think I finally got a pretty good grasp on it. [Here is the PDF with my notes and thought process](https://github.com/L42ARO/CIS4900_RoboticNavigator/blob/main/literature_review/GRAPH%20Slam%20Notes.pdf). But I feel I finally have a good understanding of GraphSLAM being an algorithm that I need to input mainly Landmark data and Robot Controls data and I can expect to get out of it the Estimated Robot Trajectory data and Landmark Location Data,([again here are my notes with more details on my understanding](https://github.com/L42ARO/CIS4900_RoboticNavigator/blob/main/literature_review/GRAPH%20Slam%20Notes.pdf)).
Based on that understanding I created this system design based on the robot hardware available and also some of the algorithms I found on my previous report that I think is capable of providing the GraphSLAM algorithm with the inputs needed to Compute a Robot's location in a real world hallway, this system design is still lacking a navigation algorithm, but given the complexity of understanding the GraphSLAM algorithm I decided to first implement this and work on the navigation algorithm from there:
![image](https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/dd030651-f289-478f-b2bc-6bcad612bc25)


## Plans for next week
- Write the GraphSLAM algorithm in actual code ([good repo for reference](https://github.com/JeffLIrion/python-graphslam))
- Test out system design by modifying the SLAM algorithm used in this basic python robot simulation [this tutorial](https://www.youtube.com/watch?v=2GJuEIh4xGo&list=PL9RPomGb9IpRJLw5UTdSy4eJeoLrwNcfC&index=1)
- Hopefully get access to the robot and start setting it up