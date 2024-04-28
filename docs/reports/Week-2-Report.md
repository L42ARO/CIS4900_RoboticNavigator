# Accomplishments This Week

- Explored and tested the Kornia library with the feature matching model (`LoFTR`) for landmark recognition needs. [Kornia Documentation](https://kornia.readthedocs.io/en/latest/applications/image_matching.html)
- Investigated SLAM (Simultaneous Localization and Mapping) with a focus on Graph-based and EKF (Extended Kalman Filter) approaches.
- Identified a suitable algorithm for mapping with LIDAR sensors that is compatible with SLAM EKF.
- Explored a research paper on feature extraction from raw LIDAR data and found a promising algorithm.

# Overview
## Landmark recognition Library
This week was dedicated to researching real-world implementations for localization, mapping, and landmark recognition beyond the scope of our class materials. One significant finding was the Kornia library, an open-source extension of OpenCV, with a feature matching model called `LoFTR`. Tests with unique and messy environment images showed promising results, though its actual use will depend on hardware and camera quality.

IMAGE SIZE | LATENCY | RESULT
--- | --- | --- |
600x375 px | 20 seconds | <img src="https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/d0414b0c-893d-4fe5-8348-056caa74ad4d" height="200" width="400">
300x375 px | 7 seconds | <img src="https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/6afb90e1-08ca-491d-b8ae-acdc9b845b45" width="400">

It also demonstrates low matches when it's an incorrect image

<img src="https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/8419ce35-bcac-45d2-8bb7-6c4dcc0f99d6" width="300">


## Localization and Mapping

Exploring real world implementations of Localization and Mapping directly led to SLAM, from my findings I got to know that the main types of backend used for slam are `Pose Graph`, `Particle Fitler` and `Extended Kalman Filter`. Given that it was suggested that `Graph based` systems are the preferred approach I attempted to gain a better understanding of them through these videos:
- [Understanding SLAM Using Pose Graph Optmization](https://www.youtube.com/watch?v=saVZtgPyyJQ)
- [Graph Based SLAM](https://www.youtube.com/watch?v=mZBdPgBtrCM)

I consider Graph Based SLAM to be maybe too advanced right now, therfore I pivoted to looking into the `Extended Kalman Filter` type of SLAM and my research led me to this other [Tutorial Series](https://www.youtube.com/watch?v=6mivXP3rAfg&list=PL9RPomGb9IpRJLw5UTdSy4eJeoLrwNcfC&index=3). These videos show an actual program implementation of `EKF` SLAM. Through these videos I managed to replicate mapping algorithm on my computer using a *simulated LIDAR* therefore I now have a promising algorithm for potential use in the project if the robotic platform that I will use has LIDAR:
```python
class buildEnvironemnt:
    ...

    def AD2Pos(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = -distance * math.sin(angle) + robotPosition[1]
        return (int(x), int(y))
    
    def dataStorage(self, data):
        # print(len(self.pointCloud))
        for element in data:
            point = self.AD2Pos(element[0], element[1], element[2])
            if point not in self.pointCloud:
                self.pointCloud.append(point)
    def show_sensorData(self):
        self.infomap = self.map.copy()
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]), int(point[1])), self.Red)
```


https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/bc3f0d51-89ed-49f6-993e-14fe10e6edb8


I additionally found a [research paper on feature extraction from raw LIDAR data](https://journals.sagepub.com/doi/full/10.1177/1729881418755245). I liked this paper since it was very thorough and provided an algorithm similar to the way algorithms were provided in the *Control of mobile robots class* last semester, so I feel confident I can replicate it and have both a mapping and feature extraction system using LIDAR, which is of course a necessary component so that our localization doesn't soley rely on the landmark recognition.

# Plans for Next Week

- Assist Chance in building the robots to determine hardware choices and tool feasibility.
- Based on the selected hardware, develop a system design incorporating identified algorithms and libraries.
- Create a task list for coding, aiming to start programming as soon as possible, contingent on hardware build completion.

# Some Notes

- The use of Kornia's feature matching model is promising, pending consideration of specific computing hardware and camera quality.
- Graph-based SLAM systems require further exploration, particularly understanding their practical implementation in robotics.
- The combination of EKF SLAM mapping and probability grid approach may result in a powerful mapping solution.
- The research paper on feature extraction from raw LIDAR data offers a potential backup localization system.