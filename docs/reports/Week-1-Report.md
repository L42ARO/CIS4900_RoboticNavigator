## Accomplishments This Week
- Successfully obtained approval for the class from the CS department.
- Established a Github repository for storing project files.
- Expanded the repository to also serve for project planning and documentation (like weekly reports).
- Reviewed slides from the `Control Of Mobile Robots` class, focusing on localization, trilateration, particle filters, and mapping.
- Revisited the paper *Trilateration-Based Robot Localization with Learned Visual Landmarks* and compared its content with trilateration slides.

## Overview
In this initial week of classes, my primary focus was on reviewing relevant slides to initiate the formulation of ideas. I must concede this task didn't take too much time to perform, I was honestly more worried in solving the issue of getting the permit for the class, so next week I must make up with a heavier workload for literature review.
However, I do consider that setting up the project environment on GitHub will be of great help moving forward, as stated I leveraged some github automations and features to make it possible to set it up as a project management tool as well as a wiki to keep track of documentation.
### Project plan
>![image](https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/a2d848a8-7b79-430b-b4bd-17c103e525a5)



## Plans for Next Week
1. Conduct a comprehensive review of `navigation planning` from the `Control of Mobile Robots` class slides
2. Explore various algorithms related to Localization, Mapping, and Navigation, extending beyond the topics covered in the `Control Of Mobile Robots` class.
3. Document new findings and select a set of algorithms for the upcoming milestone of `Algorithm Implementation.`
   
## Some Notes
- Right now I'm considering a preference for the familiar algorithms learned in class to expedite testing, like the `Monte Carlo` implementation for `Particle Filter` for which I already have some python code from last semester:
>```
>for i in range(len(sensor_readings)):
>   print("Iteration: ", i)
>   particle_model, raw = motion_model(maze[0], particle_model, fwd_prob=0.2, stay_prob=0.2)
>        
>   sense_model = sensor_model(maze[0], sensor_readings[i])
>   sense_model = normalize(sense_model)
>        
>   importance_factor = get_weights(particle_model, sense_model)
>   importance_factor = normalize(importance_factor)
>        
>   resampled_model = resample(importance_factor, TOTAL_PARTICLES)
>   particle_model = resampled_model
>   print(print_arr(resampled_model))
>   time.sleep(0.5)
>```
- Intend to consult with Dr. Alfredo to coordinate access to robots and what exact sensors will I have available to better determine the chosen algorithms.

