# CSE180-Robotics-Final
CSE180 Robotics Labs using ROS (indigo) and Gazebo - Spring 2017.

I wrote a set of topics and subscribers to control a husky in a random Gazebo map full of obstacles and targets. 

## Objectives
1. Explore entire map in a reasonable amount of time
2. Avoid obstacles
3. Detect vertical posts, which came in pairs and were two meters high and one meter apart

We were able to use any open source packages, topics, or navigation stacks to meet these objectives.

## Overview
I implemented a pseudo-random walk that used the laser scanner of the husky to pick the direction which it should walk in next.
Essentially, I randomly picked the minimum distances from the array of distances the laser scanner topic returned and used that as the direction
the husky would turn and move towards. The minimum distance being the furthest obstacle detected by the laser. This algorithm allowed
the husky to visit each corner of the map rapidly and consistently. It also had the positive side effect of preventing the husky from getting stuck in areas with multiple obstacles. However, due to the use of the laser scanner as the only navigation system and the bias towards distant open spaces the husky took more time to explore parts of the map that were enclosed or obfuscated by obstacles. 

I attempted to do post detection in variety of ways, from using trigonometry, actual width of the post, and distance to find the expected width of the post
and comparing that to the ranges detected by the laser to using a point cloud topic and looking for two similar objects one meter apart.
Eventually, I settled on using the ranges from the laser scanner and the profile a post in the ranges array for detection. Through
trial and error, we figured out that a post X distance from the husky will have the Y number of ranges with a delta of Z in the laser
scanner's ranges array. Using this information, we were able to detect individual posts because the posts always came in pairs, we
were able to summarize the existence of the second post by detecting the first post. I chose this method because of its simplicity and speed.

## Contributions
I built and designed the entire thing - the two other members of my team were responsible for testing it.

## Score - 90/100
The professor was incredulous but delighted that we accomplished all of this with just the laser scanner. Unfortunately, due to the
use of the random walk, he believed the husky would eventually explore the entire map but this was not guaranteed to happen in a reasonable amount of time every time.

