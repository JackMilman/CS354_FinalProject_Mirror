Current State of Code: 

What Worked and What Didn't:
Our robot was able to successfully navigate across the entire course. By generating random goals within the accessible map space, and prioritizing those random goals that were nearer to itself, the robot was able to perform navigation through the map and reach high coverage. A recurring issue, however, was that the navigation would occasionally miss a victim due to pure chance conspiring to make the robot never look at a victim and allow it to pass by without knowing it was there.

Returning to the starting position was highly successful, and we always returned within the time limit. We simply marked the first position-update we received as the "initial" pose and had the robot choose that as its final navigation goal when it reached the "timeout" (the time limit minus some constant, which we fiddled with during the competition) limit.

Taking close-up pictures of the victims was completed with relatively high accuracy. By choosing a point in front of the victim in question, we were able to transform that point into a reachable goal for the robot to take a picture from. These points were, however, sometimes inaccurate due to how we stored the goal position. Since we never updated the goal-position for an individual victim, we would occasionally have pictures from bad angles due to inaccuracy/noise in the camera's estimate of the victim position.

Victim location recording was performed with very high accuracy (barring phantom victims) by taking the positions reported by the camera and storing them in a list. These values would then be updated by further "duplicate" scans of the same victim by taking the average of the two reported locations and replacing the stored location with that average.