Function:
This function will be used to measure and help maintain a distance of 3ft from the iRobot to the blind person 

Execution:
Ultimately I want this function to be run on start of the navigation program, I want the robot to:
1. drive forward three ft
2. turn around
3. Take a referance image of the person to aquire a known distance
4. Pull the person distance periodically during the run

Features:
- Measure the distance of a person
- Output focal length of camera based on reference image
- Identify when the camera is pointed directly at a person, by identifying the minimum distance of the person from the camera as the camera rotates
