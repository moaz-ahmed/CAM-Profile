# cam-profile-generator

This a python program that I used to create the cam profile of a double dwell cam for a uni project. It produces the cam coordinates and normalized variabeles $S,V,A,J$ values to an `xlsx` file. These data can be later stripped into only the $x,y$ coordinates and imported into any CAD software. 
  
## Requirements 

- Pandas
- Numpy
- matplotlib (if you want plots like this one)

<img style="margin-left: auto;margin-right: auto;width: 50%;" src="https://github.com/moaz-ahmed/CAM-Profile/blob/main/cam_profile.png" alt="cam profile" width="400"/>


## Objective 

- Design the motion curves of a double-dwell cam to move a roller follower 50 mm.
- Choose suitable parameters to minimize velocities for both rise and fall.
- The total cycle must take 4 seconds.

### Follower motion:

- Rise: 60 degrees
- Dwell: 120 degrees
- Fall: 30 degrees
- Dwell: 150 degrees 
