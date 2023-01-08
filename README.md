# Trajectory-made-by-a-vehicle-with-a-Visual-odometry-approach-

## Visual Odometry

In this part we will use the transformations provided by the Kitti dataset : https://www.cvlibs.net/datasets/kitti/

<p>
  <img src="/Result/KITTI_ENVIREMNT.PNG" width="1000" />
</p>

Our objective is to calculate the trajectory made by the vehicle with a visual odometry approach.</br> 
We will use the stereo image sequence provided by Kitti to calculate the odometry. Then, we will compare the calculated trajectory with the data
provided by the IMU.

<p>
  <img src="/Result/KITTI_POV_camera2.gif" width="1000" />
</p>

Using the simple notions of projective geometry , we can easily calculate the relative pose for each of the images in the sequence (matching →matrix fundamentally →essential →rotation and translation).
</br></br>The problem is that the translation is calculated with an unknown scale, but thanks to the known configurations of the stereo system (fixed relative position
between the stereo pairs), it is possible to find this scale.

#### The tasks to be accomplished:

- Start with two consecutive stereo pairs (4 images).</br>
- Determine the pairs of images where it is necessary to calculate the relative position (according to the proposed method).</br>
- For each pair, calculate the fundamental matrix.</br>
- Compute the essential matrix.</br>
- Extract the rotation and the translation (scale ambiguity) with the recoverPose function.</br>
- Compute the scale of the translation.</br>
- Make a loop to compute the complete trajectory (50 couple of images).</br>
- Display the computed trajectory with the trajectory provided by the IMU on the same figure,</br>
- compare.</br>

<p>
  <img src="/Result/methode.PNG" width="1000" />
</p>

## Result : 
<p>
  <img src="/Result/Figure_1.png" width="1000" />
</p>


