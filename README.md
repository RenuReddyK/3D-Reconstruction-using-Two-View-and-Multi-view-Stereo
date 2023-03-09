# 3D-Reconstruction-using-Two-View-and-Multi-view-Stereo

# Using Optical Flow the point correspondences and depth are estimated.

More about this is mentioned [here](https://github.com/vuejs/vue)

# 3D Reconstruction from two 2D images using 2 view SFM:

More about this is mentioned [here](https://github.com/vuejs/vue)

|![image](https://user-images.githubusercontent.com/68454938/224098587-b792b27a-4304-4aac-b062-7d6c8ff67b65.png)
|*Left and Right view of temple model*|

First, computing the right to left transformation and the baseline B. Then, the rectification rotation matrix is computed to transform the coordinates in the left view to the rectified coordinate frame of left view. Since, the images in the dataset are rotated clockwise and the epipoles are placed at y-infinity. Then homography is computed and the image is warped. Zero padding at the border is used and then each left image patch and each right image patch are compared using the SSD kernel, SAD kernel and ZNCC kernel.

![image](https://user-images.githubusercontent.com/68454938/224098917-2a9fb1f9-dd0e-493d-913b-d7b5a5636226.png)
Input Vs Rectified views

L-R consistency: The best matched right patch for each left patch is found.

![image](https://user-images.githubusercontent.com/68454938/224099005-5a7282c8-9c8f-4f89-b564-20cc2ac093b5.png)
L-R consistency check mask

The following disparity map is computed:
![image](https://user-images.githubusercontent.com/68454938/224099081-07de3c82-b02b-4749-b284-c02b0158584a.png)
Cost Vs Disparity of one left pixel and cost map of one left horizon col

Using the disparity map the below Depth map and the back-projected Point Cloud are generated.
![image](https://user-images.githubusercontent.com/68454938/224099250-0a22de82-417f-4e64-a119-7142802fc1bc.png)
Image, Raw disparity, Raw depth

![image](https://user-images.githubusercontent.com/68454938/224099298-d7b18e9d-ab86-4d0d-97ed-8b9f3a2badfd.png)
Image, Post-processed disparity, Post-processed depth

Reconstruction using SSD:
![image](https://user-images.githubusercontent.com/68454938/224099407-c5bce663-b00f-4121-8972-16f4de238192.png)
SSD Reconstruction

Reconstruction using SAD:
![image](https://user-images.githubusercontent.com/68454938/224099511-1b0b74c3-d0eb-46dc-9baf-b5fd5957f547.png)
SAD Reconstruction

Reconstruction using ZNCC kernel:
![image](https://user-images.githubusercontent.com/68454938/224099585-a3432a89-c66c-4f74-9c84-eeaef1096224.png)
ZNCC Reconstruction

Multi-pair aggregation: full reconstructed point cloud of the temple:
![image](https://user-images.githubusercontent.com/68454938/224099685-cd5477e0-e262-44ef-a5e3-33fb8598e867.png)
Full Reconstruction

# Reconstructing the 3D model from multi view Stereo
![image](https://user-images.githubusercontent.com/68454938/224099816-c6848859-1cfb-499e-9484-8d79a84bfab1.png)
Reference view

![image](https://user-images.githubusercontent.com/68454938/224099871-084ab23c-1373-4774-bba1-80498fa0b91f.png)
Neighbouring views

Using 5 different views, choosing the middle-most view as the reference view, the 3D model is reconstructed.  The main idea is that a series of imaginary depth planes are swept across a range of candidate depths and the neighboring views are projected onto the imaginary depth plane and back onto the reference view via a computed collineation.

Cost map: The cost/similarity map is computed between the reference view and the warped neighboring view by taking patches centered around each pixel in the images and by computing similarity for every pixel location. At patches with the correct candidate depth, the similarity will be high, and the similarity will be low for incorrect depths. 

Cost volume: For each depth plane, the above steps of computing a cost map between the reference view and each of the 4 neighboring views and sum the results of each of the 4 pairs to aggregate into a single cost map per depth plane are repeated. By stacking each resulting cost map along the depth axis, the cost volume is produced.

https://vimeo.com/787164708

Depth map: The depth map is extracted from the cost volume ,by choosing a depth candidate and by taking the argmax of the costs across the depths at each pixel.
<img width="572" alt="image" src="https://user-images.githubusercontent.com/68454938/224100070-f77f3083-293c-404d-9eeb-4c619c775944.png">
Reference View Vs Raw Depth map

Point cloud: The pointcloud is obtained from the computed depth map, by backprojecting the reference image pixels to their corresponding depths, which produces a series of 3D coordinates with respect to the camera frame. 

Post-processing: These coordinates are then expressed with respect to the world frame instead of the camera frame. Post-processing is done to get a colorized pointcloud from the depth map by filtering the black background and any potential noisy outlier points.

<img width="547" alt="image" src="https://user-images.githubusercontent.com/68454938/224100184-12c378bd-4db5-49db-8ed3-28ba92bc2a8e.png">
Post processing

<img width="479" alt="image" src="https://user-images.githubusercontent.com/68454938/224100222-4e5b5ac5-00c5-4aa5-b0f5-0c94d795d432.png">
Full 3D Reconstruction

