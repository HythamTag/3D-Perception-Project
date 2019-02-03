# Robotics Nanodegree Project 3 - 3D Perception

In this project we focus on 3D perception PR2 robot simulation in Gazebo. The robot use RGBD camera to capture both colors and depth of different objects

## Main View (No Filters)

The following image shows the original view without any filters

![1](https://user-images.githubusercontent.com/42402820/52170139-77f4ae80-274d-11e9-81df-089489b7fe28.jpeg)

## Outlier Removal Filter

Outlier filters can remove noise form the point cloud. The noise happens due to some dust in the environment of due to sunlight or some other environmental conditions. One of the filters used is PCL's StatisticalOutlierRemoval filter that computes the distance to each point's neighbors, then calculates a mean distance. Any points whose mean distances are outside a defined interval are removed.
Here is the cloud after performing the outlier removal filter.

![2](https://user-images.githubusercontent.com/42402820/52170198-b048bc80-274e-11e9-86bc-5f05147c0045.jpeg)


For the PR2 simulation, I found that a mean k value of 20, and a standard deviation threshold of 0.3 provided the optimal outlier filtering using the following function.



```python
def statistical_outlier(cloud , mean_k , threshold_scale_factor):
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(mean_k)

    # Set threshold scale factor
    x = threshold_scale_factor

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud = outlier_filter.filter()
    return cloud
```
`\
## Voxel Grid Filter

The cloud often has to many information that isnt required and will increase the processing time during analyzing so to avoid that we downsample the data by taking average for each 3d matrix of points (group of voxels). Ex taking average of 2 by 2 matrix = (2+1+4+1)/4 = 2 the new assigned point of value 2
Here is the cloud after performing the Voxel Grid Filter.

![3](https://user-images.githubusercontent.com/42402820/52170333-d40d0200-2750-11e9-9959-a2bd8e573861.jpeg)

I used an X, Y, Z voxel grid filter of leaf size 0.005. The value represents the voxels per cubic meter, thus increasing the number adds more detail and processing time. Using 0.005 was enoguh to leave enough detail while minimizing processing time, however, increasing the number would increase the model accuracy (increasing processing time).


```python
def vowel_grid(ros_msg , LEAF_SIZE):
    # Create a VoxelGrid filter object for our input point cloud
    vox = ros_msg.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    return cloud_filtered
```
## Passthrough Filter

3rd stage is PassThrough filter which is like a cropping tool that crops any given 3D point cloud by specifying an axis and two values along it to cut of all the remaining and this is called ROR or region of interest.

![4](https://user-images.githubusercontent.com/42402820/52170729-82687580-2758-11e9-83d0-18f80f02d76a.jpeg)

I cropped in X, Y and Z to focus on the table and its contents. in x from 0.34 to 1, in y from -0.5 to 0.5 and in z from 0.34 to 1 using the following function

``` python
def PassThrough_filter(cloud_filtered , axis , axis_min , axis_max):
    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()
    if axis == 'x':
    # Assign axis and range to the passthrough filter object.
		filter_axis = 'x'
		passthrough.set_filter_field_name(filter_axis)

		passthrough.set_filter_limits(axis_min, axis_max)
		cloud_filtered = passthrough.filter()   
		passthrough = cloud_filtered.make_passthrough_filter()

    elif axis == 'y':
    # Assign axis and range to the passthrough filter object.
		filter_axis = 'y'
		passthrough.set_filter_field_name(filter_axis)

		passthrough.set_filter_limits(axis_min, axis_max)
		cloud_filtered = passthrough.filter()   
		passthrough = cloud_filtered.make_passthrough_filter()

    elif axis == 'z':
    # Assign axis and range to the passthrough filter object.
		filter_axis = 'z'
		passthrough.set_filter_field_name(filter_axis)

		passthrough.set_filter_limits(axis_min, axis_max)
		cloud_filtered = passthrough.filter()   
		passthrough = cloud_filtered.make_passthrough_filter()

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    # Callback function for your Point Cloud Subscriber
    return cloud_filtered
```

## RANSAC Place Segmentation
It stands for Random Sample Consensus it is and algorithm used to identify dataset that belong to a model which are the objects while the other is the table.
The extracted objects look like this.

![5](https://user-images.githubusercontent.com/42402820/52170912-daed4200-275b-11e9-89c9-ef55901c62c8.jpeg)

```python
def RANSAC(cloud_filtered,max_distance):
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    return inliers, cloud_filtered

def extract_liers(inliers, cloud_filtered):
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    return extracted_inliers,extracted_outliers
```
The extracted table look like this:

![6](https://user-images.githubusercontent.com/42402820/52170923-0ff99480-275c-11e9-87e7-356152e5ec05.jpeg)

## Clustering for Segmentation

After the objects cloud has been clearly extracted we need to identify individual objects and this is performed by Euclidean clustering using 2 main algorithms 
- K-means
- DBSCAN  

In Euclidean clustering
- Each cluster has a well-­‐defined centroid    
- average across all the points in the cluster    
- Represent each cluster by its centroid    
- Distance between clusters = distance between centroids

``` python
def Euclidean(extracted_outliers, ClusterTolerance , MinClusterSize , MaxClusterSize):
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(ClusterTolerance)
    ec.set_MinClusterSize(MinClusterSize)
    ec.set_MaxClusterSize(MaxClusterSize)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
        #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    return color_cluster_point_list,cluster_indices,white_cloud
```

## K-means

K-means clustering algorithm is able to group data points into n groups based on their distance to randomly chosen centroids. However, K-means clustering requires that you know the number of groups to be clustered, which may not always be the case.

## DBSCAN

It stands for Density-based spatial cluster of applications with noise (DBSCAN) is a clustering algorithm that creates clusters by grouping data points that are within some threshold distance from their nearest neighbor.

in contrary to k-means DBSCAN don't need to know how many clusters to expect in the data. But ypu need to know something about the density of the data points being clustered.

Performing a DBSCAN need only x,y,z position and doesnt need the RGB so we first convert the XYZRGB point cloud to a XYZ point cloud.

assigning random colors to the objects give us this 3 different distinct objects with 3 different colors:

![7](https://user-images.githubusercontent.com/42402820/52171163-a039d880-2760-11e9-9fdb-61ed8b01edb8.jpeg)

## Object Recognition

Object recognition allow us to identify objects that is clustered and isolated so we take each object and try to identify it and this is done by training a model to learn how object looks like once the system had a model it can predict and identify the objects

## Capturing Object Features

This is done by color Histogram as it measures how each object looks like when captured as an image as the object is positioned in random orientation to teach the model how the object look like in different orientations But the image is converted first to HSV from RGB to be easier to deal with

## Training SVM Model 

It stands for Support Vector Machine and it is and algorithm used to train the Model and i trained this model 200 times to be able to achieve high accuracy

```
Features in Training Set: 1600
Invalid Features in Training set: 4
Scores: [ 0.946875    0.96238245  0.98432602  0.95297806  0.95924765]
Accuracy: 0.96 (+/- 0.03)
accuracy score: 0.961152882206
```
![acc](https://user-images.githubusercontent.com/42402820/52171325-c8770680-2763-11e9-8e8c-6e3eeef7a151.jpeg)

## Trial 1

![try1](https://user-images.githubusercontent.com/42402820/52171558-6f10d680-2767-11e9-8632-298eb5180eeb.jpeg)

## Trial 2

![try2](https://user-images.githubusercontent.com/42402820/52171560-6f10d680-2767-11e9-81b8-2bc6d3b27cd7.jpeg)
## Trial 3
![try3](https://user-images.githubusercontent.com/42402820/52171561-6f10d680-2767-11e9-8ae6-fb887205273b.jpeg)

