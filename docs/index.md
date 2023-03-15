# CS 184: Computer Graphics and Imaging, Spring 2023

## Project 3-1: PathTracer
## Michael Lin, Rachel Lee
 *** 
### Overview
In this project, we generated camera rays by transforming normalized image coordinates from image space to sensors in camera space and then transforming the camera ray into a ray in the world space. We also generated pixel samples and implemented the Möller Trumbore algorithm for calculating ray-triangle intersections.
### Part 1
- **Walk through the ray generation and primitive intersection parts of the rendering pipeline.**
  - To generate the ray, we first transform the normalized input coordinates ```(x,y)``` from the world space to camera space by setting ```xTransform``` to ```x * tan(.5 * radians(hFov)) + (x - 1) * tan(.5 * radians(hFov))``` and ```yTransform``` to  ```y * tan(.5 * radians(hFov)) + (x - 1) * tan(.5 * radians(hFov))```. After transforming the coordinates from image space to sensor in camera space, we then transform the ```xTransform``` and ```yTransform``` coordinates into the world space by finding the updated direction and multiplying by the camera-to-world rotation matrix ```c2w```. Then, we create and return a new ```cameraRay``` using the camera position in the world space pos and updated direction vector while also setting the ```min_t``` and ```max_t``` of the ray to be within the bounds of the two clipping planes ```nclip``` and ```fclip```. After generating the ray and pixel samples using ```raytrace_pixel```, we test whether there is an intersection between the triangle and input ray using the Möller Trumbore algorithm and reporting the location of the nearest intersection point. Finally, we check if the found intersection point is within the triangle’s boundaries.

- **Explain the triangle intersection algorithm you implemented in your own words.**
  - In ```Triangle::intersect``` , we implemented the Möller Trumbore algorithm by using linear interpolation to calculate the barycentric coordinates and determine if the intersection point of the ray lies within the triangle. First, we found two edges of the triangle py calculating the difference of ```p2 - p1 ```and ```p3 - p1``` and stored them in ```e1``` and ```e2```, respectively. Then we found the distance of ```p1``` from the given ray’s origin and used it to find the cross product between e1 and store this in ```s1```. We also found the cross product between the ray’s direction and ```e2``` and store this value in ```s2```. Next, we calculate the inverse of the dot products using: ```1 / dot(s1, e1) * Vector3D(dot(s2, e2), dot(s1, s0), dot(s2, r.d))``` and store the result in a ```Vector3D```. Finally, we check that the coordinates of the resulting vector are within the range (0,1) and that the intersection that occurs at t lies within the ```min_t``` and ```max_t``` of the input array and update ```max_t``` if necessary.
- **Show images with normal shading for a few small .dae files.**
  - ```CBEmpty.dae```
  - ![CBEmpty.png](./images/CBEmpty.png)
  - ```CBSpheres.dae```
  - ![CBSpheres.png](./images/CBSpheres.png)
  - ```banana.dae```
  - ![banana.png](./images/banana.png)

### Part 2
- **Walk through your BVH construction algorithm. Explain the heuristic you chose for picking the splitting point.**
  - In BVHAccel:construct_bvh, we first compute the bounding box from the given vector of primitives using get_bbox() and initialize a new BVHNode with the bounding box. Then, we check to see if the current node is a leaf node by seeing if there are no more than max_leaf_size primitives in the list, and then update the start and end primitive iterators. If the node is an internal node, then we split the primitives into “left” and “right” sections along the longest axis of the bounding box within bbox.extent. After finding the longest axis for the split, we calculate the midpoint of the longest axis as the split point to divide the primitives: 

```
double splitPoint = (bbox.min[splitAxis] + bbox.max[splitAxis]) / 2;
auto partitionPoint = partition(start, end, [splitAxis, splitPoint](Primitive *p) {
        Vector3D bbCenter = p->get_bbox().centroid();
        return splitPoint > bbCenter[splitAxis];
    });
```
  - If the split point is at the start or end, then we know that all primitives lie on only one side of the split point and can return the current node. Otherwise, we create a new node with updated start and end primitives and recursively call construct_bvh on the new Primitie vector.

- **Show images with normal shading for a few large .dae files that you can only render with BVH acceleration.**
    - ```CBbunny.dae```
    ![bunny.png](./images/CBbunny.png)
    - ```cow.dae```
    ![cow.png](./images/cow.png)
    - ```CBlucy.dae```
    ![CBlucy.png](./images/CBlucy.png)

- **Compare rendering times on a few scenes with moderately complex geometries with and without BVH acceleration. Present your results in a one-paragraph analysis.**
- **Without BVH Acceleration:**
![without_bvh.png](./images/without_bvh.png)
- **With BVH Acceleration:**
![with_bvh.png](./images/with_bvh.png)
