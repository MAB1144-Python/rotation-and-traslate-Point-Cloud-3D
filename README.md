# Rotation-and-traslate-Point-Cloud-3D.
### Application Euler angle and position vector of ORB-SLAM.

### To apply the translation and rotation vectors that ORB-SLAM gives us, the librari numpy:

```python
import numpy as np
```
### Subscribe to the message line of the ORB-SLAM node.
```python
rospy.Subscriber("orb_slam2_rgbd/pose", PoseStamped, self.odometrycb)
```
### Function to extract the values of the message.
```python
def odometrycb(self, msg):
   x_t = msg.pose.position.x
   y_t = msg.pose.position.y
   z_t = msg.pose.position.z
   roll = msg.pose.orientation.x
   pitch = msg.pose.orientation.y
   yaw = msg.pose.orientation.z
```   
   
### We will define the rotation matrix, this is created from the rotation in each of the axes, and it will have the three angles as input.
```python
def euler_to_rotMat(  roll, pitch, yaw ):
    #rotación en Z
    Rz_yaw =   np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw),  np.cos(yaw), 0],
                         [0          ,            0, 1]])
    #rotación en Y
    Ry_pitch = np.array([[ np.cos(pitch),  0,  np.sin(pitch)],
                         [0             ,  1,              0],
                         [-np.sin(pitch),  0, np.cos(pitch)]])
    #rotación X
    Rx_roll =  np.array([[1,            0,             0],
                         [0, np.cos(roll), -np.sin(roll)],
                         [0, np.sin(roll),  np.cos(roll)]])
    #Matriz de rotación
    rotMat = np.dot( np.dot(Rx_roll, Ry_pitch), Rz_yaw)
    return rotMat  
```    
### Apply the product between each point of the point cloud, with the trotation matrix.

```python
np.dot(Rmat,[x,y,z])
```
### To translate, add the translation of each axis.
```python
x += x_t
y += y_t
z += z_t
```
### To find more content about SLAM, you can go to:
### https://github.com/MAB1144-Python/Research-in-SLAM/blob/main/README.md
