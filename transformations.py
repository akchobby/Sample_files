import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math 




def euler_to_quaternion():
    pass

def view_quaternion_as_axes():
    pass

def euler_to_rotation_matrix():
    pass

def quaternion_to_rotation_matrix():
    pass



class Quaternion:

    def __init__(self, data, deg=False):
        """
        Function takes a list which has 
        - quaternion values in the order x,y,z,w
        - rotation matrix 3x3
        - euler angles in order of ZYX rotation

        """
        data = np.array(data)
        if len(data.shape) < 2:
            if len(data) == 4:
                self.x = data[0]
                self.y = data[1]
                self.z = data[2]
                self.w = data[3]

            elif len(data) == 3:
                if deg:
                    data = map(self.deg2rad, data)
                cy = math.cos(data[0] * 0.5)
                sy = math.sin(data[0] * 0.5)
                cp = math.cos(data[1] * 0.5)
                sp = math.sin(data[1] * 0.5)
                cr = math.cos(data[2] * 0.5)
                sr = math.sin(data[2] * 0.5)
                self.w = cr * cp * cy + sr * sp * sy
                self.x = sr * cp * cy - cr * sp * sy
                self.y = cr * sp * cy + sr * cp * sy
                self.z = cr * cp * sy - sr * sp * cy

        elif data.shape[0] == 3 and data.shape[1] == 3:
            pass
        
        else: 
            print("[ERROR] invalid arguments")


        

        self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
    
    def __add__(self, q):
        return Quaternion(self.x+q.x,self.y+q.y, self.z+q.z, self.w+q.w)
    
    def __sub__(self, q):
        return Quaternion(self.x-q.x,self.y-q.y, self.z-q.z, self.w-q.w)
    
    def __str__(self):
        return "x:{}, y:{}, z:{}, w:{}\n".format(self.x, self.y, self.z, self.w)

    def __mul__(self, q):
        x =  self.x * q.w + self.y * q.z - self.z * q.y + self.w * q.x
        y = -self.x * q.z + self.y * q.w + self.z * q.x + self.w * q.y
        z =  self.x * q.y - self.y * q.x + self.z * q.w + self.w * q.z
        w = -self.x * q.x - self.y * q.y - self.z * q.z + self.w * q.w
        return Quaternion([x,y,z,w])

    def divide(self, scalar):
        assert not isinstance(scalar, Quaternion), "can only divide scalars"
        return Quaternion([self.x/scalar, self.y/scalar, self.z/scalar, self.w/scalar])
    
    def conjugate(self):
        return Quaternion([-self.x, -self.y, -self.z, self.w])
    
    def inv(self):
        return self.conjugate().normalize()
    
    def normalize(self):
        return(self.divide(self.magnitude))
    
    def deg2rad(self, val):
        return np.pi/180.0 * val
    
    def rad2deg(self, val):
        return val *180.0/np.pi
    
    def to_euler(self, deg=False):
        """
        Returns roll, pitch and yaw in rad unless 
        """
        # roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if (math.fabs(sinp) >= 1):
            pitch = math.copysign(np.pi / 2, sinp); # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw] if not deg else map(self.rad2deg,[roll, pitch, yaw])
    
    def to_rotation_matrix(self):
        pass
    
    def to_list(self):
        return [self.x, self.y, self.z, self.w]
    
    def rotate(self,point):
        q = Quaternion(point+[0])
        return (self * q) * self.conjugate()
    
    def to_rvec(self):
        angle = 2 * math.acos(self.w)
        x = self.x / math.sqrt(1-self.w*self.w)
        y = self.y / math.sqrt(1-self.w*self.w)
        z = self.z / math.sqrt(1-self.w*self.w)
        return [x*angle, y*angle, z*angle]





def plot_3d(ax, transform=None):
    ax = plt.axes(projection='3d', azim=170, elev=20)
    ax.plot([0,1],[0,0],[0,0], alpha=0.25, color="r")
    ax.plot([0,0],[0,1],[0,0], alpha=0.25, color="g")
    ax.plot([0,0],[0,0],[0,1], alpha=0.25,color="b")
    ax.legend(["x","y","z"])
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_zlim(-2,2)


    q =Quaternion([-90,0,-90], True)
    p4 = np.array(q.rotate([0.5,0,0]).to_list()[:3])
    p1 = np.array(q.rotate([1,0,0]).to_list()[:3]) + p4
    p2 = np.array(q.rotate([0,1,0]).to_list()[:3]) + p4
    p3 = np.array(q.rotate([0,0,1]).to_list()[:3]) + p4
    

    print(p1,p2,p3)

    ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
    ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
    ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")






    if isinstance(transform, Quaternion):
        point = np.matmul(transform.to_euler(), np.array([[1,1,1]]).T)

        x = None
        ax.plot([0,0,point[0]],[0,0,0],[0,0,0], color="r")
        ax.plot([0,0,0],[0,0,[1]],[0,0,0], color="g")
        ax.plot([0,0,0],[0,0,0],[0,0,point[2]], color="b")



    
    
q =Quaternion([-90,0,-90], True)

 

fig, ax = plt.subplots(1)
plot_3d(ax)
plt.show(fig)
    
    

    
    


