'''
Created on May 18, 2013

@author: wng
'''

import copy
import numpy as np

class Quaternion(object):
    '''
    Quaternion class.
    '''
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        '''
        Constructor
        '''
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        
        self.normalize()
                
    def vec(self):
        return np.array([self.x, self.y, self.z, self.w])

    def setFromAngleAxis(self, angle, axis):
        self.x = np.sin(angle/2)*axis[0]
        self.y = np.sin(angle/2)*axis[1]
        self.z = np.sin(angle/2)*axis[2]
        self.w = np.cos(angle/2)
        
        self.normalize()
        
        return self
        
    def setFromRotationMatrix(self, mat):
        
        # Copied from Eigen.
        # This algorithm comes from  "Quaternion Calculus and Fast Animation",
        # Ken Shoemake, 1987 SIGGRAPH course notes
        
        assert mat.shape == (3, 3)
        
        t = np.trace(mat)
        if t > 0:
            t = np.sqrt(t + 1.0)
            self.w = 0.5*t
            t = 0.5 / t
            self.x = (mat[2, 1] - mat[1, 2]) * t
            self.y = (mat[0, 2] - mat[2, 0]) * t
            self.z = (mat[1, 0] - mat[0, 1]) * t
        else:
            i = 0
            if (mat[1, 1] > mat[0, 0]):
                i = 1
            if (mat[2, 2] > mat[i, i]):
                i = 2
            j = (i+1) % 3
            k = (j+1) % 3
            
            t = np.sqrt(mat[i, i] - mat[j, j] - mat[k, k] + 1.0)
            
            if i == 0:                
                self.x = 0.5 * t
            elif i == 1:
                self.y = 0.5 * t
            elif i == 2:
                self.z = 0.5 * t
            elif i == 3:
                self.w = 0.5 * t
                        
            t = 0.5 / t
                    
            self.w = (mat[k, j] - mat[j, k])*t
            
            if j == 0:                    
                self.x = (mat[j, i] + mat[i, j])*t
            elif j == 1:
                self.y = (mat[j, i] + mat[i, j])*t
            elif j == 2:
                self.z = (mat[j, i] + mat[i, j])*t
                    
            if k == 0:                
                self.x = (mat[k, i] + mat[i, k])*t
            elif k == 1:
                self.y = (mat[k, i] + mat[i, k])*t
            elif k == 2:
                self.z = (mat[k, i] + mat[i, k])*t

        self.normalize()

        return self
        
    def norm(self):
        return np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    def normalize(self):
        n = self.norm()
        
        self.x /= n
        self.y /= n
        self.z /= n
        self.w /= n
        
        return self
        
    def getRotationMatrix(self):
        # Copied from Eigen.  
        tx  = 2.0*self.x
        ty  = 2.0*self.y
        tz  = 2.0*self.z
        twx = tx*self.w
        twy = ty*self.w
        twz = tz*self.w
        txx = tx*self.x
        txy = ty*self.x
        txz = tz*self.x
        tyy = ty*self.y
        tyz = tz*self.y
        tzz = tz*self.z
        
        R = np.zeros((3, 3))
        R[0, 0] = 1.0-(tyy+tzz)
        R[0, 1] = txy-twz
        R[0, 2] = txz+twy
        R[1, 0] = txy+twz
        R[1, 1] = 1.0-(txx+tzz)
        R[1, 2] = tyz-twx
        R[2, 0] = txz-twy
        R[2, 1] = tyz+twx
        R[2, 2] = 1.0-(txx+tyy)
        
        return R
    
    def slerp(self, t, other):
        '''
        Spherical linearl interpolation (from Eigen).
        '''
        vec0 = self.vec()
        vec1 = other.vec()

        thresh = float(1.0)  - np.spacing(1.0)
        d = np.dot(vec0, vec1)
        abs_d = np.abs(d)
        
        scale0 = 0
        scale1 = 0

        if abs_d >= thresh:
            scale0 = 1.0 - t
            scale1 = t
        else:
            theta = np.arccos(abs_d)
            sin_theta = np.sin(theta)

            scale0 = np.sin((1.0 - t ) * theta)/sin_theta;
            scale1 = np.sin(( t * theta))/sin_theta;
            
        if d < 0:
            scale1 = -scale1

        new_vec =  scale0 * vec0 + scale1 * vec1
        
        q = Quaternion(new_vec[0], new_vec[1], new_vec[2], new_vec[3])

        return q.normalize()

    def __str__(self):
        return "[%.17f, %.17f, %.17f, %.17f]" %(self.x, self.y, self.z, self.w)
        
def skewSym(w):
    """
    Convert a 3 vector to a skew-symmetric matrix.
    """
    assert(len(w) == 3)

    ret = np.zeros((3, 3))
    ret[1, 0] = w[2]
    ret[2, 0] = -w[1]
    ret[0, 1] = -w[2]
    ret[2, 1] = w[0]
    ret[0, 2] = w[1]
    ret[1, 2] = -w[0]

    return ret
    
def so3LieToMat(w):
    """
    Converts Lie algebra in SO3 to rotation matrix using 
    exponential map (i.e. Rodrigues formula).
    http://ethaneade.com/latex2html/lie/node16.html
    """
    assert(len(w) == 3)

    theta = np.linalg.norm(w)

    if theta > 1e-3:
        A = np.sin(theta)/theta
        B = (1 - np.cos(theta))/(theta**2)
    else:
        A = 1
        B = 0.5

    wx = skewSym(w)

    # Rodrigues formula.
    R = np.eye(3)
    R += A*wx + B*np.dot(wx, wx)
    
    return R    

def se3LieToRotTrans3(w, u):
    """
    Converts Lie algebra in SE3 (rotation params in u, translation
    params in u) to Rotation matrix and translation vector using
    exponential map.
    See http://ethaneade.com/latex2html/lie/node16.html
    """

    assert(len(w) == 3)
    assert(len(u) == 3)

    # First get rotation matrix.    
    theta = np.linalg.norm(w)

    if theta > 1e-3:
        A = np.sin(theta)/theta
        B = (1 - np.cos(theta))/(theta**2)
    else:
        A = 1
        B = 0.5

    wx = skewSym(w)

    # Rodrigues formula.
    R = np.eye(3)
    R += A*wx + B*np.dot(wx, wx)

    # Now get translation vector.
    if theta > 1e-3:
        C = (1 - A)/(theta**2)
    else:
        C = 1.0/6

    V = np.eye(3)
    V += B*wx + C*np.dot(wx, wx)
    t = np.dot(V, u)

    return R, t

def se3LieToMat4(w, u):
    """
    Converts Lie algebra in SE3 (rotation params in u, translation
    params in u) to 4x4 homogenous transformation matrix using
    exponential map.
    See http://ethaneade.com/latex2html/lie/node16.html
    """
    
    assert(len(w) == 3)
    assert(len(u) == 3)

    R, t = se3LieToRotTrans3(w, u)

    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    T[3, 3] = 1.0

    return T

def rotTrans3ToMat4(R, t):
    """
    Convert a 3D rotation matrix R and translation t to 
    a homogeneous transformation matrix.
    """

    assert(R.shape == (3, 3))
    assert(len(t) == 3)

    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    T[3, 3] = 1.0

    return T

    
