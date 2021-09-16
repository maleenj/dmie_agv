
import numpy as np
from numpy.linalg import multi_dot
from numpy.linalg import inv
from numpy.linalg import cond
import scipy.io #For matlab
from decimal import Decimal
from scipy.interpolate import interp1d

def predict_pose(pose,U_k,dt):
  
  #FIGURE OUT VELOCITY OR DISTANCE ISSUE AND FIX COVAIRANCES ACCORDINGLY
  X_k1k=np.zeros([3])
  #Motion model
  X_k=np.array([pose.X, pose.Y, pose.Phi])
  P_k=pose.P
  X_k1k= X_k + np.array([U_k[0]*np.cos(X_k[2]), U_k[0]*np.sin(X_k[2]), dt*U_k[1]])
  
  #print('predict!')
  

  var_v=(0.05)**2
  var_w=(np.deg2rad(5))**2

  #Control noise
  Q=np.array([[var_v, 0],[0, var_w]])

  #Jacobians

  dFX=np.array([[1, 0, -U_k[0]*np.sin(X_k[2])], [0, 1, U_k[0]*np.cos(X_k[2])], [0, 0, 1]])

  dFU=np.multiply(np.array([[np.cos(X_k[2]), 0], [np.sin(X_k[2]), 0],[0, 1]]),dt)

  P_k1k=multi_dot([dFX,P_k,np.transpose(dFX)])+multi_dot([dFU,Q,np.transpose(dFU)])

  return X_k1k,P_k1k
