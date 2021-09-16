#Wrap2pi
import numpy as np

def wrapTo2Pi(angles):
  
  wrapped=(2*np.pi + angles) * (angles < 0) + angles*(angles >= 0)
  
  return wrapped