import numpy as np
from numpy.linalg import multi_dot
from numpy.linalg import inv
from numpy.linalg import cond
import scipy.io #For matlab
from decimal import Decimal


#Wrap2pi
def wrapTo2Pi(angles):
  
  wrapped=(2*np.pi + angles) * (angles < 0) + angles*(angles >= 0)
  
  return wrapped


def obs_update(X_k1k, P_k1k, Obs, GTmap):
      
  #print('observe!')
      
  var_theta=(np.radians(14))**2
  IG_gate=0.46
  detects=0
    
  cam=Obs.camera
  Z=Obs.bearings
  #print(cam)
  #print(np.degrees(Z))
  
         
  map_num=GTmap.shape[1]
  Z_num=Z.shape[0]
      
  #PREDICTED OBSERVATIONS

  a=-0.07
      
  if cam=='C1':    
    b=0.065
                 
  elif cam=='C2':
    b=-0.065


  xr = X_k1k[0] + a*np.cos(X_k1k[2]) - b*np.sin(X_k1k[2])
  yr = X_k1k[1] + a*np.sin(X_k1k[2]) + b*np.cos(X_k1k[2])

  dx=GTmap[0,:]-np.ones(map_num)*xr
  dy=GTmap[1,:]-np.ones(map_num)*yr
  
  theta_p=wrapTo2Pi(np.arctan2(dy,dx)-np.ones(map_num)*X_k1k[2])
  range_p=np.sqrt((dx**2)+(dy**2))
  
  # print(np.arctan2(dy,dx))
  # print(X_k1k[2])
  
  #SELECT LANDMARKS
  
  # select_indx=np.where(range_p <10)
  # select_map=map[:,select_indx]
  # map_num=select_map.shape[1]

  DA_MAT=np.zeros([Z_num,map_num])
  IG_MAT=np.ones([Z_num,map_num])*1000
  IG_MAT2=np.ones([Z_num,map_num])*1000
  
  I=np.zeros([map_num,Z_num])


  zcount=0
  
  for z in Z:
    
        
    I[:,zcount]=np.ones(map_num)*z-theta_p
    
    #INNOVATION COVARIANCE
    
    a_count=0
    
    for m in range(0,map_num-1):
        
        xm=GTmap[0,m]
        ym=GTmap[1,m]
        x=X_k1k[0]
        y=X_k1k[1]
        phi=X_k1k[2]
        
        JH=np.array([np.true_divide(-(y-ym+b*np.cos(phi)+a*np.sin(phi)),((x-xm+a*np.cos(phi)-b*np.sin(phi))**2 +(y-ym+b*np.cos(phi)+a*np.sin(phi))**2)),
                     np.true_divide((x-xm+a*np.cos(phi)-b*np.sin(phi)),((x-xm+a*np.cos(phi)-b*np.sin(phi))**2+(y-ym+b*np.cos(phi)+a*np.sin(phi))**2)),
                     np.true_divide(((np.true_divide((a*np.cos(phi)-b*np.sin(phi)),(x-xm+a*np.cos(phi)-b*np.sin(phi)))+np.true_divide(((b*np.cos(phi)+a*np.sin(phi))*(y-ym+b*np.cos(phi)+a*np.sin(phi))),(x-xm+a*np.cos(phi)-b*np.sin(phi))**2))*(x-xm+a*np.cos(phi)-b*np.sin(phi))**2),((x-xm+a*np.cos(phi)-b*np.sin(phi))**2+(y-ym+b*np.cos(phi)+a*np.sin(phi))**2))-1])
      
        R=var_theta
        
        S=R+multi_dot([JH,P_k1k,np.transpose(JH)])
        
        
        IG=np.transpose(I[m,zcount])*(1/S)*I[m,zcount]
        
        if IG < IG_gate and range_p[m]<20:
            IG_MAT[zcount,m]=IG
            a_count=a_count+1
            
    zcount=zcount+1
            
  da_count=np.ones(Z_num)
  
  
  for m in range(0,map_num-1):
    
    minINDX=np.argmin(IG_MAT[:,m])
    min_lm=IG_MAT[minINDX,m]
    
    if min_lm<1000:
        IG_MAT2[minINDX,m]=min_lm

  zcount=0
  for z in Z:
    
    minINDX=np.argmin(IG_MAT2[zcount,:])
    minIG=IG_MAT2[zcount,minINDX]
    
    if minIG<1000:
      DA_MAT[zcount,minINDX]=1
      da_count[zcount]=1
    else:
      da_count[zcount]=0
    zcount=zcount+1
  

         
  finz_count=1; 
  LM_ID=np.array([])
  fin_inov=np.array([])
  
  zcount=0
  
  xm=np.array([])
  ym=np.array([])
  
  for z in Z:
    
    if da_count[zcount]>0:
          
        indx=np.where(DA_MAT[zcount,:]==1)
      
        fin_inov=np.append(fin_inov,I[indx[0],zcount])
        LM_ID=np.append(LM_ID,indx[0])
        xm=np.append(xm,GTmap[0,indx[0]])
        ym=np.append(ym,GTmap[1,indx[0]])
       
        
        finz_count=finz_count+1
  zcount=zcount+1

  if finz_count>1:
        
    detects=LM_ID.size
    # xm=GTmap[0,LM_ID[:]]
    # ym=GTmap[1,LM_ID[:]]
    x=np.ones(LM_ID.size)*X_k1k[0]
    y=np.ones(LM_ID.size)*X_k1k[1]
    phi=np.ones(LM_ID.size)*X_k1k[2]
    
    JH=np.array([np.true_divide(-(y-ym+b*np.cos(phi)+a*np.sin(phi)),((x-xm+a*np.cos(phi)-b*np.sin(phi))**2 +(y-ym+b*np.cos(phi)+a*np.sin(phi))**2)),
                     np.true_divide((x-xm+a*np.cos(phi)-b*np.sin(phi)),((x-xm+a*np.cos(phi)-b*np.sin(phi))**2+(y-ym+b*np.cos(phi)+a*np.sin(phi))**2)),
                     np.true_divide(((np.true_divide((a*np.cos(phi)-b*np.sin(phi)),(x-xm+a*np.cos(phi)-b*np.sin(phi)))+np.true_divide(((b*np.cos(phi)+a*np.sin(phi))*(y-ym+b*np.cos(phi)+a*np.sin(phi))),(x-xm+a*np.cos(phi)-b*np.sin(phi))**2))*(x-xm+a*np.cos(phi)-b*np.sin(phi))**2),((x-xm+a*np.cos(phi)-b*np.sin(phi))**2+(y-ym+b*np.cos(phi)+a*np.sin(phi))**2))-1])
    
    R=np.eye(LM_ID.size)*var_theta
    
    
    
    S=R + multi_dot([np.transpose(JH),P_k1k,JH])
    
    #Update
    
    if cond(S) <10:
        K=multi_dot([P_k1k,JH,inv(S)])
        
    else:
        K=np.zeros([3,S.shape[0]])
        
    
    #print(K.shape)
    X_k1k1=X_k1k+np.dot(K,fin_inov)
    
    P_k1k1=P_k1k-multi_dot([K,S,np.transpose(K)])
    print('detects ',detects)
  else:
        
    X_k1k1=X_k1k
    P_k1k1=P_k1k
    
    innov=0
    innovcov=0
    obs_count=0
    
  
  X_k1k1[2]=wrapTo2Pi(X_k1k1[2])
  return X_k1k1,P_k1k1,detects