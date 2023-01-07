
import pykitti
import numpy as np
import cv2
import matplotlib.pyplot as plt

raw_dir = 'KITTI_SAMPLE/RAW/'
date = '2011_09_26'
drive = '0009'

raw_data = pykitti.raw(raw_dir, date, drive, frames=range(0, 50, 1))

'''Two consecutive stereo pairs'''

imgL_01 = raw_data.get_cam2(0)#image couleur gauche
imgR_01 = raw_data.get_cam3(0)#image couleur droite
imgL_02 = raw_data.get_cam2(1)#image couleur gauche
imgR_02 = raw_data.get_cam3(1)#image couleur droite
imgL_01 = np.array(imgL_01)
imgR_01 = np.array(imgR_01)
imgL_02 = np.array(imgL_02)
imgR_02 = np.array(imgR_02)
K = raw_data.calib.K_cam2

'''DETECTION  MATCH POINTS'''

def findRT(img1,img2 , k1, k2):
    sift = cv2.SIFT_create()
    pts1, descr1 = sift.detectAndCompute(img1, None)
    pts2, descr2 = sift.detectAndCompute(img2, None)
    '''pair of stereo image and homologous points'''
    # create BFMatcher objet
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    # Matcher descriptors
    matches = bf.match(descr1, descr2)
    matches = sorted(matches,key=lambda x: x.distance)
    '''fundamental matrix with the 8-point method and RANSAC '''

    pt1 = []
    pt2 = []

    for i in matches:
        pt2.append(pts2[i.trainIdx].pt)
        pt1.append(pts1[i.queryIdx].pt)
    pt1 = np.array(pt1, dtype='float32')
    pt2 = np.array(pt2, dtype='float32')
    # la matrice fondamentale
    F, mask = cv2.findFundamentalMat(pt1, pt2, cv2.FM_8POINT + cv2.FM_RANSAC)
    pt1 = pt1[mask.ravel() == 1]
    pt2 = pt2[mask.ravel() == 1]
    #essentielle matrix .
    E = k1.T @ F @ k2
    #xtract  rotation and translation
    [_, R,t, _,] = cv2.recoverPose(E, pt1, pt2)
    return  R, t
'''Get the scale of the  translation.'''
R13 , t13 = findRT(imgL_01,imgL_02 , K, K)
R21 , t21 = findRT(imgR_01,imgL_01, K, K)
R23 , t23 = findRT(imgR_01,imgL_02 , K, K)
Mat = np.concatenate((t23,-t13), axis = 1)
s = (np.linalg.inv(Mat.T @ Mat))@Mat.T@t21
print((s[1]*t13))
print(s[0]*t23)

st31_1=s[1]*t13[0]
s_t31_y=0

''' the complete trajectory (50 pair of images).'''
f, ax = plt.subplots(2, figsize=(15, 15))
for i in range(1,49):
    L_01 = raw_data.get_cam2(i)  # left image colors
    R_01 = raw_data.get_cam3(i)  # right image colors
    L_02 = raw_data.get_cam2(i+1)  
    L_01 = np.array(L_01)
    R_01 = np.array(R_01)
    L_02 = np.array(L_02)

    R13, t13 = findRT(L_01, L_02, K, K)
    R21, t21 = findRT(R_01, L_01, K, K)
    R23, t23 = findRT(R_01, L_02, K, K)
    Mat = np.concatenate((t23, -t13), axis=1)
    s = (np.linalg.inv(Mat.T @ Mat)) @ Mat.T@t21
    s_t31_y=s[1]*t13[1]+s_t31_y
    ax[0].scatter(st31_1, s_t31_y, s=2, c='b')
ax[0].set_title('the complete trajectory (50 pair of images).')

tab = range(0,50,1)
for i in tab:
    pose = raw_data.oxts[i].T_w_imu[0:3,3]
    pose = raw_data.calib.T_cam2_imu @ (raw_data.oxts[i].T_w_imu)
    ax[1].scatter(pose[0,3],pose[2,3],s=5,c='r')
ax[1].set_title('List of OXTS packets and 6-dof poses as named tuples')


plt.show()