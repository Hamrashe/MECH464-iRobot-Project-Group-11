import os
import numpy as np
import cv2
from sympy import homogeneous_order

from tqdm import tqdm
import time

from matplotlib import pyplot as plt

#from visualization import plotting
#from visualization.video import play_trip

from tqdm import tqdm

from capture_vo import video_to_frames, write_images
import importlib

#importlib.reload(video_to_frames)

import keyboard

class VisualOdometry():
    
    def __init__(self, img_dir):
        
        #data_dir is the directory of the images
        #print(data_dir)
        with open('camera_setup\intrinsicNew.npy', 'rb') as f:
            intrinsic = np.load(f)

        self.K = intrinsic
        self.extrinsic = np.array(((1,0,0,0),(0,1,0,0),(0,0,1,0)))
        self.P = self.K @ self.extrinsic
        self.images = self._load_images(img_dir)
        #self.images = imgs

        self.orb = cv2.ORB_create(3000)
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)
        self.cap = cv2.VideoCapture('odomotry\VO\playback\VO_video.avi')
        #self.world_points = []

        #self.current_pose = None
    
    def _load_images(self,filepath):

        image_paths = [os.path.join(filepath, file) for file in sorted(os.listdir(filepath))]

        return [cv2.imread(path, cv2.IMREAD_GRAYSCALE) for path in image_paths]
    
    def _form_transf(self, R, t):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t 
        return T
    
    def get_matches(self, i):

        #cv2.imwrite('odomotry\VO\playback\image1', self.images[i-1])
        #cv2.imwrite('odomotry\VO\playback\image2', self.images[i])

        #img1 = cv2.imread('odomotry\VO\playback\image1', cv2.IMREAD_GRAYSCALE)
        #img2 = cv2.imwrite('odomotry\VO\playback\image2', cv2.IMREAD_GRAYSCALE)
        _, frame = self.cap.read()
        cv2.imshow('Frame',frame)
        cv2.waitKey(0)
        keypoints1, descriptiors1 = self.orb.detectAndCompute(frame, None)
        _, frame = self.cap.read()
        keypoints2, descriptors2 = self.orb.detectAndCompute(frame, None)

        matches = self.flann.knnMatch(descriptiors1, descriptors2,k=2)

        good = []
        # BFMatcher with default params
        matches = self.flann.knnMatch(descriptiors1, descriptors2, k=2)
        try:
            for m, n in matches:
                if m.distance < 0.8 * n.distance:
                    good.append(m)
        except ValueError:
            pass


        q1 = np.float32([ keypoints1[m.queryIdx].pt for m in good])
        q2 = np.float32([ keypoints2[m.trainIdx].pt for m in good])

        draw_params = dict(matchColor = -1, # draw matches in green color
                 singlePointColor = None,
                 matchesMask = None, # draw only inliers
                 flags = 2)

        img3 = cv2.drawMatches(self.images[i], keypoints1, self.images[i-1],keypoints2, good ,None,**draw_params)
        cv2.imshow("image", img3)
        cv2.waitKey(200)

        return q1, q2
    
    def get_pose(self, q1, q2):
        
        print('q1 = ' + str(q1) + '\n\nq2 = ' + str(q2) + '\n\n')
        Essential, mask = cv2.findEssentialMat(q1, q2, self.K)
        
        R, t = self.decomp_essential_mat(Essential, q1, q2)

        return self._form_transf(R,t)
    
    def decomp_essential_mat(self, E, q1, q2):

        print('E = ' + str(E) + '\n')
        R1, R2, t = cv2.decomposeEssentialMat(E)
        T1 = self._form_transf(R1, np.ndarray.flatten(t))
        T2 = self._form_transf(R2, np.ndarray.flatten(t))
        T3 = self._form_transf(R1, np.ndarray.flatten(-t))
        T4 = self._form_transf(R2, np.ndarray.flatten(-t))
        transformations = [T1, T2, T3, T4]

        K = np.concatenate((self.K, np.zeros((3,1)) ), axis = 1)

        projections = [K @ T1, K @ T2, K @ T3, K @ T4]

        np.set_printoptions(suppress=True)

        positives = []
        for P, T in zip(projections, transformations):
            hom_Q1 = cv2.triangulatePoints(self.P, P, q1.T, q2.T)
            hom_Q2 = T @ hom_Q1

            Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            Q2 = hom_Q2[:3, :] / hom_Q2[3, :]

            total_sum = sum(Q2[2,:] > 0 ) + sum(Q1[2, :] > 0)
            relative_scale = np.mean(np.linalg.norm(Q1.T[:-1] - Q1.T[1:], axis=-1)/
                                     np.linalg.norm(Q2.T[:-1] - Q2.T[1:], axis=-1))
            positives.append(total_sum+relative_scale)


        max = np.argmax(positives)
        if (max == 2):
            return R1, np.ndarray.flatten(-t)
        elif (max == 3):
            return R2, np.ndarray.flatten(-t)
        elif (max == 0):
            return R1, np.ndarray.flatten(t)
        elif (max == 1):
            return R2, np.ndarray.flatten(t)

def run_vo(initial_pose):
    # I want this function to capture images until a flag is returned, then I want it to read the images and perform odometry on them.


    #the initial path is in the form of a homogenous transformation matrix

    #data_dir = 'odomotry\VO\images'
    

    vo = VisualOdometry('odomotry\VO\images')

    #imgs = vo._load_images('odomotry\VO\images')
    #print(imgs[1])
    #cv2.imshow('test',imgs[1])

    estimated_path = []
    x = []
    y = []
    z = []
    for i in range(len(vo.images)):
        if i == 0:
            cur_pose = initial_pose
        else:
            print(i)
            q1, q2 = vo.get_matches(i)
            transf = vo.get_pose(q1,q2)
            cur_pose = np.matmul(cur_pose, np.linalg.inv(transf))
            print("\nThe cuurrent pose is:\n" + str(cur_pose))
            estimated_path.append((cur_pose[0,3], cur_pose[1,3]))
            x.append(cur_pose[0,3])
            y.append(cur_pose[1,3])
            z.append(cur_pose[2,3])
            print(f'i is {i} \n')
            #print(f'image is {}')

            
            #fig = plt.figure()
            #ax = plt.axes(projection='3d')

            
            #ax.plot3D(x,y,z)
            #time.sleep(1)
            #plt.show()
            print(cur_pose[0,3])

    
    fig = plt.figure()
    ax = plt.axes(projection='3d')

            
    ax.plot3D(x,y,z)
        
    #plt.plot(x,y)
    plt.show()
    #plotting.visualize_paths(estimated_path, estimated_path, "visual odometry", "VO Exercise", "plot.html")

def main():
    initial_pose = [[1, 0, 0, 0,], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    imgs = video_to_frames('odomotry\VO\playback\VO_video.avi')
    write_images(imgs)
    print(len(imgs))
    cv2.imshow('test', imgs[5])
    cv2.waitKey(0)
    run_vo(initial_pose)


main()

