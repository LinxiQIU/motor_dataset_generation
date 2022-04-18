
   
import os
import numpy as np
import random
import open3d 
import csv
import math
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors

def Visuell_PointCloud(sampled, SavePCDFile = False, FileName = None):
    #get only the koordinate from sampled
    sampled = np.asarray(sampled)
    PointCloud_koordinate = sampled[:, 0:3]
    label=sampled[:,6]
    labels = np.asarray(label)
    print(labels.shape)
    max_label = labels.max()

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label>0 else 1))


    #visuell the point cloud
    point_cloud = open3d.geometry.PointCloud()
    point_cloud.points = open3d.utility.Vector3dVector(PointCloud_koordinate)
    point_cloud.colors = open3d.utility.Vector3dVector(colors[:, :3])
    open3d.visualization.draw_geometries([point_cloud])

    if SavePCDFile is True:
    # #save the pcd file
        open3d.io.write_point_cloud(FileName +'.pcd', point_cloud)







 
def main():

    save_dir='E:\Result\TypeA1\Training_TypeA1_0002_augmentation3.npy'
    patch_motor=np.load(save_dir)      
    print(len(patch_motor))
    
    Visuell_PointCloud(patch_motor)


if __name__ == '__main__':
    main()