# Bosch-Motors-Dataset
This project is my master thesis and also a sub-project of the research project **AgiProbot** from KIT and Bosch. We develop a benchmark including 2D synthetic image datasets and 3D synthetic point cloud datasets, in which the key objects have multiple learn-able attributes with ground truth provided. In this project, small electric motors are used as the key objects. This part explains how to generate the datasets, including motor mesh model dataset, image dataset, point cloud dataset and noise point cloud dataset. We also build a synthetic **clamping** **system** with Blender to simulate the motor being clamped by the fixture in the real scenario.
## Software Preparation
* As our programming is based on the python language, we recommend a [Python>=3.7.0](https://www.python.org/) environment, including [Numpy](https://numpy.org/) and [Matplotlib](https://matplotlib.org/).
* We use [Blender 2.9](https://www.blender.org/) to generate the synthetic motor mesh model with the addon named motor factory. For specific details including how to install, you can see [Motor Factory](https://github.com/cold-soda-jay/blenderMotorFactoryVer2.0).
* The generation of image dataset is with the help of Blenderproc, a procedural Blender pipeline for photorealistic rendering. For Installation and tutorials, seen [Blenderproc](https://github.com/DLR-RM/BlenderProc). (I did this part of the work in an Ubuntu 20.04 environment)
* After generated the image dataset, we use another Blender addon named [Blensor](https://www.blensor.org/) to generate the point cloud dataset. Blensor is based on the Blender 2.79 version, may require additional libraries. (I used the third party released Blensor for Windows)
### 1. Motor Mesh Model Dataset Generation
We generate 5 kinds of synthetic motor mesh (Type A0, A1, A2, B0, B1) with Blender addon in randomly different size ranges in each motor's parts. 
> Run the Blender (version 2.9 above is recommended), open Text Editor(hotkey: Shift + F11) and open the script named `synthetic_motor_generate.py`. You should specify the path to save in BASE_DIR and the number of total motors in main() function. (Tips: The input here is the total number of motors. Since 5 types of motors will be generated, please ensure that the input is an integer multiple of 5) Click "Run Script".
```python
BASE_DIR = '/home/linxi/KIT/Thesis/Dataset/motor_mesh_model_50/' 
num_motor_for_dataset=50
```

### 2. Image Dataset Generation
In the image dataset, we merge motors and clamping system to generate 5 images for each scene, which are RGB image, distance image, normal image, semantic segmentated image for each part of the motor and a COCO-annotated image with a 2D bounding box of the motor, for the following tasks of the main project, we also add the 2D bounding box of the bolts. 
> The scripts here should be running by `blenderproc run script_name.py`. For example, you can generate one set of images for a motor by running the script `try.py`. You shoud set up the path of the clamping system in line 300, specify a motor type and number in line 310, line 311 and line 314 of the script.
![](https://github.com/LinxiQIU/Bosch-Motors-Dataset-generate/blob/main/try_setup.png)

```python
blenderproc run try.py 
```
> If you want to generate the dataset for the specified motor type, please run the script `batch_generation.py` and set the path of the motor type.
 
In each scene, we changed the position and euler rotation of the camera to increase the diversity of the scene. At the same time, we also randomly rotated the motor a little bit to simulate the improper placement of the motor in reality. This information will be saved in the csv file named camera_motor_setting.csv.
### 3. Point Cloud Dataset Generation
The point cloud dataset is composed of scene and cuboid point cloud. We use the corresponding camera information saved in the camera_motor_setting.csv to scan the scene in point cloud to maintain correspondence with the image dataset. In order to improve the deep learning efficiency, we cut out a cuboid with the motor as the center. 
> Run the Blensor, open the script `assemble_cut_win.py` in Text Editor, set up the csv_path, file_path, Clamping_path and save_path in the main() function.

> We mark the position of the motor in the point cloud scene with a 3D bounding box, and save the three-dimensional coordinates of the center of each motor and the length, width and height of the entire motor in motor_3D_bounding_box.csv for the deep learning task of 3D object detection by running `display.py`

> We provide each motor with both scene and cuboid point cloud in Numpy and PCD format. You can convert the generated Numpy file to PCD by running `points2pcd.py`
### 4. Point Cloud Dataset Augmentation
On the basis of the point cloud dataset in the previous step, we add more random noises to obtain data augmentation. For example, we add a cover randomly above the motor. You can get the whole noise cuboid point cloud dataset by running `augmented_pc_generation.py` with Blensor.
> open 'Command Prompt' in Windows, change to the Blensor directory and type in following command:
```python
blender -b -P path/of/augmented_pc_generation.py -- -i path/of/input -o path/of/output -clp path/of/clamping_system -ss(save scene) -sf(scene file format) -bb(3d bounding box) -sc(save cuboid) -cf(cuboid file format) -roim(rotation from image dataset) -csvp path/of/csv -n(number of generation)
```
 
| cmd  | Description          | Type | Property |
| ------- | ----------------------------------------------------------| --- | ---------- |
| -b   | run Blender in background mode                        |       |            |
| -P   | python script                                          |      |            |
| -i   | path of motor mesh model                                | string     | obligatory |
| -o   | path of save directory                                  | string     | obligatory |
| -clp | path of clamping system                                 | string     | obligatory |
| -ss   | whether to save scene file (default=True)               | boolean    | optional   |
| -sf   | scene file format, option: numpy, pcd, both (default: numpy)  | string | optional |
| -bb   | whether to save 3D bounding box of motor (default=True)    | boolean |  optional  |
| -sc   | whether to save cuboid file (default=True)     | boolen | optional |
| -cf   | cuboid file format, option: numpy, pcd, both (default: numpy)  | string | optional |
| -roim | default=False: apply random rotation info and save. True: load rotation info from given csv file | boolen | optional |
| -csvp | if -roim is False, save directory of rotation info.(default is save directory). if -roim is True, path of given csv file | string | optional/obligatory |
| -n | number of total generation (an integer multiple of 5)     | integer | obligatory  |

If you want to modify the value of a boolean type of argument, just enter this cmd. Here is the example command for the demo.
![](https://github.com/LinxiQIU/Bosch-Motors-Dataset-generate/blob/main/blensor_cmd.png)
