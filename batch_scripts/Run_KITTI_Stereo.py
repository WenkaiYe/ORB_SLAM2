# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqNameList = ['Seq05_10'];

Result_root = '/home/wye/rosbuild_ws/package_dir/Results/Baseline/'

# Number_GF_List = [60, 80, 100, 130, 160, 200, 240]; # [80, 100, 120]; # 
Number_GF_List = [800]; #  [1500] # 
Num_Repeating = 10 # 50 # 20 #  3 # 5 # 
SleepTime = 1 # 10 # 25

config_path = '/home/wye/rosbuild_ws/package_dir/ORB_Data'

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = config_path + '/KITTI_yaml/KITTI_04_12_stereo_lmk1500.yaml'

            # File_Vocab = config_path + '/ORBvoc.txt'
            File_Vocab = config_path + '/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/Kitti_Dataset/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName

            # do viz
            # cmd_slam   = str('rosrun GF_ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false true /left_rect/image_raw /right_rect/image_raw ' + File_traj)
            # no viz
            cmd_slam   = str('rosrun GF_ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false false /left_rect/image_raw /right_rect/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s ' + str(SeqStartTime[sn]) + ' -u ' + str(SeqDuration[sn]) # + ' -r 0.3' # + ' -s 17' #

            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill Stereo', shell=True)
            time.sleep(SleepTime)
            subprocess.call('pkill Stereo', shell=True)
