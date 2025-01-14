# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# Corridor, failed to close loop between 2 runs
# SeqStartTime = [210] # [300] # 
# SeqDuration = [300]
# SeqNameList = ['2019-01-25-12-27_stereo'];

# * Corridor (short; no loop)
# SeqNameList = ['2019-01-23-06-22_stereo'];
# SeqStartTime = [0] 
# SeqDuration = [100]

# * Corridor (short)
# SeqStartTime = [250]
# SeqDuration = [100]
# SeqNameList = ['2019-01-24-15-09_stereo'];

# * Corridor (2 loops)
# SeqStartTime = [0] 
# SeqDurationTime = [300]
# SeqNameList = ['2019-01-25-15-10_stereo'];

# * Room I
# SeqStartTime = [60] 
# SeqDuration = [130]
# SeqNameList = ['2019-01-25-17-30_stereo'];

# * Room II
# SeqStartTime = [230] 
# SeqDuration = [999]
# SeqNameList = ['2019-01-24-18-09_stereo'];

# * Batch Eval
SeqNameList = ['loop_corridor_stereo']#'meeting_room1_stereo', 'meeting_room2_stereo'];
# SeqStartTime = [0, 60, 230] 
# SeqDuration = [300, 130, 999]
SeqStartTime = {
    'loop_corridor_stereo': 0,
    'meeting_room1_stereo': 60,
    'meeting_room2_stereo': 230
}
SeqDuration = {
    'loop_corridor_stereo': 300,
    'meeting_room1_stereo': 130,
    'meeting_room2_stereo': 245#509
};

# Result_root = '/mnt/DATA/tmp/Hololens/ORB2_Stereo_Baseline/'
# Result_root = '/mnt/DATA/tmp/Hololens/ORB2_Stereo_GT/'
Result_root = '/home/wye/rosbuild_ws/package_dir/GF_ORB_SLAM2/exp/t/'

Number_GF_List = [800]; # [400, 600, 800, 1000, 1500, 2000]; # 
# Number_GF_List = [60, 80, 100, 150, 200];
Num_Repeating = 1 # 1 # 10 # 20 #  
SleepTime = 1 # 10 # 25

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

for iteration in range(0, Num_Repeating):

    for ri, num_gf in enumerate(Number_GF_List):
        
        Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        # mkdir for pose traj
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)
        # mkdir for map as well
        cmd_mkdir = 'mkdir -p ' + Experiment_dir + '_Map'
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn] # + '_blur_5'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = '../../ORB_Data/Hololens_yaml/Hololens_stereo_lmk800.yaml'

            # File_Vocab   = '../ORB_Data/ORBvoc.txt'
            File_Vocab   = '../../ORB_Data/ORBvoc.bin'
            File_rosbag  = '/home/wye/Documents/Data/Hololens/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName
            File_log = Experiment_dir + '/' + SeqName + '_verbose_details.log'

            # pre-rect
            # cmd_slam   = str('rosrun GF_ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' true true /left_cam/image_raw /right_cam/image_raw ' + File_traj)
            # do viz
            # cmd_slam   = str('rosrun GF_ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' false true /left_cam/image_raw /right_cam/image_raw ' + File_traj)
            # no viz
            cmd_slam   = str('rosrun GF_ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' false true /left_cam/image_raw /right_cam/image_raw ' + File_traj + " >> " + File_log)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s ' + str(SeqStartTime[sname]) + ' -u ' + str(SeqDuration[sname]) #+ ' -r 0.3'

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
