import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd
import scipy
import sys
import os
import shutil
from scipy.spatial.transform import Rotation as Rot

RAD2DEG = 57.2957795131
THRUST2ACC = 1/0.042 #444932




# log = np.loadtxt('/home/dji/guozheng_ws/Simulator/MaRS_Offboard/src/offboard/src/gene_src/figure/log/log_data_1218_C7_6.txt')
# log = np.loadtxt('/home/dji/guozheng_ws/Simulator/MaRS_Offboard/src/offboard/src/gene_src/figure/log/log_data_1222_C13.txt')

class traj_t():
    def __init__(self):
        self.q = 0
        self.pos = 0
        self.vel = 0
        self.acc = 0
        self.omega = 0
        self.ang_acc = 0
        self.euler = np.zeros((len(log), 3))
        self.actuator_cmd = np.zeros((len(log), 4))
        self.exc_signal = 0

class ctrl_t():
    def __init__(self):
        self.pos_err = 0
        self.vel_err = 0
        self.atti_err = 0
        self.vel_cmd = 0
        self.acc_cmd = 0
        self.omega_cmd = 0
        self.q_cmd = 0
        self.thrust_cmd = 0
        self.euler_cmd = np.zeros((len(log), 3))
        self.omega_err = np.zeros((len(log), 3))
        self.torq_cmd = np.zeros((len(log), 3))

class imu_t():
    def __init__(self):
        self.gyro = 0
        self.acc = 0


def rot2euler( R ) :
    euler = np.zeros((1, 3))
    euler[0] = np.arcsin(R[2,1])
    euler[1]= -np.arctan2(R[2,0], R[2,2])
    euler[2] = -np.arctan2(R[0,1], R[1,1])
    return euler

if __name__ == '__main__':
    name = sys.argv[1]
    file_dir = name[0:len(name)-4]
    file_dir = "./"+file_dir
    isFileExists = os.path.exists("./"+name)
    isExists=os.path.exists(file_dir)

    if not isExists:
        os.makedirs(file_dir)
        print(" -- DIR ["+ file_dir+'] creat success')
    else:
        print (" -- DIR [" +file_dir+'] already exist, skip')
        
    if not isFileExists:
        print(" -- [LOAD]: Log already in a child file.")
        name = file_dir+"/"+name
    else:
        from_path = "./"+name
        desi_path = file_dir+"/"+name
        shutil.move(from_path, desi_path)
        name = file_dir+"/"+name


    log = np.loadtxt(name)
    plot_cnt = 0
    feedback = traj_t()
    trajectory = traj_t()
    control = ctrl_t()
    imu = imu_t()
    debug = np.zeros((len(log), 3))
    ######################################################################

    Time = log[:,0]

    feedback.q          = log[:,1:5]
    feedback.pos        = log[:,5:8]
    feedback.vel        = log[:,8:11]
    feedback.acc        = log[:,11:14]
    feedback.omega      = log[:,14:17]
    feedback.ang_acc    = log[:,17:20]

    trajectory.q        = log[:,20:24]
    trajectory.pos      = log[:,24:27]
    trajectory.vel      = log[:,27:30]
    trajectory.omega    = log[:,30:33]

    control.pos_err     = log[:,33:36]
    control.vel_err     = log[:,36:39]
    control.atti_err    = log[:,39:42]
    control.omega_err   = log[:,42:45]
    control.vel_cmd     = log[:,45:48]
    control.acc_cmd     = log[:,48:51]
    control.omega_cmd   = log[:,51:54]
    control.q_cmd       = log[:,54:58]
    control.thrust_cmd  = log[:,58]
    control.torq_cmd    = log[:,59:62]

    feedback.actuator_cmd = log[:,62:66]
    feedback.exc_signal = log[:,66]

    imu.gyro = log[:,67:70]
    imu.acc = log[:,70:73]

    debug = log[:,73:76]

    aT_mean = np.mean(control.acc_cmd[:,2])
    print(" -- [aT_mean]: "+ str(aT_mean))

    # actuator_mean = np.mean(feedback.actuator_cmd[20*50:,3])
    # print(actuator_mean)


    # R = Rot.from_quat([ -0.707106781, -0.707106781, 0.0, 0.0])
    # euler = R.as_euler('xyz', degrees=True)
    # print(euler)
    #########################################################################

    for index in range(len(Time)):
        if (np.linalg.norm(trajectory.q[index,:]) != 0):
            R_traj = Rot.from_quat(np.concatenate((trajectory.q[index,1:4] , trajectory.q[index,0]), axis=None))
            trajectory.euler[index,:] = R_traj.as_euler('xyz', degrees=True)
            # trajectory.euler[index,:] = rot2euler( R_traj )

        if (np.linalg.norm(feedback.q[index,:]) != 0):
            R_fb = Rot.from_quat(np.concatenate((feedback.q[index,1:4] , feedback.q[index,0]), axis=None))
            feedback.euler[index,:] = R_fb.as_euler('xyz', degrees=True)

        if (np.linalg.norm(control.q_cmd[index,:]) != 0):
            R_ctrl = Rot.from_quat(np.concatenate((control.q_cmd[index,1:4] , control.q_cmd[index,0]), axis=None))
            control.euler_cmd[index,:] = R_ctrl.as_euler('xyz', degrees=True)

            control.omega_err = control.omega_cmd - feedback.omega


    ##########################################################################

    mpl.rcParams['legend.fontsize'] = 10
    fig_size_x = 20
    fig_size_y = 15
    fig = plt.figure(0,figsize=(fig_size_x, fig_size_y))
    ax = fig.gca(projection='3d')
    ax.plot(trajectory.pos[:,0], trajectory.pos[:,1], trajectory.pos[:,2], label='Reference')
    ax.plot(feedback.pos[:,0], feedback.pos[:,1], feedback.pos[:,2], label='Feedback')
    ax.legend()
    # ax.invert_yaxis()
    # ax.invert_zaxis()
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.grid
    ax.axis('equal')
    # plt.show()
    plot_cnt=plot_cnt+1
    # save_name = "./"+name[0:len(name)-4]+"/plot"+str(plot_cnt)+".svg"
    save_name = file_dir+"/trajectory.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')

    fig = plt.figure(1,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(321)
    plt.plot(Time, trajectory.pos[:,0] , label='trajectory')
    plt.plot(Time, feedback.pos[:,0] ,label='feedback', marker = '.')
    ax1.set_ylabel('Pos_x (m)')
    ax1.legend()
    plt.grid(True)
    ax2 = plt.subplot(322)
    plt.plot(Time, trajectory.vel[:,0] )
    plt.plot(Time, feedback.vel[:,0], marker = '.')
    ax2.set_ylabel('Vel_x (m)')
    plt.grid(True)
    ax3 = plt.subplot(323)
    plt.plot(Time, trajectory.pos[:,1] )
    plt.plot(Time, feedback.pos[:,1], marker = '.' )
    ax3.set_ylabel('Pos_y (m)')
    plt.grid(True)
    ax4 = plt.subplot(324)
    plt.plot(Time, trajectory.vel[:,1] )
    plt.plot(Time, feedback.vel[:,1] , marker = '.')
    ax4.set_ylabel('Vel_y (m)')
    plt.grid(True)
    ax5 = plt.subplot(325)
    plt.plot(Time, trajectory.pos[:,2] )
    plt.plot(Time, feedback.pos[:,2], marker = '.' )
    ax5.set_ylabel('Pos_z (m)')
    plt.grid(True)
    ax6 = plt.subplot(326)
    plt.plot(Time, trajectory.vel[:,2] )
    plt.plot(Time, feedback.vel[:,2] , marker = '.')
    ax6.set_ylabel('Vel_z (m)')
    plt.grid(True)
    save_name = file_dir+"/posi_vel.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')

    fig = plt.figure(2,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(411)
    lines = plt.plot(Time, control.pos_err[:,:] )
    plt.legend(lines[:3], ['x', 'y', 'z'])
    ax1.set_ylabel('Pos err (m)')
    plt.grid(True)
    ax2 = plt.subplot(412)
    plt.plot(Time, control.vel_err[:,:] )
    ax2.set_ylabel('Vel err (m/s)')
    plt.grid(True)
    ax3 = plt.subplot(413)
    plt.plot(Time, control.atti_err[:,:]*RAD2DEG)
    ax3.set_ylabel('Atti err (deg)')
    plt.grid(True)
    ax4 = plt.subplot(414)
    plt.plot(Time, control.omega_err[:,:]*RAD2DEG )
    ax4.set_ylabel('Omega err (deg/s)')
    ax4.set_xlabel('Time (s)')
    plt.grid(True)
    save_name = file_dir+"/errors.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')

    fig = plt.figure(3,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(411)
    plt.plot(Time, feedback.actuator_cmd[:,0] )
    plt.grid(True)
    plt.title('actuator cmd')
    ax2 = plt.subplot(412)
    plt.plot(Time, feedback.actuator_cmd[:,1])
    plt.grid(True)
    ax3 = plt.subplot(413)
    plt.plot(Time, feedback.actuator_cmd[:,2])
    plt.grid(True)
    ax4 = plt.subplot(414)
    plt.plot(Time, feedback.actuator_cmd[:,3])
    plt.grid(True)
    save_name = file_dir+"/actuator_cmd.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')


    fig = plt.figure(4,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(511)
    plt.plot(Time, control.vel_cmd[:,:] )
    ax1.set_ylabel('Vel cmd (m/s)')
    plt.grid(True)
    ax2 = plt.subplot(512)
    plt.plot(Time, control.acc_cmd[:,:])
    ax2.set_ylabel('Acc cmd (m/s^2)')
    plt.grid(True)
    ax3 = plt.subplot(513)
    plt.plot(Time, control.omega_cmd[:,:]*RAD2DEG)
    ax3.set_ylabel('omega cmd (deg/s^2)')
    plt.grid(True)
    ax4 = plt.subplot(514)
    plt.plot(Time, control.thrust_cmd)
    ax4.set_ylabel('Thrust cmd ')
    plt.grid(True)
    ax4 = plt.subplot(515)
    plt.plot(Time, control.torq_cmd[:,:])
    ax4.set_ylabel('Torq cmd ')
    ax4.set_xlabel('Time (s)')
    plt.grid(True)
    save_name = file_dir+"/cmds.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')


    fig = plt.figure(5,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(311)
    plt.plot(Time, control.omega_cmd[:,0]*RAD2DEG , label='ctrl_cmd')
    plt.plot(Time, feedback.omega[:,0]*RAD2DEG , label='feedback')
    plt.plot(Time, trajectory.omega[:,0]*RAD2DEG , label='trajectory')
    ax1.set_ylabel('omega_x (deg/s)')
    ax1.legend()
    plt.grid(True)
    ax2 = plt.subplot(312)
    plt.plot(Time, control.omega_cmd[:,1]*RAD2DEG )
    plt.plot(Time, feedback.omega[:,1]*RAD2DEG )
    plt.plot(Time, trajectory.omega[:,1]*RAD2DEG)
    ax2.set_ylabel('omega_y (deg/s)')
    plt.grid(True)
    ax3 = plt.subplot(313)
    plt.plot(Time, control.omega_cmd[:,2]*RAD2DEG )
    plt.plot(Time, feedback.omega[:,2]*RAD2DEG )
    plt.plot(Time, trajectory.omega[:,2]*RAD2DEG )
    ax3.set_ylabel('omega_z (deg/s)')
    ax3.set_xlabel('Time (s)')
    plt.grid(True)
    save_name = file_dir+"/omega.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')



    fig = plt.figure(6,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(311)
    plt.plot(Time, trajectory.euler[:,0] , label='trajectory')
    plt.plot(Time, feedback.euler[:,0] , label='feedback', marker = '.')
    plt.plot(Time, control.euler_cmd[:,0] , label='ctrl_cmd')
    ax1.set_ylabel('Roll (deg)')
    ax1.legend()
    plt.grid(True)
    ax2 = plt.subplot(312)
    plt.plot(Time, trajectory.euler[:,1] )
    plt.plot(Time, feedback.euler[:,1] , marker = '.')
    plt.plot(Time, control.euler_cmd[:,1] )
    ax2.set_ylabel('Pitch (deg)')
    plt.grid(True)
    ax3 = plt.subplot(313)
    plt.plot(Time, trajectory.euler[:,2] )
    plt.plot(Time, feedback.euler[:,2] , marker = '.')
    plt.plot(Time, control.euler_cmd[:,2] )
    ax3.set_ylabel('Yaw (deg)')
    ax3.set_xlabel('Time (s)')
    plt.grid(True)
    save_name = file_dir+"/orientation.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')

    fig = plt.figure(7,figsize=(fig_size_x, fig_size_y))
    ax1 = plt.subplot(321)
    plt.plot(Time, imu.gyro[:,0] *RAD2DEG  , label='gyro')
    plt.plot(Time, feedback.euler[:,0] +1 , label='feedback euler')
    ax1.set_ylabel('gyro_euler_x (m)')
    ax1.legend()
    plt.grid(True)
    ax2 = plt.subplot(322)
    plt.plot(Time, imu.acc[:,0] )
    # plt.plot(Time, feedback.vel[:,0] )
    ax2.set_ylabel('acc_x (m)')
    plt.grid(True)
    ax3 = plt.subplot(323)
    plt.plot(Time, imu.gyro[:,1] *RAD2DEG )
    plt.plot(Time, feedback.euler[:,1] -1)
    ax3.set_ylabel('gyro_euler_y (m)')
    plt.grid(True)
    ax4 = plt.subplot(324)
    plt.plot(Time, imu.acc[:,1] )
    # plt.plot(Time, feedback.vel[:,1] )
    ax4.set_ylabel('acc_y (m)')
    plt.grid(True)
    ax5 = plt.subplot(325)
    plt.plot(Time, imu.gyro[:,2] *RAD2DEG)
    plt.plot(Time, feedback.euler[:,2] -90)
    ax5.set_ylabel('gyro_euler_z (m)')
    plt.grid(True)
    ax6 = plt.subplot(326)
    plt.plot(Time, imu.acc[:,2] )
    # plt.plot(Time, feedback.vel[:,2] )
    ax6.set_ylabel('acc_z (m)')
    plt.grid(True)
    save_name = file_dir+"/acceleration.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')

    fig = plt.figure(8,figsize=(fig_size_x, fig_size_y))
    ax = fig.gca()
    ax.plot(Time, -imu.acc[:,2], label='acc_z')
    ax.plot(Time, control.thrust_cmd*THRUST2ACC, label='a_T')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acc cmd (m/s^2)')
    ax.legend()
    ax.grid(True)
    save_name = file_dir+"/thrust.svg"
    print(" -- [SAVE SUCCESS]" + save_name)
    plt.savefig(save_name, format='svg')
    # fig = plt.figure(50)
    # plt.plot(debug[:,0], marker = '.')
    # plt.grid(True)
    # fig = plt.figure(51)
    # plt.plot(debug[:,1], marker = '.')
    # plt.grid(True)
    # fig = plt.figure(52)
    # plt.plot(debug[:,2], marker = '.')
    # plt.grid(True)

    ctrl_run_time_mean = np.mean(debug[:,1])*1000
    print(" -- [ctrl runtime mean]: "+str(ctrl_run_time_mean))


