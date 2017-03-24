import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
# from FilterLib import P
import math
import SimpleKF
import SimplePF
# import numba

# @jit
def findRealPose(Data,time_pose):

    Pose = np.zeros([Data.shape[0],3])
    time_stamp = np.zeros(Data.shape[0])



    for i in range(Data.shape[0]):
        Pose[i,:] = (time_pose[np.argmin(np.abs(Data[i,0]-time_pose[:,0])),1:])
        time_stamp[i] = time_pose[np.argmin(np.abs(Data[i,0]-time_pose[:,0])),0]
        # tmp_centre = np.mean(time_pose[np.argmin(np.abs(Data[i,0]-time_pose[:,0])),1:],0)
        # if np.linalg.norm(Pose[i,:]-tmp_centre) > 1.5:
        #     Pose[i,:] = Pose[i-1,:]
        Pose[i,1] *= -1.0
        # Pose[i,0] += 1.0
    tPose = np.zeros_like(Pose)
    tPose = np.copy(Pose)

    plt.figure(123)
    # plt.plot(time_stamp-Pose[:,0],'r-+')
    plt.plot(time_stamp-Data[:,0],'y-.')
    # plt.plot(time_stamp,'r-.')
    # plt.plot(Data[:,0],'b-.')

    # kf = SimpleKF.KFSimple(Pose[0,:])
    # kf = SimplePF.PFSimple(Pose[0,:],100)
    #
    # for i in range(Data.shape[0]):
    #     Pose[i,:] = kf.Process(State = Pose[i,:])




    # for i in range(5,Data.shape[0]-15):
    #     if np.mean(np.linalg.norm(tPose[i,:]-tPose[i-2:i,:],1))>0.5 :
    #         Pose[i,:] = 2*tPose[i-1,:]-tPose[i-2,:]
            # v1 = tPose[i,:]-tPose[i-1,:]
            # v2 = tPose[i,:]-tPose[i-2,:]
            # print(np.abs(np.sum(v1*v2))/np.linalg.norm(v1)/np.linalg.norm(v2))
            # if (np.abs(np.sum(v1*v2))/np.linalg.norm(v1)/np.linalg.norm(v2)) < 0.8:
            #     Pose[i,:] = 2*Pose[i-1,:]-Pose[i-2,:]


        #     a = 1
        # near_dis = np.sum((Pose[i,:]-Pose[i-3:i+3,:])**2.0,1)**0.5
        # near_dis = np.sort(near_dis)
        # print(near_dis)
        # for k in range(1,near_dis.shape[0]):
        #     if near_dis[k]>0.5
        # print(near_dis.shape)
        # if near_dis[1]>0.5 and near_dis[2] >0.5 and near_dis[4]>0.5 and near_dis[5] > 0.5:
        #     print(near_dis)
        #     Pose[i,:] = np.mean(Pose[[i-2,i-1,i+1,i+2],:],0)
        #     plt.plot(Pose[i,0],Pose[i,1],'yD')


    # left offset
    Pose[:-18,:] = Pose[18:,:]

    return Pose



if __name__ == '__main__':
    time_pose = np.loadtxt("time2pose.csv",delimiter=',')

    # print(time_pose)
    plt.figure(2)
    plt.title('time pose real')
    plt.plot(time_pose[:,1],time_pose[:,2],'b-')
    plt.grid(True)

    dir_name = "/home/steve/Data/locate/5"

    # np.savetxt(dir_name+'UwbData.data',dc.UwbData)
# np.savetxt(dir_name+'UwbData.data.csv',dc.UwbData,delimiter=',')
#
# np.savetxt(dir_name+'UwbResult.data',dc.UWBResult)
# np.savetxt(dir_name+'UwbResult.data.csv',dc.UWBResult,delimiter=',')

    UwbData = np.loadtxt(dir_name+'UwbData.data')
    ImuData = np.loadtxt(dir_name+'ImuData.data')

    UwbRealPose = findRealPose(UwbData,time_pose)
    ImuRealPose = findRealPose(ImuData,time_pose)

    np.savetxt(dir_name+'UwbRealPose.data.csv',UwbRealPose,delimiter=',')
    np.savetxt(dir_name+'ImuRealPose.data.csv',ImuRealPose,delimiter=',')

    plt.figure(1)
    plt.grid(True)
    plt.plot(UwbRealPose[:,0],UwbRealPose[:,1],'r+-')
    plt.figure(3)
    plt.grid(True)
    plt.plot(ImuRealPose[:,0],ImuRealPose[:,1],'b+-')
    plt.show()


