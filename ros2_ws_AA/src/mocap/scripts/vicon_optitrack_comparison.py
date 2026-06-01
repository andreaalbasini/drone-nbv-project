#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python world_comparison.py
#  Ex. python world_comparison.py

# change cd and dc

import bagpy
from bagpy import bagreader
import pandas as pd
import rosbag
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import tf
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import sys
import scipy
import matplotlib.font_manager as font_manager
import matplotlib.patches as mpl_patches
from matplotlib.ticker import PercentFormatter
from statistics import mean

if __name__ == '__main__':

    #### font
    font = font_manager.FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(8)

    #### params
    num_data = 7 #position x, y, z. orientation x, y, z, w
    agents = ['NX03', 'NX08', 'NX09']
    bagname = "test3/2022-11-09-16-05-13.bag"
    offset_num_msgs = 100

    #### list initialization
    ### vicon[idx(agent id)][data (ig. x)][actual msgs value]
    vicon = [[[] for i in range(num_data)] for j in range(len(agents))]
    vicon_t_list = [[] for i in range(len(agents))]
    vicon_velocity = [[[] for i in range(num_data)] for j in range(len(agents))]
    vicon_t_list_vel = [[] for i in range(len(agents))]
    optitrack = [[[] for i in range(num_data)] for j in range(len(agents))]
    optitrack_t_list = [[] for i in range(len(agents))]

    #### data extraction
    with rosbag.Bag(bagname) as outbag:
        for topic, msg, t in rosbag.Bag(bagname).read_messages():
            for idx, agent in enumerate(agents):
                if topic == '/'+agent+'/world':
                    vicon[idx][0].append(float(msg.pose.position.x))
                    vicon[idx][1].append(float(msg.pose.position.y))
                    vicon[idx][2].append(float(msg.pose.position.z))
                    vicon[idx][3].append(float(msg.pose.orientation.x))
                    vicon[idx][4].append(float(msg.pose.orientation.y))
                    vicon[idx][5].append(float(msg.pose.orientation.z))
                    vicon[idx][6].append(float(msg.pose.orientation.w))
                    vicon_t_list[idx].append(msg.header.stamp.secs + msg.header.stamp.nsecs/10**9)
                if topic == '/optitrack/'+agent+'/world':
                    # seconds = msg.header.stamp.secs
                    # nseconds = msg.header.stamp.secs
                    optitrack[idx][0].append(float(msg.pose.position.x))
                    optitrack[idx][1].append(float(msg.pose.position.y))
                    optitrack[idx][2].append(float(msg.pose.position.z))
                    optitrack[idx][3].append(float(msg.pose.orientation.x))
                    optitrack[idx][4].append(float(msg.pose.orientation.y))
                    optitrack[idx][5].append(float(msg.pose.orientation.z))
                    optitrack[idx][6].append(float(msg.pose.orientation.w))
                    optitrack_t_list[idx].append(msg.header.stamp.secs + msg.header.stamp.nsecs/10**9)
                if topic == '/'+agent+'/mocap/twist':
                    vicon_velocity[idx][0].append(float(msg.twist.linear.x))
                    vicon_velocity[idx][1].append(float(msg.twist.linear.y))
                    vicon_velocity[idx][2].append(float(msg.twist.linear.z))
                    vicon_t_list_vel[idx].append(msg.header.stamp.secs + msg.header.stamp.nsecs/10**9)
    
    #### offset calculation
    offset = [[[] for i in range(num_data)] for j in range(len(agents))]
    mean_offset = [[[] for i in range(num_data)] for j in range(len(agents))]
    for idx, agent in enumerate(agents):
        # calculate offset
        ## data
        for i in range(offset_num_msgs):
            offset[idx][0].append(vicon[idx][0][i] - optitrack[idx][0][i])
            offset[idx][1].append(vicon[idx][1][i] - optitrack[idx][1][i])
            offset[idx][2].append(vicon[idx][2][i] - optitrack[idx][2][i])
            offset[idx][3].append(vicon[idx][3][i] - optitrack[idx][3][i])
            offset[idx][4].append(vicon[idx][4][i] - optitrack[idx][4][i])
            offset[idx][5].append(vicon[idx][5][i] - optitrack[idx][5][i])
            offset[idx][6].append(vicon[idx][6][i] - optitrack[idx][6][i])
        ## time
        time_offset = vicon_t_list[idx][0] - optitrack_t_list[idx][0]

        # un-offset optitrack
        ## data
        for i in range(num_data):
            mean_offset[idx][i].append(mean(offset[idx][i][:]))
            # print('bef offset', optitrack[idx][i][1:10])
            optitrack[idx][i] = [x+mean_offset[idx][i][0] for x in optitrack[idx][i]]
            # print('aft offset', optitrack[idx][i][1:10])
        ## time
        optitrack_t_list[idx] = [x+time_offset for x in optitrack_t_list[idx]]

    #### offset printout
    print("offset")
    for idx, agent in enumerate(agents):
        print(agent + ": pos x = ",round(mean_offset[idx][0][0],2))
        print(agent + ": pos y = ",round(mean_offset[idx][1][0],2))
        print(agent + ": pos z = ",round(mean_offset[idx][2][0],2))
        print(agent + ": qx = ",round(mean_offset[idx][3][0],2))
        print(agent + ": qy = ",round(mean_offset[idx][4][0],2))
        print(agent + ": qz = ",round(mean_offset[idx][5][0],2))
        print(agent + ": qw = ",round(mean_offset[idx][6][0],2))

    # import pdb; pdb.set_trace();

    #### calculate err
    print('if these two are two different you need to time sync')
    print('vicon list length', len(vicon[0][0]))
    print('optitrack list length', len(optitrack[0][0]))
    err = [[[] for i in range(num_data)] for j in range(len(agents))]
    for idx, agent in enumerate(agents):
        for i in range(num_data):
            for j in range(len(vicon_t_list[idx])):
                err[idx][i].append(vicon[idx][i][j] - optitrack[idx][i][j])

    ##################################################################################
    #################################### PLOT ########################################
    ##################################################################################
    
    #### xyz
    fig, axs = plt.subplots(2,2)
    fig.tight_layout(pad=2)
    axes = []
    axes.append(axs[0,0])
    axes.append(axs[0,1])
    axes.append(axs[1,0])
    axes.append(axs[1,1])
    ## xy
    for idx, agent in enumerate(agents):
        axes[idx].plot(vicon[idx][0], vicon[idx][1], label=agents[idx]+'/vicon')
        axes[idx].plot(optitrack[idx][0], optitrack[idx][1], label=agents[idx]+'/optitrack')
    ## z
    for idx, agent in enumerate(agents):
        axes[3].plot(vicon_t_list[idx], vicon[idx][2], label=agents[idx]+'/vicon')
        axes[3].plot(optitrack_t_list[idx], optitrack[idx][2], label=agents[idx]+'/optitrack')
    for i in range(3):
        axes[i].legend(prop=font)
        axes[i].set_title(agents[i], font=font)
        axes[i].grid(color='black', linewidth=1, alpha=0.2)
        axes[i].set_ylabel('y [m]', font=font)
        axes[i].set_xlabel('x [m]', font=font)
        axes[i].tick_params(labelsize=5)
    axes[3].set_title('z axis', font=font)
    axes[3].legend(prop={'size': 6})
    axes[3].grid(color='black', linewidth=1, alpha=0.2)
    axes[3].set_xlabel('time', font=font)
    axes[3].set_ylabel('z [m]', font=font)
    axes[3].tick_params(labelsize=5)
    plt.savefig("/home/kota/data/optitrack_vicon/test3/vicon_optitrack.png", bbox_inches="tight")
    # plt.show()
    plt.close()

    # time history of err btwn vicon and optitrack
    # len(vicon[0][0]) is a little bit shorter than len(optitrack[0][0])
    # so we will use vicon_t_list
    for idx, agent in enumerate(agents):
        fig, axs = plt.subplots(2,1)
        fig.tight_layout(pad=2)
        fig.suptitle(agent, font=font)
        axs[0].plot(vicon_t_list[idx], err[idx][0], label='x err')
        axs[0].plot(vicon_t_list[idx], err[idx][1], label='y err')
        axs[0].plot(vicon_t_list[idx], err[idx][2], label='z err')
        axs[1].plot(vicon_t_list[idx], err[idx][3], label='qx err')
        axs[1].plot(vicon_t_list[idx], err[idx][4], label='qy err')
        axs[1].plot(vicon_t_list[idx], err[idx][5], label='qz err')
        axs[1].plot(vicon_t_list[idx], err[idx][6], label='qw err')
        for i in range(2):
            axs[i].legend(prop=font)
            axs[i].grid(color='black', linewidth=1, alpha=0.2)
            axs[i].set_xlabel('time [s]', font=font)
        axs[0].set_ylabel('err position [m]', font=font)
        axs[1].set_ylabel('err quaternion', font=font)
        plt.savefig("/home/kota/data/optitrack_vicon/test3/time_history_"+agent+".png", bbox_inches="tight")
        # plt.show()
        plt.close()

    # time history of err and actual value in z on NX03 (optitrack has bad tracking in z on NX03) 
    # so we will use vicon_t_list
    fig, axs = plt.subplots(2,1)
    fig.tight_layout(pad=2)
    fig.suptitle(agent, font=font)
    axs[0].plot(vicon_t_list[0], err[0][2], label='z err')
    axs[1].plot(vicon_t_list[0], vicon[0][2], label='z pos (vicon)')
    axs[1].plot(vicon_t_list_vel[0], vicon_velocity[0][2], label='z vel (vicon)')
    for i in range(2):
        axs[i].legend(prop=font)
        axs[i].grid(color='black', linewidth=1, alpha=0.2)
        axs[i].set_xlabel('time [s]', font=font)
    axs[0].set_ylabel('err in z [m]', font=font)
    axs[1].set_ylabel('actual value in z [m and m/s]', font=font)
    plt.savefig("/home/kota/data/optitrack_vicon/test3/time_history_NX03_z_invest.png", bbox_inches="tight")
    # plt.show()
    plt.close()

    # NX09 qz and qw's signs are flipped (maybe the rigid body creation error)
    idx = 2
    agent = 'NX09'

    fig, axs = plt.subplots(2,1)
    fig.tight_layout(pad=2)
    fig.suptitle(agent, font=font)
    axs[0].plot(vicon_t_list[idx], vicon[idx][5], label='qz vicon')
    axs[1].plot(optitrack_t_list[idx], optitrack[idx][5], label='qz optitrack')
    axs[0].plot(vicon_t_list[idx], vicon[idx][6], label='qw vicon')
    axs[1].plot(optitrack_t_list[idx], optitrack[idx][6], label='qw optitrack')
    for i in range(2):
        axs[i].legend(prop=font)
        axs[i].grid(color='black', linewidth=1, alpha=0.2)
        axs[i].set_xlabel('time [s]', font=font)
    axs[0].set_ylabel('err position [m]', font=font)
    axs[1].set_ylabel('err quaternion', font=font)
    plt.savefig("/home/kota/data/optitrack_vicon/test3/time_history_NX09_investigation.png", bbox_inches="tight")
    # plt.show()
    plt.close()

    # NX09 quaternion to rpy
    idx = 2
    agent = 'NX09'

    vicon_rpy = [[] for i in range(3)]
    optitrack_rpy = [[] for i in range(3)]

    vqx = vicon[idx][3]
    vqy = vicon[idx][4]
    vqz = vicon[idx][5]
    vqw = vicon[idx][6]

    oqx = optitrack[idx][3]
    oqy = optitrack[idx][4]
    oqz = optitrack[idx][5]
    oqw = optitrack[idx][6]

    for i in range(len(vicon[idx][5])):
        vicon_rpy[0].append(math.atan2(2.0*(vqy[i]*vqz[i] + vqw[i]*vqx[i]), vqw[i]*vqw[i] - vqx[i]*vqx[i] - vqy[i]*vqy[i] + vqz[i]*vqz[i])) #roll
        vicon_rpy[1].append(math.asin(-2.0*(vqx[i]*vqz[i] - vqw[i]*vqy[i]))) #pitch
        vicon_rpy[2].append(math.atan2(2.0*(vqx[i]*vqy[i] + vqw[i]*vqz[i]), vqw[i]*vqw[i] + vqx[i]*vqx[i] - vqy[i]*vqy[i] - vqz[i]*vqz[i])) #yaw
        optitrack_rpy[0].append(math.atan2(2.0*(oqy[i]*oqz[i] + oqw[i]*oqx[i]), oqw[i]*oqw[i] - oqx[i]*oqx[i] - oqy[i]*oqy[i] + oqz[i]*oqz[i])) #roll
        optitrack_rpy[1].append(math.asin(-2.0*(oqx[i]*oqz[i] - oqw[i]*oqy[i]))) #pitch
        optitrack_rpy[2].append(math.atan2(2.0*(oqx[i]*oqy[i] + oqw[i]*oqz[i]), oqw[i]*oqw[i] + oqx[i]*oqx[i] - oqy[i]*oqy[i] - oqz[i]*oqz[i])) #yaw

    fig, axs = plt.subplots(2,1)
    fig.tight_layout(pad=2)
    fig.suptitle(agent, font=font)
    axs[0].plot(vicon_t_list[idx], vicon_rpy[0], label='roll vicon')
    axs[0].plot(vicon_t_list[idx], vicon_rpy[1], label='pitch vicon')
    axs[0].plot(vicon_t_list[idx], vicon_rpy[2], label='yaw vicon')
    axs[1].plot(vicon_t_list[idx], optitrack_rpy[0], label='roll optitrack')
    axs[1].plot(vicon_t_list[idx], optitrack_rpy[1], label='pitch optitrack')
    axs[1].plot(vicon_t_list[idx], optitrack_rpy[2], label='yaw optitrack')
    for i in range(2):
        axs[i].legend(prop=font)
        axs[i].grid(color='black', linewidth=1, alpha=0.2)
        axs[i].set_xlabel('time [s]', font=font)
    axs[0].set_ylabel('rpy vicon [rad]', font=font)
    axs[1].set_ylabel('rpy optitrack [rad]', font=font)
    plt.savefig("/home/kota/data/optitrack_vicon/test3/time_history_NX09_rpy.png", bbox_inches="tight")
    # plt.show()
    plt.close()
