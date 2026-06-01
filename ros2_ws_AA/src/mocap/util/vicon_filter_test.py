import rosbag_pandas
import matplotlib.pyplot as plt
import matplotlib
import rosbag
import numpy as np
import pandas as pd


def get_df(bag_filename,topic_name):
    return rosbag_pandas.bag_to_dataframe(bag_filename,include=[topic_name])

def to_column_name(topic_name,field_name):
    topic_name_str = topic_name.replace('/','_')
    if topic_name_str[0] == '_':
        topic_name_str = topic_name_str[1:]
    topic_name_str = topic_name_str + "__" + field_name.replace(".","_")
    return topic_name_str

def get_values(df, topic_name,field_name,start_index = None, end_index = None):
    return df.ix[start_index:end_index,to_column_name(topic_name,field_name)].values


# bag_filename = 'raw_with_marker_7_sim.bag'
# veh_name = 'tracker_sim'
bag_filename = 'compare_sim.bag'
veh_name = 'tracker_sim_new'
bag = rosbag.Bag(bag_filename)
topics = bag.get_type_and_topic_info().topics.keys()
# print topics

df_raw = rosbag_pandas.bag_to_dataframe(bag_filename,include=['/%s/viconraw' %(veh_name)])
df_flip = rosbag_pandas.bag_to_dataframe(bag_filename,include=['/%s/measurement' %(veh_name)])
df_filter = rosbag_pandas.bag_to_dataframe(bag_filename,include=['/%s/pose' %(veh_name)])

# df_diff = rosbag_pandas.bag_to_dataframe(bag_filename,include=['/tracker_sim/anglediff'])
# print "df_raw fields: %s" %(df_raw.columns.values.tolist())
# print "df_flip fields: %s" %(df_flip.columns.values.tolist())
# print "df_filter fields: %s" %(df_filter.columns.values.tolist())

plt.close("all")
fig_ori = plt.figure(1)
index_start = None
index_end = None

# === Plot orientations === #
for i,field_char in enumerate(['x','y','z','w']):
    plt.subplot(5,1,i+1)
    plt.plot(df_flip.ix[index_start:index_end,'%s_measurement__pose_orientation_'%(veh_name) + field_char],label='flipped',linewidth=4.0,color='b')
    plt.plot(df_filter.ix[index_start:index_end,'%s_pose__pose_orientation_'%(veh_name) + field_char],label='filtered',linewidth=2.0,color='g')
    plt.ylabel(field_char)
    plt.legend()

# # Plot viconraw (Coordinate transformaiton requried)
# for i,field_char in enumerate(['y','x','z','w']):
#     plt.subplot(4,1,i+1)
#     flip_temp = 1.0
#     if field_char=='x':
#         flip_temp = -1.0
#     plt.plot(flip_temp*df_raw.ix[index_start:index_end,'%s_viconraw__measurement_orientation_'%(veh_name) + field_char],label='raw',color='r')
#     plt.legend()

plt.subplot(5,1,5)
plt.plot(df_raw.ix[index_start:index_end,'%s_viconraw__marker_count_data' %(veh_name)])
plt.ylabel('Maker Count')

# ===Plot positions=== #
fig_pos = plt.figure(2)
for i,field_char in enumerate(['x','y','z']):
    plt.subplot(4,1,i+1)
    plt.plot(df_flip.ix[index_start:index_end,'%s_measurement__pose_position_' %(veh_name)+field_char],label='flipped',linewidth=4.0,color='b')
    plt.plot(df_filter.ix[index_start:index_end,'%s_pose__pose_position_' %(veh_name)+field_char],label='filtered',linewidth=2.0,color='g')
    plt.ylabel(field_char)
    plt.legend()

# # ====Plot Viconraw====#
# for i,field_char in enumerate(['y','x','z']):
#     plt.subplot(4,1,i+1)
#     flip_temp = 1.0
#     if field_char=='x':
#         flip_temp = -1.0
#     plt.plot(flip_temp*df_raw.ix[index_start:index_end,'%s_viconraw__measurement_position_' %(veh_name)+field_char],label='raw',color='r')
#     plt.legend()

plt.subplot(4,1,4)
plt.plot(df_raw.ix[index_start:index_end,'%s_viconraw__marker_count_data' %(veh_name)])
plt.ylabel('Maker Count')

plt.show()









