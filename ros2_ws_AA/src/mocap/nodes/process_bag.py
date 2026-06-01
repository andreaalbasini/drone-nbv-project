#!/usr/bin/env python

import rosbag

import numpy as np
import tf

import vispy as vp
import vispy.scene



def draw(event):
    if canvas.events.key_press.blocked():
        canvas.events.key_press.unblock()

def key_press(event):
    canvas.events.key_press.block()
    if event.key == 'Q' or event.key == 'Escape':
        canvas.close()
        vp.app.quit()


if __name__ == '__main__':
    # bag = rosbag.Bag('/home/plusk01/Documents/vicon/2022-03-14-16-08-59.bag', 'r')
    # bag = rosbag.Bag('/home/plusk01/Documents/vicon/2022-03-14-16-39-25.bag', 'r')
    bag = rosbag.Bag('/home/plusk01/Documents/vicon/2022-03-14-17-01-52.bag', 'r')

    camerasmsg = None
    for topic, msg, t in bag.read_messages(topics=['/vicon/cameras']):
        camerasmsg = msg
        break



    canvas = vp.scene.SceneCanvas(keys='interactive', show=True, size=(1600,1200), bgcolor='black')
    canvas.events.key_press.connect(key_press)
    canvas.events.draw.connect(draw)

    view = canvas.central_widget.add_view()
    view.camera = vp.scene.cameras.TurntableCamera(fov=0)

    # world origin
    Fw = vp.scene.visuals.XYZAxis(width=3, parent=view.scene)
    Fw.transform = vp.visuals.transforms.MatrixTransform()
    # Fw.transform.scale(np.ones(3,)*0.05)

    h = np.mean([c.pose.position.z for c in camerasmsg.cameras]) * 2.10

    for cameramsg in camerasmsg.cameras:

        q = (cameramsg.pose.orientation.x, cameramsg.pose.orientation.y,
                cameramsg.pose.orientation.z, cameramsg.pose.orientation.w)
        T_WC = tf.transformations.quaternion_matrix(q)
        T_WC[:3,3] = (cameramsg.pose.position.x, cameramsg.pose.position.y,
                        cameramsg.pose.position.z)

        # a hack to fix camera orientations
        T_WC = tf.transformations.rotation_matrix(np.pi/2, (0,0,1)) @ T_WC
        T_WC[:3,:3] = T_WC[:3,:3].T
        T_WC = tf.transformations.rotation_matrix(-np.pi/2, (0,0,1)) @ T_WC

        # import ipdb; ipdb.set_trace()

        Fc = vp.scene.visuals.XYZAxis(width=2, parent=view.scene)
        Fc.transform = vp.visuals.transforms.MatrixTransform()
        # Fc.transform.scale(np.ones(3,)*0.025)
        Fc.transform.matrix = T_WC.T


        K = np.eye(3)
        K[0,0] = K[1,1] = cameramsg.camera_info.K[0]
        K[0,2] = cameramsg.camera_info.K[2]
        K[1,2] = cameramsg.camera_info.K[5]

        for centroid in cameramsg.centroids:

            # m = vp.scene.visuals.Markers(parent=view.scene)
            # m.set_data(np.zeros((1,3)))

            px = np.array((centroid.u, centroid.v, 1))

            pt0 = np.linalg.inv(K) @ px
            wpt0 = T_WC @ np.hstack((pt0,1))
            pt1 = h * np.linalg.inv(K) @ px
            wpt1 = T_WC @ np.hstack((pt1,1))


            L = np.zeros((2,3))
            L[0,:] = wpt0[:3]
            L[1,:] = wpt1[:3]

            l = vp.scene.visuals.Line(parent=view.scene)
            l.set_data(L)


            # import ipdb; ipdb.set_trace()



    # L = np.zeros((3,2))
    # L[:,0] = (0,0,0)
    # L[:,1] = (1,0,0)

    # L = np.zeros((2,3))
    # L[0,:] = (0,0,0)
    # L[1,:] = (1,1,1)
    # l = vp.scene.visuals.Line(parent=view.scene)
    # l.set_data(L)

    # import ipdb; ipdb.set_trace()

    vp.app.run()


    # import ipdb; ipdb.set_trace()