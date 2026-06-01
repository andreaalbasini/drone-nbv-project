#!/usr/bin/env python

import rosbag

import numpy as np
from scipy.spatial.transform import Rotation as R

import vispy as vp
import vispy.scene

import xmlrpc.client

def draw(event):
    if canvas.events.key_press.blocked():
        canvas.events.key_press.unblock()

def key_press(event):
    canvas.events.key_press.block()
    if event.key == 'Q' or event.key == 'Escape':
        canvas.close()
        vp.app.quit()


if __name__ == '__main__':
    
    URI = 'http://192.168.0.9:1250'
    proxy = xmlrpc.client.ServerProxy(URI)

    cameras = proxy.getCameras()


    canvas = vp.scene.SceneCanvas(keys='interactive', show=True, size=(1600,1200), bgcolor='black')
    canvas.events.key_press.connect(key_press)
    canvas.events.draw.connect(draw)

    view = canvas.central_widget.add_view()
    view.camera = vp.scene.cameras.TurntableCamera(fov=0)

    # world origin
    Fw = vp.scene.visuals.XYZAxis(width=3, parent=view.scene)
    Fw.transform = vp.visuals.transforms.MatrixTransform()
    # Fw.transform.scale(np.ones(3,)*0.05)

    # h = np.mean([c.pose.position.z for c in camerasmsg.cameras]) * 1.10

    for camera in cameras:

        if not camera['enabled']:
            continue

        T_WC = np.eye(4)
        qx, qy, qz, qw = camera['orientation']
        T_WC[:3,:3] = R.from_quat((qx, qy, qz, qw)).as_matrix().T
        T_WC[:3,3] = camera['position']

        print(f"C#{camera['index']+1}: {camera['orientation']}")

        # T_WC = tf.transformations.rotation_matrix(-np.pi/2, (0,0,1)) @ T_WC

        # import ipdb; ipdb.set_trace()

        Fc = vp.scene.visuals.XYZAxis(width=1, parent=view.scene)
        Fc.transform = vp.visuals.transforms.MatrixTransform()
        # Fc.transform.scale(np.ones(3,)*0.025)
        Fc.transform.matrix = T_WC.T


        # K = np.eye(3)
        # K[0,0] = K[1,1] = cameramsg.camera_info.K[0]
        # K[0,2] = cameramsg.camera_info.K[2]
        # K[1,2] = cameramsg.camera_info.K[5]

        # for centroid in cameramsg.centroids:

        #     # m = vp.scene.visuals.Markers(parent=view.scene)
        #     # m.set_data(np.zeros((1,3)))

        #     px = np.array((centroid.u, centroid.v, 1))

        #     pt0 = np.linalg.inv(K) @ px
        #     wpt0 = T_WC @ np.hstack((pt0,1))
        #     pt1 = h * np.linalg.inv(K) @ px
        #     wpt1 = T_WC @ np.hstack((pt1,1))


        #     L = np.zeros((2,3))
        #     L[0,:] = wpt0[:3]
        #     L[1,:] = wpt1[:3]

        #     # l = vp.scene.visuals.Line(parent=view.scene)
        #     # l.set_data(L)


        #     # import ipdb; ipdb.set_trace()



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