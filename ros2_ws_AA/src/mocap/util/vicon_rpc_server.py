"""
VICON RPC Server
----------------

This script should always be running on ARMSTRONG (Windows computer). It's
purpose is to give the mocap ROS node on Sikorsky access to VSK files and
calibration files.

VSK files contain the expected marker positions for a given rigid body model.
The calibration file contains the intrinsics and extrinsics of each camera.

Parker Lusk
plusk@mit.edu
12 March 2022
"""

import argparse
import pathlib
import socket
import sys
import threading

import xml.etree.ElementTree as ET

from xmlrpc.server import SimpleXMLRPCServer

VSK_DIR = 'C:/ProgramData/Vicon/Tracker/Objects'
CAL_DIR = 'C:/ProgramData/Vicon/Calibrations'
# VSK_DIR = '/home/plusk01/Documents/vicon'
# CAL_DIR = '/home/plusk01/Documents/vicon/calib'

# this will be set in main
args = None


class Camera:
    ALLOWED_DISABLED_PROPS = ('enabled', 'index', 'deviceid', 'type', 'image_size')
    def __init__(self, ecamera):
        self.ecamera = ecamera

    def as_dict(self):
        props = [p for p in dir(Camera) if isinstance(getattr(Camera, p), property)]
        def should_use(p):
            return self.enabled or p in Camera.ALLOWED_DISABLED_PROPS
        return {p:getattr(self, p) for p in props if should_use(p)}

    @property
    def enabled(self):
        keyframe = self.ecamera.find('.//KeyFrames/KeyFrame')
        return keyframe is not None

    @property
    def index(self):
        return int(self.ecamera.attrib['USERID'])

    @property
    def deviceid(self):
        return int(self.ecamera.attrib['DEVICEID'])

    @property
    def type(self):
        return self.ecamera.attrib['DISPLAY_TYPE']

    @property
    def image_size(self):
        size_str = self.ecamera.attrib['SENSOR_SIZE'].split()
        return [int(v) for v in size_str] # horizontal, vertical
    
    @property
    def position(self):
        pos_mm_str = self.ecamera.find('.//KeyFrames/KeyFrame').attrib['POSITION'].split()
        return [float(mm) * 1e-3  for mm in pos_mm_str]

    @property
    def orientation(self):
        q_str = self.ecamera.find('.//KeyFrames/KeyFrame').attrib['ORIENTATION'].split()
        qx, qy, qz, qw = [float(v) for v in q_str] # R_CW
        return (qw, -qx, -qy, -qz) # R_WC

    @property
    def image_error(self):
        return float(self.ecamera.find('.//KeyFrames/KeyFrame').attrib['IMAGE_ERROR'])

    @property
    def focal_length(self):
        return float(self.ecamera.find('.//KeyFrames/KeyFrame').attrib['FOCAL_LENGTH'])
    
    @property
    def principal_point(self):
        pp_str = self.ecamera.find('.//KeyFrames/KeyFrame').attrib['PRINCIPAL_POINT'].split()
        return [float(v) for v in pp_str]

    @property
    def vicon_radial(self):
        vr_str = self.ecamera.find('.//KeyFrames/KeyFrame').attrib['VICON_RADIAL'].split()
        return [float(v) for v in vr_str]

def parse_cameras(file):
    tree = ET.parse(file)
    root = tree.getroot()

    cameras = []
    ecameras = root.findall('Camera')
    for ecamera in ecameras:
        cameras.append(Camera(ecamera))
    return cameras


class Marker:
    def __init__(self, emarker, epx, epy, epz):
        self.emarker = emarker
        self.epx = epx
        self.epy = epy
        self.epz = epz

    def as_dict(self):
        props = [p for p in dir(Marker) if isinstance(getattr(Marker, p), property)]
        return {p:getattr(self, p) for p in props}

    @property
    def name(self):
        return self.emarker.attrib['NAME']

    @property
    def position(self):
        pos_mm = [float(e.attrib['VALUE']) for e in (self.epx, self.epy, self.epz)]
        return [mm * 1e-3 for mm in pos_mm]

    @position.setter
    def position(self, pos):
        pos_mm = pos * 1e3
        self.epx.attrib['VALUE'] = str(pos_mm[0])
        self.epy.attrib['VALUE'] = str(pos_mm[1])
        self.epz.attrib['VALUE'] = str(pos_mm[2])
    
def parse_markers(modelname, file):
    tree = ET.parse(file)
    root = tree.getroot()
    eparameters = root.find('Parameters')
    emarkers = root.find('MarkerSet').find('Markers')

    markers = []
    markernames = [em.attrib['NAME'] for em in emarkers.findall('Marker')]
    for markername in markernames:
        em = emarkers.find(f".//Marker[@NAME='{markername}']")
        epx = eparameters.find(f".//Parameter[@NAME='{modelname}_{markername}_x']")
        epy = eparameters.find(f".//Parameter[@NAME='{modelname}_{markername}_y']")
        epz = eparameters.find(f".//Parameter[@NAME='{modelname}_{markername}_z']")
        markers.append(Marker(em, epx, epy, epz))
    return markers


class RPC:
    @staticmethod
    def retrieveVSK(modelname):
        vskfile = f"/home/plusk01/Documents/{modelname}.vsk"
        tree = ET.parse(vskfile)
        root = tree.getroot()

        return ET.tostring(root, encoding='utf-8')

    @staticmethod
    def getCameras():
        calibfile = pathlib.Path(args.cal_dir) / 'LatestCalibration.xcp'
        cameras = parse_cameras(calibfile)
        return [c.as_dict() for c in cameras]

    @staticmethod
    def getModelMarkers(modelname):
        vskfile = pathlib.Path(args.vsk_dir) / f"{modelname}.vsk"
        markers = parse_markers(modelname, vskfile)
        return [m.as_dict() for m in markers]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(conflict_handler='resolve',
        description='Serves VSK model files and camera calibrations that reside on VICON computer')
    parser.add_argument('--host', '-h', default='0.0.0.0')
    parser.add_argument('--port', '-p', default=1250, type=int)
    parser.add_argument('--vsk-dir', '-v', default=VSK_DIR)
    parser.add_argument('--cal-dir', '-c', default=CAL_DIR)
    args = parser.parse_args()

    print()
    print(f"Starting RPC Server at http://{args.host}:{args.port}")
    print(f"  VSK dir: {args.vsk_dir}")
    print(f"  Cal dir: {args.cal_dir}")
    print()

    server = SimpleXMLRPCServer((args.host, args.port))
    # server.register_function(RPC.retrieveVSK)
    server.register_function(RPC.getCameras)
    server.register_function(RPC.getModelMarkers)
    server.serve_forever()