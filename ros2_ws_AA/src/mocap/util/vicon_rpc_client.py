import pickle

import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

import xmlrpc.client

if __name__ == '__main__':
    
    URI = 'http://127.0.0.1:1250'
    URI = 'http://192.168.0.9:1250'
    proxy = xmlrpc.client.ServerProxy(URI)

    cameras = proxy.getCameras()

    print()
    print(f"Got {len(cameras)} cameras:")
    for c in cameras:
        if c['enabled']:
            print(f"  - Camera #{c['index']} ({c['type']} - {c['deviceid']}): {c['position']}")
        else:
            print(f"  - Camera #{c['index']} ({c['type']} - {c['deviceid']}): DISABLED")
    print()

    modelname = 'HX18'
    markers = proxy.getModelMarkers(modelname)

    print()
    print(f"Got {len(markers)} markers for {modelname}")
    for m in markers:
        print(f"  - {m['name']}: {m['position']}")
    print()