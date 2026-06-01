#!/usr/bin/env python3
"""
SIYI A8 Camera - Stream viewer + frame saver + gimbal control
UDP porta 37260

Dipendenze: pip install opencv-python numpy
Uso:        python3 siyi_capture.py

Tasti:
    p          → toggle pitch +90 / -90 gradi
    c          → centra gimbal (pitch=0, yaw=0)
    z / x      → zoom in / out
    s          → salva frame
    SPAZIO     → pausa/riprendi
    q          → esci
"""

import cv2, os, time, socket, struct, threading, queue, subprocess
from datetime import datetime

RTSP_URL    = "rtsp://192.168.145.25:8554/main.264"
CAMERA_IP   = "192.168.145.25"
GIMBAL_PORT = 37260
OUTPUT_DIR  = "frames"

# ── CRC16 ──────────────────────────────────────────────────────────────────────
def _make_crc16_table():
    t = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
        t.append(crc)
    return t

_CRC16 = _make_crc16_table()

def crc16(data):
    crc = 0
    for b in data:
        crc = _CRC16[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc

def build_packet(cmd_id, data, seq=0):
    body = b'\x01' + struct.pack('<H', len(data)) + struct.pack('<H', seq) + struct.pack('B', cmd_id) + data
    return b'\x55\x66' + body + struct.pack('<H', crc16(body))

# ── Gimbal controller ──────────────────────────────────────────────────────────
class GimbalController:
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.seq  = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        #try:
        #    self.sock.bind(('0.0.0.0', port))
        #    print(f"[GIMBAL] UDP bind porta {port} OK")
        #except OSError as e:
        #    print(f"[GIMBAL] Bind fallito: {e}")

        # Heartbeat thread
        self._running = True
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    def _heartbeat_loop(self):
        HB = bytes.fromhex('556601010000000000598B')
        while self._running:
            try: self.sock.sendto(HB, self.addr)
            except: pass
            time.sleep(2.0)

    def _send(self, cmd_id, data):
        pkt = build_packet(cmd_id, data, self.seq)
        self.seq = (self.seq + 1) & 0xFFFF
        try:
            self.sock.sendto(pkt, self.addr)
        except Exception as e:
            print(f"[GIMBAL] Errore: {e}")

#    def set_angle(self, yaw_deg: float, pitch_deg: float):
#        """
#        Comando 0x0E: imposta angolo assoluto.
#        Il valore viene moltiplicato x10 e inviato come int16 little-endian.
#        Esempio: pitch=90.0 → 900 → 0x0384
#        """
#        yaw_val   = int(yaw_deg   * 10)
#        pitch_val = int(pitch_deg * 10)
#        data = struct.pack('<hh', yaw_val, pitch_val)
#        self._send(0x0E, data)
#        print(f"[GIMBAL] Angolo → yaw={yaw_deg}° pitch={pitch_deg}°")

    def test_send(self):
        while True:
            self.sock.sendto(bytes.fromhex('556601010000000000598B'), self.addr)
            print("sent")
            time.sleep(1)

    def center(self):
        # Pacchetto CENTER esatto dal manuale SIYI
        pkt = bytes.fromhex('556601010000000801d112')
        try: self.sock.sendto(pkt, self.addr)
        except: pass
        print("[GIMBAL] Centro (0°, 0°)")

    def zoom_in(self):
        pkt = bytes.fromhex('5566010100000005018d64')
        try: self.sock.sendto(pkt, self.addr)
        except: pass

    def zoom_out(self):
        pkt = bytes.fromhex('556601010000000 5FF5c6a'.replace(' ',''))
        try: self.sock.sendto(pkt, self.addr)
        except: pass

    def close(self):
        self._running = False
        self.sock.close()

# ── ffmpeg reader ──────────────────────────────────────────────────────────────
def ffmpeg_reader(url, fq, stop_event):
    import numpy as np
    cmd = ["ffmpeg", "-loglevel", "error", "-rtsp_transport", "tcp",
           "-fflags", "nobuffer+discardcorrupt", "-flags", "low_delay",
           "-i", url, "-vf", "scale=1280:720", "-pix_fmt", "bgr24",
           "-f", "rawvideo", "-"]
    W, H = 1280, 720
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    try:
        while not stop_event.is_set():
            raw = proc.stdout.read(W * H * 3)
            if len(raw) != W * H * 3: break
            frame = np.frombuffer(raw, dtype='uint8').reshape((H, W, 3))
            if fq.full():
                try: fq.get_nowait()
                except queue.Empty: pass
            fq.put(frame.copy())
    finally:
        proc.kill(); proc.wait()

# ── main ──────────────────────────────────────────────────────────────────────
def main():
    try:
        import numpy as np
    except ImportError:
        print("[ERRORE] pip install numpy"); return

    print("[INFO] Avvio...")
    gimbal = GimbalController(CAMERA_IP, GIMBAL_PORT)

    fq = queue.Queue(maxsize=2)
    se = threading.Event()
    threading.Thread(target=ffmpeg_reader, args=(RTSP_URL, fq, se), daemon=True).start()

    print("[INFO] Attesa primo frame...")
    last = None
    for _ in range(100):
        try: last = fq.get(timeout=0.2); break
        except queue.Empty: pass

    if last is None:
        print("[ERRORE] Nessun frame."); se.set(); gimbal.close(); return

    print("[INFO] Stream attivo!")
    print("[INFO] [p] pitch ±90°  [c] centra  [z/x] zoom  [s] salva  [q] esci")
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    paused     = False
    fc = sc    = 0
    pitch_pos  = True   # True=+90, False=-90
    t0         = time.time()

    while True:
        if not paused:
            try: last = fq.get(timeout=0.033); fc += 1
            except queue.Empty: pass

        disp = last.copy()
        fps  = fc / (time.time() - t0 + 0.001)
        pitch_label = "+90°" if pitch_pos else "-90°"

        cv2.rectangle(disp, (0,0), (1280,36), (0,0,0), -1)
        cv2.putText(disp,
            f"{'PAUSA' if paused else 'LIVE'}  {fps:.1f}fps  salvati:{sc}  pitch:{pitch_label}",
            (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
        cv2.rectangle(disp, (0,700), (1280,720), (0,0,0), -1)
        cv2.putText(disp,
            "[p] pitch ±90  [c] centra  [z/x] zoom  [s] salva  [spazio] pausa  [q] esci",
            (10,715), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)
        cv2.imshow("SIYI A8", disp)

        key = cv2.waitKey(30) & 0xFF

        if   key == ord('q'): break
        elif key == ord('p'):
            pitch_pos = not pitch_pos
            gimbal.set_angle(yaw_deg=0.0, pitch_deg=90.0 if pitch_pos else -90.0)
        elif key == ord('c'):
            pitch_pos = True
            gimbal.center()
        elif key == ord('z'): gimbal.zoom_in();  print("[GIMBAL] Zoom+")
        elif key == ord('x'): gimbal.zoom_out(); print("[GIMBAL] Zoom-")
        elif key == ord('s'):
            path = os.path.join(OUTPUT_DIR,
                   f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg")
            cv2.imwrite(path, last, [cv2.IMWRITE_JPEG_QUALITY, 95])
            sc += 1; print(f"[SAVED] {path}")
        elif key == ord(' '):
            paused = not paused

    se.set(); gimbal.close(); cv2.destroyAllWindows()
    print(f"[INFO] Fine. Frame:{fc}, Salvati:{sc}")

if __name__ == "__main__":
    main()
