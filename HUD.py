"""
HUD Program for Raspberry Pi 5 + RayNeo Air 2S
- GPS (NEO-M8N via UART)
- Compass (MPU9250 via I²C)
- Dummy: Battery + Gas
- Camera feed: Picamera2
"""

import cv2
import numpy as np
import time
import serial
import pynmea2
import math
import datetime
from staticmap import StaticMap, CircleMarker
import smbus2
from picamera2 import Picamera2

# --- Dummy placeholders ---
battery_left = 75
battery_right = 40
gas_left = 0.55
gas_right = 0.25

# --- GPS Setup ---
gps_serial = serial.Serial('/dev/serial0', 9600, timeout=1)
lat, lon = 32.0853, 34.7818

# --- Compass Setup (MPU9250 on I²C) ---
bus = smbus2.SMBus(1)
MPU_ADDR = 0x68
AK8963_ADDR = 0x0C
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU

def read_word(adr, addr):
    high = bus.read_byte_data(adr, addr)
    low = bus.read_byte_data(adr, addr+1)
    val = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

def get_heading():
    try:
        hx = read_word(AK8963_ADDR, 0x03)
        hy = read_word(AK8963_ADDR, 0x05)
        heading = math.degrees(math.atan2(hy, hx))
        if heading < 0:
            heading += 360
        return heading
    except:
        return 0

# --- Map setup ---
last_map_fetch = 0
MAP_REFRESH_INTERVAL = 2
map_img = None

def fetch_map(lat, lon, zoom=16, size=400):
    m = StaticMap(size, size, url_template='http://a.tile.openstreetmap.org/{z}/{x}/{y}.png')
    m.add_marker(CircleMarker((lon, lat), 'red', 0))
    image = m.render(zoom=zoom)
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

# --- HUD Elements ---
def draw_battery(frame, level, x, y):
    bar_w, bar_h = 20, 60
    color = (0,0,255)  # red
    cv2.rectangle(frame,(x,y),(x+bar_w,y+bar_h),(100,100,100),2)
    fill_h = int(bar_h * (level/100))
    cv2.rectangle(frame,(x,y+bar_h-fill_h),(x+bar_w,y+bar_h),color,-1)
    cv2.putText(frame,f"{level}%",(x-5,y+bar_h+20),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)

def draw_gas(frame, level, x, y):
    bar_w, bar_h = 20, 60
    color = (200,200,200)  # gray
    cv2.rectangle(frame,(x,y),(x+bar_w,y+bar_h),(100,100,100),2)
    fill_h = int(bar_h*level)
    cv2.rectangle(frame,(x,y+bar_h-fill_h),(x+bar_w,y+bar_h),color,-1)
    perc = int(level*100)
    cv2.putText(frame,f"{perc}%",(x-5,y+bar_h+20),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)

def draw_circular_minimap(frame, map_img, heading=0):
    h,w,_ = frame.shape
    radius = 100
    cx, cy = w-radius-20, h-radius-20

    # Resize map to fit circle area
    map_resized = cv2.resize(map_img, (2*radius, 2*radius))

    # Create circular mask
    mask = np.zeros((2*radius, 2*radius), dtype=np.uint8)
    cv2.circle(mask, (radius, radius), radius, 255, -1)

    # Apply circular mask
    circular_map = cv2.bitwise_and(map_resized, map_resized, mask=mask)

    # Place onto HUD frame using mask
    roi = frame[cy-radius:cy+radius, cx-radius:cx+radius]
    np.copyto(roi, circular_map, where=mask[:,:,None].astype(bool))

    # Player dot
    cv2.circle(frame,(cx,cy),6,(0,0,255),-1)

    # Heading arrow
    angle_rad = np.deg2rad(-heading+90)
    ax = int(cx+20*np.cos(angle_rad))
    ay = int(cy-20*np.sin(angle_rad))
    cv2.arrowedLine(frame,(cx,cy),(ax,ay),(0,0,255),2)

    # Circle outline
    cv2.circle(frame,(cx,cy),radius,(0,0,255),2)
    return frame

def draw_compass(frame, heading=0):
    h,w,_ = frame.shape
    cw = w//2
    ch = 50
    cx, cy = w//2, ch//2+10
    overlay = frame.copy()
    directions=['N','NE','E','SE','S','SW','W','NW','N']
    num_ticks=36
    tick_spacing = cw/num_ticks
    tick_height=15
    red=(0,0,255)
    line_y=cy+10
    cv2.line(overlay,(cx-cw//2,line_y),(cx+cw//2,line_y),red,2)
    for i in range(num_ticks+1):
        tx=int(cx-cw/2+i*tick_spacing)
        th=tick_height if i%(num_ticks//8)==0 else tick_height//2
        cv2.line(overlay,(tx,line_y-th//2),(tx,line_y+th//2),red,1)
    for idx,dir_label in enumerate(directions):
        angle=idx*45
        pos=((angle-heading)%360)/360
        tx=int(cx-cw//2+pos*cw)
        cv2.putText(overlay,dir_label,(tx-10,line_y-tick_height),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,red,2)
    frame=cv2.addWeighted(overlay,0.7,frame,0.3,0)
    return frame

def draw_clock(frame):
    now = datetime.datetime.now().strftime("%H:%M:%S")
    color = (0,0,255)  # red
    cv2.putText(frame, now, (20,40), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, color, 2)
    return frame

# --- Camera (Picamera2) ---
picam2 = Picamera2(camera_num=0)  # 0 = normal, 1 = IR-cut if available
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# === Main Loop ===
while True:
    frame = picam2.capture_array()

    # --- Read GPS ---
    if gps_serial.in_waiting:
        line = gps_serial.readline().decode(errors="ignore").strip()
        if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
            try:
                msg = pynmea2.parse(line)
                if hasattr(msg,"latitude") and hasattr(msg,"longitude"):
                    lat, lon = msg.latitude, msg.longitude
            except:
                pass

    # --- Read Compass ---
    heading = get_heading()

    # --- Refresh map every 2s ---
    if time.time() - last_map_fetch > MAP_REFRESH_INTERVAL or map_img is None:
        map_img = fetch_map(lat, lon)
        last_map_fetch = time.time()

    # --- Draw HUD ---
    draw_battery(frame,battery_left,50,100)
    draw_battery(frame,battery_right,frame.shape[1]-70,100)
    draw_gas(frame,gas_left,50,180)
    draw_gas(frame,gas_right,frame.shape[1]-70,180)
    frame = draw_circular_minimap(frame,map_img,heading)
    frame = draw_compass(frame,heading)
    frame = draw_clock(frame)

    cv2.imshow("HUD",frame)
    if cv2.waitKey(1)&0xFF==27:  # ESC to exit
        break

picam2.stop()
cv2.destroyAllWindows()
