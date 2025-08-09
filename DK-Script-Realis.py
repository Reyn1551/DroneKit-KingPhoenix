from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import random

# === PENGATURAN & VARIABEL (UNTUK DRONE FISIK) ===
# Ganti '/dev/ttyACM0' dengan port serial drone Anda (misal: /dev/ttyUSB0, COM3, dll.)
CONNECTION_STRING = '/dev/ttyACM0'
# CONNECTION_STRING = 'udp:127.0.0.1:14550' # Baris ini untuk simulasi

TAKEOFF_ALTITUDE = 2
TARGET_WAYPOINT = LocationGlobalRelative(-35.362902, 149.165113, TAKEOFF_ALTITUDE)
LANDING_SPEED = 0.3

# Pengaturan Kamera
CAM_WIDTH_PX = 640
CAM_HEIGHT_PX = 480
CENTER_X = CAM_WIDTH_PX / 2
CENTER_Y = CAM_HEIGHT_PX / 2
KP_GAIN = 0.005

# === KONEKSI KE DRONE ===
print(f"Menyambungkan ke drone di: {CONNECTION_STRING}")
try:
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
except Exception as e:
    print(f"Gagal terhubung ke drone: {e}")
    exit(1)
print("Drone terhubung!")

# === FUNGSI-FUNGSI UTAMA ===
def arm_and_takeoff(target_altitude):
    print("Mengecek status drone sebelum takeoff...")
    while not vehicle.is_armable:
        print(" Menunggu drone siap untuk di-arm...")
        time.sleep(1)

    print("Drone siap untuk di-arm!")
    print("Mengganti mode ke GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(" Menunggu mode GUIDED...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Menunggu arming...")
        time.sleep(1)

    print(f"Takeoff menuju {target_altitude} meter!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f" Ketinggian (GPS/Baro): {current_altitude:.2f}m")
        if current_altitude >= target_altitude * 0.95:
            print("Ketinggian takeoff tercapai")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def detect_object_and_get_center():
    is_detected = True
    if is_detected:
        simulated_x = CENTER_X + random.uniform(-50, 50)
        simulated_y = CENTER_Y + random.uniform(-50, 50)
        return (True, simulated_x, simulated_y)
    return (False, None, None)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def precision_land():
    print("Memulai prosedur pendaratan presisi...")
    while True:
        # Untuk drone fisik, pastikan Lidar/Rangefinder benar-benar ada dan dikonfigurasi di Mission Planner
        current_lidar_distance = vehicle.rangefinder.distance
        if current_lidar_distance is None:
            print("ERROR: Data Lidar tidak tersedia. Pastikan sensor terpasang dan dikonfigurasi.")
            break
        
        print(f"Ketinggian (Lidar): {current_lidar_distance:.2f}m")

        if current_lidar_distance < 0.2:
            print("Target pendaratan tercapai. Menghentikan motor.")
            send_ned_velocity(0, 0, 0)
            vehicle.armed = False
            while vehicle.armed:
                time.sleep(1)
            break
            
        detected, target_x, target_y = detect_object_and_get_center()

        if detected:
            error_x = target_x - CENTER_X
            error_y = target_y - CENTER_Y
            vel_x = KP_GAIN * error_y 
            vel_y = KP_GAIN * error_x
            vel_x = max(min(vel_x, 0.5), -0.5)
            vel_y = max(min(vel_y, 0.5), -0.5)
            
            print(f"Mengirim V_N:{vel_x:.2f} m/s, V_E:{vel_y:.2f} m/s, V_D:{LANDING_SPEED:.2f} m/s")
            send_ned_velocity(vel_x, vel_y, LANDING_SPEED)
        else:
            print("Target tidak terdeteksi. Turun vertikal perlahan.")
            send_ned_velocity(0, 0, LANDING_SPEED)

        time.sleep(0.1)

# === MAIN EXECUTION SCRIPT ===
if __name__ == '__main__':
    try:
        arm_and_takeoff(TAKEOFF_ALTITUDE)
        print("Terbang menuju waypoint...")
        vehicle.simple_goto(TARGET_WAYPOINT)

        while True:
            distance_to_target = get_distance_metres(vehicle.location.global_relative_frame, TARGET_WAYPOINT)
            print(f"Jarak ke waypoint: {distance_to_target:.2f}m")
            if distance_to_target < 1.0:
                print("Waypoint tercapai.")
                break
            time.sleep(1)
        
        print("Menstabilkan posisi sebelum landing presisi...")
        time.sleep(2)
        
        precision_land()

    except Exception as e:
        print(f"Terjadi error pada misi: {e}")

    finally:
        print("Misi selesai atau dibatalkan. Mendaratkan drone...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        print("Menutup koneksi.")
        vehicle.close()
