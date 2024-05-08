
import subprocess
import time

def main():
    # Start roscore
    roscore_process = subprocess.Popen(['roscore'])
    print("Starting roscore...")
    time.sleep(10)

    # Paths to the original publisher scripts
    accelerometer_magnetometer_script = '/home/team5/catkin_ws/src/jetbot_ros/scripts/imu_F.py'
    tof_distance_script = '/home/team5/catkin_ws/src/jetbot_ros/scripts/publishTOFData.py'
    runencoder_script = '/home/team5/catkin_ws/src/jetbot_ros/scripts/encoder.py'
    encoder_script = '/home/team5/catkin_ws/src/jetbot_ros/scripts/encoder_publisher.py'
    detect_script = '/home/team5/catkin_ws/src/jetbot_ros/scripts/duckDetect.py'
    nav_script = '/home/team5/catkin_ws/src/jetbot_ros/scripts/navTue.py'
    
    # Launch each script as a separate subprocess
    processes = []
    processes.append(subprocess.Popen(['python3', accelerometer_magnetometer_script]))
    processes.append(subprocess.Popen(['python3', tof_distance_script]))
    processes.append(subprocess.Popen(['python3', runencoder_script]))
    processes.append(subprocess.Popen(['python3', encoder_script]))
    processes.append(subprocess.Popen(['python', detect_script]))
    processes.append(subprocess.Popen(['python3', nav_script]))
    
    # Wait for all processes to complete
    for process in processes:
        process.wait()

if __name__ == '__main__':
    main()
