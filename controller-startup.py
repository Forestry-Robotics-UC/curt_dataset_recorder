#!/usr/bin/env python3
import struct
import os
import sys
import subprocess
import pygame
import time

DEVICE = '/dev/input/js0'
COMMAND = '/home/curt/Documents/Default/curt_dataset_recorder/startup.sh --quiet'

def rumble_controller(duration=0.3, intensity=0.5):
    """Simple rumble using pygame"""
    try:
        pygame.init()
        pygame.joystick.init()
        
        # Check if joystick is connected
        if pygame.joystick.get_count() == 0:
            print("No joystick found")
            return False
        
        # Get first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        
        # Rumble! (duration in milliseconds)
        joystick.rumble(intensity, intensity, int(duration * 1000))
        
        # Wait for rumble to finish
        time.sleep(duration + 0.1)
        
        return True
        
    except Exception as e:
        print(f"Rumble error: {e}")
        return False


def wait_for_new_bag(directory, check_interval=1):
    # Get initial items
    initial_items = set(os.listdir(directory))
    
    while True:
        current_items = set(os.listdir(directory))
        new_items = current_items - initial_items
        
        if new_items:
            return new_items
        
        time.sleep(check_interval)


def wait_for_containers(check_interval=1):
    print("Waiting for all Docker containers to stop...")
    while True:
        try:
            # Run docker ps and capture output
            result = subprocess.run(
                ['docker', 'ps', '-q'],  # -q shows only container IDs
                capture_output=True,
                text=True,
                check=False  # Don't raise exception on non-zero exit
            )
            
            # Count containers (split output, filter empty lines)
            containers = [line for line in result.stdout.split('\n') if line.strip()]
            container_count = len(containers)
            
            if container_count == 0:
                return True
            
            time.sleep(check_interval)
            
        except FileNotFoundError:
            print("Error: Docker command not found. Is Docker installed?")
            return False
        except Exception as e:
            print(f"Error checking Docker: {e}")
            time.sleep(check_interval)


def main():
    global DEVICE

    running = False
    
    # Try to find the joystick device
    for dev in ['/dev/input/js0', '/dev/input/js1', '/dev/input/js2']:
        if os.path.exists(dev):
            DEVICE = dev
            break
    else:
        print("No joystick found!")
        sys.exit(1)
    
    try:
        with open(DEVICE, 'rb') as f:
            while True:
                data = f.read(8)
                if len(data) != 8:
                    continue

                time_val, value, etype, number = struct.unpack('IhBB', data)

                if etype == 1 and value == 1:
                    match number:
                        case 0:
                            print("GREEN BUTTON PRESSED!")

                            if not running:
                                print(f"Running: {COMMAND}")
                                running = True
                            
                                # Run command
                                subprocess.Popen([COMMAND], shell=True)

                                #time.sleep(20) # Wait for 20s after the command has finished to ensure that the containers are running.
                                wait_for_new_bag('/home/curt/Documents/Default/rosbags')
                            
                                # Rumble
                                rumble_controller(duration=0.4, intensity=0.8)
                            
                        case 1:
                            print("RED BUTTON PRESSED!")
                            if running:
                                running = False

                                subprocess.Popen(["docker exec $(docker ps | grep recorder | awk '{print $1}') bash -c \"source /home/rosuser/ros2_ws/install/setup.bash && ros2 service call /hector_recorder/stop_recording hector_recorder_msgs/srv/StopRecording\""], shell=True)

                                time.sleep(2)

                                subprocess.Popen(["docker stop $(docker ps -q)"], shell=True)
                                wait_for_containers()

                                rumble_controller(duration=0.4, intensity=0.8)
                            
                        case 2:
                            print("BLUE BUTTON PRESSED!")
                            rumble_controller(duration=0.4, intensity=0.6)
                            
                        case 3:
                            print("YELLOW BUTTON PRESSED!")
                            rumble_controller(duration=0.5, intensity=0.9)
                    
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()