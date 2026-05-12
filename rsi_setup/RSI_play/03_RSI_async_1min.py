import socket
import xml.etree.ElementTree as ET
import time
import random
import threading
import matplotlib.pyplot as plt
import numpy as np

# --- Configuration ---
HOST = '0.0.0.0'
PORT = 59152
RSI_CYCLE_TIME = 0.004  # 4ms cycle

# Motion parameters: target 150mm in ~100s
Z_LIMIT = 150        # Total limit (mm)
TARGET_TIME = 100.0      # Target completion time (seconds)
MOVE_SPEED = Z_LIMIT / TARGET_TIME  # 1.5 mm/s
STEP_INCREMENT = MOVE_SPEED * RSI_CYCLE_TIME  # 0.01mm/cycle

# Async test parameters
RANDOM_INTERVAL_MIN = 0.5   # Min interval before motion starts (s)
RANDOM_INTERVAL_MAX = 2.0   # Max interval before motion starts (s)
MOVE_DURATION = 3.0         # Each motion lasts 3 seconds, then wait for next trigger

# Global variables
motion_lock = threading.Lock()
is_moving = False
motion_start_time = None
stop_flag = threading.Event()

# Data recording for plotting
data_timestamps = []  # Unix timestamps
data_z_positions = []  # Real Z positions
data_delta_z_sent = []  # Delta Z values sent

def input_listener_thread():
    """
    Listener thread: monitor keyboard input, press 'q' to stop
    """
    print("\n[INFO] Press 'q' + Enter to stop the program\n")
    while not stop_flag.is_set():
        try:
            user_input = input()
            if user_input.lower() == 'q':
                print("\n[INFO] Stop signal received...\n")
                stop_flag.set()
                break
        except EOFError:
            break
        except Exception:
            continue

def trigger_controller_thread(start_time):
    """
    Trigger thread: control random motion trigger logic independently
    """
    global is_moving, motion_start_time
    
    next_trigger_time = start_time + random.uniform(RANDOM_INTERVAL_MIN, RANDOM_INTERVAL_MAX)
    
    while not stop_flag.is_set():
        current_time = time.time()
        time_to_next = next_trigger_time - current_time
        
        if time_to_next > 0:
            time.sleep(min(time_to_next * 0.9, 0.01))
        else:
            with motion_lock:
                if not is_moving:
                    is_moving = True
                    motion_start_time = current_time
                    elapsed = current_time - start_time
                    print(f"[TRIGGER_START] t={elapsed:.2f}s")
            
            next_trigger_time = current_time + random.uniform(
                RANDOM_INTERVAL_MIN, 
                RANDOM_INTERVAL_MAX
            )

def run_async_controller():
    """
    Async test: maintain 4ms communication frequency, random motion triggers
    Target: 300mm in ~60 seconds
    """
    global is_moving, motion_start_time
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.010)
    
    try:
        sock.bind((HOST, PORT))
        print(f"RSI Async Server started on port {PORT}")
        print(f"Target: {Z_LIMIT}mm in {TARGET_TIME}s ({MOVE_SPEED:.2f}mm/s)")
        print(f"Trigger Interval: {RANDOM_INTERVAL_MIN}s - {RANDOM_INTERVAL_MAX}s | Motion Duration: {MOVE_DURATION}s")
        print("=" * 80 + "\n")
    except Exception as e:
        print(f"Bind Error: {e}")
        return

    accumulated_dist = 0.0
    packet_count = 0
    start_time = time.time()
    
    # Start threads
    input_thread = threading.Thread(target=input_listener_thread, daemon=True)
    input_thread.start()
    
    trigger_thread = threading.Thread(target=trigger_controller_thread, args=(start_time,), daemon=True)
    trigger_thread.start()
    
    try:
        while True:
            if stop_flag.is_set():
                break
            
            try:
                data, addr = sock.recvfrom(2048)
                packet_count += 1
                current_time = time.time()
                elapsed = current_time - start_time
                
                try:
                    root = ET.fromstring(data)
                    ipoc = root.find('IPOC').text
                    rist = root.find('RIst')
                    current_real_z = float(rist.get('Z'))
                    
                    # Motion logic
                    delta_z = 0.0
                    
                    with motion_lock:
                        if is_moving:
                            elapsed_motion = current_time - motion_start_time
                            
                            if elapsed_motion < MOVE_DURATION:
                                delta_z = STEP_INCREMENT
                                accumulated_dist += delta_z
                            else:
                                is_moving = False
                                print(f"[TRIGGER_END] t={elapsed:.2f}s | Packet #{packet_count} | "
                                      f"Real Z: {current_real_z:.2f}mm | Total: {accumulated_dist:.2f}mm")
                    
                    # Record data for plotting
                    data_timestamps.append(elapsed)
                    data_z_positions.append(current_real_z)
                    data_delta_z_sent.append(delta_z)
                    
                    # Send response
                    reply_xml = f'<Sen Type="ImFree"><RKorr X="0.0" Y="0.0" Z="{delta_z:.4f}" A="0.0" B="0.0" C="0.0" /><IPOC>{ipoc}</IPOC></Sen>'
                    sock.sendto(reply_xml.encode(), addr)
                    
                    # Check completion
                    if accumulated_dist >= Z_LIMIT:
                        print(f"\n[COMPLETE] Z_LIMIT ({Z_LIMIT}mm) reached at t={elapsed:.2f}s | Packet #{packet_count}")
                        time.sleep(1)
                        break
                    
                except ET.ParseError:
                    continue
            
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Socket Error: {e}")
                continue

    except KeyboardInterrupt:
        print(f"\n{'=' * 80}")
        print(f"[STOPPED] Distance: {accumulated_dist:.2f}mm | Packets: {packet_count}")
    finally:
        sock.close()
        print("Socket closed.")
    
    # Generate plots
    generate_plots(start_time, accumulated_dist, packet_count)

def generate_plots(start_time, total_distance, total_packets):
    """
    Generate s-t (displacement-time) and v-t (velocity-time) graphs
    """
    if len(data_timestamps) < 2:
        print("[WARNING] Not enough data points for plotting")
        return
    
    timestamps = np.array(data_timestamps)
    z_positions = np.array(data_z_positions)
    delta_z_sent = np.array(data_delta_z_sent)
    
    # Calculate displacement from initial position
    displacement = z_positions - z_positions[0]
    
    # Calculate velocity (mm/s)
    # Use delta_z_sent / RSI_CYCLE_TIME
    velocity = (delta_z_sent / RSI_CYCLE_TIME)
    # Smooth out noise with moving average
    window_size = 250  # ~1 second of data
    if len(velocity) > window_size:
        velocity_smooth = np.convolve(velocity, np.ones(window_size)/window_size, mode='same')
    else:
        velocity_smooth = velocity
    
    # Create figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot s-t (displacement vs time)
    ax1.plot(timestamps, displacement, 'b-', linewidth=1.5, label='Displacement')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Displacement (mm)', fontsize=12)
    ax1.set_title(f's-t Graph: Displacement vs Time (Total: {total_distance:.2f}mm in {timestamps[-1]:.2f}s)', 
                  fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot v-t (velocity vs time)
    ax2.plot(timestamps, velocity_smooth, 'r-', linewidth=1.5, label='Velocity (smoothed)')
    ax2.axhline(y=MOVE_SPEED, color='g', linestyle='--', linewidth=2, label=f'Target Speed ({MOVE_SPEED:.2f}mm/s)')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Velocity (mm/s)', fontsize=12)
    ax2.set_title('v-t Graph: Velocity vs Time', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    
    # Save figure
    filename = f'RSI_async_plot_{int(time.time())}.png'
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"\n[PLOT SAVED] {filename}")
    
    # Display statistics
    print("\n" + "=" * 80)
    print("[STATISTICS]")
    print(f"  Total Time: {timestamps[-1]:.2f}s")
    print(f"  Total Distance: {total_distance:.2f}mm")
    print(f"  Average Velocity: {total_distance / timestamps[-1]:.4f}mm/s")
    print(f"  Target Velocity: {MOVE_SPEED:.4f}mm/s")
    print(f"  Peak Velocity: {velocity_smooth.max():.4f}mm/s")
    print(f"  Total Packets: {total_packets}")
    print(f"  Effective Frequency: {total_packets / timestamps[-1]:.1f}Hz")
    print("=" * 80)
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    run_async_controller()
