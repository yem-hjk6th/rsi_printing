import cv2
import numpy as np
from config.config import load_config
from src.zed_video import ZEDVideo
from src.printing_detector import PrintingDetector
from src.width_measurement import WidthMeasurement
from src.data_logger import DataLogger


if __name__ == "__main__":

    # Load configuration from YAML
    config = load_config()

    fps_constant = int(config['length']*30/(config['speed'])) # 16mm * 30 fps / speed (mm/s)

    # Initialize measurement storage and counters
    all_widths = []
    counting_fps, relative_time, acc_length = 0, 0.0, 0.0
    timestamp = config['timestamp']  # Initial timestamp in "YYYYMMDDHHMMSSmmm" format
    video_num = int(config['svo_path'].split("_")[-1].split('.')[0])  # extract video number from filename 
    # video_num = 1 # use this to apply fillflood

    # Set start frame depending on video number
    start_frame = { 1: 10000,
                    2: 950,
                    3: 1500,
                    }.get(video_num, 0)
    
    # Threshold value for printing detection (can vary per video)
    thresh_value = { 1: 200,
                     2: 200, 
                     3: 200, # 0-255
                   }.get(video_num, 200)
    
    # Adjust crop y0 value for video_num >= 5
    if video_num>=5:
        config['y0'] = 400
    
    frame_count = start_frame
    end_frame = start_frame + config['limit_frames']  # Limit number of frames to process

    # Initialize ZED video object
    zed_video = ZEDVideo(config['svo_path'], start_frame)
    
    # Initialize printing detector
    detector = PrintingDetector(thresh_value)
    
    # Initialize width measurement object
    width_measurer = WidthMeasurement(zed_video.fx, config)
    
    try:
        while True:
            # Grab a new frame, depth map and depth image
            frame, depth, depth_image = zed_video.grab_frame()
            frame_count += 1
            counting_fps +=1
            width_mm = -1.0

            # Stop if end of video reached
            if frame is None:
                print("End of the video")
                break

            # Crop region of interest (ROI)
            crop_image = frame[config['y0']:config['y0']+150, config['x0']:config['x0']+300]

            # Detect printed area
            mask = detector.detect(video_num, crop_image=crop_image)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                contour_max = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(contour_max)

                # Decide whether to process width based on area and video type
                if area < 1500 and video_num==1 or area<3500 and video_num!=1:
                    process_width = True
                    if video_num==1:
                        process_width = width_measurer.should_process_width(contour_max)

                    if process_width:   
                        # Draw reference measurement line 
                        width_measurer.draw_measurement_line(frame)  
                        
                        # Create mask from largest contour                            
                        mask = width_measurer.create_mask_from_contour(mask.shape, contour_max)
                        # Extract skeleton (centerline) of the mask
                        skeleton, centerline_points = width_measurer.extract_centerline(mask)
                        # Trim skeleton ends to avoid extremes
                        centerline_points = width_measurer.trim_centerline(centerline_points)
                        
                        # Measure widths along the centerline
                        segment_widths = width_measurer.measure_perpendicular_widths(frame, mask, contour_max, centerline_points, depth)
                        # Measure width at a specific control point
                        width_mm = width_measurer.measure_width_at_control_point(frame, mask, skeleton, depth)
                        

            # Save width data every 15 frames (or first measurement)
            # Specifically every 1.6 cm (in straight line, video 1) which is every 15 frames because robot velocity == 32mm/s and 30 fps
            if (counting_fps >= fps_constant or not all_widths):
                counting_fps = 0
                all_widths, relative_time, acc_length, timestamp = DataLogger.save(
                    all_widths, width_mm, timestamp,
                    relative_time, acc_length, fps_constant, config['length'])

            cv2.imshow("LEFT IMAGE", cv2.resize(frame, None, fx=0.7, fy=0.7))
            if cv2.waitKey(1) & 0xFF == ord('q') or frame_count >= end_frame:
                break

    finally:
        zed_video.close()
        cv2.destroyAllWindows()
        # Save all measured widths, timestamp and length to CSV
        np.savetxt(f"print_data_{video_num}.csv", all_widths, fmt="%s", delimiter=",")
