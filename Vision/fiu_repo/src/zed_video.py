import pyzed.sl as sl

class ZEDVideo:
    """Handles ZED video loading and frame retrieval."""
    def __init__(self, svo_path, start_frame=0):
        """
        Initialize the ZED camera with an SVO file.
        
        Parameters:
            svo_path (str): Path to the SVO video file.
            start_frame (int): Frame number to start playback.
        """
        self.svo_path = svo_path
        self.start_frame = start_frame
        self.zed = sl.Camera()  # Create ZED camera object 
        self.init_parameters = sl.InitParameters()
        self.init_parameters.svo_real_time_mode = False
        input_type = sl.InputType()
        input_type.set_from_svo_file(svo_path)
        self.init_parameters.input = input_type
        
        # Open ZED camera
        if self.zed.open(self.init_parameters) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Cannot open SVO file.")
        
        # Set start frame position
        self.zed.set_svo_position(start_frame)
        self.runtime_params = sl.RuntimeParameters()

        # Allocate ZED Mat objects for images and depth
        self.image_zed = sl.Mat()
        self.depth_zed = sl.Mat()
        self.depth_image_zed = sl.Mat()

        # Get camera calibration parameters (focal length fx)
        calib = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.fx = calib.fx

    def grab_frame(self):
        """
        Grab the next frame from the SVO video.
        
        Returns:
            tuple: (image, depth_map, depth_image) as numpy arrays
        """
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return None, None, None
        
        # Retrieve left image, depth measure, and depth image
        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)
        self.zed.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU)
        return self.image_zed.get_data(), self.depth_zed.get_data(), self.depth_image_zed.get_data()

    def close(self):
        """Release the ZED camera resources."""
        self.zed.close()

