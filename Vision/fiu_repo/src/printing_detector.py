import cv2 
import numpy as np

class PrintingDetector:
    """Handles segmentation/mask detection for printing."""
    def __init__(self, thresh_value):
        """
        Initialize the detector with a threshold value for binary segmentation.
        
        Parameters:
            thresh_value (int): Threshold used in simple thresholding method.
        """
        self.thresh_value = thresh_value

    def threshold(self, crop_image):
        """
        Detect printed area using simple thresholding.

        Steps:
        1. Convert image to grayscale.
        2. Apply Gaussian blur to reduce noise.
        3. Apply binary threshold to create mask.
        4. Apply morphological opening to remove small artifacts.
        5. Display the detected mask for debugging.

        Parameters:
            crop_image (np.array): Cropped image region.

        Returns:
            mask (np.array): Binary mask of detected printing.
        """
        gray = cv2.cvtColor(crop_image, cv2.COLOR_BGRA2GRAY)
        cv2.imshow("gray", gray)
        gray = cv2.GaussianBlur(gray, (1, 3), 0, 1)
        _, mask = cv2.threshold(gray, self.thresh_value, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        cv2.imshow("Detected mask", mask)
        return mask

    def flood_fill(self, crop_image):
        """
        Detect printed area using a seeded flood fill approach.

        Steps:
        1. Convert to BGR if image has 4 channels (BGRA).
        2. Apply Gaussian blur to reduce noise.
        3. Define seed points along x/y positions.
        4. Perform flood fill from each seed with small color difference tolerance.
        5. Combine results into a binary mask.
        6. Apply morphological closing to fill gaps.
        7. Display segmented image and mask.

        Parameters:
            crop_image (np.array): Cropped image region.

        Returns:
            mask (np.array): Binary mask of detected printing.
        """
        val = 14
        img = crop_image.copy()
        if img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img = cv2.GaussianBlur(img, (1, 3), 0, 1)

        x_positions = [134 for _ in range(10)]
        y_positions = list(range(40, 140, 10))
        x_offsets = [0, -4]
        mask_flood = np.zeros((img.shape[0]+2, img.shape[1]+2), np.uint8)
        mask = np.zeros(img.shape[:2], np.uint8)

        for x, y in zip(x_positions, y_positions):
            for x_off in x_offsets:
                pos = (x + x_off, y)
                cv2.floodFill(img, mask_flood, pos, (0, 0, 255),
                              loDiff=(val,val,val), upDiff=(val,val,val),
                              flags=4 | cv2.FLOODFILL_FIXED_RANGE)
                mask += mask_flood[1:-1,1:-1]
        mask = np.clip(mask, 0, 1) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))

        cv2.imshow("Segmentation", img)
        cv2.imshow("Detected mask", mask)
        return mask

    def detect(self, video_num, crop_image):
        """
        Select detection method based on video number.
        
        Video 1 uses flood fill; others use thresholding.

        Parameters:
            video_num (int): Number identifying the video type.
            crop_image (np.array): Cropped image region.

        Returns:
            mask (np.array): Binary mask of detected printing.
        """
        if video_num == 1:
            return self.flood_fill(crop_image)
        return self.threshold(crop_image)
