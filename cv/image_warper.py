import cv2
import numpy as np


class ImageWarper(object):
    """
    Warps binary image into bird-view
    """
    DEFAULT_DESTINATION_SIZE = (1280, 720)
    DEFAULT_SOURCE_ROI = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    DEFAULT_DESTINATION_ROI = np.float32([(0, 0), (1, 0), (0, 1), (1, 1)])

    @classmethod
    def warp(cls, img,
             destination_size=DEFAULT_DESTINATION_SIZE,
             source_roi_proportion=DEFAULT_SOURCE_ROI,
             destination_roi_proportion=DEFAULT_DESTINATION_ROI
             ):
        """
        Warps an image according to the specified ROIs (if any)
        :param img: Binary image to be processed
        :param destination_size: size of output image
        :param source_roi_proportion
        :param destination_roi_proportion
        :return: warped image
        """
        source_size = np.float32([(img.shape[1], img.shape[0])])
        source_roi = source_roi_proportion * source_size
        destination_roi = destination_roi_proportion * np.float32(destination_size)
        transformation_matrix = cv2.getPerspectiveTransform(source_roi, destination_roi)
        return cv2.warpPerspective(img, transformation_matrix, destination_size)
