from typing import List

import cv2
import numpy as np

from .image_warper import ImageWarper


class CurveCalculator(object):
    DEBUG = False

    DEFAULT_METERS_PER_PIXEL_X = 3.7 / 720
    DEFAULT_METERS_PER_PIXEL_Y = 30.5 / 720

    def __init__(self,
                 meters_per_pixel_x=DEFAULT_METERS_PER_PIXEL_X,
                 meters_per_pixel_y=DEFAULT_METERS_PER_PIXEL_Y,
                 debug=False,
                 mode="B"
                 ):
        CurveCalculator.DEBUG = debug
        self.mode = mode

        self.left = [[], [], []]
        self.right = [[], [], []]

        self.right_fitted_curve: List[float] = None
        self.left_fitted_curve: List[float] = None

        self.meters_per_pixel_x = meters_per_pixel_x
        self.meters_per_pixel_y = meters_per_pixel_y

    def recognize_curve(self, img, n_windows=9, window_margin=150,
                       minpix=1, draw_windows=True):
        if self.mode == "B" or self.mode == "L":
            self.sliding_window(img, True, n_windows, window_margin, minpix, draw_windows)
        if self.mode == "B" or self.mode == "R":
            self.sliding_window(img, False, n_windows, window_margin, minpix, draw_windows)

    def sliding_window(self, img, left_lane, n_windows=9, window_margin=150,
                        minimum_pixels=1, draw_windows=True):
        image_height = img.shape[0]
        window_height = np.int(image_height / n_windows)
        y_range = np.linspace(0, image_height - 1, image_height)
        fit = np.empty(3)

        out_img = None
        if CurveCalculator.DEBUG: out_img = np.dstack((img, img, img)) * 255

        # Starting Point calculation
        histogram = self.calculate_histogram(img)
        img_middle = int(histogram.shape[0] / 2)
        starting_point = None
        if left_lane:
            starting_point = np.argmax(histogram[:img_middle])
        else:
            starting_point = np.argmax(histogram[img_middle:]) + img_middle

        # Identify x and y positions of nonzero pixels
        nonzero = img.nonzero()
        nonzero_y_pixels = np.array(nonzero[0])
        nonzero_x_pixels = np.array(nonzero[1])

        current_x_position = starting_point
        lane_pixels = []
        for window in range(n_windows):
            # Calculate image boundries
            window_y_lower_boundry = image_height - (window + 1) * window_height
            window_y_upper_boundry = image_height - window * window_height
            window_x_lower = current_x_position - window_margin
            window_x_upper = current_x_position + window_margin
            
            if CurveCalculator.DEBUG:
                cv2.rectangle(out_img, (window_x_lower, window_y_lower_boundry),
                                (window_x_upper, window_y_upper_boundry), (100, 225, 225), 3)

            # get nonzero areas within window
            non_zero_pixels = ((nonzero_y_pixels >= window_y_lower_boundry) &
                                    (nonzero_y_pixels < window_y_upper_boundry) &
                                    (nonzero_x_pixels >= window_x_lower) &
                                    (nonzero_x_pixels < window_x_upper)
                                ).nonzero()[0]

            lane_pixels.append(non_zero_pixels)

            # Recenter next window (based on mean)
            if len(non_zero_pixels) > minimum_pixels:
                current_x_position = np.int(np.mean(nonzero_x_pixels[non_zero_pixels]))

        lane_pixels = np.concatenate(lane_pixels)
        if len(lane_pixels) < minimum_pixels*(n_windows/2):
            if left_lane: self.left_fitted_curve = None 
            if not left_lane: self.right_fitted_curve = None 
            print("The expected number of pixels was at least 200, but only ", len(lane_pixels), "were found")
            return

        # Extract pixel positions
        x_lane_pixels = nonzero_x_pixels[lane_pixels]
        y_lane_pixels = nonzero_y_pixels[lane_pixels]

        # Fit second order polinomial
        fit = np.polyfit(y_lane_pixels, x_lane_pixels, 2)

        # For averaging, not implemented
        # if left_lane:
        #     for i in range(3):
        #         self.left[i].pop()
        #         self.left[i].append(fit[i])
        # else:
        #     for i in range(3):
        #         self.right[i].pop()
        #         self.right[i].append(fit[i])


        # Generate x and y values for plotting
        if left_lane: self.left_fitted_curve = fit[0] * y_range ** 2 + fit[1] * y_range + fit[2]
        else: self.right_fitted_curve = fit[0] * y_range ** 2 + fit[1] * y_range + fit[2]

        # Plot lane
        if CurveCalculator.DEBUG:
            lane_color = [255, 0, 100]
            if left_lane: lane_color = [0, 0, 255]

            out_img[nonzero_y_pixels[lane_pixels], nonzero_x_pixels[lane_pixels]] = lane_color
        
            cv2.imshow("out", out_img)
            cv2.waitKey(0)

    def calculate_histogram(self, img):
        lower_half_img = img[img.shape[0] // 2:, :]
        return np.sum(lower_half_img, axis=0)

    def fit_curve_worldspace(self, img, mode="B"):

        left_curve_radius = None
        right_curve_radius = None
        if self.left_fitted_curve is not None:
            left_curve_radius, current_left_x = self.get_curve_radius(img, self.left_fitted_curve)
        if self.right_fitted_curve is not None:
            right_curve_radius, current_right_x = self.get_curve_radius(img, self.right_fitted_curve)

        bycicle_position = img.shape[1] / 2  # assumed to be in the middle of image
        offset = None
        if mode == "B":
            if right_curve_radius is not None and left_curve_radius is not None:
                offset = (bycicle_position-(current_right_x + current_left_x)/2) * self.meters_per_pixel_x / 10
        if mode == "R":
            if right_curve_radius is not None:
                offset = (bycicle_position-current_right_x) * self.meters_per_pixel_x / 10

        if CurveCalculator.DEBUG:
            print("Calculated Curve in Worldspace for mode", mode, "is:", (left_curve_radius, right_curve_radius, offset))

        return (left_curve_radius, right_curve_radius, offset)

    def get_curve_radius(self, img, x_points):
        img_height: int = img.shape[0]
        lowest_y_pixel: float = img_height - 1  # "closest y pixel of curve to current position"
        curve_pixels_y: np.ndarray = np.linspace(0, lowest_y_pixel, img_height)
        fitted_curve = np.polyfit(curve_pixels_y * self.meters_per_pixel_y, x_points * self.meters_per_pixel_x, 2)
        # Calculate radius
        curve_radius = ((1 + (2 * fitted_curve[0] * lowest_y_pixel * self.meters_per_pixel_y + fitted_curve[
            1]) ** 2) ** 1.5) / np.absolute((2 * fitted_curve[0]))
        current_x = fitted_curve[0] * img.shape[0] ** 2 + fitted_curve[1] * img.shape[0] + fitted_curve[2]
        return curve_radius, current_x

    def draw_lanes(self, img):
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        color_img = np.zeros_like(img)

        left = None
        right = None
        if self.left_fitted_curve is not None:
            left = np.int_(np.array([np.transpose(np.vstack([self.left_fitted_curve, ploty]))]))
            color_img[np.int_(ploty), np.int_(self.left_fitted_curve)] = [225, 0, 0]
        if self.right_fitted_curve is not None:
            right = np.int_(np.array([np.flipud(np.transpose(np.vstack([self.right_fitted_curve, ploty])))]))
            color_img[np.int_(ploty), np.int_(self.right_fitted_curve)] = [0, 225, 0]
       
        if left is not None and right is not None:
            points = np.hstack((left, right))
            cv2.fillPoly(color_img, np.int_(points), (0, 200, 255))

        inv_perspective = ImageWarper().inv_perspective_warp(color_img, dst_size=(img.shape[1], img.shape[0]))
        inv_perspective = cv2.addWeighted(img, 1, inv_perspective, 0.7, 0)
        return inv_perspective
