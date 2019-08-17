from typing import List

import cv2
import numpy as np


class CurveCalculator(object):
    DEBUG = False

    DEFAULT_METERS_PER_PIXEL_X = 3.7 / 720
    DEFAULT_METERS_PER_PIXEL_Y = 30.5 / 720

    def __init__(self,
                 meters_per_pixel_x=DEFAULT_METERS_PER_PIXEL_X,
                 meters_per_pixel_y=DEFAULT_METERS_PER_PIXEL_Y
                 ):
        self.left = [[], [], []]
        self.right = [[], [], []]

        self.right_fitted_curve: List[float] = [0.0]
        self.left_fitted_curve: List[float] = [0.0]

        self.meters_per_pixel_x = meters_per_pixel_x
        self.meters_per_pixel_y = meters_per_pixel_y

    def sliding_window(self, img, n_windows=9, window_margin=150,
                       minpix=1, draw_windows=True):
        image_height = img.shape[0]
        image_width = img.shape[1]

        left_fit = np.empty(3)
        right_fit = np.empty(3)
        out_img = np.dstack((img, img, img)) * 255

        # Find starting points
        histogram = self.calculate_histogram(img)
        img_middle = int(histogram.shape[0] / 2)
        left_starting_point = np.argmax(histogram[:img_middle])
        right_starting_point = np.argmax(histogram[img_middle:]) + img_middle

        # Set height of windows
        window_height = np.int(image_height / n_windows)

        # Identify x and y positions of nonzero pixels
        nonzero = img.nonzero()
        nonzero_y_pixels = np.array(nonzero[0])
        nonzero_x_pixels = np.array(nonzero[1])

        # TODO: set values directly?
        current_left_x_position = left_starting_point
        current_right_x_position = right_starting_point

        left_lane_pixels = []
        right_lane_pixels = []

        for window in range(n_windows):
            window_y_lower_boundry = image_height - (window + 1) * window_height
            window_y_upper_boundry = image_height - window * window_height
            window_x_left_lower = current_left_x_position - window_margin
            window_x_left_upper = current_left_x_position + window_margin
            window_x_right_lower = current_right_x_position - window_margin
            window_x_right_upper = current_right_x_position + window_margin

            if CurveCalculator.DEBUG:
                cv2.rectangle(out_img, (window_x_left_lower, window_y_lower_boundry),
                              (window_x_left_upper, window_y_upper_boundry), (100, 225, 225), 3)
                cv2.rectangle(out_img, (window_x_right_lower, window_y_lower_boundry),
                              (window_x_right_upper, window_y_upper_boundry), (100, 225, 225), 3)

            # get nonzero areas within window
            non_zero_pixels_left = ((nonzero_y_pixels >= window_y_lower_boundry) &
                                    (nonzero_y_pixels < window_y_upper_boundry) &
                                    (nonzero_x_pixels >= window_x_left_lower) &
                                    (nonzero_x_pixels < window_x_left_upper)
                                    ).nonzero()[0]
            non_zero_pixels_right = ((nonzero_y_pixels >= window_y_lower_boundry) &
                                     (nonzero_y_pixels < window_y_upper_boundry) &
                                     (nonzero_x_pixels >= window_x_right_lower) &
                                     (nonzero_x_pixels < window_x_right_upper)
                                     ).nonzero()[0]

            left_lane_pixels.append(non_zero_pixels_left)
            right_lane_pixels.append(non_zero_pixels_right)

            # Recenter next window
            # TODO: Mean is probably not the way to go here
            if len(non_zero_pixels_left) > minpix:
                current_left_x_position = np.int(np.mean(nonzero_x_pixels[non_zero_pixels_left]))
            if len(non_zero_pixels_right) > minpix:
                current_right_x_position = np.int(np.mean(nonzero_x_pixels[non_zero_pixels_right]))

        left_lane_pixels = np.concatenate(left_lane_pixels)
        right_lane_pixels = np.concatenate(right_lane_pixels)

        # Extract pixel positions
        left_x_lane_pixels = nonzero_x_pixels[left_lane_pixels]
        left_y_lane_pixels = nonzero_y_pixels[left_lane_pixels]
        right_x_lane_pixels = nonzero_x_pixels[right_lane_pixels]
        right_y_lane_pixels = nonzero_y_pixels[right_lane_pixels]

        # Fit second order polinomial
        left_fit = np.polyfit(left_y_lane_pixels, left_x_lane_pixels, 2)
        right_fit = np.polyfit(right_y_lane_pixels, right_x_lane_pixels, 2)

        # TODO: Maybe do a mean over past 10 polynomials, otherwise remove block
        self.left[0].append(left_fit[0])
        self.left[1].append(left_fit[1])
        self.left[2].append(left_fit[2])

        self.right[0].append(right_fit[0])
        self.right[1].append(right_fit[1])
        self.right[2].append(right_fit[2])

        # Generate x and y values for plotting
        ploty = np.linspace(0, image_height - 1, image_height)
        self.left_fitted_curve = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        self.right_fitted_curve = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        if CurveCalculator.DEBUG:
            out_img[nonzero_y_pixels[left_lane_pixels], nonzero_x_pixels[left_lane_pixels]] = [255, 0, 100]
            out_img[nonzero_y_pixels[right_lane_pixels], nonzero_x_pixels[right_lane_pixels]] = [0, 0, 255]

        if CurveCalculator.DEBUG:
            cv2.imshow("out", out_img)
            cv2.waitKey(0)

    def calculate_histogram(self, img):
        lower_half_img = img[img.shape[0] // 2:, :]
        return np.sum(lower_half_img, axis=0)

    def fit_polynom_to_lane(self, starting_poinnt):
        pass

    def fit_curve_worldspace(self, img):
        img_height: int = img.shape[0]
        lowest_y_pixel: float = img_height - 1  # "closest y pixel of curve to current position"
        curve_pixels_y: np.ndarray = np.linspace(0, lowest_y_pixel, img_height)

        # Fit polynomial in wolrld space
        left_fit_curve = np.polyfit(curve_pixels_y * self.meters_per_pixel_y,
                                    self.left_fitted_curve * self.meters_per_pixel_x,
                                    2)
        right_fit_curve = np.polyfit(curve_pixels_y * self.meters_per_pixel_y,
                                     self.right_fitted_curve * self.meters_per_pixel_x,
                                     2)

        # Calculate radii
        left_curve_radius = ((1 + (2 * left_fit_curve[0] * lowest_y_pixel * self.meters_per_pixel_y + left_fit_curve[
            1]) ** 2) ** 1.5) / np.absolute((2 * left_fit_curve[0]))
        right_fit_radius = ((1 + (2 * right_fit_curve[0] * lowest_y_pixel * self.meters_per_pixel_y + right_fit_curve[
            1]) ** 2) ** 1.5) / np.absolute((2 * right_fit_curve[0]))

        #  get center
        car_position = img.shape[1] / 2  # car assumed to be in the middle of image
        l_fit_x_int = left_fit_curve[0] * img.shape[0] ** 2 + left_fit_curve[1] * img.shape[0] + left_fit_curve[2]
        r_fit_x_int = right_fit_curve[0] * img.shape[0] ** 2 + right_fit_curve[1] * img.shape[0] + right_fit_curve[2]
        lane_center = (r_fit_x_int + l_fit_x_int) / 2
        center = (car_position - lane_center) * self.meters_per_pixel_x / 10
        return (left_curve_radius, right_fit_radius, center)

    def draw_lanes(self, img):
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
        color_img = np.zeros_like(img)

        print(self.left_fitted_curve.shape)
        print(ploty.shape)
        print(np.vstack([self.left_fitted_curve, ploty]))
        print(np.transpose(np.vstack([self.left_fitted_curve, ploty])))
        left = np.array([np.transpose(np.vstack([self.left_fitted_curve, ploty]))])
        right = np.array([np.flipud(np.transpose(np.vstack([self.right_fitted_curve, ploty])))])
        points = np.hstack((left, right))

        cv2.fillPoly(color_img, np.int_(points), (0, 200, 255))
        inv_perspective = self.inv_perspective_warp(color_img)
        print(inv_perspective.shape)
        print(img.shape)
        inv_perspective = cv2.addWeighted(img, 1, inv_perspective, 0.7, 0)
        return inv_perspective

    # TODO: DOES NOT BELONG HERE
    def inv_perspective_warp(self, img,
                             dst_size=(720, 1280),
                             # dst_size=(608, 800),
                             src=np.float32([(0, 0), (1, 0), (0, 1), (1, 1)]),
                             dst=np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])):
        img_size = np.float32([(img.shape[1], img.shape[0])])
        src = src * img_size
        # For destination points, I'm arbitrarily choosing some points to be
        # a nice fit for displaying our warped result
        # again, not exact, but close enough for our purposes
        dst = dst * np.float32(dst_size)
        # Given src and dst points, calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(src, dst)
        # Warp the image using OpenCV warpPerspective()
        warped = cv2.warpPerspective(img, M, dst_size)
        return warped
