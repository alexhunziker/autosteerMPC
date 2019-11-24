import cv2
import matplotlib.pyplot as plt
import numpy as np


class ImagePreprocessor(object):
    """
    This aims at recognizing curved lanes;
    """
    DEFAULT_DESTINATION_SIZE = (1280, 720)
    DEFAULT_SOURCE_ROI = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    DEFAULT_DESTINATION_ROI = np.float32([(0, 0), (1, 0), (0, 1), (1, 1)])

    DEFAULT_GRADIENT_THRESHOLD = (15, 255)
    DEFAULT_SATURATION_THRESHOLD = (100, 255)

    def __init__(self,
                 gradient_threshold=DEFAULT_GRADIENT_THRESHOLD,
                 saturation_threshold=DEFAULT_SATURATION_THRESHOLD):
        self.gradient_threshold = gradient_threshold
        self.saturation_threshold = saturation_threshold

    def process(self, img):
        # May require undistorting the image
        hls_channels: list = ImagePreprocessor.convert_to_hls(img)
        sobel = ImagePreprocessor.sobel_edge_detection(hls_channels[1])
        sobel_x_binary = self.apply_gradient_threshold(sobel)
        saturation_binary = self.apply_saturation_gradient(hls_channels[2])
        combined_binary = self.combine_binaries(sobel_x_binary, saturation_binary)

        return combined_binary

    @classmethod
    def convert_to_hls(cls, img):
        hls_img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        h_channel = hls_img[:, :, 0]
        l_channel = hls_img[:, :, 1]
        s_channel = hls_img[:, :, 2]
        return [h_channel, l_channel, s_channel]

    @classmethod
    def sobel_edge_detection(cls, lightness):
        lightness_denoised = cv2.fastNlMeansDenoising(lightness, searchWindowSize=9, templateWindowSize=5)
        ddepth_datatype = cv2.CV_64F  # Allows to detect both edges, alternatively use cv2.CV8U
        sobel_x_derivative = cv2.Sobel(lightness_denoised, ddepth_datatype, 1, 0)
        absolute_sobel_x = np.absolute(sobel_x_derivative)
        return np.uint8(255 * absolute_sobel_x / (np.max(absolute_sobel_x) * 0.5))

    def apply_gradient_threshold(self, data):
        """
        Convert Sobel scaled derivative into binary image
        """
        gradient_binary = np.zeros_like(data)
        gradient_binary[(data >= self.gradient_threshold[0]) & (data <= self.gradient_threshold[1])] = 1
        return gradient_binary

    def apply_saturation_gradient(self, data):
        """
        If the road is dark, it will have low saturation
        TODO: Will we drive on light brown stones? then this assumption sucks
        """
        saturation_binary = np.zeros_like(data)
        saturation_binary[(data >= self.saturation_threshold[0]) & (data <= self.saturation_threshold[1])] = 1
        return saturation_binary

    def combine_binaries(self, binary_one, binary_two):
        """
        Combines two binaries by OR operator
        TODO: in general, this np.zeros_like is stupid, i guess we can do this directly
        """
        combined_binary = np.zeros_like(binary_one)
        combined_binary[(binary_one == 1) | (binary_two == 1)] = 1
        return combined_binary

    def plot_warp_result(self, original, result):
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        ax1.imshow(original)
        ax1.set_title('Original Image', fontsize=30)
        ax2.imshow(result, cmap='gray')
        ax2.set_title('Warped Image', fontsize=30)
        plt.show()

if __name__ == "__main__":
    img = cv2.imread('resources/lane_curve_1.jpg')
    ImagePreprocessor().process(img)
