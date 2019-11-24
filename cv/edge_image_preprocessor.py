import cv2
import matplotlib.pyplot as plt
import numpy as np


class EdgeImagePreprocessor(object):
    """
    This class prepares images to detect the edge of roads
    """
    DEFAULT_DESTINATION_SIZE = (800, 600)
    DEFAULT_SOURCE_ROI = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    DEFAULT_DESTINATION_ROI = np.float32([(0, 0), (1, 0), (0, 1), (1, 1)])

    DEFAULT_GRADIENT_THRESHOLD = (30, 255)
    DEFAULT_SATURATION_THRESHOLD = (30, 255)

    def __init__(self,
                 gradient_threshold=DEFAULT_GRADIENT_THRESHOLD,
                 saturation_threshold=DEFAULT_SATURATION_THRESHOLD,
                 destination_size=DEFAULT_DESTINATION_SIZE,
                 show_intermediate_steps=False):
        self.destination_size = destination_size
        self.show_intermediate_steps = show_intermediate_steps
        self.gradient_threshold = gradient_threshold
        self.saturation_threshold = saturation_threshold

    def process(self, img):
        """
        Given a 3 channel input image, a binary output will be generated
        """
        # May require undistorting the image
        hls_channels: list = EdgeImagePreprocessor.convert_to_hls(img)
        if self.show_intermediate_steps is True:
            self.compare_images(img, hls_channels[1], "original", "light channel")
        if self.show_intermediate_steps is True:
            self.compare_images(img, hls_channels[2], "original", "saturation channel")

        sobel = EdgeImagePreprocessor.sobel_edge_detection(hls_channels[1])
        sobel_x_binary = self.apply_gradient_threshold(sobel)
        if self.show_intermediate_steps is True:
            self.compare_images(hls_channels[1], sobel_x_binary, "light channel", "sobel filter binary")
        saturation_binary = self.apply_saturation_gradient(hls_channels[2])
        if self.show_intermediate_steps is True:
            self.compare_images(hls_channels[2], saturation_binary, "saturation channel", "saturation filter binary")
        combined_binary = self.combine_binaries(sobel_x_binary, saturation_binary)
        if self.show_intermediate_steps is True:
            self.compare_images(img, saturation_binary, "Original Image", "Resulting binary")

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
        """
        Getting the edges based on Sobel,
        assumption: Lightness does differ from road to non road
        """
        ddepth_datatype = cv2.CV_64F  # Allows to detect both edges, alternatively use cv2.CV8U
        sobel_x_derivative = cv2.Sobel(lightness, ddepth_datatype, 1, 1)
        absolute_sobel_x = np.absolute(sobel_x_derivative)
        return np.minimum(np.full(cls.DEFAULT_DESTINATION_SIZE, 255, dtype=np.uint8).transpose(),
                          np.uint8(255 * absolute_sobel_x / np.max(absolute_sobel_x)))

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
        """
        saturation_binary = np.zeros_like(data)
        saturation_binary[(data >= self.saturation_threshold[0]) & (data <= self.saturation_threshold[1])] = 1
        return saturation_binary

    def combine_binaries(self, binary_one, binary_two):
        """
        Combines two binaries by OR operator
        """
        # This may be done without the zeros_like directly, this may minimally improve performance
        combined_binary = np.zeros_like(binary_one)
        combined_binary[(binary_one == 1) | (binary_two == 1)] = 1
        return combined_binary

    def compare_images(self, original, result, descr1, descr2):
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        ax1.imshow(original)
        ax1.set_title(descr1, fontsize=30)
        ax2.imshow(result, cmap='gray')
        ax2.set_title(descr2, fontsize=30)
        plt.show()


if __name__ == "__main__":
    img = cv2.imread('resources/straight_5.jpg')
    EdgeImagePreprocessor(show_intermediate_steps=True).process(img)
