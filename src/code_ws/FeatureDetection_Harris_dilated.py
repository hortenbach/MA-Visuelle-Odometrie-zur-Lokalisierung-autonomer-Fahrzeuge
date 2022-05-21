#!/bin/python3
import cv2 as cv
import numpy as np


def read_img(img_num):
    img = cv.imread(f"./pictures/image_left_{img_num}.jpg")
    return img


def read_stereoFromFiles(img_num):
    return [read_img(img_num), read_img(img_num)]


if __name__ == "__main__":
    # read images from files
    img_left, img_right = read_stereoFromFiles(352)
    # rpepare images
    gray_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
    gray_left = np.float32(gray_left)
    # apply corner detection
    dst = cv.cornerHarris(gray_left, 2, 3, 0.04)

    # result is dilated for marking the corners, not important
    dst = cv.dilate(dst, None)
    # Threshold for an optimal value, it may vary depending on the image.
    img_left[dst > 0.01 * dst.max()] = [0, 0, 255]
    cv.imshow("dst", img_left)
    cv.waitKey(0)
