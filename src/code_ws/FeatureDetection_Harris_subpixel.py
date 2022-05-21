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

    dst = cv.dilate(dst, None)
    ret, dst = cv.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)
    # find centroids
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv.cornerSubPix(
        gray_left, np.float32(centroids), (5, 5), (-1, -1), criteria
    )
    # Now draw them
    res = np.hstack((centroids, corners))
    res = np.int0(res)
    img_left[res[:, 1], res[:, 0]] = [0, 0, 255]
    img_left[res[:, 3], res[:, 2]] = [0, 255, 0]
    cv.imshow("subpixel5.png", img_left)
    cv.waitKey(0)

    # # Normalizing
    # dst_norm = np.empty(dst.shape, dtype=np.float32)
    # cv.normalize(dst, dst_norm, alpha=0, beta=255, norm_type=cv.NORM_MINMAX)
    # dst_norm_scaled = cv.convertScaleAbs(dst_norm)
    # cv.imshow("dst norm scaled", dst_norm_scaled)
    # cv.waitKey(0)

    # thresh = 180
    # # Drawing a circle around corners
    # for i in range(dst_norm.shape[0]):
    #     for j in range(dst_norm.shape[1]):
    #         if int(dst_norm[i, j]) > thresh:
    #             # cv.circle(dst_norm_scaled, (j, i), 5, (0), 2)
    #             cv.circle(img_left, (j, i), 5, (0, 255, 0), 2)

    # # cv.imshow("dst", dst_norm_scaled)
    # cv.imshow("dst", img_left)
    # cv.waitKey(0)
