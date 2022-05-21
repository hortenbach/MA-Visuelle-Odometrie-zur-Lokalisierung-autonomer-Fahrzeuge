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

    orb = cv.ORB_create()

    # Find keypoints and descriptors with ORB
    keypoints_left, descriptors_left = orb.detectAndCompute(img_left, None)
    keypoints_right, descriptors_right = orb.detectAndCompute(img_left, None)

    # Create a BFMatcher object.
    # It will find all of the matching keypoints on two images
    bf = cv.BFMatcher_create(cv.NORM_HAMMING, crossCheck=True)

    matches = bf.match(descriptors_left, descriptors_right)
    single_match = matches[0]
    single_match.distance

    ORB_matches = cv.drawMatches(
        img_left,
        keypoints_left,
        img_right,
        keypoints_right,
        matches[:30],
        None,
        flags=2,
    )
    cv.imshow("orb matches", ORB_matches)

    # cv.imshow("dst", dst_norm_scaled)
    cv.imshow("dst", img_left)
    cv.waitKey(0)
