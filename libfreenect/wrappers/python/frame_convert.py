import numpy as np
import cv2 as cv

def pretty_depth(depth):
    """Converts raw 16-bit depth into an 8-bit format for display.

    Args:
        depth: A NumPy array of dtype uint16 (2 bytes per pixel).

    Returns:
        A NumPy array of dtype uint8 that can be shown with cv2.imshow.
    """
    # Clamp to [0, 1023], then shift right by 2 bits to map 10-bit → 8-bit
    np.clip(depth, 0, 2**10 - 1, out=depth)
    depth8 = (depth >> 2).astype(np.uint8)
    return depth8

def pretty_depth_cv(depth):
    """Wraps pretty_depth for use with cv2.

    Args:
        depth: A NumPy array of dtype uint16, shape (H, W).

    Returns:
        A single‐channel (grayscale) NumPy uint8 image of shape (H, W).
    """
    return pretty_depth(depth)


def video_cv(video):
    """Converts raw RGB video (uint8) into BGR for cv2.

    Args:
        video: A NumPy array of dtype uint8 with shape (H, W, 3), in RGB order.

    Returns:
        A NumPy array of dtype uint8 with shape (H, W, 3), in BGR order.
    """
    # Simply flip from RGB → BGR
    return video[:, :, ::-1]
