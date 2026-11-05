import numpy as np

from so101_vision.segmentation import hsv_mask, largest_blob


def image_with_square(u, v, size=10, shape=(120, 160)):
    hsv = np.zeros((*shape, 3), np.uint8)
    hsv[v:v + size, u:u + size] = (170, 200, 200)   # saturated red
    return hsv


RED = [((0, 120, 50), (6, 255, 255)), ((160, 120, 50), (180, 255, 255))]


def test_largest_blob_centroid():
    blob = largest_blob(hsv_mask(image_with_square(40, 30), RED))
    assert np.allclose(blob.center, (44.5, 34.5))
    assert not blob.touches_border((120, 160))


def test_blob_cut_by_the_image_edge():
    hsv = image_with_square(70, 115)          # bottom rows fall outside the image
    blob = largest_blob(hsv_mask(hsv, RED))
    assert blob.touches_border(hsv.shape)


def test_nothing_found():
    assert largest_blob(hsv_mask(np.zeros((50, 50, 3), np.uint8), RED)) is None
