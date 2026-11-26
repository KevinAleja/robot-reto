import logging

import cv2
import numpy as np
import pytest

from filters import DEFAULT_PARAMS, MODES, apply_filter, parse_filter_command, resolve_mode

EDGE_COLOR = np.array(DEFAULT_PARAMS["edge_color"], np.uint8)
H, W = 240, 320
LINE_Y = 120


def edge_pixels(output):
    """Boolean mask of pixels painted with the overlay color."""
    return np.all(output == EDGE_COLOR, axis=2)


@pytest.fixture
def line_image():
    """Flat gray background with a thin dark horizontal line, like a crack."""
    image = np.full((H, W, 3), 150, np.uint8)
    cv2.line(image, (20, LINE_Y), (W - 20, LINE_Y), (40, 40, 40), 2)
    return image


@pytest.fixture
def noisy_flat_image():
    """Harsh per-pixel texture (uniform 90..140), worse than a JPEG camera frame."""
    rng = np.random.default_rng(0)
    gray = rng.integers(90, 140, (H, W), dtype=np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


@pytest.mark.parametrize("mode", MODES)
def test_every_mode_returns_uint8_bgr_of_same_shape(mode, line_image):
    output = apply_filter(line_image, mode)
    assert output.dtype == np.uint8
    assert output.shape == line_image.shape
    assert output is not line_image


@pytest.mark.parametrize("mode", MODES)
def test_grayscale_input_is_returned_as_bgr(mode, line_image):
    gray = cv2.cvtColor(line_image, cv2.COLOR_BGR2GRAY)
    assert apply_filter(gray, mode).shape == (H, W, 3)


def test_none_returns_an_unmodified_copy(line_image):
    output = apply_filter(line_image, "none")
    assert np.array_equal(output, line_image)


@pytest.mark.parametrize("mode", ["canny", "blackhat"])
def test_thin_dark_line_is_detected_along_the_line_only(mode, line_image):
    painted = edge_pixels(apply_filter(line_image, mode))
    near_line = np.zeros_like(painted)
    near_line[LINE_Y - 6:LINE_Y + 7, :] = True

    columns_hit = painted[near_line.nonzero()].reshape(13, W).any(axis=0)
    # The line spans x = 20..300; most of those columns must be marked.
    assert columns_hit[30:290].mean() > 0.9
    assert painted[~near_line].sum() < 0.01 * (~near_line).sum()


@pytest.mark.parametrize("mode", ["canny", "blackhat"])
def test_contour_cleanup_keeps_noisy_flat_image_mostly_clean(mode, noisy_flat_image):
    painted = edge_pixels(apply_filter(noisy_flat_image, mode))
    assert painted.mean() < 0.02


def test_cleanup_is_what_removes_the_noise(noisy_flat_image):
    loose = {"min_length": 0, "min_aspect": 0}
    raw_noise = edge_pixels(apply_filter(noisy_flat_image, "canny", loose)).mean()
    cleaned = edge_pixels(apply_filter(noisy_flat_image, "canny")).mean()
    assert cleaned < raw_noise


def test_blackhat_finds_a_crack_on_textured_noise(noisy_flat_image):
    image = noisy_flat_image.copy()
    points = np.array([[20, 110], [90, 125], [160, 112], [230, 130], [300, 118]], np.int32)
    cv2.polylines(image, [points], False, (30, 30, 30), 3)
    crack_band = np.zeros((H, W), np.uint8)
    cv2.polylines(crack_band, [points], False, 255, 9)

    painted = edge_pixels(apply_filter(image, "blackhat"))
    assert painted[crack_band > 0].mean() > 0.3
    assert painted[crack_band == 0].mean() < 0.005


@pytest.mark.parametrize("mode", ["sobel", "laplacian"])
def test_comparison_modes_respond_to_the_line(mode, line_image):
    output = apply_filter(line_image, mode)
    brightness = output.astype(np.int32).sum(axis=2)
    assert brightness[LINE_Y - 3:LINE_Y + 4].mean() > brightness[:60].mean() + 50


def test_unknown_mode_falls_back_to_none_with_warning(line_image, caplog):
    with caplog.at_level(logging.WARNING, logger="filters"):
        output = apply_filter(line_image, "thermal")
    assert np.array_equal(output, line_image)
    assert "thermal" in caplog.text


def test_resolve_mode_accepts_aliases():
    assert resolve_mode(" Canny ") == "canny"
    assert resolve_mode("Black-hat") == "blackhat"
    assert resolve_mode("Laplaciano") == "laplacian"
    assert resolve_mode("original") == "none"


@pytest.mark.parametrize("payload, expected", [
    (b"canny", ("canny", {})),
    ('"sobel"', ("sobel", {})),
    ('{"filter": "blackhat", "params": {"blackhat_kernel": 21}}', ("blackhat", {"blackhat_kernel": 21})),
    ("unknown", ("none", {})),
])
def test_parse_filter_command(payload, expected):
    assert parse_filter_command(payload) == expected


@pytest.mark.parametrize("payload", ["", "{not json", '{"params": {}}', '{"filter": "canny", "params": 3}'])
def test_parse_filter_command_rejects_malformed(payload):
    with pytest.raises(ValueError):
        parse_filter_command(payload)


def test_params_are_tunable(line_image):
    default = apply_filter(line_image, "canny")
    red = apply_filter(line_image, "canny", {"edge_color": (0, 0, 255), "dim": 0.0})
    assert not np.array_equal(default, red)
    assert red[..., 1].max() == 0  # dim 0 and pure red: no green anywhere
