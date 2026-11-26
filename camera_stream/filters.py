"""Crack/line highlighting filters. Pure functions: no I/O, no global state.

Every mode takes a BGR uint8 frame and returns a new BGR uint8 frame of the same
size: the detected edges painted in a bright color over a dimmed original.

Modes:
    none       original frame
    canny      main mode: Canny with median-based automatic thresholds
    blackhat   crack-specific: morphological black-hat picks thin dark structures
    sobel      gradient magnitude (comparison)
    laplacian  absolute Laplacian (comparison)

Shared preprocessing: grayscale -> CLAHE -> bilateral filter. Canny and black-hat
masks are cleaned up by dropping contours that are short or not elongated, which
removes texture and sensor noise while keeping line-like cracks, and then closed
to join the surviving segments. The cleanup runs before the close on purpose:
closing first lets texture noise bridge into one large blob that swallows the crack.
"""
import json
import logging

import cv2
import numpy as np

log = logging.getLogger(__name__)

MODES = ("none", "canny", "blackhat", "sobel", "laplacian")

# Friendly names accepted from the dashboard or the command line.
ALIASES = {
    "original": "none",
    "raw": "none",
    "off": "none",
    "black-hat": "blackhat",
    "black_hat": "blackhat",
    "laplaciano": "laplacian",
}

DEFAULT_PARAMS = {
    # Preprocessing
    "clahe_clip": 2.0,            # 0 disables CLAHE
    "clahe_tile": 8,
    "bilateral_diameter": 9,      # 0 disables the bilateral filter
    "bilateral_sigma_color": 75.0,
    "bilateral_sigma_space": 75.0,
    # Canny
    "canny_sigma": 0.33,          # thresholds = (1 -/+ sigma) * median intensity
    # Black-hat
    "blackhat_kernel": 15,        # elliptical kernel size; wider than the cracks to find
    "blackhat_threshold": 0,      # 0 = Otsu, otherwise a fixed threshold (0..255)
    "blackhat_min_threshold": 40, # floor for Otsu: on textured surfaces Otsu alone splits the noise
    # Contour cleanup (canny and blackhat)
    "close_kernel": 3,            # morphological close after cleanup, joins broken segments; 0 disables
    "min_length": 40,             # px, long side of the contour's rotated bounding box
    "min_aspect": 3.0,            # long side / short side of that box
    # Comparison modes
    "sobel_kernel": 3,
    "laplacian_kernel": 3,
    "laplacian_blur": 3,          # Gaussian kernel before the Laplacian; 0 disables
    # Overlay
    "edge_color": (0, 255, 255),  # BGR, yellow
    "dim": 0.4,                   # brightness kept from the original under the overlay
}


def resolve_mode(name):
    """Return the canonical mode name; unknown names fall back to 'none' with a warning."""
    key = str(name).strip().lower()
    key = ALIASES.get(key, key)
    if key not in MODES:
        log.warning("Unknown filter mode %r; falling back to 'none'", name)
        return "none"
    return key


def _odd(value, minimum=1, maximum=99):
    value = max(minimum, min(maximum, int(round(float(value)))))
    return value if value % 2 else value + 1


def merge_params(params=None):
    """Combine user params with the defaults, coercing types and clamping ranges.

    Unknown keys are ignored so a newer dashboard cannot break an older server.
    Raises ValueError/TypeError when a known key has an unusable value.
    """
    p = dict(DEFAULT_PARAMS)
    for key, value in (params or {}).items():
        if key in p and value is not None:
            p[key] = value
    p["clahe_clip"] = max(0.0, float(p["clahe_clip"]))
    p["clahe_tile"] = max(1, int(p["clahe_tile"]))
    p["bilateral_diameter"] = max(0, int(p["bilateral_diameter"]))
    p["bilateral_sigma_color"] = float(p["bilateral_sigma_color"])
    p["bilateral_sigma_space"] = float(p["bilateral_sigma_space"])
    p["canny_sigma"] = max(0.0, min(1.0, float(p["canny_sigma"])))
    p["blackhat_kernel"] = _odd(p["blackhat_kernel"], minimum=3)
    p["blackhat_threshold"] = max(0, min(255, int(p["blackhat_threshold"])))
    p["blackhat_min_threshold"] = max(0, min(255, int(p["blackhat_min_threshold"])))
    p["close_kernel"] = max(0, int(p["close_kernel"]))
    p["min_length"] = max(0.0, float(p["min_length"]))
    p["min_aspect"] = max(0.0, float(p["min_aspect"]))
    p["sobel_kernel"] = _odd(p["sobel_kernel"], maximum=7)
    p["laplacian_kernel"] = _odd(p["laplacian_kernel"], maximum=31)
    p["laplacian_blur"] = 0 if int(p["laplacian_blur"]) <= 0 else _odd(p["laplacian_blur"])
    p["edge_color"] = tuple(int(max(0, min(255, c))) for c in p["edge_color"])[:3]
    p["dim"] = max(0.0, min(1.0, float(p["dim"])))
    return p


def parse_filter_command(payload):
    """Parse an MQTT payload into (mode, params).

    Accepts a plain name ("canny"), a JSON string ("\\"canny\\"") or a JSON object
    ({"filter": "canny", "params": {...}}). Unknown names resolve to 'none'.
    Raises ValueError for empty payloads or malformed JSON objects.
    """
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8", errors="replace")
    text = str(payload).strip()
    if not text:
        raise ValueError("empty filter command")
    params = {}
    if text[0] in "{\"":
        try:
            data = json.loads(text)
        except json.JSONDecodeError as error:
            raise ValueError(f"invalid filter JSON: {error}") from error
        if isinstance(data, str):
            name = data
        elif isinstance(data, dict) and "filter" in data:
            name = data["filter"]
            params = data.get("params") or {}
            if not isinstance(params, dict):
                raise ValueError("'params' must be an object")
        else:
            raise ValueError("filter JSON must contain a 'filter' field")
    else:
        name = text
    return resolve_mode(name), params


# --------------------------------------------------------------------------- stages


def preprocess(frame, p):
    """Grayscale -> CLAHE -> bilateral filter; returns uint8 gray."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if p["clahe_clip"] > 0:
        clahe = cv2.createCLAHE(clipLimit=p["clahe_clip"], tileGridSize=(p["clahe_tile"], p["clahe_tile"]))
        gray = clahe.apply(gray)
    if p["bilateral_diameter"] > 0:
        gray = cv2.bilateralFilter(gray, p["bilateral_diameter"], p["bilateral_sigma_color"], p["bilateral_sigma_space"])
    return gray


def auto_canny(gray, sigma):
    median = float(np.median(gray))
    low = int(max(0, (1.0 - sigma) * median))
    high = int(min(255, (1.0 + sigma) * median))
    return cv2.Canny(gray, low, max(high, low + 1))


def blackhat_mask(gray, p):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p["blackhat_kernel"], p["blackhat_kernel"]))
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)
    if p["blackhat_threshold"] > 0:
        threshold = p["blackhat_threshold"]
    else:
        otsu, _ = cv2.threshold(hat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        threshold = max(otsu, p["blackhat_min_threshold"])
    return np.where(hat > threshold, 255, 0).astype(np.uint8)


def close(mask, size):
    if size <= 0:
        return mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def keep_elongated(mask, min_length, min_aspect):
    """Drop contours whose rotated box is shorter than min_length or less elongated than min_aspect."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    keep = np.zeros_like(mask)
    for contour in contours:
        (_, _), (w, h), _ = cv2.minAreaRect(contour)
        long_side, short_side = max(w, h), max(min(w, h), 1.0)
        if long_side >= min_length and long_side / short_side >= min_aspect:
            cv2.drawContours(keep, [contour], -1, 255, thickness=cv2.FILLED)
    return cv2.bitwise_and(mask, keep)


def normalize_to_uint8(values):
    peak = float(values.max()) if values.size else 0.0
    if peak <= 0:
        return np.zeros(values.shape, np.uint8)
    return np.clip(values * (255.0 / peak), 0, 255).astype(np.uint8)


def edge_map(gray, mode, p):
    """Return a uint8 edge-strength map (0..255) for an edge mode."""
    if mode in ("canny", "blackhat"):
        mask = auto_canny(gray, p["canny_sigma"]) if mode == "canny" else blackhat_mask(gray, p)
        return close(keep_elongated(mask, p["min_length"], p["min_aspect"]), p["close_kernel"])
    if mode == "sobel":
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=p["sobel_kernel"])
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=p["sobel_kernel"])
        return normalize_to_uint8(cv2.magnitude(gx, gy))
    if mode == "laplacian":
        if p["laplacian_blur"]:
            gray = cv2.GaussianBlur(gray, (p["laplacian_blur"], p["laplacian_blur"]), 0)
        return normalize_to_uint8(np.abs(cv2.Laplacian(gray, cv2.CV_32F, ksize=p["laplacian_kernel"])))
    raise ValueError(f"{mode!r} is not an edge mode")


def overlay_edges(frame, edges, color, dim):
    """Blend `color` over a dimmed `frame`, weighted by edge strength (0..255)."""
    # Native OpenCV ops: several times faster than the equivalent float NumPy math.
    base = cv2.convertScaleAbs(frame, alpha=dim)
    paint = np.empty_like(frame)
    paint[:] = color
    weight = cv2.multiply(edges, 1.0 / 255.0, dtype=cv2.CV_32F)
    return cv2.blendLinear(paint, base, weight, 1.0 - weight)


def apply_filter(frame, mode="none", params=None):
    """Apply a filter mode; always returns a new BGR uint8 frame of the input size.

    Grayscale input is accepted and converted to BGR. Unknown modes fall back to 'none'.
    """
    frame = np.ascontiguousarray(frame, dtype=np.uint8)
    if frame.ndim == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    mode = resolve_mode(mode)
    if mode == "none":
        return frame.copy()
    p = merge_params(params)
    edges = edge_map(preprocess(frame, p), mode, p)
    return overlay_edges(frame, edges, p["edge_color"], p["dim"])
