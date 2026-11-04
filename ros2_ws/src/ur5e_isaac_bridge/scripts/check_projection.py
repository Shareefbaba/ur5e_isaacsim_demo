#!/usr/bin/env python3
"""Project the known cube center onto the five saved wrist-camera images."""

import json
import math
from pathlib import Path

import cv2
import yaml


RESULTS = Path(__file__).resolve().parent.parent / "resuts"


def base_to_camera(point, pose):
    """Invert the saved base_link <- camera_optical_frame transform."""
    t = pose["translation_xyz"]
    x, y, z, w = pose["quaternion_xyzw"]
    length = math.sqrt(x*x + y*y + z*z + w*w)
    if length == 0:
        raise ValueError("Camera quaternion has zero length")
    x, y, z, w = -x/length, -y/length, -z/length, w/length
    a, b, c = [p - o for p, o in zip(point, t)]
    tx, ty, tz = 2*(y*c - z*b), 2*(z*a - x*c), 2*(x*b - y*a)
    return (a + w*tx + y*tz - z*ty,
            b + w*ty + z*tx - x*tz,
            c + w*tz + x*ty - y*tx)


def main():
    info = yaml.safe_load((RESULTS / "camera_info.yaml").read_text())
    k = info["camera_matrix"]["data"]
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    distortion = info.get("distortion_coefficients", {}).get("data", [])
    if any(abs(value) > 1e-9 for value in distortion):
        raise ValueError("This checker expects zero camera distortion")

    records = [json.loads(line) for line in
               (RESULTS / "poses.jsonl").read_text().splitlines() if line.strip()]
    if len(records) != 5:
        raise ValueError(f"Expected 5 captures; found {len(records)}")

    output = RESULTS / "projected"
    output.mkdir(exist_ok=True)

    for i, record in enumerate(records, start=1):
        image_path = RESULTS / record["image"]
        image = cv2.imread(str(image_path))
        if image is None:
            raise FileNotFoundError(image_path)

        cube_camera = base_to_camera(record["cube_center_base_xyz"],
                                     record["camera_pose"])
        x, y, z = cube_camera
        if z <= 0:
            print(f"View {i}: cube is behind the camera (z={z:.3f} m)")
            continue

        u, v = fx*x/z + cx, fy*y/z + cy
        pixel = (round(u), round(v))
        visible = 0 <= u < image.shape[1] and 0 <= v < image.shape[0]
        if visible:
            cv2.circle(image, pixel, 12, (0, 0, 0), 4)
            cv2.drawMarker(image, pixel, (0, 255, 0), cv2.MARKER_CROSS, 22, 3)
        else:
            cv2.putText(image, "CUBE PROJECTION OUTSIDE IMAGE", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        destination = output / f"img_{i:02d}_proj.png"
        cv2.imwrite(str(destination), image)
        print(f"View {i}: pixel ({u:.1f}, {v:.1f}), depth {z:.3f} m"
              f" - {'in frame' if visible else 'outside image'}")

    print(f"Projection images: {output}")
    print("Open each image and verify the green cross lands on the orange cube.")


if __name__ == "__main__":
    main()
