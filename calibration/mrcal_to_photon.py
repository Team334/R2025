import argparse
import base64
import json
import os
import cv2
import numpy as np
import mrcal # type: ignore (this exists when running in wsl)
from dataclasses import dataclass, asdict

# Assuming similar dataclass structures for simplicity and compatibility
@dataclass
class Size:
    width: int
    height: int

@dataclass
class JsonMatOfDoubles:
    rows: int
    cols: int
    type: int  # Assuming type follows OpenCV conventions, 6 for CV_64F
    data: list[float]

@dataclass
class JsonMat:
    rows: int
    cols: int
    type: int  # Assuming type follows OpenCV conventions
    data: str  # Base64-encoded image data

@dataclass
class CameraCalibration:
    resolution: Size
    cameraIntrinsics: JsonMatOfDoubles
    distCoeffs: JsonMatOfDoubles
    observations: list  # This would need to be fleshed out based on actual use
    calobjectWarp: list[float]
    calobjectSize: Size
    calobjectSpacing: float

def __convert_mrcal_to_photon_cameramodel(cameramodel_path: str) -> CameraCalibration:
    model = mrcal.cameramodel(cameramodel_path)

    # Extracting necessary data from mrcal model
    intrinsics = model.intrinsics()
    lensmodel, intrinsics_data = intrinsics
    dist_coeffs = intrinsics_data[4:]  # Assuming the first four are the core intrinsics
    resolution = model.imagersize()

    # Placeholder for generating/synthesizing observation data
    observations = []

    # Constructing the CameraCalibration object
    camera_calibration = CameraCalibration(
        resolution=Size(width=resolution[0], height=resolution[1]),
        cameraIntrinsics=JsonMatOfDoubles(rows=1, cols=4, type=6, data=intrinsics_data[:4]),
        distCoeffs=JsonMatOfDoubles(rows=1, cols=len(dist_coeffs), type=6, data=dist_coeffs),
        observations=observations,
        calobjectWarp=[],  # Placeholder, as warp is not directly available from mrcal
        calobjectSize=Size(width=0, height=0),  # Placeholder
        calobjectSpacing=0  # Placeholder
    )

    return camera_calibration

def __convert_mrcal_to_photon_cameramodel(cameramodel_path: str) -> CameraCalibration:
    model = mrcal.cameramodel(cameramodel_path)

    # Extracting necessary data from mrcal model
    intrinsics = model.intrinsics()
    lensmodel, intrinsics_data = intrinsics
    dist_coeffs = intrinsics_data[4:].tolist()  # Convert to list for JSON serialization
    resolution = model.imagersize()

    # Placeholder for generating/synthesizing observation data
    observations = []

    intrinsics_list = intrinsics_data[:4].tolist()
    camera_intrinsics = [intrinsics_list[0], 0, intrinsics_list[2], 0, intrinsics_list[1], intrinsics_list[3], 0, 0, 1]

    # Constructing the CameraCalibration object, ensuring all data is in a serializable form
    camera_calibration = CameraCalibration(
        resolution=Size(width=int(resolution[0]), height=int(resolution[1])),  # Convert to int
        cameraIntrinsics=JsonMatOfDoubles(rows=3, cols=3, type=6, data=camera_intrinsics),
        distCoeffs=JsonMatOfDoubles(rows=1, cols=len(dist_coeffs), type=6, data=dist_coeffs),
        observations=observations,
        calobjectWarp=[],  # Placeholder, as warp is not directly available from mrcal
        calobjectSize=Size(width=0, height=0),  # Placeholder
        calobjectSpacing=0  # Placeholder
    )

    return camera_calibration


def convert_mrcal_to_photon(mrcal_model_path: str, output_json_path: str):
    camera_cal_data = __convert_mrcal_to_photon_cameramodel(mrcal_model_path)

    # Writing the calibration data to a JSON file
    with open(output_json_path, "w") as outfile:
        json.dump(asdict(camera_cal_data), outfile, indent=4)

def main():
    parser = argparse.ArgumentParser(description="Convert mrcal cameramodel to PhotonVision calibration JSON")
    parser.add_argument("input", type=str, help="Path to mrcal cameramodel file")
    parser.add_argument("output", type=str, help="Output path for the PhotonVision calibration JSON")

    args = parser.parse_args()

    convert_mrcal_to_photon(args.input, args.output)

if __name__ == "__main__":
    main()
