import cv2
import numpy as np
from app.mesh_processing import reconstruct_mesh, calculate_volume, scale_point_cloud
import open3d as o3d

def process_video(video_path):
    frames = extract_frames(video_path)
    if not frames:
        return {"error": "No frames extracted from the video. Please ensure the video is not corrupted or too short."}

    scale = estimate_scale(frames[0])  # Assume first frame contains visible coin

    if scale is None:
        return {"error": "Reference coin not detected. Please include it in the first frame."}

    point_cloud = generate_point_cloud(frames)
    if point_cloud.size == 0:
        return {"error": "Point cloud generation failed. Ensure the video has sufficient detail and motion."}

    scaled_pc = scale_point_cloud(point_cloud, scale)

    mesh = reconstruct_mesh(scaled_pc)
    if mesh is None:
        return {"error": "Mesh reconstruction failed."}

    volume = calculate_volume(mesh)

    # Save mesh to static directory
    o3d.io.write_triangle_mesh("static/mesh.ply", mesh)

    return {
        "volume": round(volume, 2),
        "mesh_info": f"Vertices: {len(mesh.vertices)}, Faces: {len(mesh.triangles)}"
    }

def extract_frames(video_path, max_frames=30):
    frames = []
    cap = cv2.VideoCapture(video_path)
    frame_count = 0

    while cap.isOpened() and frame_count < max_frames:
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
        frame_count += 1

    cap.release()
    return frames

def estimate_scale(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray, 7)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                param1=50, param2=30, minRadius=10, maxRadius=50)

    if circles is not None:
        coin_radius_px = circles[0][0][2]
        coin_diameter_mm = 26  # Standard coin diameter
        px_per_mm = (2 * coin_radius_px) / coin_diameter_mm
        return px_per_mm

    return None

def generate_point_cloud(frames):
    orb = cv2.ORB_create()
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    point_cloud = []

    for i in range(len(frames) - 1):
        kp1, des1 = orb.detectAndCompute(cv2.cvtColor(frames[i], cv2.COLOR_BGR2GRAY), None)
        kp2, des2 = orb.detectAndCompute(cv2.cvtColor(frames[i + 1], cv2.COLOR_BGR2GRAY), None)

        if des1 is None or des2 is None:
            continue

        matches = bf.match(des1, des2)
        for m in matches:
            pt1 = kp1[m.queryIdx].pt
            pt2 = kp2[m.trainIdx].pt
            x, y = pt1
            z = np.linalg.norm(np.array(pt1) - np.array(pt2))
            point_cloud.append([x, y, z])

    return np.array(point_cloud)
