import numpy as np
import cv2 as cv

def inverse_proj(arr_inv, K_inv, pix_coord):
    """
    Inverse projection from image coordinates to world coordinates.
    """
    eps = 1e-24
    img_coord = pix_coord
    world_coord = arr_inv @ K_inv @ img_coord
    world_coord[:2, :] = world_coord[:2, :] / (world_coord[2, :] + eps)
    world_coord[2, :] = 0
    return world_coord

def direct_proj(arr, K, p1):
    """
    Direct projection from world coordinates to image coordinates.
    """
    eps = 1e-24

    img_points =  K @ arr @ p1 #the order is key here, first the homography then the intrinsic matrix!
    img_points[:2, :] = img_points[:2, :] / (img_points[2, :] + eps)
    img_points[2, :] = 1 # Set z to 1 for image coordinates

    return img_points

class InverseProjector:
    def __init__(self, p):
        self.pitch = p 
        self.roll = 0.0
        self.yaw = 1.57  

        # extrinsic position of camera [m] 
        self.x = -0.28119
        self.y = 0.2016
        self.z = 1.2         #modify for checking closest point in visible FOV. Real is 1.06787

        # intrinsic parameters of camera [px]
        self.fx = 901.1387623297165
        self.fy = 902.8926336853814
        self.u0 = 645.2704047048309
        self.v0 = 366.13035318739895

        # Intrinsic parameter matrix
        self.K = np.array([[self.fx, 0, self.u0],
                           [0, self.fy, self.v0],
                           [0,  0,  1]])

        # Compute rotation and translation matrices
        self.R_c2w = self.rotation_matrix()
        self.T_c2w = np.array([[self.x, self.y, self.z]]).T

        self.R_w2c = self.R_c2w.T
        self.T_w2c = -(self.R_w2c @ self.T_c2w) 

        self.R_cam2img = np.array([[0, -1, 0],
                                   [0, 0, -1],
                                   [1, 0,  0]])

        self.arr = np.zeros((3,3))
        self.R_w2i = self.R_cam2img @ self.R_w2c
        self.T_w2i = (self.R_cam2img @ self.T_w2c) # why didn't we do negative here like T_w2c calculation above?

        self.arr[:, 0] = self.R_w2i[:,0] #rx
        self.arr[:, 1] = self.R_w2i[:,1] #ry
        self.arr[:, 2:3] = self.T_w2i
        # apparently this is a homography matrix

        self.arr_inv = np.linalg.inv(self.arr) 
        self.K_inv = np.linalg.inv(self.K)


    def rotation_matrix(self):
        roll, pitch, yaw = self.roll, self.pitch, self.yaw
        si, sj, sk = np.sin(roll), np.sin(pitch), np.sin(yaw)
        ci, cj, ck = np.cos(roll), np.cos(pitch), np.cos(yaw)
        cc, cs = ci * ck, ci * sk
        sc, ss = si * ck, si * sk

        R = np.identity(3)
        R[0, 0] = cj * ck
        R[0, 1] = sj * sc - cs
        R[0, 2] = sj * cc + ss
        R[1, 0] = cj * sk
        R[1, 1] = sj * ss + cc
        R[1, 2] = sj * cs - sc
        R[2, 0] = -sj
        R[2, 1] = cj * si
        R[2, 2] = cj * ci
        return R


def main():
    original_pitch = np.deg2rad(7) # 7 deg
    error_pitch = np.deg2rad(8) 

    print(f"Original pitch: {np.rad2deg(original_pitch)} deg, Error pitch: {np.rad2deg(error_pitch)} deg\n")

    projector_error = InverseProjector(error_pitch) 
    projector_original = InverseProjector(original_pitch) 

    # Define a line in world ground plane, x, y and w = 1
    p1 = np.array([[0, 3.5, 1.0]]).T
    p2 = np.array([[0, 9, 1.0]]).T
    euclidean_distance = np.linalg.norm(p1 - p2)

    print("World coordinates of line points:")
    print("Point One:", p1.flatten())
    print("Point Two:", p2.flatten())    
    print("Expected euclidean distance of line:", euclidean_distance, "m\n")
    
    # Direct projection of p1 and p2 to find pixel coordinates using the error pitch
    pixel_one = direct_proj(projector_error.arr, projector_error.K, p1)
    pixel_two = direct_proj(projector_error.arr, projector_error.K, p2)

    print("Direct projection of world coordinates to pixel coordinates using error pitch (make sure within image dimensions - 1280x720):")
    print("Pixel One:", pixel_one.flatten())
    print("Pixel Two:", pixel_two.flatten(), "\n")

    # Inverse projection of pixel coordinates back to world coordinates using the original pitch
    world_one = inverse_proj(projector_original.arr_inv, projector_original.K_inv, pixel_one)
    world_two = inverse_proj(projector_original.arr_inv, projector_original.K_inv, pixel_two)
    euclidean_distance_error = np.linalg.norm(world_one - world_two)

    print("Inverse projection of image coordinates to pixel coordinates using original pitch:")
    print(f"pixel_one: {world_one.flatten()}")
    print(f"pixel_two: {world_two.flatten()}")
    print(f"Euclidean distance of line: {euclidean_distance_error:.4f} m\n")

    print("Euclidean distance error of", np.abs(euclidean_distance_error - euclidean_distance), "m at pitch error of", np.rad2deg(error_pitch) - np.rad2deg(original_pitch), "degrees")

    px1 = np.array([[640, 720, 1.0]]).T
    px1_world = inverse_proj(projector_original.arr_inv, projector_original.K_inv, px1)
    print("Inverse projection of pixel (640, 720) to world coordinates using original pitch:")
    print("World coordinates:", px1_world.flatten())


if __name__ == '__main__':
    main()