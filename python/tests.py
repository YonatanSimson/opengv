import pyopengv
import numpy as np


def normalized(x):
    return x / np.linalg.norm(x)


def generateRandomPoint(maximumDepth, minimumDepth):
    cleanPoint = np.random.uniform(-1.0, 1.0, 3)
    direction = normalized(cleanPoint)
    return (maximumDepth - minimumDepth) * cleanPoint + minimumDepth * direction


def generateRandomTranslation(maximumParallax):
    return np.random.uniform(-maximumParallax, maximumParallax, 3)


def generateRandomRotation(maxAngle):
    rpy = np.random.uniform(-maxAngle, maxAngle, 3)

    R1 = np.array([[1.0,  0.0,  0.0],
                   [0.0,  np.cos(rpy[0]), -np.sin(rpy[0])],
                   [0.0,  np.sin(rpy[0]),  np.cos(rpy[0])]])

    R2 = np.array([[np.cos(rpy[1]),  0.0,  np.sin(rpy[1])],
                   [0.0,  1.0,  0.0],
                   [-np.sin(rpy[1]),  0.0,  np.cos(rpy[1])]])

    R3 = np.array([[np.cos(rpy[2]), -np.sin(rpy[2]),  0.0],
                   [np.sin(rpy[2]),  np.cos(rpy[2]),  0.0],
                   [0.0,  0.0,  1.0]])

    return R3.dot(R2.dot(R1))


def addNoise(noiseLevel, cleanPoint):
    noisyPoint = cleanPoint + np.random.uniform(-noiseLevel, noiseLevel, 3)
    return normalized(noisyPoint)


def extractRelativePose(position1, position2, rotation1, rotation2):
    relativeRotation = rotation1.T.dot(rotation2)
    relativePosition = rotation1.T.dot(position2 - position1)
    return relativePosition, relativeRotation


def essentialMatrix(position, rotation):
    # E transforms vectors from vp 2 to 1: x_1^T * E * x_2 = 0
    # and E = (t)_skew*R
    t_skew = np.zeros((3, 3))
    t_skew[0, 1] = -position[2]
    t_skew[0, 2] = position[1]
    t_skew[1, 0] = position[2]
    t_skew[1, 2] = -position[0]
    t_skew[2, 0] = -position[1]
    t_skew[2, 1] = position[0]

    E = t_skew.dot(rotation)
    return normalized(E)


def getPerturbedPose(position, rotation, amplitude):
    dp = generateRandomTranslation(amplitude)
    dR = generateRandomRotation(amplitude)
    return position + dp, rotation.dot(dR)


def proportional(x, y, tol=1e-2):
    xn = normalized(x)
    yn = normalized(y)
    return (np.allclose(xn, yn, rtol=1e20, atol=tol) or
            np.allclose(xn, -yn, rtol=1e20, atol=tol))


def matrix_in_list(a, l):
    for b in l:
        if proportional(a, b):
            return True
    return False


def same_transformation(position, rotation, transformation):
    R = transformation[:, :3]
    t = transformation[:, 3]
    return proportional(position, t) and proportional(rotation, R)


class RelativePoseDataset:

    def __init__(self, num_points, noise, outlier_fraction, rotation_only=False):
        # generate a random pose for viewpoint 1
        position1 = np.zeros(3)
        rotation1 = np.eye(3)

        # generate a random pose for viewpoint 2
        if rotation_only:
            position2 = np.zeros(3)
        else:
            position2 = generateRandomTranslation(2.0)
        rotation2 = generateRandomRotation(0.5)

        # derive correspondences based on random point-cloud
        self.generateCorrespondences(
            position1, rotation1, position2, rotation2,
            num_points, noise, outlier_fraction)

        # Extract the relative pose
        self.position, self.rotation = extractRelativePose(
            position1, position2, rotation1, rotation2)
        if not rotation_only:
            self.essential = essentialMatrix(self.position, self.rotation)

    def generateCorrespondences(self,
                                position1, rotation1,
                                position2, rotation2,
                                num_points,
                                noise, outlier_fraction):
        min_depth = 4
        max_depth = 8

        # initialize point-cloud
        self.points = np.empty((num_points, 3))
        for i in range(num_points):
            self.points[i] = generateRandomPoint(max_depth, min_depth)

        self.bearing_vectors1 = np.empty((num_points, 3))
        self.bearing_vectors2 = np.empty((num_points, 3))
        for i in range(num_points):
            # get the point in viewpoint 1
            body_point1 = rotation1.T.dot(self.points[i] - position1)

            # get the point in viewpoint 2
            body_point2 = rotation2.T.dot(self.points[i] - position2)

            self.bearing_vectors1[i] = normalized(body_point1)
            self.bearing_vectors2[i] = normalized(body_point2)

            # add noise
            if noise > 0.0:
                self.bearing_vectors1[i] = addNoise(noise, self.bearing_vectors1[i])
                self.bearing_vectors2[i] = addNoise(noise, self.bearing_vectors2[i])

        # add outliers
        num_outliers = int(outlier_fraction * num_points)
        for i in range(num_outliers):
            # create random point
            p = generateRandomPoint(max_depth, min_depth)

            # project this point into viewpoint 2
            body_point = rotation2.T.dot(p - position2)

            # normalize the bearing vector
            self.bearing_vectors2[i] = normalized(body_point)


def test_relative_pose():
    print("Testing relative pose")

    d = RelativePoseDataset(10, 0.0, 0.0)

    # running experiments
    twopt_translation = pyopengv.relative_pose_twopt(
        d.bearing_vectors1, d.bearing_vectors2, d.rotation)
    fivept_nister_essentials = pyopengv.relative_pose_fivept_nister(
        d.bearing_vectors1, d.bearing_vectors2)
    fivept_kneip_rotations = pyopengv.relative_pose_fivept_kneip(
        d.bearing_vectors1, d.bearing_vectors2)
    sevenpt_essentials = pyopengv.relative_pose_sevenpt(d.bearing_vectors1, d.bearing_vectors2)
    eightpt_essential = pyopengv.relative_pose_eightpt(d.bearing_vectors1, d.bearing_vectors2)
    t_perturbed, R_perturbed = getPerturbedPose(d.position, d.rotation, 0.01)
    eigensolver_rotation = pyopengv.relative_pose_eigensolver(
        d.bearing_vectors1, d.bearing_vectors2, R_perturbed)
    t_perturbed, R_perturbed = getPerturbedPose(d.position, d.rotation, 0.1)
    nonlinear_transformation = pyopengv.relative_pose_optimize_nonlinear(
        d.bearing_vectors1, d.bearing_vectors2, t_perturbed, R_perturbed)

    assert proportional(d.position, twopt_translation)
    assert matrix_in_list(d.essential, fivept_nister_essentials)
    assert matrix_in_list(d.rotation, fivept_kneip_rotations)
    assert matrix_in_list(d.essential, sevenpt_essentials)
    assert proportional(d.essential, eightpt_essential)
    assert proportional(d.rotation, eigensolver_rotation)
    assert same_transformation(d.position, d.rotation, nonlinear_transformation)

    print("Done testing relative pose")


def test_relative_pose_ransac():
    print("Testing relative pose ransac")

    d = RelativePoseDataset(100, 0.0, 0.3)

    ransac_transformation = pyopengv.relative_pose_ransac(
        d.bearing_vectors1, d.bearing_vectors2, "NISTER", 0.01, 1000)

    assert same_transformation(d.position, d.rotation, ransac_transformation)

    print("Done testing relative pose ransac")


def test_relative_pose_ransac_rotation_only():
    print("Testing relative pose ransac rotation only")

    d = RelativePoseDataset(100, 0.0, 0.3, rotation_only=True)

    ransac_rotation = pyopengv.relative_pose_ransac_rotation_only(
        d.bearing_vectors1, d.bearing_vectors2, 0.01, 1000)

    assert proportional(d.rotation, ransac_rotation)

    print("Done testing relative pose ransac rotation only")


def test_triangulation():
    print("Testing triangulation")

    d = RelativePoseDataset(10, 0.0, 0.0)

    points1 = pyopengv.triangulation_triangulate(
        d.bearing_vectors1, d.bearing_vectors2, d.position, d.rotation)

    assert np.allclose(d.points, points1)

    points2 = pyopengv.triangulation_triangulate2(
        d.bearing_vectors1, d.bearing_vectors2, d.position, d.rotation)

    assert np.allclose(d.points, points2)

    print("Done testing triangulation")


# =============================================================================
# ABSOLUTE POSE TESTS
# =============================================================================

class AbsolutePoseDataset:
    """Dataset for absolute pose estimation tests."""
    
    def __init__(self, num_points, noise, outlier_fraction):
        # Generate random camera pose
        self.position = generateRandomTranslation(2.0)
        self.rotation = generateRandomRotation(0.5)
        
        # Generate correspondences
        self.generateCorrespondences(num_points, noise, outlier_fraction)
    
    def generateCorrespondences(self, num_points, noise, outlier_fraction):
        min_depth = 4.0
        max_depth = 8.0
        
        # Generate random 3D points in world frame
        self.points = np.empty((num_points, 3))
        for i in range(num_points):
            self.points[i] = generateRandomPoint(max_depth, min_depth)
        
        # Project points to bearing vectors
        self.bearing_vectors = np.empty((num_points, 3))
        for i in range(num_points):
            # Transform point to camera frame
            body_point = self.rotation.T.dot(self.points[i] - self.position)
            
            # Normalize to get bearing vector
            self.bearing_vectors[i] = normalized(body_point)
            
            # Add noise
            if noise > 0.0:
                self.bearing_vectors[i] = addNoise(noise, self.bearing_vectors[i])
        
        # Add outliers (replace first N correspondences with random ones)
        num_outliers = int(outlier_fraction * num_points)
        for i in range(num_outliers):
            # Random bearing vector (not corresponding to actual point)
            random_dir = np.random.uniform(-1.0, 1.0, 3)
            self.bearing_vectors[i] = normalized(random_dir)


def transformation_close(position, rotation, transformation, pos_tol=0.1, rot_tol=0.1):
    """Check if transformation is close to ground truth."""
    R = transformation[:, :3]
    t = transformation[:, 3]
    
    pos_error = np.linalg.norm(t - position)
    
    # Rotation error using Rodrigues vector norm
    R_rel = R.T.dot(rotation)
    # Convert to angle-axis and get angle
    trace = np.trace(R_rel)
    trace = np.clip(trace, -1.0, 3.0)  # Numerical stability
    angle = np.arccos((trace - 1.0) / 2.0)
    
    return pos_error < pos_tol and angle < rot_tol


def test_absolute_pose():
    """Test absolute pose solvers: p3p_kneip, p3p_gao, epnp, sqpnp"""
    print("Testing absolute pose")
    
    d = AbsolutePoseDataset(20, 0.0, 0.0)
    
    # Test P3P Kneip (returns list of transformations)
    p3p_kneip_results = pyopengv.absolute_pose_p3p_kneip(
        d.bearing_vectors, d.points)
    found_kneip = False
    for T in p3p_kneip_results:
        if transformation_close(d.position, d.rotation, T):
            found_kneip = True
            break
    assert found_kneip, "P3P Kneip failed to find correct pose"
    
    # Test P3P Gao (returns list of transformations)
    p3p_gao_results = pyopengv.absolute_pose_p3p_gao(
        d.bearing_vectors, d.points)
    found_gao = False
    for T in p3p_gao_results:
        if transformation_close(d.position, d.rotation, T):
            found_gao = True
            break
    assert found_gao, "P3P Gao failed to find correct pose"
    
    # Test EPNP (returns single transformation)
    epnp_result = pyopengv.absolute_pose_epnp(d.bearing_vectors, d.points)
    assert transformation_close(d.position, d.rotation, epnp_result), \
        "EPNP failed to find correct pose"
    
    # Test SQPNP (returns single transformation)
    sqpnp_result = pyopengv.absolute_pose_sqpnp(d.bearing_vectors, d.points)
    assert transformation_close(d.position, d.rotation, sqpnp_result), \
        "SQPNP failed to find correct pose"
    
    print("Done testing absolute pose")


def test_absolute_pose_ransac():
    """Test absolute pose RANSAC with different algorithms."""
    print("Testing absolute pose RANSAC")
    
    # Dataset with 30% outliers
    d = AbsolutePoseDataset(100, 0.0, 0.3)
    
    threshold = 1.0 - np.cos(np.arctan(np.sqrt(2.0) * 0.5 / 800.0))
    
    # Test RANSAC with KNEIP (P3P)
    ransac_kneip = pyopengv.absolute_pose_ransac(
        d.bearing_vectors, d.points, "KNEIP", threshold, 1000)
    assert transformation_close(d.position, d.rotation, ransac_kneip, 0.2, 0.2), \
        "RANSAC KNEIP failed"
    print("  RANSAC KNEIP: OK")
    
    # Test RANSAC with EPNP
    ransac_epnp = pyopengv.absolute_pose_ransac(
        d.bearing_vectors, d.points, "EPNP", threshold, 1000)
    assert transformation_close(d.position, d.rotation, ransac_epnp, 0.2, 0.2), \
        "RANSAC EPNP failed"
    print("  RANSAC EPNP: OK")
    
    # Test RANSAC with SQPNP
    ransac_sqpnp = pyopengv.absolute_pose_ransac(
        d.bearing_vectors, d.points, "SQPNP", threshold, 1000)
    assert transformation_close(d.position, d.rotation, ransac_sqpnp, 0.2, 0.2), \
        "RANSAC SQPNP failed"
    print("  RANSAC SQPNP: OK")
    
    print("Done testing absolute pose RANSAC")


def test_absolute_pose_lmeds():
    """Test absolute pose LMedS with SQPNP."""
    print("Testing absolute pose LMedS")
    
    # Dataset with 30% outliers
    d = AbsolutePoseDataset(100, 0.0, 0.3)
    
    threshold = 1.0 - np.cos(np.arctan(np.sqrt(2.0) * 0.5 / 800.0))
    
    # Test LMedS with KNEIP
    lmeds_kneip = pyopengv.absolute_pose_lmeds(
        d.bearing_vectors, d.points, "KNEIP", threshold, 1000)
    assert transformation_close(d.position, d.rotation, lmeds_kneip, 0.2, 0.2), \
        "LMedS KNEIP failed"
    print("  LMedS KNEIP: OK")
    
    # Test LMedS with SQPNP
    lmeds_sqpnp = pyopengv.absolute_pose_lmeds(
        d.bearing_vectors, d.points, "SQPNP", threshold, 1000)
    assert transformation_close(d.position, d.rotation, lmeds_sqpnp, 0.2, 0.2), \
        "LMedS SQPNP failed"
    print("  LMedS SQPNP: OK")
    
    print("Done testing absolute pose LMedS")


def test_absolute_pose_comparison():
    """Compare accuracy of different absolute pose methods."""
    print("Testing absolute pose accuracy comparison")
    
    # Clean data for accuracy comparison
    d = AbsolutePoseDataset(50, 0.0, 0.0)
    
    # Get results from each method
    epnp_result = pyopengv.absolute_pose_epnp(d.bearing_vectors, d.points)
    sqpnp_result = pyopengv.absolute_pose_sqpnp(d.bearing_vectors, d.points)
    
    # Calculate errors
    def calc_errors(T, position, rotation):
        pos_error = np.linalg.norm(T[:, 3] - position)
        R_rel = T[:, :3].T.dot(rotation)
        trace = np.clip(np.trace(R_rel), -1.0, 3.0)
        rot_error = np.arccos((trace - 1.0) / 2.0)
        return pos_error, rot_error
    
    epnp_pos_err, epnp_rot_err = calc_errors(epnp_result, d.position, d.rotation)
    sqpnp_pos_err, sqpnp_rot_err = calc_errors(sqpnp_result, d.position, d.rotation)
    
    print(f"  EPNP:  pos_err={epnp_pos_err:.2e}, rot_err={np.degrees(epnp_rot_err):.4f}°")
    print(f"  SQPNP: pos_err={sqpnp_pos_err:.2e}, rot_err={np.degrees(sqpnp_rot_err):.4f}°")
    
    # Both should achieve very low error with clean data
    assert epnp_pos_err < 1e-6, "EPNP position error too large"
    assert sqpnp_pos_err < 1e-6, "SQPNP position error too large"
    
    print("Done testing absolute pose accuracy comparison")


if __name__ == "__main__":
    # Relative pose tests
    test_relative_pose()
    test_relative_pose_ransac()
    test_relative_pose_ransac_rotation_only()
    test_triangulation()
    
    # Absolute pose tests
    test_absolute_pose()
    test_absolute_pose_ransac()
    test_absolute_pose_lmeds()
    test_absolute_pose_comparison()
