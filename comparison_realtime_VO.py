import numpy as np 
import cv2
import OptFlow as OF

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 500

feature_params = dict(maxCorners=500,
                      qualityLevel=0.3,
                      minDistance=10,
                      blockSize=7)

lk_params = dict(winSize  = (21, 21), 
				maxLevel = 3,
				criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
	
def featureTracking(image_ref, image_cur, px_ref):
	kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

	st = st.reshape(st.shape[0])
	kp1 = px_ref[st == 1]
	kp2 = kp2[st == 1]
	# feature points in image coordinate 
	# kp1: prev_point, kp2: cur_point
	return kp1, kp2

class PinholeCamera:
	def __init__(self, width, height, fx, fy, cx, cy, 
				k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
		self.width = width
		self.height = height
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.distortion = (abs(k1) > 0.0000001)
		self.d = [k1, k2, p1, p2, k3]


class VisualOdometry:
	def __init__(self, cam):
		self.frame_stage = 0
		self.cam = cam
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.px_ref = None
		self.px_cur = None
		self.Scale = None
		self.skip_frame = False
		self.focal = cam.fx
		self.T_vectors = []
		self.R_matrices = []
		self.new_cloud = None
		self.last_cloud = None
		self.K = np.array([[cam.fx,0,cam.cx],[0,cam.fy,cam.cy],[0,0,1]])
		self.pp = (cam.cx, cam.cy)
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

		self.Rot = np.zeros((3,3))
		self.imu_scale = 0.

		self.rotation_radius = 0.055
		self.prev_R = None
	
	def detectNewFeatures(self, cur_img):
		"""
		Detects new features in the current frame.
        Uses the Feature Detector selected.
        """
        
		if self.detector == 'SHI-TOMASI':
			feature_pts = cv2.goodFeaturesToTrack(cur_img, **feature_params)
			feature_pts = np.array([x for x in feature_pts], dtype=np.float32).reshape((-1, 2))
		else:
			feature_pts = self.detector.detect(cur_img, None)
			feature_pts = np.array([x.pt for x in feature_pts], dtype=np.float32)

		return feature_pts
	
	def triangulatePoints(self, R, t):
		"""
		Triangulates the feature correspondence points with
        the camera intrinsic matrix, rotation matrix, and translation vector.
        It creates projection matrices for the triangulation process.
        """

        # The canonical matrix (set as the origin)
		P0 = np.array([[1, 0, 0, 0],
						[0, 1, 0, 0],
						[0, 0, 1, 0]])
		P0 = self.K.dot(P0)
        # Rotated and translated using P0 as the reference point
		P1 = np.hstack((R, t))
		P1 = self.K.dot(P1)
        # Reshaped the point correspondence arrays to cv2.triangulatePoints's format
		point1 = self.px_ref.reshape(2, -1)
		point2 = self.px_cur.reshape(2, -1)

		return cv2.triangulatePoints(P0, P1, point1, point2).reshape(-1, 4)[:, :3]

	def angle_between_rot(R1, R2):
		
		# Compute the relative rotation matrix
		R_rel = R1.T @ R2
		
		# Calculate the angle from the relative rotation matrix
		angle = abs(np.arccos((np.trace(R_rel) - 1) / 2))
		return angle
	
	def getRelativeScale(self):

		"""
		Returns the relative scale based on the 3-D point clouds
		produced by the triangulation_3D function. Using a pair of 3-D corresponding points
		the distance between them is calculated. This distance is then divided by the
		corresponding points' distance in another point cloud.
		"""

		min_idx = min([self.new_cloud.shape[0], self.last_cloud.shape[0]])
		ratios = []  # List to obtain all the ratios of the distances
		for i in range(min_idx):
			if i > 0:
				Xk = self.new_cloud[i]
				p_Xk = self.new_cloud[i - 1]
				Xk_1 = self.last_cloud[i]
				p_Xk_1 = self.last_cloud[i - 1]

				if np.linalg.norm(p_Xk - Xk) != 0:
					ratios.append(np.linalg.norm(p_Xk_1 - Xk_1) / np.linalg.norm(p_Xk - Xk))

		d_ratio = np.median(ratios) # Take the median of ratios list as the final ratio
		return d_ratio

	def processFirstFrame(self):
		self.px_ref = self.detector.detect(self.new_frame)
		self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
		self.frame_stage = STAGE_SECOND_FRAME

	def processSecondFrame(self):
		self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)

		if E is not None:
			E = E.copy()
			self.px_ref = self.px_ref.copy()
			self.px_cur = self.px_cur.copy()
		self.px_cur = np.ascontiguousarray(self.px_cur)
		self.px_ref = np.ascontiguousarray(self.px_ref)
		E = np.ascontiguousarray(E)
		_, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		#_, R, t, mask = cv2.recoverPose(E, self.px_cur.get(), self.px_ref.get(), focal=self.focal, pp=self.pp)
		self.T_vectors.append(tuple(self.cur_R.dot(self.cur_t)))
		self.R_matrices.append(tuple(self.cur_R))
		self.new_cloud = self.triangulatePoints(self.cur_R, self.cur_t)
		self.OFF_prev, self.OFF_cur = self.px_ref, self.px_cur
		self.frame_stage = STAGE_DEFAULT_FRAME 
		self.px_ref = self.px_cur
		self.last_cloud = self.new_cloud

	def frame_Skip(self, pixel_diff):
		"""
		Determines if the current frame needs to be skipped.
		A frame is skipped on the basis that the current feature points
		are almost identical to the previous feature points, meaning the image
		was probably taken from the same place and the translation should be zero.
		"""

        # We tried this parameter with 20, 15, 10, 5, 3, 2, 1 and 0
        # for one dataset and found that 3 produces the best results.
		return pixel_diff < 3

	def updatePrevRotation(self):
		# Store current rotation matrix as previous for next frame
		if self.cur_R is not None:
			self.prev_R = self.cur_R.copy()
	
	def processFrame(self, frame_id):
		prev_img, cur_img = self.last_frame, self.new_frame


		'''
		For now, undistortion is disabled
    	'''
    
		# prev_img = cv2.undistort(prev_img, self.K)
		# cur_img = cv2.undistort()

		self.px_ref, self.px_cur, px_diff = OF.KLT_featureTracking(prev_img, cur_img, self.px_ref)
		#self.px_ref, selef.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
		
		self.skip_frame = self.frame_Skip(px_diff)

		if self.skip_frame:
			if self.px_ref.shape[0] < kMinNumFeature:  # Verify if features on last_frame are sparse
				self.px_cur = self.detectNewFeatures(prev_img)
				self.px_ref = self.px_cur
				self.last_cloud = self.new_cloud
			return

		# Calculation of Essential Matrix
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)

		# Solve Essential Matrix
		_, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)

		self.new_cloud = self.triangulatePoints(R, t)

		# absolute_scale = self.getAbsoluteScale(frame_id)
		# if self.prev_R is not None and self.cur_R is not None:
		# 	angle_difference = self.angle_between_rot(self.cur_R, self.prev_R)
		# 	absolute_scale = 2 * self.rotation_radius * np.sin(angle_difference/2)
		# else:
		# 	angle_difference = 0.0
		# 	absolute_scale = 0.0
			
		self.Scale = self.getRelativeScale()
		
		if self.px_ref.shape[0] < kMinNumFeature:                     # Verify if the amount of feature points
			self.px_cur = self.detectNewFeatures(cur_img)  			  # is above the kMinNumFeature threshold

		self.OFF_prev, self.OFF_cur = self.px_ref, self.px_cur
		self.px_ref = self.px_cur
		self.last_cloud = self.new_cloud

		if(self.imu_scale > 0.01):									  # update if only displacement obtained from IMU exceeds 1cm
			print("VO updated")
			self.cur_t = self.cur_t + self.imu_scale * self.cur_R.dot(t) 
			self.cur_R = R.dot(self.cur_R)
			self.T_vectors.append(tuple(self.cur_t))
			self.R_matrices.append(tuple(self.cur_R))
		else:
			self.cur_R = self.Rot
			# self.cur_t = self.cur_t + absolute_scale * self.cur_R.dot(t)
			self.cur_t = self.cur_t

		if(self.px_ref.shape[0] < kMinNumFeature):
			self.px_cur = self.detector.detect(self.new_frame)
			self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
		self.px_ref = self.px_cur

	def update(self, img, frame_id, data, displacement = 0.):
		# Ensure image dimensions match the camera model and is grayscale
		assert(img.ndim == 2 and img.shape[0] == self.cam.height and img.shape[1] == self.cam.width), \
			"Frame: provided image has not the same size as the camera model or image is not grayscale"
		
		# Check if data (rotation matrix) is provided
		if data is not None:
			# Directly assign the 3x3 rotation matrix to self.Rot
			self.Rot = data
		else:
			# If data is None, retain the current rotation matrix
			self.Abs = False
			self.Rot = self.cur_R
		
		self.imu_scale = displacement

		# Update the current frame and region of interest (ROI)
		self.new_frame = img
		self.new_roi = img[int(img.shape[0] * 0.40):img.shape[0], 0:img.shape[1]]
		
		# Process frames based on the current frame stage
		if self.frame_stage == STAGE_DEFAULT_FRAME:
			self.processFrame(frame_id)
		elif self.frame_stage == STAGE_SECOND_FRAME:
			self.processSecondFrame()
		elif self.frame_stage == STAGE_FIRST_FRAME:
			self.processFirstFrame()
		
		# Update the last frame and frame ID
		self.last_frame = self.new_frame
		self.last_id = frame_id
		self.last_roi = self.new_roi

		# Skip frame handling if specified
		if self.skip_frame:
			return False

		return True