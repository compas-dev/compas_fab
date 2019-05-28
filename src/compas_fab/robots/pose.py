from __future__ import print_function

import compas
from compas.geometry import Transformation
from compas.geometry.transformations import matrix_from_quaternion
from compas.geometry.transformations import matrix_from_euler_angles
from compas.geometry.transformations import matrix_from_basis_vectors
from compas.geometry.transformations import basis_vectors_from_matrix
from compas.geometry.transformations import quaternion_from_matrix
from compas.geometry.basic import add_vectors
from compas.geometry.basic import multiply_matrices

import math as m
import copy
import numpy as np


class Pose(compas.geometry.Frame):

	"""
	A pose is defined by a point (origin) and a quaternion (orientation).

	Parameters
	----------
	point : point
		The origin of the pose.
	quaternion : list of 4 floats
		The quaternion describing the orientation of the pose.
	convention: string
		Convention of the input quarternion
	"""

	def __init__(self, point, quaternion, convention='wxyz'):
	   
		#transform the input quarternion into "wxyz" form if needed
		if convention == 'wxyz':
			self._quaternion = quaternion
		elif convention == 'xyzw':
			self._quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
		
		# unitized orientation quaternion
		self._quaternion = self.unitized_quaternion

		R = matrix_from_quaternion(self._quaternion)
		xaxis, yaxis = basis_vectors_from_matrix(R)

		super(Pose, self).__init__(point, xaxis, yaxis)

		self._quaternion = self.get_quaternion()


	# ==========================================================================
	# factory
	# ==========================================================================

	@classmethod
	def from_list(cls, values, convention='wxyz'):
		"""Construct a pose from a list of 7 :obj:`float` values.

		Parameters
		----------
		values : :obj:`list` of :obj:`float`
			The list of 7 values representing a pose.

		Returns
		-------
		Pose
			The constructed pose.
		"""
		point = values[0:3]
		quaternion = values[3:7]
		return cls(point, quaternion, convention=convention)
	
	@classmethod
	def from_EGM_data(cls, data):
		"""Construct a pose from its data representation.

		Parameters
		----------
		data : :obj:`dict`
			The data dictionary.

		Returns
		-------
		Pose
			The constructed pose.
		
		Note:
			Mainly used from creating Pose from EGM data, where quaternion is described as q0, q1, q2, q3 = w, x, y, z
		"""

		point = [data['x'], data['y'], data['z']]
		quaternion = [data['q0'], data['q1'], data['q2'], data['q3']]
		return cls(point, quaternion, convention='wxyz')

	@classmethod
	def from_euler_angles(cls, euler_angles, static=True, axes='xyz', point=[0, 0, 0]):
		"""Construct a pose from a rotation represented by Euler angles.

		Parameters
		----------
		euler_angles : :obj:`list` of :obj:`float`
			Three numbers that represent the angles of rotations about the defined axes.
		static : :obj:`bool`, optional
			If true the rotations are applied to a static frame.
			If not, to a rotational.
			Defaults to true.
		axes : :obj:`str`, optional
			A 3 character string specifying the order of the axes.
			Defaults to 'xyz'.
		point : :obj:`list` of :obj:`float`, optional
			The point of the frame.
			Defaults to [0, 0, 0].

		Returns
		-------
		Pose
			The constructed pose.
		"""
		R = matrix_from_euler_angles(euler_angles, static, axes)
		quaternion = quaternion_from_matrix(R)
		return cls(point, quaternion, convention='wxyz')

	# ==========================================================================
	# descriptors
	# ==========================================================================

	def get_quaternion(self, convention='wxyz', canonic=True):
		"""
		convention: format of the output quarternion
		canonic: W component of the questernion always positive
		"""
		if canonic:
			canonic_quat = self.canonic_quaternion
			if convention == 'wxyz':
				return canonic_quat
			elif convention == 'xyzw':
				return [canonic_quat[1], canonic_quat[2], canonic_quat[3], canonic_quat[0]]
		else:
			if convention == 'wxyz':
				return self._quaternion
			elif convention == 'xyzw':
				return [self._quaternion[1], self._quaternion[2], self._quaternion[3], self._quaternion[0]]

	@property
	def data(self):
		"""Returns the data dictionary that represents the pose.

		Returns
		-------
		dict
			The pose data.
		"""
		return {'point': list(self.point),
				'quaternion': list(self._quaternion)}

	@data.setter
	def data(self, data):
		self.point = data['point']
		self._quaternion = data['quaternion']

	def to_data(self):
		"""Returns the data dictionary that represents the pose.

		Returns
		-------
		dict
			The pose data.
		"""
		return self.data
	
	def to_list(self, convention='wxyz'):
		"""Returns the list that represents the pose.

		Returns
		-------
		list
			The pose list.
		"""
		return [self.point[0], self.point[1], self.point[2]] + list(self.get_quaternion(convention=convention))
	
	def to_lists(self, convention='wxyz', canonic=True):
		"""Returns the 2 lists that represents the pose.

		Returns
		-------
		lists
			The origin point list.
			The quaternion list (canonic or not canonic).
		"""
		return list(self.point), list(self.get_quaternion(convention=convention, canonic=canonic))
	

	# ==========================================================================
	# representation
	# ==========================================================================

	def __repr__(self):
		return "Pose({0}, Quaternion{1})".format(self.point, self._quaternion)

	# ==========================================================================
	# helpers
	# ==========================================================================

	def copy(self):
		"""Make a copy of this ``Pose``.

		Returns
		-------
		Pose
			The copy.
		"""
		cls = type(self)
		return cls(self.point.copy(), copy.copy(self._quaternion), convention='wxyz')

	# ==========================================================================
	# methods
	# ==========================================================================

	def quaternion_sum_of_squares(self, quaternion):
		return np.dot(quaternion, quaternion)
	
	def is_quaternion_unit(self, tolerance=1e-14):
		"""Determine whether the quaternion is of unit length to within a specified tolerance value.
		
		Params:
			tolerance: [optional] maximum absolute value by which the norm can differ from 1.0 for the object to be considered a unit quaternion. Defaults to `1e-14`.
		
		Returns:
			`True` if the Quaternion object is of unit length to within the specified tolerance value. `False` otherwise.
		Note:
			if _sum_of_squares is 1, norm is 1. This saves a call to sqrt()
		"""
		return abs(1.0 - self.quaternion_sum_of_squares(self._quaternion)) < tolerance

	@property 
	def quaternion_norm(self):
		q = self._quaternion
		return m.sqrt(np.dot(q,q))

	@property
	def unitized_quaternion(self, tolerance=1e-14):
		n = self.quaternion_norm
		if abs(n-1.0)>tolerance:
			q = self._quaternion
			self._quaternion = [i/n for i in q]
		return self._quaternion

	@property
	def canonic_quaternion(self):
		if (self._quaternion[0] < 0):
			return [v * -1.0 for v in self._quaternion]
		else:
			return self._quaternion
	
	def pose_noise(self, noise):
		#TODO: add pose noise function
		raise NotImplementedError

	def multiply_by_quaternion(self, other_quaternion):
		#NOTE: to be reviewed!

		w1, x1, y1, z1 = self._quaternion
		w2, x2, y2, z2 = other_quaternion
		w = w1*w2 - x1*x2 - y1*y2 - z1*z2
		x = w1*x2 + x1*w2 + y1*z2 - z1*y2
		y = w1*y2 + y1*w2 + z1*x2 - x1*z2
		z = w1*z2 + z1*w2 + x1*y2 - y1*x2
		return [w, x, y, z]

	
	# ==========================================================================
	# transformations
	# ==========================================================================

	def transform(self, transformation):
		"""Trasform the pose.

		Parameters
		----------
		transformation : :class:`Transformation`
			The transformation used to transform the Pose.
		"""
		T = transformation * Transformation.from_frame(self)
		point = T.translation
		xaxis, yaxis = T.basis_vectors
		self.point = point
		self.xaxis = xaxis
		self.yaxis = yaxis
		self._quaternion = self.quaternion
		self._quaternion = self.get_quaternion()

	def transformed(self, transformation):
		"""Returns a transformed copy of the current pose.

		Parameters
		----------
		transformation : :class:`Transformation`
			The transformation used to transform the Pose.

		Returns
		-------
		:class:`Pose`
			The transformed pose.
		"""
		pose = self.copy()
		pose.transform(transformation)
		return pose

	def translated(self, vector):
		"""
		Returns a copy of the current pose translated along a vector.
		"""
		pose = self.copy()
		pose.point = add_vectors(pose.point, vector)
		return pose


	def add_deltas(self, linear_translation, angular_rotation):
		"""
		Returns a copy of the current pose translated along a vector and rotated by delta rotations
		"""
		new_origin = add_vectors(self.point, linear_translation)
		
		current_orientation_rotation_matrix = matrix_from_quaternion(self._quaternion)
		delta_orientation_rotation_matrix = matrix_from_euler_angles(reversed(angular_rotation),
																	 static=False,
																	 axes='zyx')
		new_orientation_rotation_matrix = multiply_matrices(delta_orientation_rotation_matrix, current_orientation_rotation_matrix)
		new_orientation_quaternion = quaternion_from_matrix(new_orientation_rotation_matrix)

		cls = type(self)
		return cls(new_origin, new_orientation_quaternion, convention='wxyz')


	@property
	def ABB_euler(self):
		# Returns ABB (ZYX)-Euler angles from quaternion orientation
		
		q0_, q1_, q2_, q3_ = self._quaternion

		# rotation matrix
		r11 = 1 - 2*q2_*q2_ - 2*q3_*q3_
		r12 = 2*q1_*q2_ - 2*q3_*q0_
		r13 = 2*q1_*q3_ + 2*q2_*q0_
		r21 = 2*q1_*q2_ + 2*q3_*q0_
		r22 = 1 - 2*q1_*q1_ -2*q3_*q3_
		r23 = 2*q2_*q3_ - 2*q1_*q0_
		r31 = 2*q1_*q3_ - 2*q2_*q0_
		r32 = 2*q2_*q3_ + 2*q1_*q0_
		r33 = 1 - 2*q1_*q1_ -2*q2_*q2_


		# (ZYX)-Euler angles for ABB
		if (r11 == 0):
			rx = m.atan2(r12, r22)
			if (r31 == 1):
				ry = -m.pi/2
			else:
				ry = m.pi/2
			rz = 0
		else:
			rx = m.atan2(r32, r33)
			ry = m.asin(-r31)
			rz = m.atan2(r21, r11)

		return [rx, ry, rz]


	def ABB_euler_to_quaternion(self, euler_angles):
		# Converts ABB (ZYX)-Euler angles to quaternion

		rx, ry, rz = euler_angles

		# Check range of Euler angles
		if (abs(rx) > m.pi or abs(ry) > m.pi / 2 or abs(rz) > m.pi):
			print("ERROR: Angle out of range in conversion from Abb Euler angles to rotation matrix!")


		# Rotation matrix from Abb (ZYX)-Euler
		sx = m.sin(rx)
		sy = m.sin(ry)
		sz = m.sin(rz)

		cx = m.cos(rx)
		cy = m.cos(ry)
		cz = m.cos(rz)

		r11 = cz * cy
		r12 = cz * sy * sx - sz * cx
		r13 = cz * sy * cx + sz * sx
		r21 = sz * cy
		r22 = sz * sy * sx + cz * cx
		r23 = sz * sy * cx - cz * sx
		r31 = -sy
		r32 = cy * sx
		r33 = cy * cx

		rotation_matrix = [
			[r11, r12, r13, 0],
			[r21, r22, r23, 0],
			[r31, r32, r33, 0],
			[0,   0,   0,   1]]
		
		return quaternion_from_matrix(rotation_matrix)