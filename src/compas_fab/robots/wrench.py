from __future__ import print_function

import copy
import numpy as np
from scipy import stats

import compas.geometry as cg

__all__ = ['Wrench']


class Wrench():

	"""
	A wrench is defined by a list of 6 float describing the 3 components of force and 3 components of torque

	Parameters
	----------
	wrench : list of 6 floats
		List of 6 axis forces [Fx,Fy,Fz] in Newtons. and moments [Tx,Ty,Tz] in Newton-meters
	bias : list of 6 floats
	mass: float [Kg]
	center_of_mass : list of 3 float [meters]
	"""

	def __init__(self,
				 wrench,
				 bias = None,
				 mass = None,
				 center_of_mass = None):
		
		# wrench
		self.wrench = wrench

		# bias
		self.bias = bias

		# mass
		if mass:
			self.mass = mass
		else:
			self.mass = 8.559916898585978
		
		# center of mass
		if center_of_mass:
			self.center_of_mass = cg.Vector(center_of_mass)
		else:
			self.center_of_mass = cg.Vector(-0.03397799730572294, 0.042647988599212705, 0.2900206244556921)

		# gravity
		self.gravity = 9.80665	# [m/s**2]
		self.gravity_vector_WCS = cg.Vector(0.0, 0.0, -self.gravity)
	
	# ==========================================================================
	# factory
	# ==========================================================================

	@classmethod
	def from_msg(cls, msg, bias = None, mass = None, center_of_mass = None):
		"""Construct a wrench from its ROS message representation.
		http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/WrenchStamped.html

		Parameters
		----------
		data : :obj:`dict`
			The data dictionary.

		Returns
		-------
		Wrench
			The constructed wrench.
		
		Note:
			Used from creating Wrench from ROS message
		"""
		wrench = [
			msg["wrench"]["force"]["x"],
			msg["wrench"]["force"]["y"],
			msg["wrench"]["force"]["z"],
			msg["wrench"]["torque"]["x"],
			msg["wrench"]["torque"]["y"],
			msg["wrench"]["torque"]["z"]
		]
		return cls(wrench, bias = bias, mass = mass, center_of_mass = center_of_mass)
	
	@classmethod
	def by_samples(cls, wrenches, bias = None, mass = None, center_of_mass = None, proportiontocut = 0.0):
		"""
		Construct the compensator by sampled data, allowing to filter lists of wrenches
		"""
		wrench = stats.trim_mean(wrenches, proportiontocut, axis=0).tolist()
		return cls(wrench, bias = bias, mass = mass, center_of_mass = center_of_mass)
	
	# ==========================================================================
	# descriptors
	# ==========================================================================

	@property
	def data(self):
		"""Returns the data dictionary that represents the wrench.

		Returns
		-------
		dict
			The wrench data.

		"""
		return {'wrench': {
			'force': {
				'x': self.wrench[0],
				'y': self.wrench[1],
				'z': self.wrench[2],
			},
			'torque': {
				'x': self.wrench[3],
				'y': self.wrench[4],
				'z': self.wrench[5]}}}

	def to_data(self):
		"""Returns the data dictionary that represents the wrench.

		Returns
		-------
		dict
			The wrench data.
		"""
		return self.data
	
	def to_list(self):
		"""Returns the list that represents the wrench.

		Returns
		-------
		list
			The wrench list.
		"""
		return self.wrench
	
	def to_lists(self):
		"""Returns the 2 lists that represents the forces and torques.

		Returns
		-------
		lists
			The forces list.
			The torques list.
		"""
		return list(self.wrench[0:3]), list(self.wrench[3:6])

	# ==========================================================================
	# representation
	# ==========================================================================

	def __repr__(self):
		return '''Wrench(Fx {0:.2f}, Fy {1:.2f}, Fz {2:.2f}, Tx {3:.2f}, Ty {4:.2f}, Tz {5:.2f})'''.format(*self.wrench)

	# ==========================================================================
	# helpers
	# ==========================================================================

	def copy(self):
		"""Make a copy of this ``Wrench``.

		Returns
		-------
		Wrench
			The copy.
		"""
		cls = type(self)
		return cls(copy.copy(self.wrench))

	# ==========================================================================
    # methods
    # ==========================================================================
	
	@staticmethod
	def binary(wrench, forces_threshold = 10.0, torques_threshold = 5.0):
		"""
		Discretize the wrench
		"""
		binary_ft = [0,0,0,0,0,0]

		#forces
		for idx, w in enumerate(wrench[0:3]):
			if (w > forces_threshold): binary_ft[idx] = 1
			elif (w < -forces_threshold): binary_ft[idx] = -1
			else: binary_ft[idx] = 0
		#torques
		for idx, w in enumerate(wrench[3:6]):
			if (w > torques_threshold): binary_ft[idx+3] = 1
			elif (w < -torques_threshold): binary_ft[idx+3] = -1
			else: binary_ft[idx+3] = 0
		
		return Wrench(binary_ft)
	
	# ==========================================================================
	# operators
	# ==========================================================================

	def __mul__(self, n):
		"""Create a ``Wrench`` multiplied by the given factor.

		Parameters
		----------
		n : float
			The multiplication factor.

		Returns
		-------
		Wrench
			The resulting new ``Wrench``.

		"""
		return Wrench([
			n * self.wrench[0],
			n * self.wrench[1],
			n * self.wrench[2],
			n * self.wrench[3],
			n * self.wrench[4],
			n * self.wrench[5] ])
	
	def __sub__(self, other):
		"""Return a ``Wrench`` that is the the difference between this ``Wrench`` and another wrench.

		Parameters
		----------
		other : wrench
			The wrench to subtract.

		Returns
		-------
		Wrench
			The resulting new ``Wrench``.
		"""
		return Wrench([
			self.wrench[0] - other[0],
			self.wrench[1] - other[1],
			self.wrench[2] - other[2],
			self.wrench[3] - other[3],
			self.wrench[4] - other[4],
			self.wrench[5] - other[5] ])
	
	def __neg__(self):
		"""Return the negated ``Wrench``.

		Returns
		-------
		Wrench
			The negated ``Wrench``.
		"""
		return Wrench([
			-1.0 * self.wrench[0],
			-1.0 * self.wrench[1],
			-1.0 * self.wrench[2],
			-1.0 * self.wrench[3],
			-1.0 * self.wrench[4],
			-1.0 * self.wrench[5] ])

	def __len__(self):
		return len(self.wrench)
	
	def __getitem__(self, item):
		return self.wrench[item]
	
	def __eq__(self, other):
		return np.allclose(self.wrench, other)

	# ==========================================================================
	# transformations
	# ==========================================================================

	def rotated(self, rotation_matrix):
		"""Rotate a wrench or force-torque vector given a rotation matrix.

		Parameters
		----------
		rotation_matrix: matrix, 3x3 rotation matrix [[xx, yx, zx], [xy, yy, zy], [xz, yz, zz]]

		Returns
		-------
		Wrench
			rotated wrench.
		"""
		rotated_wrench = []
		for i in range(2):
			vector = self.wrench[i * 3: i * 3 + 3]
			# Transpose to [[x], [y], [z]]
			vector = np.matrix(vector).T
			# Apply rotation
			vector = np.matmul(rotation_matrix, vector).T
			# Convert back to vector
			vector = vector.tolist()[0]
			# Include in rotated_wrench
			rotated_wrench.extend(vector)
		return Wrench(rotated_wrench)

	@property
	def unbiased(self):
		"""
		Removes the force and torque in effect of bias.
		"""
		if self.bias:
			return self.__sub__(self.bias)
		else:
			return None

	def gravity_compensated(self, pose):
		"""
		Removes the force and torque in effect of gravity.
		"""
		if (pose is not None):
			# transform gravity to FT Sensor coordinate system (FTSCS)
			FTSCS_transformation = cg.Transformation.from_frame_to_frame(pose, cg.Frame.worldXY())
			gravity_vector_FTSCS = self.gravity_vector_WCS.transformed(FTSCS_transformation)
			
			### F gravity compensation
			# F = gravity * mass
			F_gravity = gravity_vector_FTSCS * self.mass
			
			### T gravity compensation
			# T = (lever_arm * m) X gravity_vector_FTSCS
			T_gravity = cg.Vector( *cg.cross_vectors((self.center_of_mass * self.mass), gravity_vector_FTSCS) )
			
			gravity_wrench = [F_gravity.x, F_gravity.y, F_gravity.z, T_gravity.x, T_gravity.y, T_gravity.z]
			return self.__sub__(gravity_wrench)
		else:
			return None

	def unbiased_and_gravity_compensated(self, pose):
		return self.unbiased.gravity_compensated(pose)


# ==============================================================================
# Main
# ==============================================================================

if __name__ == '__main__':

	from wrench import Wrench
	from pose import Pose

	b = [-29.70, 0.34, 178.32, 3.46, 5.44, 0.78]
	w = Wrench([-27.9185252588, -2.64034876282, 94.2414889533, 0.288691484311, 2.95823662774, 0.657809409512], bias=b)
	p = Pose([0,0,0], [1,0,0,0])

	print("wrench =", w)
	print("pose =", p)
	print("bias =", w.bias)
	print("unbiased wrench =", w.unbiased)
	print("gravity compensated wrench =", w.gravity_compensated(p))
	print("unbiased and gravity compensated wrench =", w.unbiased_and_gravity_compensated(p))
	print((w.unbiased_and_gravity_compensated(p)) == (w.gravity_compensated(p) - w.bias ))

