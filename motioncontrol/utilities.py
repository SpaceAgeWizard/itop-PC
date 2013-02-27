"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array
import time
import math

def pauseForStage(stage):
  """
  Hold python execution in null loop until stage is stopped.
  """
  while stage.getMotionStatus():
    pass

def clamp(value, min_value, max_value):
  """
  Constrain a value to between a minimum and maximum.
  """
  return max(min(max_value, value), min_value)

class ConstrainToBeam(object):
  """
  For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.
  """

  def __init__(self, controller, group_id, camera, **kwargs):
    """
    Initialize a constraint on the movement of a LBP on a stage group to travel
    along a beam.

    Keyword arguments accepted to constrain the region of group travel.
    Assume group of ILS250CC stages if no kwargs given. Units are those
    that the stages are currently programmed to.

    Option=default values are as follows:
    lower_limit_x=-125 - Lower travel limit.
    lower_limit_z=-125 - Lower travel limit.
    upper_limit_x=125 - Upper travel limit.
    upper_limit_z=125 - Upper travel limit.
    power=level - Beam-in-view power threshold.
    """
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
    self.lower_limit_x = kwargs.pop('lower_limit_x', -125)
    self.upper_limit_x = kwargs.pop('upper_limit_x',  125)
    self.lower_limit_z = kwargs.pop('lower_limit_z', -125)
    self.upper_limit_z = kwargs.pop('upper_limit_z',  125)
    self.power_level = kwargs.pop('power_level', 0.003)
    self.r_initial = array([0, 0])
    self.r_final = array([0, 0])
    self.slope = array([0, 0])

  def search(self, start_point, stop_point, step_size):
    """
    Searches through a range of position steps for the beam.

    All arguments given in millimeters.
    """
    beam_seen = False
    x_start = clamp(start_point[0], self.lower_limit_x, self.upper_limit_x)
    x_stop = clamp(stop_point[0], self.lower_limit_x, self.upper_limit_x)
    z_start = clamp(start_point[1], self.lower_limit_z, self.upper_limit_z)
    z_stop = clamp(stop_point[1], self.lower_limit_z, self.upper_limit_z)
    displacement = math.hypot((x_stop - x_start), (z_stop - z_start))
    if displacement == 0:
      return None
    count = int(math.floor(displacement/abs(float(step_size))))
    x_step = (x_stop - x_start) / float(count)
    z_step = (z_stop - z_start) / float(count)
    x_steps = [x_start + x_step*i for i in xrange(count)]
    z_steps = [z_start + z_step*i for i in xrange(count)]
    steps = map(list, zip(x_steps, z_steps))
    steps.append([x_stop, z_stop])
    for position in steps:
      self.controller.groupMoveLine(self.group_id, position)
      while self.controller.groupIsMoving(self.group_id):
        if (self.camera.read()['power'] > self.power_level):
          beam_seen = True
      cam_reading = self.camera.read()
      if (cam_reading['power'] < self.power_level and beam_seen):
        print "Passed the beam."
        return position
      elif (cam_reading['power'] > self.power_level):
        beam_seen = True
        beam_offset = cam_reading['centroid_x'] - (x_step * 500.)
        if (x_step > 0 <= beam_offset) or (x_step < 0 >= beam_offset):
          print "Passed the beam."
          return position
    else:
      print "Something's not right..."
      cam_reading = self.camera.read()
      if cam_reading['power'] > self.power_level:
        if -20 < cam_reading['centroid_x'] < 20:
          print "On the beam within thermal fluctuations."
          return self.controller.groupPosition(self.group_id)
      elif beam_seen:
        print "ERROR: Beam center not in reach of camera center."
      else:
        # The beam was not detected! The beam may be out of range, blocked, or
        # the stage moved too fast to register the beam on camera over with
        # the given serial polling frequency. Also, there may be a bug in the
        # code.
        print "ERROR: Beam not detected."
        self.controller.groupOff(self.group_id)
        return [0, 0]

  def findBeam(self, z_coordinate):
    """
    Centers the beam on a camera attached to given stage group.
    """
    self.controller.groupVelocity(self.group_id, 30)
    start_point = [self.lower_limit_x, z_coordinate]
    self.controller.groupMoveLine(self.group_id, start_point)
    self.controller.pauseForGroup(self.group_id)
    time.sleep(1)
    self.controller.groupVelocity(self.group_id, 5)
    scan_steps = [50.00, 25.00, 5.00, 1.00, 0.25, 0.12, 0.05, 0.01]
    scan_range = self.upper_limit_x - self.lower_limit_x
    for step_number, step_size in enumerate(scan_steps):
      sign = (-1)**step_number
      stop_point = map(sum, zip(start_point, [sign*scan_range, 0]))
      start_point = self.search(start_point, stop_point, step_size)
      scan_range = 2.0 * step_size
    return self.controller.groupPosition(self.group_id)

  def findSlope(self):
	"""
	Finds the trajectory of the stages needed to keep a beam centered on camera.
	"""
	self.r_initial = array(self.findBeam(self.lower_limit_z))
	self.r_final = array(self.findBeam(self.upper_limit_z))
	self.slope = self.r_final - self.r_initial
	return self.slope

  def position(self, fraction):
    """
    Moves the stage group along the currently defined trajectory.
    """
    if fraction < 0 or fraction > 1:
      print "Cannot exceed stage limits."
    else:
      return (self.r_initial + fraction * self.slope).tolist()

class FocalPoint(object):
  def __init__(self, controller, group_id, camera, **kwargs):
	self.controller = controller
	self.mirror = self.controller.axis1
	self.group_id = group_id
	self.camera = camera
	self.trajectory = ConstrainToBeam(self.controller, self.group_id, self.camera)
	self.beam_crossing_found = False
	##
	self.lower_limit_x = kwargs.pop('lower_limit_x', -125)
	self.upper_limit_x = kwargs.pop('upper_limit_x',  125)
	self.lower_limit_z = kwargs.pop('lower_limit_z', -125)
	self.upper_limit_z = kwargs.pop('upper_limit_z',  125)
	self.power_level = kwargs.pop('power_level', 0.003)
	self.slope = array([0, 0])
	self.r_focal = array([0, 0])
	##

  def moveOnBeam(self, position):
	self.controller.groupMoveLine(self.group_id,
        self.trajectory.position(position))

  def findFocalPoint(self):#, mirror_position):
	self.mirror.on()
	#self.mirror.position(250 - mirror_position)
	#pauseForStage(self.mirror)
    # Block the free beam. Done manually for now. TODO: get an automatic block.
	##
	unblockedPos = 50
	blockedPos = 0
	self.mirror.acceleration(50)
	self.mirror.velocity(30)
	self.mirror.position(blockedPos)
	pauseForStage(self.mirror)
	##
	
	self.slope = self.trajectory.findSlope()
	
    # Unblock the free beam. Done manually for now.
	##
	self.mirror.position(unblockedPos)
	pauseForStage(self.mirror)
	##
	
	self.controller.groupVelocity(self.group_id, 5)
	#
	# move cam to focal point
	
	#1: move along beam
	#2: find power spike
	#3: refine position
	#self.r_focal = array(self.findBeam2(self.lower_limit_z))
	self.r_focal = array(self.findBeam2(self.slope))
	#
	#
	
  def searchAlongBeam(self, start_point, stop_point, step_size):
    """
    Searches through a range of position steps for the beam.

    All arguments given in millimeters.
    """
    beam_seen = False
    x_start = clamp(start_point[0], self.lower_limit_x, self.upper_limit_x)
    x_stop = clamp(stop_point[0], self.lower_limit_x, self.upper_limit_x)
    z_start = clamp(start_point[1], self.lower_limit_z, self.upper_limit_z)
    z_stop = clamp(stop_point[1], self.lower_limit_z, self.upper_limit_z)
    displacement = math.hypot((x_stop - x_start), (z_stop - z_start))
    if displacement == 0:
      return None
    count = int(math.floor(displacement/abs(float(step_size))))
    x_step = (x_stop - x_start) / float(count)
    z_step = (z_stop - z_start) / float(count)
    x_steps = [x_start + x_step*i for i in xrange(count)]
    z_steps = [z_start + z_step*i for i in xrange(count)]
    steps = map(list, zip(x_steps, z_steps))
    steps.append([x_stop, z_stop])
    for position in steps:
      #self.controller.groupMoveLine(self.group_id, position)
	  #self.moveOnBeam(self, position)
	  #self.controller.groupMoveLine(self.group_id,self.trajectory.position(position))
	  self.controller.groupMoveLine(self.group_id, position)
	  while self.controller.groupIsMoving(self.group_id):
		if (self.camera.read()['power'] > self.power_level):
			##
			#define new power level based on level of a single beam
			##
			beam_seen = True
	  cam_reading = self.camera.read()
	  if (cam_reading['power'] < self.power_level and beam_seen):
			print "Passed the beam."
			return position
	  elif (cam_reading['power'] > self.power_level):
			beam_seen = True
			beam_offset = cam_reading['centroid_x'] - (x_step * 500.)
			if (x_step > 0 <= beam_offset) or (x_step < 0 >= beam_offset):
				print "Passed the beam."
				return position
    else:
      print "Something's not right..."
      cam_reading = self.camera.read()
      if cam_reading['power'] > self.power_level:
        if -20 < cam_reading['centroid_x'] < 20:
          print "On the beam within thermal fluctuations."
          return self.controller.groupPosition(self.group_id)
      elif beam_seen:
        print "ERROR: Beam center not in reach of camera center."
      else:
        # The beam was not detected! The beam may be out of range, blocked, or
        # the stage moved too fast to register the beam on camera over with
        # the given serial polling frequency. Also, there may be a bug in the
        # code.
        print "ERROR: Beam not detected."
        self.controller.groupOff(self.group_id)
        return [0, 0]

  def findBeam2(self, slope):
	"""
	Centers the beam on a camera attached to given stage group.
	"""
	self.controller.groupVelocity(self.group_id, 30)
    #start_point = [self.lower_limit_x, z_coordinate]
	##
	#Set start_point to current position
	start_point = self.controller.groupPosition(self.group_id)
	##
	#don't move to "start position"
    #self.controller.groupMoveLine(self.group_id, start_point)
	self.controller.pauseForGroup(self.group_id)
	time.sleep(1)
	self.controller.groupVelocity(self.group_id, 5)
	scan_steps = [50.00, 25.00, 5.00, 1.00, 0.25, 0.12, 0.05, 0.01]
	#scan_range_x = self.upper_limit_x - self.lower_limit_x
	scan_range_x = slope[0]
	scan_range_z = slope[1]
	for step_number, step_size in enumerate(scan_steps):
		sign = (-1)**step_number
		#stop_point = map(sum, zip(start_point, [sign*scan_range, 0]))
		stop_point = map(sum, zip(start_point, [sign*scan_range_x, sign*scan_range_z]))
		#start_point = self.search(start_point, stop_point, step_size)
		start_point = self.searchAlongBeam(start_point, stop_point, step_size)
		scan_range = 2.0 * step_size
	return self.controller.groupPosition(self.group_id)
	
  def outputInfo(self):
	#grab all variables to output
	#print "temp output"
	#give the outut a name (ex A=output) and save to a file
	#f = open (pathname)
	#f.write ('string\n')
	#f.close()
	return 0