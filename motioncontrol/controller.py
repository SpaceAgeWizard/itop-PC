"""
This module provides methods for controlling and communicating with an EPS300
motion controller.
"""

import serial
import stage
import time
from utilities import pauseForStage

class StageController(object):
  """
  Encompasses serial I/O for Newport EPS300 motion controllers, controller
  operation methods plus instances of the stage I/O classes for each
  axis attached to the controller.
  """
  
  def __init__(self, serial_device):
    """
    Creates an I/O handle on a Newport EPS300 motion controller and its axes.
    
    Arguments:
    serial_device -- Path string to serial port used by the controller.
    """
    self.io = serial.Serial(serial_device, 19200, timeout = 1)
    self.io_end = '\r'
    self.axis1 = stage.Stage(1, self)
    self.axis2 = stage.Stage(2, self)
    self.axis3 = stage.Stage(3, self)
    self.readFirmwareVersion()
    
  def send(self, command, parameter = '', axis = ''):
    """
    Send a command to the controller.
    """
    self.io.write(str(axis) + str(command) + str(parameter) + self.io_end)
    
  def read(self):
    """
    Return a line read from the controller's serial buffer.
    """
    return self.io.readline()
    
  def reset(self):
    """
    Perform a full controller reset.
    """
    self.send('RS')
    
  def readStatus(self):
    """
    Read the first error message in the error FIFO buffer.
    """
    self.send('TS')
    self.read()
    
  def readActivity(self):
    """
    Read the activity register.
    """
    self.send('TX')
    self.read()
    
  def readError(self):
    """
    Read the first error message in the error FIFO buffer.
    """
    self.send('TB?')
    self.read()
    
  def readFirmwareVersion(self):
    """
    Report the controller firmware version.
    """
    self.send('VE?')
    self.read()
    
  def wait(milliseconds='0'):
    self.send('WT', milliseconds)
    
  def eStop(self):
    """
    Emergency stop all axes.
    
    The e-stop configuration for each axis is invoked when this command is sent.
    """
    self.send('AB')
    
  def abortProgram(self):
    """
    Abort execution of current program.
    
    Stages in motion will finish their last command.
    """
    self.send('AB')
  
  def groupAcceleration(self, group_id, acceleration = '?'):
    """
    Sets the vectorial acceleration for a group.
    
    This command overides individually set accelerations.
    """
    self.send('HA', acceleration, group_id)
    if (acceleration == '?'):
      acceleration = self.read()
      print acceleration
    
  def groups(self):
    """
    Returns the IDs of all defined groups.
    """
    self.send('HB')
    group_ids = self.read()
    print group_ids
    return group_ids
  
  def groupMoveArc(self, group_id, coordinates = '?'):
    """
    Moves a group along an arc.
    
    The arc is defined by a list of coordinates:
      [center_x, center_y, deltaTheta]
    """
    if (coordinates == '?'):
      self.send('HC', coordinates, group_id)
      coordinates = self.read()
      print coordinates
    else:
      self.send('HC', ",".join(map(str,coordinates)), group_id)
  
  def groupDeceleration(self, group_id, deceleration = '?'):
    """
    Sets the vectorial deceleration for a group.
    
    This command overides individually set decelerations.
    """
    self.send('HD', deceleration, group_id)
    if (deceleration == '?'):
      deceleration = self.read()
      print deceleration
    
  def groupEStopDeceleration(self, group_id, deceleration = '?'):
    """
    Sets the vectorial deceleration for a group emergency stop.
    
    This command overides individually set decelerations.
    """
    self.send('HE', deceleration, group_id)
    if (deceleration == '?'):
      deceleration = self.read()
      print deceleration
  
  def groupOff(self, group_id):
    """
    Turns off power to all axis in a group.
    """
    self.send('HF', '', group_id)
  
  def groupJerk(self, group_id, jerk = '?'):
    """
    Sets the vectorial jerk limit for a group.
    
    This command overides individually set jerks.
    """
    self.send('HJ', jerk, group_id)
    if (jerk == '?'):
      jerk = self.read()
      print jerk
  
  def groupMoveLine(self, group_id, coordinates = '?'):
    """
    Moves a group along a line.
    
    The line is defined by a list of endpoint coordinates:
      [axis1, axis2, ..., axisN]
    """
    if (coordinates == '?'):
      self.send('HL', coordinates, group_id)
      coordinates = self.read()
      print coordinates
    else:
      self.send('HL', ",".join(map(str,coordinates)), group_id)

  def groupCreate(self, group_id, axes = '?'):
    """
    Creates a group with ID group_id over the given axes.
    
    The axes are given as an ordered list of the physical axis numbers:
      [axis1, axis2, ..., axisN]
    """
    if (axes == '?'):
      self.send('HN', axes, group_id)
      axes = self.read()
      print axes
    else:
      self.send('HN', ",".join(map(str,axes)), group_id)

  def groupOn(self, group_id):
    """
    Turns on power to all axis in a group.
    """
    self.send('HO', '', group_id)
  
  def groupPosition(self, group_id):
    """
    Returns the position of all stages in a group.
    
    The positions are given as an ordered list of the group axis numbers:
      [axis1, axis2, ..., axisN]
    """
    self.send('HP', '', group_id)
    coordinates = [float(x.strip()) for x in self.read().split(',')]
    print coordinates
    return coordinates
    
  # Wait for group via point buffer. - NOT IMPLEMENTED.
  
  def groupStop(self, group_id):
    """
    Stops all motion on all axes in a group.
    """
    self.send('HS', '', group_id)
    
  def groupIsMoving(self, group_id):
    """
    Queries controller if group is in motion or stopped.
    """
    self.send('HS', '?', group_id)
    response = self.read()
    if '0' in response:
      return True
    else:
      return False
      
  def pauseForGroup(self, group_id):
    """
    Holds python execution in loop until group is stopped.
    """
    while self.groupIsMoving(group_id):
      pass
    
  def groupVelocity(self, group_id, velocity = '?'):
    """
    Sets the vectorial velocity limit for a group.
    
    This command overides individually set velocities.
    """
    self.send('HV', velocity, group_id)
    if (velocity == '?'):
      velocity = self.read()
      print velocity

  def groupWaitForStop(self, group_id, delay = '0'):
    """
    Pauses command execution until group has stopped for given delay [ms].
    """
    self.send('HW', delay, group_id)
    
  def groupDelete(self, group_id):
    """
    Deletes group with given ID.
    """
    self.send('HX', '', group_id)
    
  def groupSize(self, group_id):
    """
    Returns the size of the group with given group_id.
    """
    self.send('HZ', '', group_id)
    size = self.read()
    print size
    
  def initializeGroup(self, group_id, axes, **kwargs):
    """
    Creates and initilizes a group.
    
    Default values are provided for all kinematic parameters unless alternatives
    are given in the form of keyword arguments. The units are those that the
    stages are currently set to. Possible option=default values are:
     velocity=10 - Set group velocity (Units/s)
     acceleration=100 - Set group acceleration (Units/s^2)
     deceleration=100 - Set group deceleration (Units/s^s)
     jerk=1000 - Set group jerk rate (Units/s^3)
     estop=200 - Set group emergency stop deceleration (Units/s^2)
     
    See core group functions for usage of each parameter.
    """
    if str(group_id) in self.groups():
      self.groupDelete(group_id)
    stages = [self.axis1, self.axis2, self.axis3]
    for axis in axes:
      stage = stages[axis - 1]
      stage.on()
      stage.goToHome()
      pauseForStage(stage)
      time.sleep(1)
    self.groupCreate(group_id, axes)
    self.groupVelocity(group_id, kwargs.pop('velocity', 10))
    self.groupAcceleration(group_id, kwargs.pop('acceleration', 100))
    self.groupDeceleration(group_id, kwargs.pop('deceleration', 100))
    self.groupJerk(group_id, kwargs.pop('jerk', 1000))
    self.groupEStopDeceleration(group_id, kwargs.pop('estop', 200))
    self.groupOn(group_id)