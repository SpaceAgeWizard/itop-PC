"""
This module provides methods for controlling stages attached to an EPS300
motion controller.
"""

class Stage(object):
  """
  A represenation of a robotic stage.
  
  The methods allow for the control and monitoring of specific servo or
  stepper motor driven robotic stages through a Newport ESP30X stage controller.
  """
  
  def __init__(self, axis, controller):
    """
    Initialize the stage. Requires a controller instance.
    """
    self.axis = str(axis)
    self.controller = controller
    
  def send(self, command, parameter=''):
    """
    Send a command to this axis.
    """
    self.controller.send(command, str(parameter), self.axis)

  def targetedPosition(self):
    """
    Returns the position the stage is currently targeting.
    """
    self.send('DP?')
    position = self.controller.read()
    print position, self.units()
    return float(position)
    
  def targetedVelocity(self):
    """
    Returns the stage's targeted velocity.
    """
    self.send('DV')
    velocity = self.controller.read()
    print velocity+self.units()+'/s'
    return float(velocity)
    
  def stageID(self):
    """
    Returns stage model and serial number.
    """
    self.send('ID')
    return self.controller.read()
    
  def getMotionStatus(self):
    """
    Return false for stopped, true for in motion.
    """
    self.send('MD?')
    if '0' in self.controller.read():
      return True
    else:
      return False
  
  def on(self):
    """
    Turns the axis motor on.
    """
    self.send('MO')
    
  def off(self):
    """
    Turns the axis motor off.
    """
    self.send('MF')
  
  def defineHome(self, position = '?'):
    """
    Sets the stage home position to given position in current units.
    """
    self.send('DH', position)
    if (position == '?'):
      position = self.controller.read()
      print position+self.units()
    return float(position)
    
  def moveToLimit(self, direction = '?'):
    """
    Given the argument '+' or '-', moves stage that hardware limit.
    """
    self.send('MT', position)
    if (position == '?'):
      finishedQ = self.controller.read()
      return int(finishedQ)
  
  def moveIndefinately(self, direction = '?'):
    """
    Initiates continuous motion in the given '+' or '-' direction.
    """
    self.send('MV', position)
    if (position == '?'):
      finishedQ = self.controller.read()
      return int(finishedQ)
    
  def moveToNextIndex(self, direction = '?'):
    """
    Moves to the nearest index in the given '+' or '-' direction.
    """
    self.send('MZ', position)
    if (position == '?'):
      finishedQ = self.controller.read()
      return int(finishedQ)
      
  def goToHome(self):
    """
    Moves the stage to the home position.
    """
    self.send('OR')
    
  def position(self, absolute_position = '?'):
    """
    Moves the stage to an absolute position.
    """
    self.send('PA', absolute_position)
    if (absolute_position == '?'):
      absolute_position = self.controller.read()
      print absolute_position, self.units()
    return float(absolute_position)
    
  def move(self, relative_position):
    """
    Moves the stage the given relative position.
    """
    self.send('PR', relative_position)
    return float(relative_position)
    
  def stop(self):
    """
    Stops motion on this axis with predefined acceleration.
    """
    self.send('ST')
  
  def followingError(self, error = '?'):
    """
    Sets or returns the maximum following error threshold.
    """
    self.send('FE', error)
    if (error == '?'):
      error = self.controller.read()
      print error+self.units()
    return float(error)
  
  def stepResoltion(self, resolution = '?'):
    """
    Sets or returns the encoder full-step resolution for a Newport Unidrive
    compatible programmable driver with step motor axis.
    """
    self.send('FR', resolution)
    if (resolution == '?'):
      resolution = self.controller.read()
      print resolution+self.units()
    return float(resolution)
    
  def gearRatio(self, gear_ratio = '?'):
    """
    Sets or returns the master-slave reduction ratio for a slave axis.
    
    Use this command very carefully. The slave axis will have its speed and
    acceleration in the same ratio as the position.
    Also, ensure that the ratio used for the slave axis does not cause
    overflow of this axis parameters (speed, acceleration), especially with
    ratios greater than 1. 
    """
    self.send('GR', gear_ratio)
    if (gear_ratio == '?'):
      gear_ratio = self.controller.read()
      print gear_ratio+self.units()
    return float(gear_ratio)
  
  def units(self, units = '?'):
    """
    Sets the stage displacement units from given integer.
    If no argument is given, current unit setting is reported.
    
    Possible units:
    0 -- Encoder counts         6 -- micro-inches
    1 -- Motor steps            7 -- degrees
    2 -- millimeters            8 -- gradient
    3 -- micrometers            9 -- radians
    4 -- inches                10 -- milliradian
    5 -- mils (milli-inches)   11 -- microradian
    """
    self.send('SN', units)
    if (units == '?'):
      response = self.controller.read()
      units = ['encoder-counts', 'motor-steps', 'mm', u'\u03BCm', 'in', 'mil',
               u'\u03BCin', u'\u00B0', 'grade', 'rad', 'mrad', u'\u03BCrad']
      return units[int(response)]
      
  def acceleration(self, acceleration = '?'):
    """
    Sets the stage acceleration.
    """
    self.send('AC', acceleration)
    if (acceleration == '?'):
      acceleration = self.controller.read()
      print acceleration+self.units()+'/s^2'
    return float(acceleration)
    
  def eStopAcceleration(self, acceleration = '?'):
    """
    Sets the stage emergency stop acceleration.
    """
    self.send('AE', acceleration)
    if (acceleration == '?'):
      acceleration = self.controller.read()
      print acceleration+self.units()+'/s^2'
    return float(acceleration)
    
  def deceleration(self, deceleration = '?'):
    """
    Sets te stage deceleration.
    """
    self.send('AG', deceleration)
    if (deceleration == '?'):
      deceleration = self.controller.read()
      print deceleration+self.units()+'/s^2'
    return float(deceleration)

  def accelerationLimit(self, acceleration = '?'):
    """
    Sets the maximum allowed stage acceleration/deceleration.
    
    Stage will error out if this limit is exceeded.
    """
    self.send('AU', acceleration)
    if (acceleration == '?'):
      acceleration = self.controller.read()
      print acceleration+self.units()+'/s^2'
    return float(acceleration)
    
  def backlashCompensation(self, compensation = '?'):
    """
    Set or report the backlash compensation in current units.
    
    Maximum compensation is equivelent of 10000 encoder counts.
    """
    self.send('BA', compensation)
    if (compensation == '?'):
      compensation = self.controller.read()
      print compensation+self.units()
    return float(compensation)
  
  def homePreset(self, home_position = '?'):
    """
    Sets the absolute position ascribed to the home position.
    """
    self.send('SH', home_position)
    if (home_position == '?'):
      home_position = self.controller.read()
      print home_position+self.units()
    return float(home_position)
  
  def velocity(self, velocity = '?'):
    """
    Sets the stage velocity.
    """
    self.send('VA', velocity)
    if (velocity == '?'):
      velocity = self.controller.read()
      print velocity+self.units()+'/s'
    return float(velocity)
  
  def velocityLimit(self, velocity = '?'):
    """
    Sets the maximum allowed stage velocity.
    
    Stage will error out if this limit is exceeded.
    """
    self.send('VU', velocity)
    if (velocity == '?'):
      velocity = self.controller.read()
      print velocity, self.units()+'/s'
    return float(velocity)
  
  def waitUntilPosition(position):
    """
    Pause EPS command execution until stage is at position.
    
    This does not pause execution of python code!
    """
    self.send('WP', position)
      
  def waitUntilStopped(time=''):
    """
    Pause EPS command execution time [ms] after stage is stopped.
    
    This does not pause execution of python code!
    """
    self.send('WS', time)

      