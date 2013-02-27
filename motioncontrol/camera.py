"""
A class to read the data from a Newport HD-LBP laser beam profiler.
"""

import serial

class LaserBeamProfiler(object):
  """
  Provides an interface to a Newport HD-LBP over serial link.
  """

  def __init__(self, device):
    """
    Establish serial communication with an HD-LBP.
    """
    self.device = device
    self.io = serial.Serial(device, 57600, timeout=1)
    self.keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r',
                 'level_1', 'level_2', 'level_3',
                 'width_1', 'width_2', 'width_3',
                 'height_1', 'height_2', 'height_3',
                 'power']

  def read(self):
    """
    Read the latest recorded data.

    The output is a dictionary that contains the following quantities:
      'time' - Time since camera reset of measurement (seconds)
      'power' - Power deposited on CCD (mW). Requires calibration for accuracy.
      'centroid_x' - Centroid horizontal position from center (micrometers)
      'centroid_y' - Centroid vertical position from center (micrometers)
      'centroid_r' - Image radius (micrometers)

      The following sizes of the x-,y-projected image are also given. See
      HD-LBP documentation for more information.
      'level_1' - Projection level 1 (13.5%)
      'level_2' - Projection level 2 (50.0 %)
      'level_3' - Projection level 3 (80.0%)
      'width_1' - Projection width at level 1
      'width_2' - Projection width at level 2
      'width_3' - Projection width at level 3
      'height_1' - Projection height at level 1
      'height_2' - Projection height at level 1
      'height_3' - Projection height at level 1
    """
    buffer = ''
    while True:
      buffer = buffer + self.io.read(self.io.inWaiting())
      if ' \n' in buffer:
        lines = buffer.split(' \n')
        if len(lines) > 2:
          last_full_line = lines[-2]
          header, values = last_full_line.split(" ",1)
          floats = [float(x) for x in values.split()]
          if len(floats) == 14:
            output = dict(zip(self.keys, floats))
            return output
