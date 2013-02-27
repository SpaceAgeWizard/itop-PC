=============
MotionControl
=============

Motion control, DAQ, analysis code, etc for the UC Belle group optics lab.

Simple usage::

    #!/usr/bin/python

    from motioncontrol import controller as sc

    eps = sc.StageController('COM3')
    mirror = eps.axis1
    ccd_x = eps.axis2
    ccd_y = eps.axis3

    ccd_x.goToHome()

StageController
===============

This module covers communication and control of an EPS300 controller itself.
It contains pointers to each of:

* axis1, axis2, and axis3 corresponding to the numbered
  axes on the controller. Unused axes are still given pointers. Communication
  to unused axes results in a controller error.

* The serial buffer on the EPS controller.

Stage
=====

The Stage module contains commands to manipulate and group individual stages.

Most stepper-based (particularly CC series) stages are accomodated, as well
as all stage formats (linear, rotational, pan-tilt, etc).


Not implemented controller functions.
=====================================
  
* Torque reduction - Not implememted.
* Microstep factor - Not implememted.
* Tachometer constant - Not implememted.
* Motor voltage - Not implememted.
* Master-slave jog update interval - Not implememted.
* Slave axis jog velocity coefficients - Not implememted.
* Left limit - Not implememted.
* Right limit - Not implememted.
* Master-slave relationship - Not implememted.
* Encoder resolution - Not implememted.
* Linear Compensation.
* Jog high speed.
* Jerk rate.
* Jog low speed.
* home search speed low.
* home search high speed.
* Home search mode.
* Update filter parameters.
* Base velocity for stepper motors.
* Define label.
* Jump to label.
* Generate service request.
* set device address.
* Digital Filters
* Set master-slave ratio.
* Set master-slave jog update interval.
* master-slave jog velocity coefficients.
* master-slave mode.
