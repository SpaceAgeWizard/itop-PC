from distutils.core import setup

setup(
    name='MotionControl',
    version='0.1.1',
    author='M.P. Belhorn',
    author_email='matt.belhorn@gmail.com',
    packages=['motioncontrol'],
    scripts=[],
    url='https://github.com/emmpbee/ucbelle-optics',
    license='LICENSE.txt',
    description='EPS300 based stage control.',
    long_description=open('README.txt').read(),
    install_requires=[
        "pyserial >= 2.6",
    ],
)
