from pathlib import Path
import os
from setuptools import setup

ROOT = Path(__file__).resolve().parent
VENDOR = ROOT.parents[1] / 'vendor' / 'unitree_g1'
assets = [(f'share/stability_lab/vendor/unitree_g1/{p.parent.relative_to(VENDOR)}', [os.path.relpath(p, ROOT)])
          for p in VENDOR.rglob('*') if p.is_file()]
setup(name='stability_lab', version='0.1.0', packages=['stability_lab'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/stability_lab']),
                  ('share/stability_lab', ['package.xml']),
                  ('share/stability_lab/launch', ['launch/sim.launch.py']), *assets],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='SHAD ROS 2 course', maintainer_email='course@shad.example',
      description='G1 simulation physics public teaching kit',
      license='MIT (course code); BSD-3-Clause (vendored Unitree assets)',
      entry_points={'console_scripts': [
          'state_bridge = stability_lab.telemetry:main',
          'motion = stability_lab.experiment:motion_main',
          'observe = stability_lab.experiment:observe_main']})
