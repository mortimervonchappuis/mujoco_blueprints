from setuptools import setup, find_packages

setup(name='mujoco-blueprints', 
      vesion='0.0.1', 
      packages=find_packages(), 
      author='Mortimer von Chappuis', 
      install_requires=['numpy>=2.0.0', 
      			'imageio>=2.0.0', 
      			'mujoco>=3.3.2', 
      			'tqdm>=4.0.0'])