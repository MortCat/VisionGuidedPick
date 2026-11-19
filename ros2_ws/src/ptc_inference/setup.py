from setuptools import setup
setup(name='ptc_inference', version='0.1.0', packages=['ptc_inference'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/ptc_inference']),
                  ('share/ptc_inference', ['package.xml', 'README.md', 'requirements-inference.txt'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='MortCat', maintainer_email='mortcj0@gmail.com',
      description='RF-DETR ROS model process', license='MIT',
      entry_points={'console_scripts': ['inference = ptc_inference.inference_main:main']})
