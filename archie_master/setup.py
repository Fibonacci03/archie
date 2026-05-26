from setuptools import setup
import os
from glob import glob

package_name = 'archie_master'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Registramos los archivos launch para que sean instalados
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Torres',
    maintainer_email='davidtorrest1097@gmail.com',
    description='Paquete maestro para el control y planificación de ARCHIE',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aquí vinculamos el comando con el script
            'plan_node = archie_master.plan:main',
            'write_word_node = archie_master.write_word:main',
            'archie_tracer = archie_master.archie_tracer:main',
        ],
    },
)