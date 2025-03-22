from setuptools import setup

package_name = 'odas_classifier'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Копируем CSV в share/odas_classifier/
        (f'share/{package_name}', ['share/yamnet_class_map_with_db.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Example ODAS classifier node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odas_classifier_node = odas_classifier.script.odas_classifier_node:main'
        ],
    },
)
