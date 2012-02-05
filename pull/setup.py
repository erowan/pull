import os

from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
README = open(os.path.join(here, 'README.txt')).read()

requires = [
     # Testing dependencies
    'coverage',
    'nose',
    ]

setup(name='pull',
      version='0.1',
      description='Web Scraper scaffolding library',
      long_description=README,
      classifiers=[
        "Programming Language :: Python",
        "Topic :: Internet :: WWW/HTTP",
        ],
      author='Rowan Shulver',
      author_email='rowan.shulver@gmail.com',
      url='',
      keywords='pull web scraper',
      packages=find_packages(),
      include_package_data=True,
      zip_safe=False,
      install_requires = requires,
      tests_require= requires,
      test_suite="pull",
      
      )

