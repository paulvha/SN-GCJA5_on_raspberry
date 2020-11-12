# SN-GCJA5 connected to Raspberry Pi

## ===========================================================

Software to connect an SN-GCJA5 with a Raspberry Pi using I2C or Serial. It is able to instruct,
read and display data from an SN-GCJA5. The monitor can optionally be
extended to include an SDS011 monitor and provide common output.

<br> A detailed description of the options and findings are in [sngcja5_on_raspberry.odt](extras/sngcja5_on_raspberry.odt) in the extras folder

<br> There is also a [library version](https://github.com/paulvha/SN-GCJA5) for the Arduino-variants

## Getting Started
This project is part of a number of projects to measure the air quality.
I bought a new sensor : SN-GCJA5 from Panasonic.
Having worked on other air quality sensors before thee aim is to compare
the output with other sensors and provide the learning.

The board is relative new at the time of writing this documentation and
as limited documents from supplier.

The monitor can optionally be extended to include a SDS011 monitor
and provide common output.

## Prerequisites
BCM2835 library (http://www.airspayce.com/mikem/bcm2835/) in case of I2C communication

## Software installation

Install latest from BCM2835 from : http://www.airspayce.com/mikem/bcm2835/

Note :
In case you only want to use serial communication, the BCM2835 library
is not needed. See 3.1.4 for setting the parameter of the document.

1. wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.68.tar.gz
2. tar -zxf bcm2835-1.68.tar.gz     // 1.68 was version number at the time of writing
3. cd bcm2835-1.68
4. ./configure
5. sudo make check
6. sudo make install

To compile the program, go in the directory and type:
    make

To create a build the SN-GCJA5 with SDS011 monitor (in case you have one):
    make BUILD=SDS011

## Program usage
### Program options
type ./gcja5 -h and/or see the detailed document

## Versioning

### version 1.0 / NOvember 2020
 * Initial version for Raspberry Pi 4 with PI-OS / Buster

## Author
 * Paul van Haastrecht (paulvha@hotmail.com)

## License
This project is licensed under the GNU GENERAL PUBLIC LICENSE 3.0

## Acknowledgements
Make sure to read the datasheet from Panasonic (also in extras).


