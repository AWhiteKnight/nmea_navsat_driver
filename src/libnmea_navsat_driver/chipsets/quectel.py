# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Stephan Kunz
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial
import logging

# configuration commands and options of quectel chip

# Control the NMEA output 
#  0 GLL GLL interval - Geographic position - latitude longitude
#  1 RMC RMC interval - Recommended minimum specific GNSS sentence
#  2 VTG VTG interval - Course over ground and ground speed
#  3 GGA GGA interval - GPS fix data
#  4 GSA GSA interval - GNSS DOPS and active satellites
#  5 GSV GSV interval - GNSS satellites in view
#  6 GRS GRS interval – GNSS range residuals
#  7 GST GST interval – GNSS pseudorange error statistics
#  8 - 16 Reserved always 0
# 17 ZDA ZDA interval - Time and date
# 18 MCHN PMTKCHN interval - GNSS channel status
# The value indicates the interval of fixes <0-5> when to send the record
#                                   G R V G G G G G - - - - - - - - - Z T
#                                   L M T G S S R S - - - - - - - - - D K
#                                   L C G A A V S T - - - - - - - - - A C 
#                                                                       H
#                                                                       N
#SET_NMEA_OUTPUT_ALL    = '$PMTK314,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1'
SET_NMEA_OUTPUT_NO_RMC  = '$PMTK314,0,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0'
SET_NMEA_OUTPUT_RMC     = '$PMTK314,0,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0'
SET_NMEA_OUTPUT_DEFAULT = '$PMTK314,-1'

#$PMTK353,GPS_Enable,GLONASS_Enable,GALILEO_Enable,GALILEO_FULL_Enable,BEIDOU_Enable
SET_SATELLITES_DEFAULT  = '$PMTK353,1,0,0,0,1'
SET_USE_GALILEO_GLONASS = '$PMTK353,1,1,1,0,0'

SET_USE_SBAS  = '$PMTK313,1'
SET_SBAS_DGPS = '$PMTK301,2'


# Configures chip and than tries to change baudrate
def quectel_serial_init(ser, driver):
    # configure records to use    
    if driver.use_RMC:
        ser.write(create_command(SET_NMEA_OUTPUT_RMC))
    else:
        ser.write(create_command(SET_NMEA_OUTPUT_NO_RMC))

    # configure satellites to use
    useGalileoGLONASS = driver.declare_parameter('useGalileoGLONASS', False).value
    if useGalileoGLONASS:
        ser.write(create_command(SET_USE_GALILEO_GLONASS))
    else:
        ser.write(create_command(SET_SATELLITES_DEFAULT))

    # configure gps qualities
    ser.write(create_command(SET_USE_SBAS))
    ser.write(create_command(SET_SBAS_DGPS))
    
    return ser

# Reset chip to defaults
def quectel_serial_reset(ser, driver):
    if ser.is_open:
        driver.get_logger().info("Reset of quectel device")
        ser.write(create_command(SET_NMEA_OUTPUT_DEFAULT))
        ser.write(create_command(SET_SATELLITES_DEFAULT))
    else:
        driver.get_logger().error("Reset of quectel device not possible")

    return

# for checksum in create_command
DIGIT = '0123456789ABCDEF*'

def create_command(data: str):
    check = ord(data[1]) 
    for i in range(2, len(data)):
        check = check ^ ord(data[i]) 
    data = data + DIGIT[16] + DIGIT[int(check/16)]+ DIGIT[int(check%16)] + '\r\n'
    return data.encode('ASCII')
