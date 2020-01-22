Cvserver is an extension of the multiServer code from FRC's WPILib
Suite. In addition to support for streaming from multiple cameras, it
provides functionality for tracking 2019 vision targets as well as
alignment lines.

Alignment line tracking

The alignment line tracking is enabled by adding an extra property in
the camera's custom properties JSON. If property with a name
"track_line" is present, and its value is "true", the camera is used
to track alignment lines. An example of custom properties that enable
the alignment line tracking:

[
	{"name":"track_line","value":true},
	{"name":"line_coeff","value":5.34}
]

The code expects a single line that is brighter than the rest of the
image. It calculates the angle of the line relative to a vertical
line. Possible angle values are from -90 degrees to 90 degrees.
Positive angle values mean that the alignment line is on the right
side of a vertical line. The angle is negative when the alignment
line is on the left side of a vertical line.

The code also calculates the displacement of the center of the line.
The value by default is between -0.5 and 0.5, where -0.5 means that
the line is at the left edge of the image, when it is 0.5 the line is
at the right edge.

Cvserver provides support for an optional property, "line_coeff"
which, when specified, is used to multiply the displacement value.
Line_coeff can be used to convert the displacement value to real
distance.

The code publishes two values in the Network Table, subtable "Line":

	Angle: the angle of the alignment line
	X: the displacement

Vision target tracking

The vision target tracking is enabled by adding an extra property in
the camera's custom properties JSON. If property with a name
"track_target" is present, and its value is "true", the camera is
used to track vision targets. An example of custom properties that
enable the vision target tracking:

[
                { "name": "track_target", "value": true },
                { "name": "h_low", "value": 60 },
                { "name": "s_low", "value": 200 },
                { "name": "v_low", "value": 136 },
                { "name": "h_high", "value": 95 },
                { "name": "s_high", "value": 255 },
                { "name": "v_high", "value": 255 }
]

The tracking works best when the exposure time is low and the camera
is encircled by LEDs that emit light of a specific color. Properties
{hsv}_low define the lowest hue/saturation/value values for the
target. Respectively, {hsv}_high values define the maximum values for
the target color. The example above defines the full HSV range (Note:
OpenCV divides the Hue range [0:360] by two so it can fit in an 8-bit
value). Also, instead of range [0:100] for Saturation and Value,
OpenCV uses the range of [0:255].

The code can track multiple targets. For each of them it calculates
(x,y,z) distances to the plane of the target as well as (yaw, pitch,
roll) angles to the floor point at the center of the target. The
distances are in centimeters, the angles are in degrees.

The code only posts the values for the target that is closest to the
center of the image. It creates a subtable "Targett" and publishes the
following values:

	X	distance on the X coordinate
	Y	distance on the Y coordinate
	Z	distance on the Z coordinate
	Yaw	rotatation parallel to the floor
	Pitch	rotation on a plane perpendicular both to the floor
		and the target plane
	Roll	rotation on a plane parallel to the target plane

Modifying the values

The additional stream published by cvserver provides convenient way to
change the custom properties. Once you are happy with the settings,
you can use the "Source Config JSON" link to get the complete
configuration, and you can copy the "properties" value to the Custom
Properties JSON for the camera. Make sure you only copy the value for
the propertties. If the "Source Config JSON" looks like:

	{
	    "fps": 30,
	    "height": 480,
	    "pixel format": "mjpeg",
	    "properties": [
	        {
	            "name": "track_target",
	            "value": true
	        },
	        {
	            "name": "track_line",
	            "value": false
	        },
	        {
	            "name": "h_low",
	            "value": 0
	        },
	        {
	            "name": "s_low",
	            "value": 0
	        },
	        {
	            "name": "v_low",
	            "value": 0
	        },
	        {
	            "name": "h_high",
	            "value": 180
	        },
	        {
	            "name": "s_high",
	            "value": 255
	        },
	        {
	            "name": "v_high",
	            "value": 255
	        },
	        {
	            "name": "line_coeff",
	            "value": "1.000000"
	        },
	        {
	            "name": "exposure_auto",
	            "value": 1
	        },
	        {
	            "name": "exposure_absolute",
	            "value": 0
	        }
	    ],
	    "width": 640
	}
	
You only need to copy:
	
	[
	        {
	            "name": "track_target",
	            "value": true
	        },
	        {
	            "name": "track_line",
	            "value": false
	        },
	        {
	            "name": "h_low",
	            "value": 0
	        },
	        {
	            "name": "s_low",
	            "value": 0
	        },
	        {
	            "name": "v_low",
	            "value": 0
	        },
	        {
	            "name": "h_high",
	            "value": 180
	        },
	        {
	            "name": "s_high",
	            "value": 255
	        },
	        {
	            "name": "v_high",
	            "value": 255
	        },
	        {
	            "name": "line_coeff",
	            "value": "1.000000"
	        },
	        {
	            "name": "exposure_auto",
	            "value": 1
	        },
	        {
	            "name": "exposure_absolute",
	            "value": 0
	        }
	]

JSON is very picky, make sure that there are no commas after the last
entry.

Important: The camera needs to be calibrated, otherwise the distances
and angles are not correct. Csrv.cpp has a cameraMatrix variable that
contains the particular calibration parameters for the camera used.
There are values for the Raspberry Pi camera as well as Microsoft
HD3000. If you need to calibrate another camera, use OpenCV
interactive camera calibration tool.
