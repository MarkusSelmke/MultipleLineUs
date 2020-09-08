# MultipleLineUs
Processing code to coordinate and control multiple Line-Us devices (robotic arms)

This program lets you control 4 Line-Us devices simultaneously in a single coordinate system, preventing the devices collision by a bounding box method.

Start by renaming your Line-Us devices to "line-us1", "line-us2", ... 
Then start the program and connect to the devices by pressing 'C' + '1', 'C' + '2', ...
You can then control the 4 devices by mouse dragging (points A=device positions, point E = pen holders, point F=extension arm end points, depending on dragging mode), the arrow keys or by a supplemented positions.cvs file containing target coordinates for the optional extension arms (F points).

Set the floating variable e to 1 if you do not need these extension arms, or remove the code bits corresponding to F points altogether.

Keyboard Controls:
'C' + '1'-'4'			        : connect to Line-Us device 1, 2, 3, 4

'1'-'4'				            : select device 1, 2, 3, 4
'E'					              : edit-mode for E points
'F'					              : edit-mode for F points
'A'					              : edit-mode for A points
Up / Down / Left / Right	: move A/E/F points (edit mode)
Ctrl + Left / Right		    : inc./dec. step size

'+' / '-'					        : go through configurations
Ctrl + '1'-'4'			      : select configuration 1, 2, 3, 4
'S'					              : save as current configuration
'I'					              : initial / default configuration (no collision check!)
Enter				              : start/cancel move to configuration
Ctrl + Up / Down		      : inc./dec. movement step size 
'O'					              : home position (no collision check!)
