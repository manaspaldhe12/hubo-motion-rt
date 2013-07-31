

from __future__ import with_statement # for python 2.5
__author__ = 'Manas Paldhe'
__license__ = 'GPLv3 license'


import hubo_ach as hubo
import ach
import sys
import time
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(hubo.HUBO_CHAN_STATE_NAME)
r = ach.Channel(hubo.HUBO_CHAN_REF_NAME)
s.flush()
r.flush()

# feed-forward will now be refered to as "state"
state = hubo.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = hubo.HUBO_REF()

# Get the current feed-forward (state)
[statuss, framesizes] = s.get(state, wait=False, last=False)

#Set Left Elbow Bend (LEB) and Right Shoulder Pitch (RSP) to -0.2 rad and 0.1 rad respectively
ref.ref[hubo.LEB] = -0.2
ref.ref[hubo.RSP] = 0.1

# Print out the actual position of the LEB
print "Joint = ", state.joint[hubo.LEB].pos

# Print out the Left foot torque in X
print "Mx = ", state.ft[hubo.HUBO_FT_L_FOOT].m_x

# Write to the feed-forward channel
r.put(ref)

# Close the connection to the channels
r.close()
s.close()
