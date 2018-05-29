#!/usr/bin/python
import sys, string, math

if len(sys.argv) != 2:
	print "Usage:", sys.argv[0], "file"
	sys.exit(1)

last_x = 0
last_y = 0
last_psi = 0
last_v = 0
lf_count = 0
lf_sum = 0
def process_data(step, x, y, psi, mph, steering, throttle):
    global last_x, last_y, last_psi, last_v
    global lf_count, lf_sum

    # Convert speed to meters per second.
    v = mph * 0.44704

    if steering >= -1 and throttle >= -1:
        dx   = x   - last_x
        dy   = y   - last_y
        dpsi = psi - last_psi

        if v >= 1 and dpsi >= 0.001:
            dt = math.sqrt(dx*dx + dy*dy) / v
            if dt >= 0.1:
                print "step", step, ": dpsi=", dpsi, " steer=", steering, " v=", last_v, " dt=", dt

                # Verify: dpsi = -v * steer * dt / LF
                lf = -v * steering * dt / dpsi
                print "    lf=", lf
                if lf > 0.01:
                    lf_count += 1
                    lf_sum += lf

    last_x = x
    last_y = y
    last_psi = psi
    last_v = v

#
# Process the trace file from mpc CarND simulator.
#
f = open (sys.argv[1])
steering = -2
throttle = -2
for line in f.readlines():
    word = line.split()

    if word[0] == "---":
        # --- step 0
        step = int(word[2])
        #print "--- step =", step
        continue

    if word[0] == "steering":
        # steering = -0.111887  throttle = 1
        steering = float(word[2])
        throttle = float(word[5])
        #print "    steering =", steering, ", throttle =", throttle
        continue

    if word[0] == "x":
        # x = -40.62, y = 108.73, psi = 3.73365, v = 0.438009
        x = float(word[2])
        y = float(word[5])
        psi = float(word[8])
        mph = float(word[11])
        #print "    x =", x, ", y =", y, ", psi =", psi, " mph =", mph

        # Process accumulated data.
        process_data(step, x, y, psi, mph, steering, throttle)
        continue

print "mean lf=", lf_sum / lf_count
