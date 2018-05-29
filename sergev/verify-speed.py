#!/usr/bin/python
import sys, string, math

if len(sys.argv) != 2:
	print "Usage:", sys.argv[0], "file"
	sys.exit(1)

last_x = 0
last_y = 0
last_psi = 0
last_v = 0
dtx_count = 0
dtx_sum = 0
dty_count = 0
dty_sum = 0
def process_data(step, x, y, psi, mph, steering, throttle):
    global last_x, last_y, last_psi, last_v
    global dtx_count, dtx_sum, dty_count, dty_sum

    # Convert speed to meters per second.
    v = mph * 0.44704

    if steering >= -1 and throttle >= -1:
        dx   = x - last_x
        dy   = y - last_y
        dpsi = psi - last_psi
        dv   = v - last_v

        sinpsi = math.sin(last_psi)
        cospsi = math.cos(last_psi)

        #TODO
        print "step", step, ": dx=", dx, " dy=", dy, " cospsi=", cospsi, " sinpsi=", sinpsi, " v=", last_v

        if v >= 1:
            # Verify: dx = v * cospsi * dt
            if cospsi < -0.1 or cospsi > 0.1:
                dtx = dx / v / cospsi
                print "    dtx=", dtx
                if dtx > 0.01:
                    dtx_count += 1
                    dtx_sum += dtx

            # Verify: dy = v * sinpsi * dt
            if sinpsi < -0.1 or sinpsi > 0.1:
                dty = dy / v / sinpsi
                print "    dty=", dty
                if dty > 0.01:
                    dty_count += 1
                    dty_sum += dty

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

print "mean dtx=", dtx_sum / dtx_count
print "mean dty=", dty_sum / dty_count
print "mean k=", (dtx_sum + dty_sum) / (dtx_count + dty_count)
